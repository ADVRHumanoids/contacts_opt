#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <ifopt_problem/ForzaGiustaOpt.h>
#include <sensor_msgs/JointState.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosImpl.h>

std::map<std::string, Eigen::Vector6d> * g_fmap_ptr;
std::map<std::string, Eigen::Vector6d> * g_fmap_est_ptr;
std::map<std::string, Eigen::Matrix3d> * g_Rmap_ptr;
Eigen::Vector6d* wrench_manip_ptr;
std::map<std::string, Eigen::Vector6d> * g_fmap_arms_ptr;
Eigen::Matrix3d imu_R_w0;

void on_joint_pos_recv(const sensor_msgs::JointStateConstPtr& msg, XBot::JointNameMap * jmap)
{
    for(int i = 0; i < msg->name.size(); i++)
    {
        (*jmap)[msg->name.at(i)] = msg->position.at(i);
    }
}

void on_force_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
    tf::wrenchMsgToEigen(msg->wrench, g_fmap_ptr->at(l));
}

void on_force_est_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
    tf::wrenchMsgToEigen(msg->wrench, g_fmap_est_ptr->at(l));
}

void on_force_arms_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
    tf::wrenchMsgToEigen(msg->wrench, g_fmap_arms_ptr->at(l));
}

void on_wrench_recv(const geometry_msgs::WrenchStampedConstPtr& msg)
{
    tf::wrenchMsgToEigen(msg->wrench, wrench_manip_ptr[0]);
}

void on_normal_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
   
    Eigen::Matrix3d R;    
    
    if (msg->wrench.force.x == 0 && msg->wrench.force.y == 0 && msg->wrench.force.z == 1) 
    {      
        R.setIdentity();             
    }     
    
    else
    {

        double _tmp = std::sqrt( (msg->wrench.force.x*msg->wrench.force.x) + (msg->wrench.force.y*msg->wrench.force.y) );

	Eigen::Matrix3d R_tmp;
	
        R_tmp.coeffRef(0, 0) =  msg->wrench.force.y/_tmp;
        R_tmp.coeffRef(0, 1) = -msg->wrench.force.x/_tmp;

        R_tmp.coeffRef(1, 0) = (msg->wrench.force.x * msg->wrench.force.z)/_tmp;
        R_tmp.coeffRef(1, 1) = (msg->wrench.force.y * msg->wrench.force.z)/_tmp;
        R_tmp.coeffRef(1, 2) = -_tmp;

        R_tmp.coeffRef(2, 0) =  msg->wrench.force.x;
        R_tmp.coeffRef(2, 1) =  msg->wrench.force.y;
        R_tmp.coeffRef(2, 2) =  msg->wrench.force.z;
	
	R = R_tmp.transpose();
    }
    
    g_Rmap_ptr->at(l) = R; 
        
}


        

int main(int argc, char ** argv)
{
    
    ros::init(argc, argv, "forza_giusta_node");
    ros::NodeHandle nh("forza_giusta");
    ros::NodeHandle nh_priv("~");
    
    auto robot = XBot::RobotInterface::getRobot(XBot::ConfigOptionsFromParamServer());
    auto model = XBot::ModelInterface::getModel(XBot::ConfigOptionsFromParamServer());
    auto imu = robot->getImu().begin()->second;
    robot->sense(false);
    Eigen::Matrix3d tmp;
    imu->getOrientation(tmp);
    imu_R_w0 = tmp.transpose();
    
    XBot::JointNameMap jmap;
    robot->getMotorPosition(jmap);
        
    XBot::Cartesian::RosImpl ci;
      
    auto j_sub = ros::NodeHandle("cartesian").subscribe<sensor_msgs::JointState>("solution", 1, 
                            std::bind(on_joint_pos_recv, std::placeholders::_1, &jmap)
                        );
    
    robot->setControlMode(XBot::ControlMode::Effort());
    
    
    double rate = nh_priv.param("rate", 100.0);
    double mu = nh_priv.param("mu", 0.5);
    auto links = nh_priv.param("links", std::vector<std::string>());
    
    /* BLACKLISTED JOINTS */    
    auto blacklist = nh_priv.param("blacklist", std::vector<std::string>());
    
    
    std::map<std::string, XBot::ControlMode> ctrl_map;
    for(auto j : blacklist)
    {
        if(!robot->hasJoint(j))
        {
            if(!robot->hasChain(j))
            {
                throw std::runtime_error("Joint or chain '" + j + "' undefined");
            }
            else
            {
                for(auto jc: robot->chain(j).getJointNames())
                {
                    ROS_INFO("Joint '%s' is blacklisted", jc.c_str());
                    ctrl_map[jc] = XBot::ControlMode::Idle();
                }
            }
            
        }
        
        ROS_INFO("Joint '%s' is blacklisted", j.c_str());
        ctrl_map[j] = XBot::ControlMode::Idle();
    }
    robot->setControlMode(ctrl_map);
    
    /* TORQUE OFFSET */  
    auto tau_off_map = nh_priv.param("torque_offset", std::map<std::string, double>());
    XBot::JointNameMap tau_off_map_xbot(tau_off_map.begin(), tau_off_map.end());
    Eigen::VectorXd tau_offset;
    tau_offset.setZero(model->getJointNum());
    model->mapToEigen(tau_off_map_xbot, tau_offset);
    std::cout << "Torque offset: " << tau_offset.transpose() << std::endl;
    
    std::map<std::string, ros::Subscriber> sub_force_map;
    std::map<std::string, ros::Subscriber> sub_n_map;
    std::map<std::string, Eigen::Vector6d> f_ref_map;
    std::map<std::string, Eigen::Matrix3d> RotM_map;
    std::map<std::string, Eigen::Vector6d> f_ForzaGiusta_map;
            
    for(auto l : links)
    {
        auto sub_force = nh.subscribe<geometry_msgs::WrenchStamped>("force_ref/" + l,
								    1, 
								    boost::bind(on_force_recv, _1, l));
        
        auto sub_n = nh.subscribe<geometry_msgs::WrenchStamped>("normal/" + l,
                                                                1, 
                                                                boost::bind(on_normal_recv, _1, l));
        
        sub_force_map[l] = sub_force;
        f_ref_map[l] = Eigen::Vector6d::Zero();
        f_ForzaGiusta_map[l] = Eigen::Vector6d::Zero();
        sub_n_map[l] = sub_n;
        RotM_map[l] =  Eigen::Matrix3d::Identity();
                
        ROS_INFO("Subscribed to topic '%s'", sub_force.getTopic().c_str());
        ROS_INFO("Subscribed to topic '%s'", sub_n.getTopic().c_str());
    }
    
       
    std::vector<std::string> arms  = {"arm1_8", "arm2_8"};
    
    std::map<std::string, ros::Subscriber> sub_force_arms_map;
    std::map<std::string, Eigen::Vector6d> f_ref_arms_map;
    
    for(auto i : arms)
    {
	auto sub_force_arms = nh.subscribe<geometry_msgs::WrenchStamped>("force_arms/" + i,
									 1, 
									 boost::bind(on_force_arms_recv, _1, i));
	
	sub_force_arms_map[i] = sub_force_arms;
        f_ref_arms_map[i] = Eigen::Vector6d::Zero();	

    }
    
    std::vector<std::string> force_links  = {"wheel_1", "wheel_2", "wheel_3", "wheel_4", "arm1_8", "arm2_8"};
    
    std::map<std::string, ros::Subscriber> sub_force_est_map;
    std::map<std::string, Eigen::Vector6d> f_est_map;
    
    for(auto j : force_links)
    {
        auto sub_force_est = ros::NodeHandle("cartesian").subscribe<geometry_msgs::WrenchStamped>("force_estimation/" + j, 1, boost::bind(on_force_est_recv, _1, j));	
	sub_force_est_map[j] = sub_force_est;
	f_est_map[j] = Eigen::Vector6d::Zero();
	
	ROS_INFO("Subscribed to topic '%s'", sub_force_est.getTopic().c_str());
	
    }
    
    g_fmap_est_ptr = &f_est_map;
       
     
    auto sub_wrench_manip = nh.subscribe<geometry_msgs::WrenchStamped>("wrench_manip/",
                                                                       1, 
                                                                       boost::bind(on_wrench_recv, _1));
    
    Eigen::Vector6d wrench_manip; 
    wrench_manip.setZero();
    
    ROS_INFO("Subscribed to topic '%s'", sub_wrench_manip.getTopic().c_str());
    

    
    g_fmap_ptr = &f_ref_map;
    g_Rmap_ptr = &RotM_map;
    wrench_manip_ptr = &wrench_manip;
    g_fmap_arms_ptr = &f_ref_arms_map;
    
    auto force_opt = boost::make_shared<forza_giusta::ForceOptimization>(model, links, mu);
    
    
    Eigen::VectorXd tau;
    ros::Rate loop_rate(rate);
    
    bool log;
    nh_priv.param("log", log, false);
    
    XBot::MatLogger::Ptr logger;
    std::stringstream ss;
    ss << "/tmp/forza_giusta_node";

    if (log)
        logger = XBot::MatLogger::getLogger(ss.str());
    
    while(ros::ok())
    {
        ros::spinOnce();
        
        /* Sense robot state and update model */
        robot->sense(false);
        model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);
        model->setFloatingBaseState(imu,imu_R_w0);
// 	Eigen::Matrix3d tmp;
// 	imu->getOrientation(tmp);
// 	model->setFloatingBaseOrientation(imu_R_w0*tmp);
// 	Eigen::Vector3d tmp_v;
// 	imu->getAngularVelocity(tmp_v);
// 	model->setFloatingBaseAngularVelocity(imu_R_w0*tmp_v);
	model->update();
        
        Eigen::Affine3d fb_pose;
        model->getFloatingBasePose(fb_pose);
                
        /* Compute gcomp */
        model->computeGravityCompensation(tau);
        tau -= tau_offset;
        tau.head(6) += wrench_manip;
       
        force_opt->compute(tau, f_ref_map, RotM_map, f_ForzaGiusta_map);
        
        force_opt->log(logger);
               
        for(const auto& pair : f_ref_map)
        {        
            Eigen::Vector6d f_world = pair.second; 
            
            if (log) 
            {          
                logger->add("F_ifopt_" + pair.first, f_world);         
		for (auto pair : RotM_map)
		{
		  logger->add("RotM_map_" + pair.first, pair.second);   
		}
            }            
        }
        


        for(const auto& pair : f_ForzaGiusta_map)
        {
      
            Eigen::Vector6d f_world = pair.second;	    
         
            std::cout << "F_" + pair.first + ": " << f_world.head(3) << std::endl;
	    
            if (log) 
            {          
                logger->add("F_" + pair.first, f_world);
                                      
            }
            
            Eigen::MatrixXd J;           
            model->getJacobian(pair.first, J);
	    
            tau -= J.transpose() * f_world;
            
        }
        
	
	for (const auto& pair : f_ref_arms_map)
	{
            Eigen::Vector6d f_world = pair.second;
	    f_world.tail(3).setZero();
	    
	    std::cout << "F_" + pair.first + ": " << f_world.head(3) << std::endl;
	    
	    if (log) 
            {          
                logger->add("F_" + pair.first, f_world);
                                      
            }
                 
            Eigen::MatrixXd J;           
            model->getJacobian(pair.first, J);
            
            tau += J.transpose() * f_world;
	}	
	
	for(const auto& pair : f_est_map)
        { 
	   
	   Eigen::Affine3d pose;  
	   model->getPose(pair.first, pose);
	   
	   Eigen::Vector3d f_est_world;
	   f_est_world = pose.linear() * pair.second.head(3);
	   
	   logger->add("f_est_" + pair.first, f_est_world);  
	   
	}
        
        
        /* Send torque to joints */
        model->setJointEffort(tau);
        robot->setReferenceFrom(*model, XBot::Sync::Effort);
        robot->move();
        
        loop_rate.sleep();
    }
    
   if (log)  
       logger->flush();
    
    return 0;
    
}
