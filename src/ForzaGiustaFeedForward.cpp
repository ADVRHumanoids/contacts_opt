#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <ifopt_problem/ForzaGiustaOpt.h>


std::map<std::string, Eigen::Vector6d> * g_fmap_ptr;
std::map<std::string, Eigen::Matrix3d> * g_Rmap_ptr;
Eigen::Vector6d* wrench_manip_ptr;
double mu;


void on_force_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
    tf::wrenchMsgToEigen(msg->wrench, g_fmap_ptr->at(l));
}

void on_mu_recv(const geometry_msgs::WrenchStampedConstPtr& msg)
{
    mu = msg->wrench.force.x;
}

void on_wrench_recv(const geometry_msgs::WrenchStampedConstPtr& msg)
{
    tf::wrenchMsgToEigen(msg->wrench, wrench_manip_ptr[0]);
}

void on_normal_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
   
    Eigen::Matrix3d R; 
    R.setZero();
    
    auto _tmp = pow( (msg->wrench.force.x*msg->wrench.force.x) + (msg->wrench.force.y*msg->wrench.force.y) , 0.5 );

    R.coeffRef(0, 0) =  msg->wrench.force.y/_tmp;
    R.coeffRef(0, 1) = -msg->wrench.force.x/_tmp;

    R.coeffRef(1, 0) = (msg->wrench.force.x * msg->wrench.force.z)/_tmp;
    R.coeffRef(1, 1) = (msg->wrench.force.y * msg->wrench.force.z)/_tmp;
    R.coeffRef(1, 2) = -_tmp;

    R.coeffRef(2, 0) = msg->wrench.force.x;
    R.coeffRef(2, 1) = msg->wrench.force.y;
    R.coeffRef(2, 2) = msg->wrench.force.z;
    
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
    
    robot->setControlMode(XBot::ControlMode::Effort());
    
    
    double rate = nh_priv.param("rate", 100.0);
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
    
    
     
    auto sub_wrench_manip = nh.subscribe<geometry_msgs::WrenchStamped>("wrench_manip/",
                                                                       1, 
                                                                       boost::bind(on_wrench_recv, _1));
    
    Eigen::Vector6d wrench_manip; 
    wrench_manip.setZero();
    
    ROS_INFO("Subscribed to topic '%s'", sub_wrench_manip.getTopic().c_str());
    
    auto sub_mu = nh.subscribe<geometry_msgs::WrenchStamped>("mu/",
                                                              1, 
                                                              boost::bind(on_mu_recv, _1));
    
    double _mu = 0.5; 
    
    ROS_INFO("Subscribed to topic '%s'", sub_mu.getTopic().c_str());
    
    
    
    g_fmap_ptr = &f_ref_map;
    g_Rmap_ptr = &RotM_map;
    wrench_manip_ptr = &wrench_manip;
    _mu = mu;
    
    auto force_opt = boost::make_shared<forza_giusta::ForceOptimization>(model, links, _mu);
    
    Eigen::VectorXd tau;
    ros::Rate loop_rate(rate);
    
    while(ros::ok())
    {
        ros::spinOnce();
        
        /* Sense robot state and update model */
        robot->sense(false);
        model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);
        model->setFloatingBaseState(imu);
        model->update();
        
        /* Compute gcomp */
        model->computeGravityCompensation(tau);
        tau -= tau_offset;
        tau.head(6) += wrench_manip;
        
        force_opt->compute(tau, f_ref_map, RotM_map, f_ForzaGiusta_map);
        
        for(const auto& pair : f_ForzaGiusta_map)
        {
            Eigen::MatrixXd J;
            
            model->getJacobian(pair.first, J);
            
            Eigen::Vector6d f_world = pair.second;
         
//             std::cout<<"f_ForzaGiusta: "<<f_world.transpose()<<std::endl;
               
            tau += J.transpose() * f_world;
            
        }
        
        /* Send torque to joints */
        model->setJointEffort(tau);
        robot->setReferenceFrom(*model, XBot::Sync::Effort);
        robot->move();
        
        loop_rate.sleep();
    }
    
    return 0;
    
}