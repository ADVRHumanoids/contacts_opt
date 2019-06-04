#include <ros/ros.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/problem/Com.h>
#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/ros/RosImpl.h>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt_problem/ifopt_contacts.h>
#include <signal.h>
#include <ifopt_problem/ForcePublisher.h>

using namespace XBot;
using namespace XBot::Cartesian;
using namespace ifopt;

XBot::MatLogger::Ptr logger;
std::map<std::string, Eigen::Vector6d> * g_fmap_ptr;
Eigen::Affine3d pose;
Eigen::Vector3d com_ref, pelvis, wheel_1, wheel_2, wheel_3, wheel_4, left_arm, right_arm;
double ground_z;  

void set_up(XBot::Cartesian::RosImpl &ci)
{
  
    /* Lowering the Homing CoM */
    ci.getPoseFromTf("ci/com", "ci/world_odom", pose);
    pose.translation().z() -= 0.05;
    com_ref = pose.translation();
    ci.setTargetPose("com", pose, 2.0);
    ci.waitReachCompleted("com");

    ci.getPoseFromTf("ci/pelvis", "ci/world_odom", pose);
    pelvis = pose.translation();
    
    double wheel_offset_x = 0.35;
    double wheel_offset_y = 0.35;

    ci.getPoseFromTf("ci/wheel_1", "ci/world_odom", pose);
    wheel_1 = pose.translation();
    wheel_1.x() =  pelvis.x() + wheel_offset_x;
    wheel_1.y() =  pelvis.y() + wheel_offset_y;

    ci.getPoseFromTf("ci/wheel_2", "ci/world_odom", pose);
    wheel_2 = pose.translation();
    wheel_2.x() =  pelvis.x() + wheel_offset_x;
    wheel_2.y() =  pelvis.y() - wheel_offset_y;

    ci.getPoseFromTf("ci/wheel_3", "ci/world_odom", pose);
    wheel_3 = pose.translation();
    wheel_3.x() =  pelvis.x() - wheel_offset_x;
    wheel_3.y() =  pelvis.y() + wheel_offset_y;

    ci.getPoseFromTf("ci/wheel_4", "ci/world_odom", pose);
    wheel_4 = pose.translation();
    wheel_4.x() =  pelvis.x() - wheel_offset_x;
    wheel_4.y() =  pelvis.y() - wheel_offset_y;
    
    ci.getPoseFromTf("ci/arm1_8", "ci/world_odom", pose);
    left_arm = pose.translation();

    ci.getPoseFromTf("ci/arm2_8", "ci/world_odom", pose);
    right_arm = pose.translation();
    
    ground_z = wheel_1.z(); 

   
}

void set_legs_initial_stiffness(XBot::RobotInterface::Ptr robot)
{
   
    robot->setControlMode(XBot::ControlMode::Stiffness() + XBot::ControlMode::Damping());
    
    Eigen::VectorXd K_0, K_end(6);
    robot->leg(0).getStiffness(K_0);
    K_end << 500, 500, 500, 250, 50, 50;
    
    const int N_ITER = 400;
    for(int k = 0; k < N_ITER; k++)
    {
        for(int i = 0; i < 4; i++)
        {
            Eigen::VectorXd leg_k(robot->leg(i).getJointNum());
            leg_k = K_0 + k/(N_ITER-1)*(K_end-K_0);
            robot->leg(i).setStiffness(leg_k);
        }
        
        robot->move();
        
        ros::Duration(0.01).sleep();
    }
   
}

void set_leg_stiffness(XBot::RobotInterface::Ptr robot, const std::string chain)
{
        
    robot->setControlMode(XBot::ControlMode::Stiffness() + XBot::ControlMode::Damping());
    
    Eigen::VectorXd K_0, K_end(6);
    robot->chain(chain).getStiffness(K_0);
    K_end = K_0;
    K_end[1] = 20.0; // hip pitch
    K_end[2] = 20.0; // knee pitch
          
    const int N_ITER = 10;
    for(int k = 0; k < N_ITER; k++)
    {
	Eigen::VectorXd leg_k(robot->chain(chain).getJointNum());
        leg_k = K_0 + k/(N_ITER-1)*(K_end-K_0);
        robot->chain(chain).setStiffness(leg_k);
        
        robot->move();
        
        ros::Duration(0.01).sleep();
    }
   
}

void set_arms_low_stiffness(XBot::RobotInterface::Ptr robot)
{
          
    robot->setControlMode(XBot::ControlMode::Stiffness() + XBot::ControlMode::Damping());
    
    Eigen::VectorXd  K_0, K_end(7);
    robot->arm(0).getStiffness(K_0);
    K_end = K_0;
    K_end.head(4) << 50.0, 50.0, 50.0, 50.0;
  
    const int N_ITER = 10;
    for(int k = 0; k < N_ITER; k++)
    {
        for(int i = 0; i < 2; i++)
        {
            Eigen::VectorXd arm_k(robot->arm(i).getJointNum());
            arm_k = K_0 + k/(N_ITER-1)*(K_end-K_0);
            robot->arm(i).setStiffness(arm_k);
        }
        
        robot->move();
        
        ros::Duration(0.01).sleep();
    }
   
}

void set_arms_high_stiffness(XBot::RobotInterface::Ptr robot)
{
     
    robot->setControlMode(XBot::ControlMode::Stiffness() + XBot::ControlMode::Damping()); 
     
    Eigen::VectorXd  K_0, K_end(7);
    robot->arm(0).getStiffness(K_0);
    K_end = K_0;
    K_end.head(4) << 50.0, 50.0, 50.0, 50.0;
 
    const int N_ITER = 200;
    for(int k = 0; k < N_ITER; k++)
    {
        for(int i = 0; i < 2; i++)
        {
            Eigen::VectorXd arm_k(robot->arm(i).getJointNum());
            arm_k = K_0 + k/(N_ITER-1)*(K_end-K_0);
            robot->arm(i).setStiffness(arm_k);
        }
        
        robot->move();
        
        ros::Duration(0.01).sleep();
    }
   
}


void solve_initial_opt(const std::vector <Eigen::Vector6d> &p_bounds, const Eigen::Vector3d &F_max, const Eigen::Vector3d &C, const Eigen::Vector3d &R, const Eigen::Vector3d &P, 
		       const Eigen::VectorXd &p_ref, double &Wp, const Eigen::Vector3d &com_ref, double &Wcom, double &m, double &mu, const Eigen::Vector6d &ext_w, Eigen::VectorXd &x_opt)
{
    Problem nlp;
    IpoptSolver ipopt;
    ipopt.SetOption("derivative_test", "first-order");
    
    x_opt.setZero(39);

    auto p1 = std::make_shared<ExVariables> ("p1");
    auto p2 = std::make_shared<ExVariables> ("p2");
    auto p3 = std::make_shared<ExVariables> ("p3");
    auto p4 = std::make_shared<ExVariables> ("p4");
    auto p5 = std::make_shared<ExVariables> ("p5");
    auto p6 = std::make_shared<ExVariables> ("p6");

    auto F1 = std::make_shared<ExVariables> ("F1");
    auto F2 = std::make_shared<ExVariables> ("F2");
    auto F3 = std::make_shared<ExVariables> ("F3");
    auto F4 = std::make_shared<ExVariables> ("F4");
    auto F5 = std::make_shared<ExVariables> ("F5");
    auto F6 = std::make_shared<ExVariables> ("F6");

    auto n1 = std::make_shared<ExVariables> ("n1");
    auto n2 = std::make_shared<ExVariables> ("n2");
    auto n3 = std::make_shared<ExVariables> ("n3");
    auto n4 = std::make_shared<ExVariables> ("n4");
    auto n5 = std::make_shared<ExVariables> ("n5");
    auto n6 = std::make_shared<ExVariables> ("n6");

    auto com = std::make_shared<ExVariables> ("com");

    auto static_constr = std::make_shared<StaticConstraint>();

    auto SE_p1 = std::make_shared<SuperEllipsoidConstraint> ("p1");
    auto SE_p2 = std::make_shared<SuperEllipsoidConstraint> ("p2");
    auto SE_p3 = std::make_shared<SuperEllipsoidConstraint> ("p3");
    auto SE_p4 = std::make_shared<SuperEllipsoidConstraint> ("p4");
    auto SE_p5 = std::make_shared<SuperEllipsoidConstraint> ("p5");
    auto SE_p6 = std::make_shared<SuperEllipsoidConstraint> ("p6");

    auto fr_F1 = std::make_shared<FrictionConstraint> ("F1");
    auto fr_F2 = std::make_shared<FrictionConstraint> ("F2");
    auto fr_F3 = std::make_shared<FrictionConstraint> ("F3");
    auto fr_F4 = std::make_shared<FrictionConstraint> ("F4");
    auto fr_F5 = std::make_shared<FrictionConstraint> ("F5");
    auto fr_F6 = std::make_shared<FrictionConstraint> ("F6");

    auto n_p1 = std::make_shared<NormalConstraint> ("p1");
    auto n_p2 = std::make_shared<NormalConstraint> ("p2");
    auto n_p3 = std::make_shared<NormalConstraint> ("p3");
    auto n_p4 = std::make_shared<NormalConstraint> ("p4");   
    auto n_p5 = std::make_shared<NormalConstraint> ("p5");
    auto n_p6 = std::make_shared<NormalConstraint> ("p6");

    auto cost = std::make_shared<ExCost>();

    nlp.AddVariableSet(p1);
    nlp.AddVariableSet(p2);
    nlp.AddVariableSet(p3);
    nlp.AddVariableSet(p4);
    nlp.AddVariableSet(p5);
    nlp.AddVariableSet(p6);
    
    p1->SetBounds(p_bounds[0].head(3), p_bounds[0].tail(3));    
    p2->SetBounds(p_bounds[1].head(3), p_bounds[1].tail(3));
    p3->SetBounds(p_bounds[2].head(3), p_bounds[2].tail(3));
    p4->SetBounds(p_bounds[3].head(3), p_bounds[3].tail(3));
    p5->SetBounds(p_bounds[4].head(3), p_bounds[4].tail(3));
    p6->SetBounds(p_bounds[5].head(3), p_bounds[5].tail(3));

    nlp.AddVariableSet(F1);
    F1->SetBounds(-F_max, F_max);
    nlp.AddVariableSet(F2);
    F2->SetBounds(-F_max, F_max);
    nlp.AddVariableSet(F3);
    F3->SetBounds(-F_max, F_max);
    nlp.AddVariableSet(F4);
    F4->SetBounds(-F_max, F_max);
    nlp.AddVariableSet(F5);
    F5->SetBounds(-F_max, F_max);
    nlp.AddVariableSet(F6);
    F6->SetBounds(-F_max, F_max);

    nlp.AddVariableSet(n1);
    nlp.AddVariableSet(n2);
    nlp.AddVariableSet(n3);
    nlp.AddVariableSet(n4);
    nlp.AddVariableSet(n5);
    nlp.AddVariableSet(n6);

    nlp.AddVariableSet(com);
    
    static_constr->SetMass(m);
    static_constr->SetExternalWrench(ext_w);
    nlp.AddConstraintSet(static_constr);

    SE_p1->SetParam(C, R, P);
    nlp.AddConstraintSet(SE_p1);
    SE_p2->SetParam(C, R, P);
    nlp.AddConstraintSet(SE_p2);
    SE_p3->SetParam(C, R, P);
    nlp.AddConstraintSet(SE_p3);
    SE_p4->SetParam(C, R, P);
    nlp.AddConstraintSet(SE_p4);
    SE_p5->SetParam(C, R, P);
    nlp.AddConstraintSet(SE_p5);
    SE_p6->SetParam(C, R, P);
    nlp.AddConstraintSet(SE_p6);

    fr_F1->set_mu(mu);
    nlp.AddConstraintSet(fr_F1);
    fr_F2->set_mu(mu);
    nlp.AddConstraintSet(fr_F2);
    fr_F3->set_mu(mu);
    nlp.AddConstraintSet(fr_F3);
    fr_F4->set_mu(mu);
    nlp.AddConstraintSet(fr_F4);
    fr_F3->set_mu(mu);
    nlp.AddConstraintSet(fr_F5);
    fr_F4->set_mu(mu);
    nlp.AddConstraintSet(fr_F6);

    n_p1->SetParam(C, R, P);
    nlp.AddConstraintSet(n_p1);
    n_p2->SetParam(C, R, P);
    nlp.AddConstraintSet(n_p2);
    n_p3->SetParam(C, R, P);
    nlp.AddConstraintSet(n_p3);
    n_p4->SetParam(C, R, P);
    nlp.AddConstraintSet(n_p4);
    n_p5->SetParam(C, R, P);
    nlp.AddConstraintSet(n_p5);
    n_p6->SetParam(C, R, P);
    nlp.AddConstraintSet(n_p6);
  
    cost->SetPosRef(p_ref, Wp);
    
    cost->SetCOMRef(com_ref, Wcom);

    nlp.AddCostSet(cost);
    
    ipopt.Solve(nlp);
    x_opt = nlp.GetOptVariables()->GetValues();
       
}

void solve_transition_opt(const std::vector <Eigen::Vector6d> &p_bounds, const std::vector <Eigen::Vector6d> &n_bounds, Eigen::Vector6d &com_bounds, const std::vector <Eigen::Vector6d> &F_bounds,
			  const Eigen::VectorXd &F_thr, const Eigen::VectorXd &p_ref, const Eigen::Vector3d &com_ref, double &m, double &mu, const Eigen::Vector6d &ext_w, Eigen::VectorXd &x_opt)
{
    Problem nlp;
    IpoptSolver ipopt;
    ipopt.SetOption("derivative_test", "first-order");
    
    x_opt.setZero(39);

    auto p1 = std::make_shared<ExVariables> ("p1");
    auto p2 = std::make_shared<ExVariables> ("p2");
    auto p3 = std::make_shared<ExVariables> ("p3");
    auto p4 = std::make_shared<ExVariables> ("p4");

    auto F1 = std::make_shared<ExVariables> ("F1");
    auto F2 = std::make_shared<ExVariables> ("F2");
    auto F3 = std::make_shared<ExVariables> ("F3");
    auto F4 = std::make_shared<ExVariables> ("F4");

    auto n1 = std::make_shared<ExVariables> ("n1");
    auto n2 = std::make_shared<ExVariables> ("n2");
    auto n3 = std::make_shared<ExVariables> ("n3");
    auto n4 = std::make_shared<ExVariables> ("n4");

    auto com = std::make_shared<ExVariables> ("com");

    auto static_constr = std::make_shared<StaticConstraint>();

    auto fr_F1 = std::make_shared<FrictionConstraint> ("F1");
    auto fr_F2 = std::make_shared<FrictionConstraint> ("F2");
    auto fr_F3 = std::make_shared<FrictionConstraint> ("F3");
    auto fr_F4 = std::make_shared<FrictionConstraint> ("F4");

    auto n_p1 = std::make_shared<NormalConstraint> ("p1");
    auto n_p2 = std::make_shared<NormalConstraint> ("p2");
    auto n_p3 = std::make_shared<NormalConstraint> ("p3");
    auto n_p4 = std::make_shared<NormalConstraint> ("p4");

    auto cost = std::make_shared<ExCost>();

    nlp.AddVariableSet(p1);
    nlp.AddVariableSet(p2);
    nlp.AddVariableSet(p3);
    nlp.AddVariableSet(p4);
    
    nlp.AddVariableSet(F1);
    nlp.AddVariableSet(F2);
    nlp.AddVariableSet(F3);
    nlp.AddVariableSet(F4);
    
    nlp.AddVariableSet(n1);
    nlp.AddVariableSet(n2);
    nlp.AddVariableSet(n3);
    nlp.AddVariableSet(n4);
    
    nlp.AddVariableSet(com);
    
    p1->SetBounds(p_bounds[0].head(3), p_bounds[0].tail(3));    
    p2->SetBounds(p_bounds[1].head(3), p_bounds[1].tail(3));
    p3->SetBounds(p_bounds[2].head(3), p_bounds[2].tail(3));
    p4->SetBounds(p_bounds[3].head(3), p_bounds[3].tail(3));

    F1->SetBounds(F_bounds[0].head(3), F_bounds[0].tail(3));
    F2->SetBounds(F_bounds[1].head(3), F_bounds[1].tail(3));
    F3->SetBounds(F_bounds[2].head(3), F_bounds[2].tail(3));
    F4->SetBounds(F_bounds[3].head(3), F_bounds[3].tail(3));
   
    n1->SetBounds(n_bounds[0].head(3), n_bounds[0].tail(3));    
    n2->SetBounds(n_bounds[1].head(3), n_bounds[1].tail(3));
    n3->SetBounds(n_bounds[2].head(3), n_bounds[2].tail(3));
    n4->SetBounds(n_bounds[3].head(3), n_bounds[3].tail(3));

    com->SetBounds(com_bounds.head(3), com_bounds.tail(3));
    
    static_constr->SetMass(m);
    static_constr->SetExternalWrench(ext_w);
    nlp.AddConstraintSet(static_constr);
    
    fr_F1->set_mu(mu); fr_F1->set_F_thr(F_thr[0]);
    fr_F2->set_mu(mu); fr_F2->set_F_thr(F_thr[1]);
    fr_F3->set_mu(mu); fr_F3->set_F_thr(F_thr[2]);
    fr_F4->set_mu(mu); fr_F4->set_F_thr(F_thr[3]);
    nlp.AddConstraintSet(fr_F1);
    nlp.AddConstraintSet(fr_F2);
    nlp.AddConstraintSet(fr_F3);
    nlp.AddConstraintSet(fr_F4);

    nlp.AddCostSet(cost);
    cost->SetPosRef(p_ref, 0);
    cost->SetCOMRef(com_ref, 1);
    cost->SetForceRef(0);

    nlp.AddCostSet(cost);
    
    ipopt.Solve(nlp);
    x_opt = nlp.GetOptVariables()->GetValues();
    
    
}

void compute_RotM(const Eigen::Vector3d &ni, Eigen::Matrix3d &R)
{
  
        R.setZero();

        R.coeffRef(0, 0) =  ni.y() / ((ni.head(2)).norm());
        R.coeffRef(0, 1) = -ni.x() / ((ni.head(2)).norm());

        R.coeffRef(1, 0) = (ni.x() * ni.z()) / ((ni.head(2)).norm());
        R.coeffRef(1, 1) = (ni.y() * ni.z()) / ((ni.head(2)).norm());
        R.coeffRef(1, 2) = - (ni.head(2)).norm();

        R.coeffRef(2, 0) = ni.x();
        R.coeffRef(2, 1) = ni.y();
        R.coeffRef(2, 2) = ni.z();
  
}

void on_force_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
    tf::wrenchMsgToEigen(msg->wrench, g_fmap_ptr->at(l));
}


void sighandler(int sig)
{   
    if (log)
      logger->flush();
}


int main(int argc, char **argv)
{
    
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    /* Init ROS node */
    ros::init(argc, argv, "ifopt_contacts_node");
    ros::NodeHandle nh("ifopt_contacts");
    ros::NodeHandle nh_priv("~");
    
    std::vector<std::string> feet  = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};
    std::vector<std::string> ankle = {"ankle2_1", "ankle2_2", "ankle2_3", "ankle2_4"};
    std::vector<std::string> chain  = {"front_left_leg", "front_right_leg", "rear_left_leg", "rear_right_leg"};
    
    force_publisher::ForcePublisher fpub(feet);

    XBot::Cartesian::RosImpl ci;  

    double rate = nh_priv.param("rate", 100.0);
    double mu = nh_priv.param("mu", 0.5);
    bool wall_balancing_flag = nh_priv.param("wall_balancing_flag", false);
    bool manip_flag = nh_priv.param("manip_flag", false);
    bool ground_balancing_flag = nh_priv.param("ground_balancing_flag", false);
    double contact_thr = nh_priv.param("contact_thr", 20.0);  
    double arm_disp = nh_priv.param("arm_disp", 0.1);
    
    XmlRpc::XmlRpcValue param_list;
    nh_priv.getParam("ext_w_init", param_list);
    Eigen::Vector6d ext_w_init; ext_w_init.setZero(); 
    for (int i = 0; i < param_list.size(); ++i) 
    {
      ext_w_init[i] = param_list[i];
    }
    
    nh_priv.getParam("ext_force_manip", param_list);
    Eigen::Vector3d ext_force_manip; ext_force_manip.setZero(); 
    for (int i = 0; i < param_list.size(); ++i) 
    {
      ext_force_manip[i] = param_list[i];
    }
    
    auto force_links = nh_priv.param("force_links", std::vector<std::string>());
            
    ros::Rate loop_rate(rate);

    ConfigOptions config = XBot::ConfigOptionsFromParamServer();
    
    auto robot = XBot::RobotInterface::getRobot(config);
    robot->sense();
    
    auto model = ModelInterface::getModel(config);  
    model->update();

    bool log;
    nh_priv.param("log", log, false);
    
    std::stringstream ss;
    ss << "/tmp/ifopt_node";

    if (log)
        logger = XBot::MatLogger::getLogger(ss.str());  
    
    /* Set-up*/ 
    set_legs_initial_stiffness(robot);    
    set_up(ci);

    /* Environment super-ellipsoid parameters*/ 
    Eigen::Vector3d C, R, P;
    
    C << pelvis.x() + 0.1, 
         pelvis.y(), 
         pelvis.z();
         
    R << C.x() + 0.6, 
         20.0, 
         pelvis.z() - ground_z;
    
    P << 10, 10, 10;

    if (log) 
    {
        logger->add("C", C);
        logger->add("R", R);
        logger->add("P", P);
    }
    
    /* INITIAL OPT - Manipulation*/
    Eigen::VectorXd p_ref;
    p_ref.setZero(18);

    p_ref.head(3) = wheel_1;
    p_ref.segment<3>(3) = wheel_2;
    p_ref.segment<3>(6) = wheel_3;
    p_ref.segment<3>(9) = wheel_4;
    p_ref.segment<3>(12) = left_arm;
    p_ref.tail(3) = right_arm;

    Eigen::VectorXd x_opt;   x_opt.setZero(57);
    Eigen::VectorXd p_opt;   p_opt.setZero(18);
    Eigen::VectorXd F_opt;   F_opt.setZero(18);
    Eigen::VectorXd n_opt;   n_opt.setZero(18);
    Eigen::VectorXd com_opt; com_opt.setZero(3);

    Eigen::Vector3d F_max;
    F_max.setOnes();
    F_max *= 500;
      
    Eigen::Vector6d p1_bounds, p2_bounds, p3_bounds, p4_bounds, p5_bounds, p6_bounds;
    double delta = 0.3;
    
    p1_bounds.setZero(); 
    p1_bounds.head(3) = wheel_1; 
    p1_bounds.tail(3) = wheel_1 + Eigen::Vector3d(delta, delta, delta);
    
    p2_bounds.setZero(); 
    p2_bounds.head(3) = wheel_2 - Eigen::Vector3d(0.0, delta, 0.0); 
    p2_bounds.tail(3) = wheel_2 + Eigen::Vector3d(delta, 0.0, delta);
    
    p3_bounds.setZero(); 
    p3_bounds.head(3) = wheel_3 - Eigen::Vector3d(delta, 0.0, 0.0); 
    p3_bounds.tail(3) = wheel_3 + Eigen::Vector3d(0.0, delta, delta);
    
    p4_bounds.setZero(); 
    p4_bounds.head(3) = wheel_4 - Eigen::Vector3d(delta, delta, 0.0); 
    p4_bounds.tail(3) = wheel_4 + Eigen::Vector3d(0.0, 0.0, delta);
    
    p5_bounds.setZero(); 
    p5_bounds.head(3) = left_arm - Eigen::Vector3d(delta, delta, delta); 
    p5_bounds.tail(3) = left_arm + Eigen::Vector3d(delta, delta, delta);
    
    p6_bounds.setZero(); 
    p6_bounds.head(3) = right_arm - Eigen::Vector3d(delta, delta, delta); 
    p6_bounds.tail(3) = right_arm + Eigen::Vector3d(delta, delta, delta);
    
    std::vector <Eigen::Vector6d> p_bounds{p1_bounds, p2_bounds, p3_bounds, p4_bounds, p5_bounds, p6_bounds};
   
    double m = model->getMass();
    double Wp = 10;
    double Wcom = 100;
   
    /*SOLVE*/
    solve_initial_opt(p_bounds, F_max, C, R, P, p_ref, Wp, com_ref, Wcom, m, mu, ext_w_init, x_opt);
    
    p_opt = x_opt.head(18);
    F_opt = x_opt.segment<18> (18);
    n_opt = x_opt.segment<18> (36);
    com_opt =  x_opt.tail(3);

    if (log) 
    {
        logger->add("com_ref", com_ref);
        logger->add("p_ref", p_ref);
        logger->add("F_ref", ext_w_init.head(3));
        logger->add("x_sol_initial", x_opt);
        logger->add("com_initial", com_opt);
        logger->add("p_initial", p_opt);
        logger->add("F_initial", F_opt);
        logger->add("n_initial", n_opt);
    }

    /* TRANSITIONS OPT - Balancing ONLY*/
    
    double F_lift_max = 800;
    
    Eigen::Vector6d F1_bounds, F2_bounds, F3_bounds, F4_bounds;
    F1_bounds.setOnes(); F1_bounds.head(3) *= -F_lift_max; F1_bounds.tail(3) *= F_lift_max;
    F2_bounds.setOnes(); F2_bounds.head(3) *= -F_lift_max; F2_bounds.tail(3) *= F_lift_max;
    F3_bounds.setOnes(); F3_bounds.head(3) *= -F_lift_max; F3_bounds.tail(3) *= F_lift_max;
    F4_bounds.setOnes(); F4_bounds.head(3) *= -F_lift_max; F4_bounds.tail(3) *= F_lift_max;
    std::vector <Eigen::Vector6d> F_bounds{F1_bounds, F2_bounds, F3_bounds, F4_bounds};

    Eigen::Vector6d n1_bounds, n2_bounds, n3_bounds, n4_bounds;
    n1_bounds.setZero(); n1_bounds[2] = 1.0; n1_bounds[5] = 1.0;
    n2_bounds.setZero(); n2_bounds[2] = 1.0; n2_bounds[5] = 1.0;
    n3_bounds.setZero(); n3_bounds[2] = 1.0; n3_bounds[5] = 1.0;
    n4_bounds.setZero(); n4_bounds[2] = 1.0; n4_bounds[5] = 1.0;
    std::vector <Eigen::Vector6d> n_bounds{n1_bounds, n2_bounds, n3_bounds, n4_bounds};

    Eigen::VectorXd F_thr;
    double thr = 200;
    F_thr.setOnes(4); F_thr *= thr; 

    std::vector< Eigen::VectorXd > x_opt_legs(4, Eigen::VectorXd::Zero(39));
    std::vector< Eigen::VectorXd > p_opt_legs(4, Eigen::VectorXd::Zero(12));
    std::vector< Eigen::VectorXd > F_opt_legs(4, Eigen::VectorXd::Zero(12));
    std::vector< Eigen::VectorXd > n_opt_legs(4, Eigen::VectorXd::Zero(12));
    std::vector< Eigen::VectorXd > com_opt_legs(4, Eigen::VectorXd::Zero(3));

    p_bounds[0].head(3) = p_ref.head(3);        p_bounds[0].tail(3) = p_bounds[0].head(3);
    p_bounds[1].head(3) = p_ref.segment<3>(3);  p_bounds[1].tail(3) = p_bounds[1].head(3);
    p_bounds[2].head(3) = p_ref.segment<3>(6);  p_bounds[2].tail(3) = p_bounds[2].head(3);
    p_bounds[3].head(3) = p_ref.tail(3);        p_bounds[3].tail(3) = p_bounds[3].head(3);
    
    Eigen::Vector6d com_bounds;
    Eigen::Vector3d delta_com;
    delta_com << 0.2, 0.2, 0.0;
    com_bounds.head(3) = com_ref - delta_com;
    com_bounds.tail(3) = com_ref + delta_com;
    
    
    /* FORCE ESTIMATION */
    std::map<std::string, ros::Subscriber> sub_force_map;
    std::map<std::string, Eigen::Vector6d> f_est_map;
    

    for(auto l : force_links)
    {
        auto sub_force = ros::NodeHandle("cartesian").subscribe<geometry_msgs::WrenchStamped>("force_estimation/" + l, 1, boost::bind(on_force_recv, _1, l));	
	sub_force_map[l] = sub_force;
	f_est_map[l] = Eigen::Vector6d::Zero();
	
	ROS_INFO("Subscribed to topic '%s'", sub_force.getTopic().c_str());
	
    }
    
    g_fmap_ptr = &f_est_map;      
    
    if ( wall_balancing_flag )
    {
   
    for (int i : {0,1,2,3})  
    {
 
	F_bounds[i] = Eigen::Vector6d::Zero();
	F_thr[i] = 0.0;
	p_bounds[i].head(3) = -1e3*Eigen::Vector3d::Ones(); p_bounds[i].tail(3) =  1e3*Eigen::Vector3d::Ones();

        /*SOLVE*/
	solve_transition_opt(p_bounds, n_bounds, com_bounds, F_bounds, F_thr, p_ref, com_ref, m, mu, Eigen::Vector6d::Zero(), x_opt_legs[i]);
        
        p_opt_legs[i] = x_opt_legs[i].head(12);
        F_opt_legs[i] = x_opt_legs[i].segment<12> (12);
        n_opt_legs[i] = x_opt_legs[i].segment<12> (24);
        com_opt_legs[i] =  x_opt_legs[i].tail(3);
         
	F_bounds[i].setOnes();
	F_bounds[i].head(3) *= -F_lift_max; F_bounds[i].tail(3) *= F_lift_max;
        F_thr[i] = thr;

        /* Put wheels on the ground */ 
        Eigen::Vector3d pi = p_opt.segment<3> (3 * i); 
	
        if( (i==0) || (i==1) )
	  pi.z() = ground_z;
	
	p_bounds[i].head(3) = pi; p_bounds[i].tail(3) = pi;

        Eigen::Vector3d ni = - n_opt.segment<3> (3 * i);
	n_bounds[i].head(3) = -ni; n_bounds[i].tail(3) = -ni; 
	
	Eigen::Matrix3d R; R.setZero();
	compute_RotM(ni, R);
	
        fpub.send_force(F_opt_legs[i]); 
	fpub.send_normal(n_opt_legs[i]);
      
	Eigen::Affine3d w_T_com;
	w_T_com.setIdentity();
        w_T_com.translation() = com_opt_legs[i];
        ci.setTargetPose("com", w_T_com, 5.0);
        ci.waitReachCompleted("com");

        Eigen::Affine3d w_T_f1;
        w_T_f1.translation() = pi;
	
	if( (i==0) || (i==1) )
	{
	    w_T_f1.translation().z() += 0.2;
	
	    Eigen::Affine3d w_T_f2;
	    w_T_f2.translation() = pi;

	    Trajectory::WayPointVector wp;
	    wp.emplace_back(w_T_f1, 2.0);    // absolute time w.r.t. start of traj
	    wp.emplace_back(w_T_f2, 4.0);
	    ci.setWayPoints(feet[i], wp);
	    Eigen::Affine3d a_T_f;
	    a_T_f.translation() = ni;
	    a_T_f.linear() =  R.transpose();
	    ci.setTargetPose(ankle[i], a_T_f, 2.0);
	    ci.waitReachCompleted(feet[i]);
	    ci.waitReachCompleted(ankle[i]);
	    
	}
 	else 
	{	   
	    ci.setTargetPose(feet[i], w_T_f1, 4.0);
	    Eigen::Affine3d a_T_f;
	    a_T_f.translation() = ni;
	    a_T_f.linear() =  R.transpose();
	    ci.setTargetPose(ankle[i], a_T_f, 4.0);
	    if(i==3)
	      fpub.send_normal(n_opt);
            ci.waitReachCompleted(ankle[i]);
	    
	    ros::Rate rate(100.0);
	    
	    bool contact_detection = false;	    
	    	    
	    ci.setControlMode(feet[i], Cartesian::ControlType::Velocity);
	    while(!contact_detection)
	    {
	      
	      ros::spinOnce();
	      
	      Eigen::Vector3d f_est = f_est_map[feet[i]].head(3);
	      
	      std::cout<< "f_est norm " + feet[i] + ": " << f_est.norm() << std::endl;
	      
	      if( f_est.norm() >= contact_thr )
		contact_detection = true;
	      
	      Eigen::Vector6d vel; 
	      vel.setZero();
	      vel.head(3) = ni; vel *= 1e-2;
	      
	      ci.setVelocityReference(feet[i],vel);
	    }
	    ci.setControlMode(feet[i], Cartesian::ControlType::Position);    

	}
	

        if (log) 
        {
            logger->add("x_sol_lift", x_opt_legs[i]);
            logger->add("com_lift", com_opt_legs[i]);
            logger->add("p_lift", p_opt_legs[i]);
            logger->add("F_lift", F_opt_legs[i]);
            logger->add("n_lift", n_opt_legs[i]);
            logger->add("R_lift", R);
        }
        

    }
    
    }
  
    fpub.send_force(Eigen::VectorXd::Zero(12)); 
    
    ci.getPoseFromTf("ci/com", "ci/world_odom", pose);
    pose.translation().y() = 0.0;
    ci.setTargetPose("com", pose, 4.0);
    ci.waitReachCompleted("com");
         
    bool right_arm_contact = false;
    bool left_arm_contact = false;   
    
    if( manip_flag )
    {

    pose.translation() << 0.68, 0.12, 0.1;
    pose.linear() = Eigen::Quaterniond(0.19, 0.67, -0.04, 0.71).toRotationMatrix(); //w x y z
    ci.setTargetPose("arm1_8", pose, 4.0);
    ci.waitReachCompleted("arm1_8");
	    
    pose.translation() << 0.68, -0.12, 0.1;
    pose.linear() = Eigen::Quaterniond(0.71, -0.04, 0.67, 0.19).toRotationMatrix();
    ci.setTargetPose("arm2_8", pose, 4.0);
    ci.waitReachCompleted("arm2_8");        
   
    ci.setControlMode("arm1_8", Cartesian::ControlType::Velocity);
    ci.setControlMode("arm2_8", Cartesian::ControlType::Velocity);

    while( !(right_arm_contact && left_arm_contact) )
    {
      
      ros::spinOnce();
      
      Eigen::Vector3d left_arm_force  = f_est_map["arm1_8"].head(3);
      Eigen::Vector3d right_arm_force = f_est_map["arm2_8"].head(3);
      
      std::cout<<"left_arm f_est norm: "  << left_arm_force.norm()  << std::endl;
      std::cout<<"right_arm f_est norm: " << right_arm_force.norm() << std::endl;
      
      if( left_arm_force.norm() >= contact_thr )
	left_arm_contact = true;
      
      if( right_arm_force.norm() >= contact_thr )
	right_arm_contact = true;
      
      Eigen::Vector6d vel; 
      vel.setZero();
      vel.head(3) << 1.5, 0.0, 0.0; vel *= 1e-2;
      
      if(!left_arm_contact)
	ci.setVelocityReference("arm1_8",vel);
      
      if(!right_arm_contact)
	ci.setVelocityReference("arm2_8",vel);
    }
    
    ci.setControlMode("arm1_8", Cartesian::ControlType::Position);
    ci.setControlMode("arm2_8", Cartesian::ControlType::Position);      

    ci.getPoseFromTf("arm1_8", "ci/world_odom", pose);
    Eigen::Vector3d left_arm = pose.translation();
    ci.getPoseFromTf("arm2_8", "ci/world_odom", pose);
    Eigen::Vector3d right_arm = pose.translation();
    
    Eigen::Vector3d f_left_arm =  ext_force_manip/2.0;
    Eigen::Vector3d f_right_arm = ext_force_manip/2.0;
    Eigen::Vector6d f_arms;

    f_arms.head(3) = f_left_arm; 
    f_arms.tail(3) = f_right_arm; 
    
    Eigen::Vector6d ext_w_manip; 
    ext_w_manip.head(3) = ext_force_manip;
    ext_w_manip.tail(3) = (left_arm - com_ref).cross(f_left_arm) + (right_arm - com_ref).cross(f_right_arm);
	    
    fpub.send_wrench_manip(ext_w_manip);

    fpub.send_force_arms(-f_arms);

    set_arms_low_stiffness(robot);
      
    bool arm_stop_push = false;

    robot->sense(false);
    model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);	
    model->getPose("arm1_8", "pelvis", pose);
    Eigen::Vector3d start_left_pos = pose.translation();
    model->getPose("arm2_8", "pelvis", pose);
    Eigen::Vector3d start_right_pos = pose.translation();

    while (!arm_stop_push) 
    {
	ros::spinOnce();
	
	robot->sense(false);
	model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);	
	model->getPose("arm1_8", "pelvis", pose);
	Eigen::Vector3d left_arm = pose.translation();
	model->getPose("arm2_8", "pelvis", pose);
	Eigen::Vector3d right_arm = pose.translation();
	
	std::cout<<"left_arm disp norm: "  << (start_left_pos - left_arm).norm()  << std::endl;
        std::cout<<"right_arm disp norn: " << (start_right_pos - right_arm).norm() << std::endl;
		
	if( ((start_left_pos - left_arm).norm() >= arm_disp) || ((start_right_pos - right_arm).norm() >= arm_disp) )
	    arm_stop_push = true;
    }
      
    fpub.send_force_arms(Eigen::Vector6d::Zero());

    fpub.send_wrench_manip(Eigen::Vector6d::Zero()); 
          
    }
    
    
    if ( wall_balancing_flag && ground_balancing_flag )
    {
      
    const int N_ITER = 400;
    for(int k = 0; k < N_ITER; k++)
        ros::Duration(0.01).sleep();

    for (int i : {3,2})  
    {
 
        Eigen::Vector3d pi = p_ref.segment<3> (3 * i);
	pi.z() = ground_z;
      
        Eigen::Vector3d ni = - n_opt.segment<3> (0);
	
	Eigen::Matrix3d R; R.setZero();
	compute_RotM(ni, R);
	
        fpub.send_force(F_opt_legs[i]); 
	fpub.send_normal(n_opt_legs[i]);
      
	Eigen::Affine3d w_T_com;
	w_T_com.setIdentity();
        w_T_com.translation() = com_opt_legs[i];
        ci.setTargetPose("com", w_T_com, 5.0);
        ci.waitReachCompleted("com");

        Eigen::Affine3d w_T_f;
	w_T_f.setIdentity();
        w_T_f.translation() = pi;  
	ci.setTargetPose(feet[i], w_T_f, 4.0);
	Eigen::Affine3d a_T_f;
	a_T_f.translation() = ni;
	a_T_f.linear() =  R.transpose();
	ci.setTargetPose(ankle[i], a_T_f, 4.0);	
	ci.waitReachCompleted(feet[i]);
        ci.waitReachCompleted(ankle[i]);


    }
    
    }
  
    fpub.send_force(Eigen::VectorXd::Zero(12)); 
    
    ci.getPoseFromTf("ci/com", "ci/world_odom", pose);
    pose.translation().y() = 0.0;
    ci.setTargetPose("com", pose, 4.0);
    ci.waitReachCompleted("com");
    

    while (ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    if (log)
      logger->flush();
    
    return 0;
}

