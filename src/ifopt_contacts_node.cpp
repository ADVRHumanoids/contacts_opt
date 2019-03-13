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

using namespace XBot;
using namespace XBot::Cartesian;
using namespace ifopt;

void set_low_stiffness(XBot::RobotInterface::Ptr robot)
{
    robot->setControlMode(XBot::ControlMode::Stiffness() + XBot::ControlMode::Damping());
    for(int i = 0; i < 4; i++)
    {
        Eigen::VectorXd leg_k(robot->leg(i).getJointNum());
        leg_k << 300, 300, 300, 150, 50, 0;
        robot->leg(i).setStiffness(leg_k);
    }
    robot->move();
}


class ForcePublisher
{
  
public:
    
    ForcePublisher(std::vector<std::string> feet);
    
    void send_force(const Eigen::VectorXd& f_opt);
    void send_normal(const Eigen::VectorXd& n_opt);
    void send_wrench_manip(const Eigen::VectorXd& tau_opt);
    void send_mu(const double& mu);
    
private:
    
    std::vector<ros::Publisher> _pubs_force;
    std::vector<ros::Publisher> _pubs_normal;
    ros::Publisher _pub_wrench_manip;
    ros::Publisher _pub_mu;
    
};

ForcePublisher::ForcePublisher(std::vector<std::string> feet)
{
    ros::NodeHandle nh;
    
    
    for(auto l : feet)
    {
//         _pubs.push_back( nh.advertise<geometry_msgs::WrenchStamped>("cartesian/force_ffwd/" + l, 1) );
        _pubs_force.push_back( nh.advertise<geometry_msgs::WrenchStamped>("forza_giusta/force_ref/" + l, 1) );
        _pubs_normal.push_back( nh.advertise<geometry_msgs::WrenchStamped>("forza_giusta/normal/" + l, 1) );
    }
    
    _pub_wrench_manip = nh.advertise<geometry_msgs::WrenchStamped>("forza_giusta/wrench_manip/", 1);
    _pub_mu = nh.advertise<geometry_msgs::WrenchStamped>("forza_giusta/mu/", 1);
    
}

void ForcePublisher::send_force(const Eigen::VectorXd &f_opt)
{
     
    for (int i : {0, 1, 2, 3}) 
    {
        Eigen::Vector3d f =  f_opt.segment<3>(3*i);
            
        geometry_msgs::WrenchStamped msg;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();        
        msg.wrench.force.x = f.x();
        msg.wrench.force.y = f.y();
        msg.wrench.force.z = f.z();
       
       _pubs_force[i].publish(msg);
        
    }
    
}

void ForcePublisher::send_normal(const Eigen::VectorXd &n_opt)
{
     
    for (int i : {0, 1, 2, 3}) 
    {
        Eigen::Vector3d n =  n_opt.segment<3>(3*i);
            
        geometry_msgs::WrenchStamped msg;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();        
        msg.wrench.force.x = n.x();
        msg.wrench.force.y = n.y();
        msg.wrench.force.z = n.z();
       
       _pubs_normal[i].publish(msg);
        
    }
    
}

void ForcePublisher::send_wrench_manip(const Eigen::VectorXd& tau_opt)
{
              
        geometry_msgs::WrenchStamped msg;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();        
        msg.wrench.force.x = tau_opt.x();
        msg.wrench.force.y = tau_opt.y();
        msg.wrench.force.z = tau_opt.z();
        msg.wrench.torque.x = tau_opt[3];
        msg.wrench.torque.y = tau_opt[4];
        msg.wrench.torque.z = tau_opt[5];
       
       _pub_wrench_manip.publish(msg);
        
    
}

void ForcePublisher::send_mu(const double& mu)
{
              
        geometry_msgs::WrenchStamped msg;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();        
        msg.wrench.force.x = mu;
      
       _pub_mu.publish(msg);
        
    
}


int main(int argc, char **argv)
{

    /* Init ROS node */
    ros::init(argc, argv, "ifopt_contacts_node");
    ros::NodeHandle nh;
    
    std::vector<std::string> feet  = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};
    std::vector<std::string> ankle = {"ankle2_1", "ankle2_2", "ankle2_3", "ankle2_4"};
    
    ForcePublisher fpub(feet);

    XBot::Cartesian::RosImpl ci;   

    double rate;
    nh.param("rate", rate, 100.);
    double mu = nh.param("mu", 0.5);
    
    fpub.send_mu(mu);
    
    ros::Rate loop_rate(rate);

    ConfigOptions config = XBot::ConfigOptionsFromParamServer();
    
    auto robot = XBot::RobotInterface::getRobot(config);
    robot->sense();
    
    auto model = ModelInterface::getModel(config);   

    bool log;
    nh.param("log", log, false);

    XBot::MatLogger::Ptr logger;
    uint T = ros::Time::now().sec;
    std::stringstream ss;

    ss << "/tmp/ifopt_contacts_node_" << T;

    if (log)
        logger = XBot::MatLogger::getLogger(ss.str());


    Eigen::Affine3d pose;
    
    ci.getPoseFromTf("ci/com", "ci/world_odom", pose);
    pose.translation().z() -= 0.1;
    Eigen::Vector3d com_ref = pose.translation();
    ci.setTargetPose("com", pose, 2.0);
    ci.waitReachCompleted("com");

    ci.getPoseFromTf("ci/pelvis", "ci/world_odom", pose);
    Eigen::Vector3d pelvis = pose.translation();
    
    double wheel_offset_x = 0.35;
    double wheel_offset_y = 0.35;

    ci.getPoseFromTf("ci/wheel_1", "ci/world_odom", pose);
    Eigen::Vector3d wheel_1 = pose.translation();
    wheel_1.x() =  pelvis.x() + wheel_offset_x;
    wheel_1.y() =  pelvis.y() + wheel_offset_y;

    ci.getPoseFromTf("ci/wheel_2", "ci/world_odom", pose);
    Eigen::Vector3d wheel_2 = pose.translation();
    wheel_2.x() =  pelvis.x() + wheel_offset_x;
    wheel_2.y() =  pelvis.y() - wheel_offset_y;

    ci.getPoseFromTf("ci/wheel_3", "ci/world_odom", pose);
    Eigen::Vector3d wheel_3 = pose.translation();
    wheel_3.x() =  pelvis.x() - wheel_offset_x;
    wheel_3.y() =  pelvis.y() + wheel_offset_y;

    ci.getPoseFromTf("ci/wheel_4", "ci/world_odom", pose);
    Eigen::Vector3d wheel_4 = pose.translation();
    wheel_4.x() =  pelvis.x() - wheel_offset_x;
    wheel_4.y() =  pelvis.y() - wheel_offset_y;
    
    Eigen::Affine3d pose_arm1_8, pose_arm2_8;
    ci.getPoseFromTf("ci/arm1_8", "ci/world_odom", pose_arm1_8);       
    ci.getPoseFromTf("ci/arm2_8", "ci/world_odom", pose_arm2_8);

    double ground_z = wheel_1.z();  

/* Environment super-ellipsoid parameters*/ 
    Eigen::Vector3d C, R, P;
    
    C << pelvis.x() + 20.0, 
         pelvis.y(), 
         pelvis.z();
         
    R << C.x() + 0.6, 
         20.0, 
         pelvis.z() - ground_z;
    
    P << 20, 20, 20; 

    if (log) {
        logger->add("C", C);
        logger->add("R", R);
        logger->add("P", P);
    }

/* Initial desired wheel position on the ground*/
    Eigen::VectorXd p_ref;
    p_ref.setZero(12);

    p_ref.head(3) = wheel_1;
    p_ref.segment<3> (3) = wheel_2;
    p_ref.segment<3> (6) = wheel_3;
    p_ref.tail(3) = wheel_4;


    Problem nlp;
    IpoptSolver ipopt;
    ipopt.SetOption("derivative_test", "first-order");
    
    Eigen::VectorXd x_opt;
    x_opt.setZero(39);
    Eigen::VectorXd p_opt;
    x_opt.setZero(12);
    Eigen::VectorXd F_opt;
    x_opt.setZero(12);
    Eigen::VectorXd n_opt;
    x_opt.setZero(12);
    Eigen::VectorXd com_opt;
    x_opt.setZero(3);

    Eigen::VectorXd ext_w;
    ext_w.setZero(6);
    ext_w << 1000, 0, 0, 0, 0, 0.0;

    Eigen::Vector3d F_max;
    F_max.setOnes();
    F_max *= 500;

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

    auto SE_p1 = std::make_shared<SuperEllipsoidConstraint> ("p1");
    auto SE_p2 = std::make_shared<SuperEllipsoidConstraint> ("p2");
    auto SE_p3 = std::make_shared<SuperEllipsoidConstraint> ("p3");
    auto SE_p4 = std::make_shared<SuperEllipsoidConstraint> ("p4");

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

    double delta = 0.3;
    
    p1->SetBounds(wheel_1 - Eigen::Vector3d(0.0, 0.0, 0.0),     wheel_1 + Eigen::Vector3d(delta, delta, delta));
    p2->SetBounds(wheel_2 - Eigen::Vector3d(0.0, delta, 0.0),   wheel_2 + Eigen::Vector3d(delta, 0.0, delta));
    p3->SetBounds(wheel_3 - Eigen::Vector3d(delta, 0.0, 0.0),   wheel_3 + Eigen::Vector3d(0.0, delta, delta));
    p4->SetBounds(wheel_4 - Eigen::Vector3d(delta, delta, 0.0), wheel_4 + Eigen::Vector3d(0.0, 0.0, delta));

    nlp.AddVariableSet(F1);
    F1->SetBounds(-F_max, F_max);
    nlp.AddVariableSet(F2);
    F2->SetBounds(-F_max, F_max);
    nlp.AddVariableSet(F3);
    F3->SetBounds(-F_max, F_max);
    nlp.AddVariableSet(F4);
    F4->SetBounds(-F_max, F_max);

    nlp.AddVariableSet(n1);
    nlp.AddVariableSet(n2);
    nlp.AddVariableSet(n3);
    nlp.AddVariableSet(n4);

    nlp.AddVariableSet(com);

    double m = model->getMass();
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

    fr_F1->set_mu(mu);
    nlp.AddConstraintSet(fr_F1);
    fr_F2->set_mu(mu);
    nlp.AddConstraintSet(fr_F2);
    fr_F3->set_mu(mu);
    nlp.AddConstraintSet(fr_F3);
    fr_F4->set_mu(mu);
    nlp.AddConstraintSet(fr_F4);

    n_p1->SetParam(C, R, P);
    nlp.AddConstraintSet(n_p1);
    n_p2->SetParam(C, R, P);
    nlp.AddConstraintSet(n_p2);
    n_p3->SetParam(C, R, P);
    nlp.AddConstraintSet(n_p3);
    n_p4->SetParam(C, R, P);
    nlp.AddConstraintSet(n_p4);

    
    double Wp = 10;
    cost->SetPosRef(p_ref, Wp);
    
    double Wcom = 100;
    cost->SetCOMRef(com_ref, Wcom);

    nlp.AddCostSet(cost);

    ipopt.Solve(nlp);
    x_opt = nlp.GetOptVariables()->GetValues();

    p_opt = x_opt.head(12);
    F_opt = x_opt.segment<12> (12);
    n_opt = x_opt.segment<12> (24);
    com_opt =  x_opt.tail(3);

    if (log) {
        logger->add("com_ref", com_ref);
        logger->add("p_ref", p_ref);
        logger->add("F_ref", ext_w.head(3));
        logger->add("x_sol_initial", x_opt);
        logger->add("com_initial", com_opt);
        logger->add("p_initial", p_opt);
        logger->add("F_initial", F_opt);
        logger->add("n_initial", n_opt);
    }

    
/* Simultaneous foot lift  */

    Eigen::Affine3d w_T_com;
    w_T_com.translation() = com_opt;
//     ci.setTargetPose("com", w_T_com, 5.0);

    for (int i : {0, 1, 2, 3}) 
    {

        Eigen::Vector3d pi = p_opt.segment<3> (3 * i);

        Eigen::Vector3d ni = - n_opt.segment<3> (3 * i);

        Eigen::Matrix3d R;
        R.setZero();

        R.coeffRef(0, 0) =  ni.y() / ((ni.head(2)).norm());
        R.coeffRef(0, 1) = -ni.x() / ((ni.head(2)).norm());

        R.coeffRef(1, 0) = (ni.x() * ni.z()) / ((ni.head(2)).norm());
        R.coeffRef(1, 1) = (ni.y() * ni.z()) / ((ni.head(2)).norm());
        R.coeffRef(1, 2) = - (ni.head(2)).norm();

        R.coeffRef(2, 0) = ni.x();
        R.coeffRef(2, 1) = ni.y();
        R.coeffRef(2, 2) = ni.z();


        Eigen::Affine3d w_T_f;
        w_T_f.translation() = pi;

// 	ci.setTargetPose(feet[i], w_T_f, 5.0);

        Eigen::Affine3d a_T_f;
        a_T_f.translation() = ni;
        a_T_f.linear() =  R.transpose();

// 	ci.setTargetPose(ankle[i], a_T_f, 5.0);

    }

/* Sequential foot lift - Balancing */

    Problem nlp_legs;

    nlp_legs.AddVariableSet(p1);
    nlp_legs.AddVariableSet(p2);
    nlp_legs.AddVariableSet(p3);
    nlp_legs.AddVariableSet(p4);

    F_max.setOnes();
    F_max *= 800;

    nlp_legs.AddVariableSet(F1);
    F1->SetBounds(-F_max, F_max);
    nlp_legs.AddVariableSet(F2);
    F2->SetBounds(-F_max, F_max);
    nlp_legs.AddVariableSet(F3);
    F3->SetBounds(-F_max, F_max);
    nlp_legs.AddVariableSet(F4);
    F4->SetBounds(-F_max, F_max);

    nlp_legs.AddVariableSet(n1);
    n1->SetBounds(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1));
    nlp_legs.AddVariableSet(n2);
    n2->SetBounds(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1));
    nlp_legs.AddVariableSet(n3);
    n3->SetBounds(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1));
    nlp_legs.AddVariableSet(n4);
    n4->SetBounds(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1));

    nlp_legs.AddVariableSet(com);

    double F_thr = 30;

    fr_F1->set_F_thr(F_thr);
    fr_F2->set_F_thr(F_thr);
    fr_F3->set_F_thr(F_thr);
    fr_F4->set_F_thr(F_thr);

    nlp_legs.AddConstraintSet(fr_F1);
    nlp_legs.AddConstraintSet(fr_F2);
    nlp_legs.AddConstraintSet(fr_F3);
    nlp_legs.AddConstraintSet(fr_F4);

    std::vector< Eigen::VectorXd > x_opt_legs(5, Eigen::VectorXd::Zero(39));
    std::vector< Eigen::VectorXd > p_opt_legs(5, Eigen::VectorXd::Zero(12));
    std::vector< Eigen::VectorXd > F_opt_legs(5, Eigen::VectorXd::Zero(12));
    std::vector< Eigen::VectorXd > n_opt_legs(5, Eigen::VectorXd::Zero(12));
    std::vector< Eigen::VectorXd > com_opt_legs(5, Eigen::VectorXd::Zero(3));

    std::vector < std::shared_ptr<ExVariables> > F {F1, F2, F3, F4};
    std::vector < std::shared_ptr<FrictionConstraint> > fr_F {fr_F1, fr_F2, fr_F3, fr_F4};
    std::vector < std::shared_ptr<ExVariables> > p {p1, p2, p3, p4};
    std::vector < std::shared_ptr<ExVariables> > n {n1, n2, n3, n4};

    static_constr->SetExternalWrench( Eigen::VectorXd::Zero(6));
    nlp_legs.AddConstraintSet(static_constr);


    nlp_legs.AddCostSet(cost);

    p1->SetBounds(p_ref.head(3), p_ref.head(3));
    p2->SetBounds(p_ref.segment<3> (3), p_ref.segment<3> (3));
    p3->SetBounds(p_ref.segment<3> (6), p_ref.segment<3> (6));
    p4->SetBounds(p_ref.tail(3), p_ref.tail(3));

    com->SetBounds(com_ref - 0.2 * Eigen::Vector3d::Ones(), com_ref + 0.2 * Eigen::Vector3d::Ones());

    for (int i : {0, 1, 2, 3}) 
    {

        cost->SetPosRef(p_ref, 0);
        cost->SetCOMRef(com_ref, 1);
        cost->SetForceRef(0);

        // No force on the lifting leg 
        F_max *= 0;
        F[i]->SetBounds(-F_max, F_max);

        fr_F[i]->set_F_thr(0);
        
        // No position bound on the lifting leg
        p[i]->SetBounds(-Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());

        ipopt.Solve(nlp_legs);
        x_opt_legs[i] = nlp_legs.GetOptVariables()->GetValues();

        p_opt_legs[i] = x_opt_legs[i].head(12);
        F_opt_legs[i] = x_opt_legs[i].segment<12> (12);
        n_opt_legs[i] = x_opt_legs[i].segment<12> (24);
        com_opt_legs[i] =  x_opt_legs[i].tail(3);

        F_max.setOnes();
        F_max *= 800;
        F[i]->SetBounds(-F_max, F_max);

        fr_F[i]->set_F_thr(F_thr);

        // Put wheel on the ground, if super-ellipsoid is close to the ground 
        Eigen::Vector3d pi = p_opt.segment<3> (3 * i);
        
        if ((pi.z() - ground_z) <= 0.015)
        {
            pi.z() = ground_z;
        }

        p[i]->SetBounds(pi, pi);

        Eigen::Vector3d ni = - n_opt.segment<3> (3 * i);

        n[i]->SetBounds(n_opt.segment<3> (3 * i), n_opt.segment<3> (3 * i));

        Eigen::Matrix3d R;
        R.setZero();

        R.coeffRef(0, 0) =  ni.y() / ((ni.head(2)).norm());
        R.coeffRef(0, 1) = -ni.x() / ((ni.head(2)).norm());

        R.coeffRef(1, 0) = (ni.x() * ni.z()) / ((ni.head(2)).norm());
        R.coeffRef(1, 1) = (ni.y() * ni.z()) / ((ni.head(2)).norm());
        R.coeffRef(1, 2) = - (ni.head(2)).norm();

        R.coeffRef(2, 0) = ni.x();
        R.coeffRef(2, 1) = ni.y();
        R.coeffRef(2, 2) = ni.z();

        Eigen::Affine3d w_T_com;
        w_T_com.translation() = com_opt_legs[i];
        ci.setTargetPose("com", w_T_com, 5.0);
        ci.waitReachCompleted("com");

        Eigen::Affine3d w_T_f1;
        w_T_f1.translation() = pi;
        w_T_f1.translation().x() += 0.08;
        w_T_f1.translation().z() += 0.1;

        Eigen::Affine3d w_T_f2;
        w_T_f2.translation() = pi;
        
        fpub.send_force( F_opt_legs[i] ); 
        fpub.send_normal( n_opt_legs[i] ); 
        
        set_low_stiffness(robot);

        Trajectory::WayPointVector wp;
        wp.emplace_back(w_T_f1, 2.0);    // absolute time w.r.t. start of traj
        wp.emplace_back(w_T_f2, 4.0);
        ci.setWayPoints(feet[i], wp);
        Eigen::Affine3d a_T_f;
        a_T_f.translation() = ni;
        a_T_f.linear() =  R.transpose();
        ci.setTargetPose(ankle[i], a_T_f, 5.0);
        ci.waitReachCompleted(feet[i]);
        ci.waitReachCompleted(ankle[i]);

        if (log) {
            logger->add("x_sol_lift", x_opt_legs[i]);
            logger->add("com_lift", com_opt_legs[i]);
            logger->add("p_lift", p_opt_legs[i]);
            logger->add("F_lift", F_opt_legs[i]);
            logger->add("n_lift", n_opt_legs[i]);
        }

    }
    
    
    pose_arm1_8.translation().x() += 0.15;
    ci.setBaseLink("arm1_8", "world");
    ci.setTargetPose("arm1_8", pose_arm1_8, 2.0);
    
    pose_arm2_8.translation().x() += 0.15;
    ci.setBaseLink("arm2_8", "world");
    ci.setTargetPose("arm2_8", pose_arm2_8, 2.0);
    ci.waitReachCompleted("arm2_8");

/* CoM-Force opt for pushing */    
    static_constr->SetExternalWrench(ext_w);
    
    ipopt.Solve(nlp_legs);
    x_opt_legs[4] = nlp_legs.GetOptVariables()->GetValues();
    
    p_opt_legs[4] = x_opt_legs[4].head(12);
    F_opt_legs[4] = x_opt_legs[4].segment<12> (12);
    n_opt_legs[4] = x_opt_legs[4].segment<12> (24);
    com_opt_legs[4] =  x_opt_legs[4].tail(3);
    
    w_T_com.translation() = com_opt_legs[4];
    ci.setTargetPose("com", w_T_com, 5.0);
    ci.waitReachCompleted("com");
    
    if (log) {
        logger->add("x_sol_final", x_opt_legs[4]);
        logger->add("com_final", com_opt_legs[4]);
        logger->add("p_final", p_opt_legs[4]);
        logger->add("F_final", F_opt_legs[4]);
        logger->add("n_final", n_opt_legs[4]);
    }

/* Pushing */      
    ci.getPoseFromTf("ci/arm1_8", "ci/world_odom", pose);
    ci.setBaseLink("arm1_8", "pelvis");
   
    ci.getPoseFromTf("ci/arm2_8", "ci/world_odom", pose);
    ci.setBaseLink("arm2_8", "pelvis");
        
   
    fpub.send_force( F_opt_legs[4] );
    fpub.send_normal( n_opt_legs[4] );
    fpub.send_wrench_manip(ext_w);
    

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    if (log)
        logger->flush();

    return 0;
}

