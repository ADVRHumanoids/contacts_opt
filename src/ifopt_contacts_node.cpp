#include <ros/ros.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/problem/Com.h>

#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/ros/RosImpl.h>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt_problem/ifopt_contacts.h>


using namespace XBot;
using namespace XBot::Cartesian;
using namespace ifopt;

ModelInterface::Ptr model;
Eigen::VectorXd q;

int main(int argc, char **argv)
{
  
    /* Init ROS node */
    ros::init(argc, argv, "ifopt_contacts_node");
    ros::NodeHandle nh;
    
    XBot::Cartesian::RosImpl ci;

    double rate;
    nh.param("rate", rate, 100.);

    ros::Rate loop_rate(rate);

    ConfigOptions config = XBot::ConfigOptionsFromParamServer();

    model = ModelInterface::getModel(config);
    q.setZero(model->getJointNum());

    bool log;
    nh.param("log", log, false);

    XBot::MatLogger::Ptr logger;
    uint T = ros::Time::now().sec;
    std::stringstream ss;
    
    ss<<"/tmp/ifopt_contacts_node_"<<T;
    
    if(log)
        logger = XBot::MatLogger::getLogger(ss.str());
    
    Eigen::Vector3d com_ref;
    com_ref.setZero();  
    model->getCOM(com_ref);   
    
    Eigen::Affine3d pose;
    
    model->getPose("wheel_1",pose);  
    Eigen::Vector3d wheel_1 = pose.translation();
    wheel_1.y() =  0.35;
    
    model->getPose("wheel_2",pose);  
    Eigen::Vector3d wheel_2 = pose.translation();
    wheel_2.y() = -0.35;
    
    model->getPose("wheel_3",pose);  
    Eigen::Vector3d wheel_3 = pose.translation();
    wheel_3.y() =  0.35;
    
    model->getPose("wheel_4",pose);  
    Eigen::Vector3d wheel_4 = pose.translation();
    wheel_4.y() = -0.35;
    
    model->getPose("pelvis",pose);  
    Eigen::Vector3d pelvis = pose.translation();
    
    Eigen::Vector3d C, R, P; 
    C << 20.0, pelvis.y(), pelvis.z();
    R <<  C.x() + 2*wheel_1.x() , 20.0,  pelvis.z()-wheel_1.z();
    P << 20.0, 20.0, 20.0; 
    
    std::cout<< "C: " << C.transpose() << std::endl;
    std::cout<< "R: " << R.transpose() << std::endl;
    
    Eigen::VectorXd p_ref; p_ref.setZero(12);
    
    p_ref.head(3) = wheel_1;
    p_ref.segment<3>(3) = wheel_2;
    p_ref.segment<3>(6) = wheel_3;
    p_ref.tail(3) = wheel_4;
  
    
    Problem nlp;
    IpoptSolver ipopt; ipopt.SetOption("derivative_test", "first-order");  
    Eigen::VectorXd x_opt; x_opt.setZero(39);
    Eigen::VectorXd p_opt; x_opt.setZero(12);
    Eigen::VectorXd F_opt; x_opt.setZero(12);
    Eigen::VectorXd n_opt; x_opt.setZero(12);
    Eigen::VectorXd com_opt; x_opt.setZero(3);
    
    Eigen::VectorXd ext_w; ext_w.setZero(6); ext_w << 90, 0, 0, 0, 0, 0.0;
    
    Eigen::Vector3d F_max; F_max.setOnes(); F_max *= 100;
    
    double mu = 0.2;
    double Wp = 10;
    double Wcom = 100;
       
    auto p1 = std::make_shared<ExVariables>("p1");
    auto p2 = std::make_shared<ExVariables>("p2");
    auto p3 = std::make_shared<ExVariables>("p3");
    auto p4 = std::make_shared<ExVariables>("p4");
	  
    auto F1 = std::make_shared<ExVariables>("F1");
    auto F2 = std::make_shared<ExVariables>("F2");
    auto F3 = std::make_shared<ExVariables>("F3");
    auto F4 = std::make_shared<ExVariables>("F4"); 
	  
    auto n1 = std::make_shared<ExVariables>("n1");
    auto n2 = std::make_shared<ExVariables>("n2");
    auto n3 = std::make_shared<ExVariables>("n3");
    auto n4 = std::make_shared<ExVariables>("n4");
	  
    auto com = std::make_shared<ExVariables>("com");
	    
    auto static_constr = std::make_shared<StaticConstraint>();
	    
    auto SE_p1 = std::make_shared<SuperEllipsoidConstraint>("p1");
    auto SE_p2 = std::make_shared<SuperEllipsoidConstraint>("p2");
    auto SE_p3 = std::make_shared<SuperEllipsoidConstraint>("p3");
    auto SE_p4 = std::make_shared<SuperEllipsoidConstraint>("p4");
	    
    auto fr_F1 = std::make_shared<FrictionConstraint>("F1");
    auto fr_F2 = std::make_shared<FrictionConstraint>("F2");
    auto fr_F3 = std::make_shared<FrictionConstraint>("F3");
    auto fr_F4 = std::make_shared<FrictionConstraint>("F4");
	    
    auto n_p1 = std::make_shared<NormalConstraint>("p1");
    auto n_p2 = std::make_shared<NormalConstraint>("p2");
    auto n_p3 = std::make_shared<NormalConstraint>("p3");
    auto n_p4 = std::make_shared<NormalConstraint>("p4");
	  
    auto cost = std::make_shared<ExCost>();	    
    
    std::vector<std::string> feet  = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};
    std::vector<std::string> ankle = {"ankle2_1", "ankle2_2", "ankle2_3", "ankle2_4"};
	    
    nlp.AddVariableSet(p1); p1->SetBounds(wheel_1 - Eigen::Vector3d( 0.0,  0.0, 0.0), wheel_1 + Eigen::Vector3d( 0.3, 0.3, 0.5)); 
    nlp.AddVariableSet(p2); p2->SetBounds(wheel_2 - Eigen::Vector3d( 0.0,  0.3, 0.0), wheel_2 + Eigen::Vector3d( 0.3, 0.0, 0.5));  
    nlp.AddVariableSet(p3); p3->SetBounds(wheel_3 - Eigen::Vector3d( 0.3,  0.0, 0.0), wheel_3 + Eigen::Vector3d( 0.0, 0.3, 0.5)); 
    nlp.AddVariableSet(p4); p4->SetBounds(wheel_4 - Eigen::Vector3d( 0.3,  0.3, 0.0), wheel_4 + Eigen::Vector3d( 0.0, 0.0, 0.5));  
      
    nlp.AddVariableSet(F1); F1->SetBounds(-F_max,F_max);
    nlp.AddVariableSet(F2); F2->SetBounds(-F_max,F_max);
    nlp.AddVariableSet(F3); F3->SetBounds(-F_max,F_max);
    nlp.AddVariableSet(F4); F4->SetBounds(-F_max,F_max);
      
    nlp.AddVariableSet(n1);
    nlp.AddVariableSet(n2);
    nlp.AddVariableSet(n3);
    nlp.AddVariableSet(n4); 
      
    nlp.AddVariableSet(com); 
	    
    static_constr->SetExternalWrench(ext_w); nlp.AddConstraintSet(static_constr);
	    
    SE_p1->SetParam(C,R,P); nlp.AddConstraintSet(SE_p1);
    SE_p2->SetParam(C,R,P); nlp.AddConstraintSet(SE_p2);
    SE_p3->SetParam(C,R,P); nlp.AddConstraintSet(SE_p3);
    SE_p4->SetParam(C,R,P); nlp.AddConstraintSet(SE_p4);
	    
    fr_F1->set_mu(mu); nlp.AddConstraintSet(fr_F1);
    fr_F2->set_mu(mu); nlp.AddConstraintSet(fr_F2);
    fr_F3->set_mu(mu); nlp.AddConstraintSet(fr_F3);
    fr_F4->set_mu(mu); nlp.AddConstraintSet(fr_F4); 
	    
    n_p1->SetParam(C,R,P); nlp.AddConstraintSet(n_p1);
    n_p2->SetParam(C,R,P); nlp.AddConstraintSet(n_p2);
    n_p3->SetParam(C,R,P); nlp.AddConstraintSet(n_p3);
    n_p4->SetParam(C,R,P); nlp.AddConstraintSet(n_p4); 
	    
    cost->SetPosRef(p_ref, Wp);
    
    cost->SetCOMRef(com_ref, Wcom);
        
    nlp.AddCostSet(cost);  
    
    ipopt.Solve(nlp);	
    x_opt = nlp.GetOptVariables()->GetValues(); 
    
    p_opt = x_opt.head(12);
    F_opt = x_opt.segment<12>(12);
    n_opt = x_opt.segment<12>(24);
    com_opt =  x_opt.tail(3);
	
    if(log)
    {
	logger->add("x_sol", x_opt);
	logger->add("com_ref", com_ref);
	logger->add("p_ref", p_ref);
	logger->add("com", com_opt);
	logger->add("p", p_opt);
	logger->add("F", F_opt);
	logger->add("n", n_opt);
    }
    
    
/* Simultaneous foot lift  */
    
    Eigen::Affine3d w_T_com;
    w_T_com.translation() = com_opt;
//     ci.setTargetPose("com", w_T_com, 5.0);   
       
    for(int i : {0, 1, 2, 3})
    {
	
        Eigen::Vector3d pi = p_opt.segment<3>(3*i);
	
	Eigen::Vector3d ni = - n_opt.segment<3>(3*i);	
		
	Eigen::Matrix3d R; R.setZero();
                 
        R.coeffRef(0, 0) =  ni.y()/((ni.head(2)).norm()); 
        R.coeffRef(0, 1) = -ni.x()/((ni.head(2)).norm());  
        
        R.coeffRef(1, 0) =  (ni.x()*ni.z())/((ni.head(2)).norm());  
        R.coeffRef(1, 1) =  (ni.y()*ni.z())/((ni.head(2)).norm());  
        R.coeffRef(1, 2) = -(ni.head(2)).norm();  
        
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
    
    std::vector< Eigen::VectorXd > x_opt_legs(4, Eigen::VectorXd::Zero(39));
    std::vector< Eigen::VectorXd > p_opt_legs(4, Eigen::VectorXd::Zero(12));
    std::vector< Eigen::VectorXd > F_opt_legs(4, Eigen::VectorXd::Zero(12));
    std::vector< Eigen::VectorXd > n_opt_legs(4, Eigen::VectorXd::Zero(12));
    std::vector< Eigen::VectorXd > com_opt_legs(4, Eigen::VectorXd::Zero(3));
    
    std::vector < std::shared_ptr<ExVariables> > F{F1, F2, F3, F4};
    std::vector < std::shared_ptr<ExVariables> > p{p1, p2, p3, p4};

    ext_w.setZero();	
    static_constr->SetExternalWrench(ext_w);
    
/* Sequential foot lift  */
    
    for(int i : {0, 1 , 2, 3})
    {
	
	cost->SetPosRef(p_ref, 100);
            
	cost->SetCOMRef(com_ref, 10);
	    
	F_max.setOnes(); F_max *= 1e-2;	    
	F[i]->SetBounds(-F_max,F_max);
	
	p[i]->SetBounds(p_opt.segment<3>(3*i),p_opt.segment<3>(3*i));
	
	ipopt.Solve(nlp);	
        x_opt_legs[i] = nlp.GetOptVariables()->GetValues(); 
    
        p_opt_legs[i] = x_opt_legs[i].head(12);
        F_opt_legs[i] = x_opt_legs[i].segment<12>(12);
        n_opt_legs[i] = x_opt_legs[i].segment<12>(24);
        com_opt_legs[i] =  x_opt_legs[i].tail(3);

	F_max.setOnes(); F_max *= 100;
	F[i]->SetBounds(-F_max,F_max);	
	
	Eigen::Vector3d pi = p_opt_legs[i].segment<3>(3*i);
	
	Eigen::Vector3d ni = - n_opt_legs[i].segment<3>(3*i);	
		
	Eigen::Matrix3d R; R.setZero();
                 
        R.coeffRef(0, 0) =  ni.y()/((ni.head(2)).norm()); 
        R.coeffRef(0, 1) = -ni.x()/((ni.head(2)).norm());  
        
        R.coeffRef(1, 0) =  (ni.x()*ni.z())/((ni.head(2)).norm());  
        R.coeffRef(1, 1) =  (ni.y()*ni.z())/((ni.head(2)).norm());  
        R.coeffRef(1, 2) = -(ni.head(2)).norm();  
        
        R.coeffRef(2, 0) = ni.x();  
        R.coeffRef(2, 1) = ni.y();  
        R.coeffRef(2, 2) = ni.z(); 

	Eigen::Affine3d w_T_com;
	w_T_com.translation() = com_opt_legs[i];
	ci.setTargetPose("com", w_T_com, 5.0); 	
	ci.waitReachCompleted("com");
	
	Eigen::Affine3d w_T_f;
	w_T_f.translation() = pi;	
	ci.setTargetPose(feet[i], w_T_f, 5.0);
	ci.waitReachCompleted(feet[i]);
	
	Eigen::Affine3d a_T_f;
	a_T_f.translation() = ni;
	a_T_f.linear() =  R.transpose();	
	ci.setTargetPose(ankle[i], a_T_f, 5.0);
	ci.waitReachCompleted(ankle[i]);
		
	if(log)
	{
	  logger->add("x_sol", x_opt_legs[i]);
	  logger->add("com", com_opt_legs[i]);
	  logger->add("p", p_opt_legs[i]);
	  logger->add("F", F_opt_legs[i]);
	  logger->add("n", n_opt_legs[i]);
	}
	
    }
   
    while(ros::ok())
    {	
	ros::spinOnce();
        loop_rate.sleep();
    }
    
    if(log)
	 logger->flush();
    
    return 0;
}

