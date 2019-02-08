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
        
    Problem nlp;
    IpoptSolver ipopt; ipopt.SetOption("derivative_test", "first-order");  
    Eigen::VectorXd x_opt; x_opt.setZero(39);
    
    Eigen::VectorXd ext_w; ext_w.setZero(6); ext_w << 90, 0, 0, 0, 0, 0.0;
    Eigen::Vector3d F_max; F_max.setOnes(); F_max *= 100;
    
    Eigen::Vector3d C, R, P; 
    C <<  1.5,  0.0,  1.0;
    R <<  2.0, 20.0,  1.0;
    P << 20.0, 20.0, 20.0; 
    
    double mu = 0.2;
    
    Eigen::VectorXd p_ref;
    p_ref.setZero(12);
    p_ref <<  0.5, -0.3, 0.0, 
	      0.5,  0.3, 0.0, 
	     -0.5, -0.3, 0.0, 
	     -0.5,  0.3, 0.0;
	     
    double Wp = 10;
	     
    Eigen::Vector3d com_ref;
    com_ref.setZero();
    
    double Wcom = 10;
       
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
    
    std::vector<std::string> feet = {"wheel_2", "wheel_1", "wheel_4", "wheel_3"};
	    
    nlp.AddVariableSet(p1); p1->SetBounds(Eigen::Vector3d( 0.1, -1.0, 0.0),Eigen::Vector3d( 2.0, -0.1, 0.4));
    nlp.AddVariableSet(p2); p2->SetBounds(Eigen::Vector3d( 0.1,  0.1, 0.0),Eigen::Vector3d( 2.0,  1.0, 0.4));
    nlp.AddVariableSet(p3); p3->SetBounds(Eigen::Vector3d(-2.0, -1.0, 0.0),Eigen::Vector3d(-0.1, -0.1, 0.4));
    nlp.AddVariableSet(p4); p4->SetBounds(Eigen::Vector3d(-2.0,  0.1, 0.0),Eigen::Vector3d(-0.1,  1.0, 0.4));
      
    nlp.AddVariableSet(F1); F1->SetBounds(-F_max,F_max);
    nlp.AddVariableSet(F2); F2->SetBounds(-F_max,F_max);
    nlp.AddVariableSet(F3); F3->SetBounds(-F_max,F_max);
    nlp.AddVariableSet(F4); F4->SetBounds(-F_max,F_max);
      
    nlp.AddVariableSet(n1);
    nlp.AddVariableSet(n2);
    nlp.AddVariableSet(n3);
    nlp.AddVariableSet(n4); 
      
    nlp.AddVariableSet(com); com->SetBounds(Eigen::Vector3d(-2.0, -1.0, 0.4),Eigen::Vector3d( 2.0, 1.0, 0.6));
	    
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
//     cost->SetCOMRef(com_ref, Wcom);
    nlp.AddCostSet(cost);
    
    ipopt.Solve(nlp);	
    x_opt = nlp.GetOptVariables()->GetValues(); 
	
    if(log)
    {
	logger->add("q", q);
	logger->add("x_sol", x_opt);
	logger->add("com", x_opt.tail(3));
	logger->add("p", x_opt.head(12));
	logger->add("F", x_opt.segment(12,12));	    
    }
    
    Eigen::Affine3d w_T_com;
    w_T_com.translation() = x_opt.tail(3);
    ci.setTargetPose("com", w_T_com, 5.0);
    
    for(int i : {0, 1, 2, 3})
    {
	Eigen::Affine3d w_T_f;
	w_T_f.translation() = x_opt.head(12).segment<3>(3*i);
	ci.setTargetPose(feet[i], w_T_f, 5.0);
    }

    if(log)
        logger->flush();
    return 0;
}

