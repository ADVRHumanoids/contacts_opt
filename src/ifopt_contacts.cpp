#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt_problem/ifopt_contacts.h>

#include <XBotInterface/MatLogger.hpp>

using namespace ifopt;

int main()
{
    
   auto logger = XBot::MatLogger::getLogger("/tmp/ifopt_contacts_log");  
    
  // 1. define the problem
  Problem nlp;
  
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
  
  
   // BOUNDS 
  p1->SetBounds(Eigen::Vector3d( 0.1, -1.0, 0.0),Eigen::Vector3d( 2.0, -0.1, 0.4));
  p2->SetBounds(Eigen::Vector3d( 0.1,  0.1, 0.0),Eigen::Vector3d( 2.0,  1.0, 0.4));
  p3->SetBounds(Eigen::Vector3d(-2.0, -1.0, 0.0),Eigen::Vector3d(-0.1, -0.1, 0.4));
  p4->SetBounds(Eigen::Vector3d(-2.0,  0.1, 0.0),Eigen::Vector3d(-0.1,  1.0, 0.4));
  
  com->SetBounds(Eigen::Vector3d(-2.0, -1.0, 0.4),Eigen::Vector3d( 2.0, 1.0, 0.6));
 
  Eigen::Vector3d F_max; 
  F_max.setOnes();
  F_max *= 100;
  
  F1->SetBounds(-F_max,F_max);
  F2->SetBounds(-F_max,F_max);
  F3->SetBounds(-F_max,F_max);
  F4->SetBounds(-F_max,F_max);
  
  
  // CENTROIDAL DYNAMICS
  auto static_constr = std::make_shared<StaticConstraint>();
  Eigen::Vector6d ext_w;
  ext_w << 90, 0, 0, 0, 0, 0.0;
  static_constr->SetExternalWrench(ext_w);
  
  nlp.AddConstraintSet(static_constr);
  
  
  // SUPERELLIPSOID ENVIRONMENT
  auto SE_p1 = std::make_shared<SuperEllipsoidConstraint>("p1");
  auto SE_p2 = std::make_shared<SuperEllipsoidConstraint>("p2");
  auto SE_p3 = std::make_shared<SuperEllipsoidConstraint>("p3");
  auto SE_p4 = std::make_shared<SuperEllipsoidConstraint>("p4");
  
  Eigen::Vector3d C; C << 1.5, 0.0, 1.0;
  Eigen::Vector3d R; R << 2, 20, 1;
  Eigen::Vector3d P; P << 8, 8, 4; 
  
  SE_p1->SetParam(C,R,P);
  SE_p2->SetParam(C,R,P);
  SE_p3->SetParam(C,R,P);
  SE_p4->SetParam(C,R,P);
  
  nlp.AddConstraintSet(SE_p1);
  nlp.AddConstraintSet(SE_p2);
  nlp.AddConstraintSet(SE_p3);
  nlp.AddConstraintSet(SE_p4);
   
  // FRICTION CONES
  auto fr_F1 = std::make_shared<FrictionConstraint>("F1");
  auto fr_F2 = std::make_shared<FrictionConstraint>("F2");
  auto fr_F3 = std::make_shared<FrictionConstraint>("F3");
  auto fr_F4 = std::make_shared<FrictionConstraint>("F4");
  
  double mu = .2;
  
  fr_F1->set_mu(mu);
  fr_F2->set_mu(mu);
  fr_F3->set_mu(mu);
  fr_F4->set_mu(mu);
  
  nlp.AddConstraintSet(fr_F1);
  nlp.AddConstraintSet(fr_F2);
  nlp.AddConstraintSet(fr_F3);
  nlp.AddConstraintSet(fr_F4); 
  
  // NORMAL CONSTRAINT
  auto n_p1 = std::make_shared<NormalConstraint>("p1");
  auto n_p2 = std::make_shared<NormalConstraint>("p2");
  auto n_p3 = std::make_shared<NormalConstraint>("p3");
  auto n_p4 = std::make_shared<NormalConstraint>("p4");
  
  n_p1->SetParam(C,R,P);
  n_p2->SetParam(C,R,P);
  n_p3->SetParam(C,R,P);
  n_p4->SetParam(C,R,P);
  
  nlp.AddConstraintSet(n_p1);
  nlp.AddConstraintSet(n_p2);
  nlp.AddConstraintSet(n_p3);
  nlp.AddConstraintSet(n_p4); 
   
  // COST
  auto cost = std::make_shared<ExCost>();
  Eigen::VectorXd p_ref;  
  p_ref.setZero(12);
  
  p_ref <<  0.5, -0.3, 0.0, 
            0.5,  0.3, 0.0, 
           -0.5, -0.3, 0.0, 
           -0.5,  0.3, 0.0;
  
  double W_p = 10;
  
  cost->SetPosRef(p_ref,W_p);
  
  nlp.AddCostSet(cost);

  nlp.PrintCurrent();
  

  // 2. choose solver and options
  IpoptSolver ipopt;
  ipopt.SetOption("derivative_test", "first-order");


  // 3 . solve
  ipopt.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  
//   for(int i = 0; i < 4; i++)
//   std::cout<<"n"<< i+1 << ": \n" << x.segment(i*3 + 24, 3).norm() <<std::endl;
  
  std::cout<<"com: \n"<< x.tail(3).transpose() <<std::endl;
  
  for(int i = 0; i < 4; i++)
  std::cout<<"p"<< i+1 << ": \n" << x.segment(i*3, 3).transpose() <<std::endl;

  for(int i = 0; i < 4; i++)
  std::cout<<"F"<< i+1 << ": \n" << x.segment(i*3 + 12, 3).transpose() <<std::endl;
  

  
   for(int k = 0; k < x.size(); k++)
   {
        logger->add("x_sol", x[k]);
   }
  
    logger->flush();
  
}