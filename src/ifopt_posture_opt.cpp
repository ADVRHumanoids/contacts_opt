#include <ifopt_problem/ifopt_posture_opt.h>
#include <ifopt/ipopt_solver.h>
#include <iostream>

int main()
{
    ifopt::Problem nlp;
    ifopt::IpoptSolver ipopt;
    ipopt.SetOption("derivative_test", "first-order");
    
    auto q = std::make_shared<ifopt::Variable3D>("q");
    auto tau = std::make_shared<ifopt::Variable3D>("tau");
    
    q->SetBounds(-3.0 * Eigen::Vector3d::Ones(), 
                 3.0 * Eigen::Vector3d::Ones());
    
    Eigen::Vector3d tau_max(150.0, 150.0, 100.0);
    
    tau->SetBounds(-tau_max, 
                 tau_max);
    
    Eigen::Vector3d d(0.40, 0.30, 0.20);
    Eigen::Vector2d F(-600.0, 300.0);
    auto static_constr = std::make_shared<ifopt::StaticConstraint>(d);
    static_constr->SetExternalWrench(F);
    
    auto cost = std::make_shared<ifopt::ExCost>(d);
    
    nlp.AddVariableSet(q);
    nlp.AddVariableSet(tau);
    
    nlp.AddConstraintSet(static_constr);
    
    nlp.AddCostSet(cost);

    ipopt.Solve(nlp);
    auto x_opt = nlp.GetOptVariables()->GetValues();   
    auto q_value = x_opt.head<3>();
    auto tau_value = x_opt.tail<3>();
    
    std::cout << "q_value: " << q_value.transpose() << std::endl;
    std::cout << "tau_value: " << tau_value.transpose() << std::endl;
    
    Eigen::Vector2d p;
    p[0] = d[0]*cos(q_value[0])+ d[1]*cos(q_value[0]+q_value[1])+ d[2]*cos(q_value[0]+q_value[1]+q_value[2]);
    p[1] = d[0]*sin(q_value[0])+ d[1]*sin(q_value[0]+q_value[1])+ d[2]*sin(q_value[0]+q_value[1]+q_value[2]);
    std::cout << "p: " << p.transpose() << std::endl;
    
}
