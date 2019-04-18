#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include <eigen3/Eigen/Dense>

#include <ifopt_problem/posture_opt_functions.h>

#include <iostream>

namespace Eigen
{
    typedef Matrix<double, 6, 1> Vector6d;
}

namespace ifopt
{


class Variable3D : public VariableSet
{

public:

    Variable3D(std::string name): 
        VariableSet(3, name)
    {
        // the initial values where the NLP starts iterating from
        _q.setZero();

        _lb.setConstant(-300.0);
        _ub.setConstant(300.0);

    }

    // Here is where you can transform the Eigen::Vector into whatever
    // internal representation of your variables you have (here two doubles, but
    // can also be complex classes such as splines, etc..
    void SetVariables ( const Eigen::VectorXd& x ) override
    {
        _q = x;
    };

    // Here is the reverse transformation from the internal representation to
    // to the Eigen::Vector
    Eigen::VectorXd GetValues() const override
    {
        return _q;
    };

    void SetBounds(const Eigen::Vector3d& lower, 
                   const Eigen::Vector3d& upper)
    {

        _lb = lower;
        _ub = upper;

        if(((_ub - _lb).array() < 0).any()) 
        {
            throw std::invalid_argument ( "Inconsistent bounds" );
        }

    }

    // Each variable has an upper and lower bound set here
    VecBound GetBounds() const override
    {
        VecBound bounds(GetRows());
        bounds.at(0) = Bounds(_lb(0), _ub(0));
        bounds.at(1) = Bounds(_lb(1), _ub(1));
        bounds.at(2) = Bounds(_lb(2), _ub(2));
        return bounds;
    }

private:

    Eigen::Vector3d _q;
    Eigen::Vector3d _lb, _ub;

};



class ExCost : public CostTerm
{
    
public:

    ExCost(const Eigen::Vector3d& d) : CostTerm ( "cost_term" ),
        _d(d)
    { 
    }
    

    double GetCost() const override
    {

        Eigen::Vector3d q = GetVariables()->GetComponent("q")->GetValues();
        
        Eigen::MatrixXd J = ::get_jacobian(q, _d);  
        
        double manip = -std::sqrt((J*J.transpose()).determinant());
   
        std::cout<< "manipulability measure: " << manip << std::endl;
        
        return manip;
        
    };

    void FillJacobianBlock(std::string var_set, Jacobian& jac) const override
    {
        jac.setZero();
        
        if(var_set == "q")
        {
            Eigen::Vector3d q = GetVariables()->GetComponent("q")->GetValues();
            Eigen::Vector3d manip_J = ::get_manip_jacobian(q, _d); 
            
            std::cout<< "manipulability jacobian: " << manip_J.transpose() << std::endl;
            
            jac.coeffRef(0, 0) = -manip_J[0];
            jac.coeffRef(0, 1) = -manip_J[1];
            jac.coeffRef(0, 2) = -manip_J[2];
            
        }
    }

private:
    
    Eigen::Vector3d _d;

};



class StaticConstraint : public ConstraintSet
{

public:

    StaticConstraint(const Eigen::Vector3d& d): 
        ConstraintSet (3, "StaticConstraint" ),
        _d(d)
    {
        _F.setZero();
    }

    // The constraint value minus the constant value "1", moved to bounds.
    Eigen::VectorXd GetValues() const override
    {
        Eigen::Vector3d q   = GetVariables()->GetComponent("q")->GetValues();
        Eigen::Vector3d tau = GetVariables()->GetComponent("tau")->GetValues();
        
        Eigen::Vector3d value = ::get_tau(q, _d, _F) - tau;
        
        return value;
        
        
    };

    void SetExternalWrench ( const Eigen::Vector2d& F )
    {
        _F = F;
    }

    VecBound GetBounds() const override
    {
        VecBound b (GetRows());
        
        for(int i = 0; i < 3; i++)
        {
            b.at(i) = Bounds (.0, .0);
        }

        return b;
    }

    void FillJacobianBlock(std::string var_set, 
                           Jacobian& jac_block) const override
    {
        jac_block.setZero();
        
        if(var_set == "tau")
        {
            jac_block = -Eigen::Matrix3d::Identity().sparseView();
        }
        
        if(var_set == "q")
        {
            Eigen::Vector3d q   = GetVariables()->GetComponent("q")->GetValues();
            
            jac_block = ::get_dtau_dq(q, _d, _F).sparseView();
        }
        
    }

private:

    Eigen::Vector2d _F;
    Eigen::Vector3d _d;
};




} // namespace opt


