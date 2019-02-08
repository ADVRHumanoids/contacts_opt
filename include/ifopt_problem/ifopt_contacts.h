#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 

namespace Eigen {
    typedef Matrix<double, 6, 1> Vector6d;
}

namespace ifopt {
    
using Eigen::Vector2d;


class ExVariables : public VariableSet {
public:
  // Every variable set has a name, here "var_set1". this allows the constraints
  // and costs to define values and Jacobians specifically w.r.t this variable set.
    
  ExVariables(const std::string& name) : VariableSet(3, name)
  {
    // the initial values where the NLP starts iterating from
    x0_ = 0.0;
    x1_ = 0.0;
    x2_ = 0.0;
    
    lb_.setConstant(-1000.0);
    ub_.setConstant(1000.0);
    
  }

  // Here is where you can transform the Eigen::Vector into whatever
  // internal representation of your variables you have (here two doubles, but
  // can also be complex classes such as splines, etc..
  void SetVariables(const VectorXd& x) override
  {
    x0_ = x(0);
    x1_ = x(1);
    x2_ = x(2);
  };

  // Here is the reverse transformation from the internal representation to
  // to the Eigen::Vector
  VectorXd GetValues() const override
  {
      Eigen::VectorXd temp; 
      temp.setZero(3);
      temp << x0_, x1_, x2_;
      
      return temp;

  };
  
  void SetBounds(const Eigen::Vector3d& lower, const Eigen::Vector3d& upper)
  {
   
      lb_ = lower;
      ub_ = upper;
      
      if( ((ub_ - lb_).array() < 0).any() )
      {
          throw std::invalid_argument("Inconsistent bounds");
      }
      
  }

  // Each variable has an upper and lower bound set here
  VecBound GetBounds() const override
  {
    VecBound bounds(GetRows());
    bounds.at(0) = Bounds(lb_(0), ub_(0));
    bounds.at(1) = Bounds(lb_(1), ub_(1));
    bounds.at(2) = Bounds(lb_(2), ub_(2));
    return bounds;
  }

private:
  double x0_, x1_, x2_;
  Eigen::Vector3d lb_, ub_;
  
};



class ExCost: public CostTerm {
public:
    
    ExCost() : ExCost("cost_term") 
    {
        _W_p = 0;
        _p_ref.setZero(12);
    }
    ExCost(const std::string& name) : CostTerm(name) {}

    
    void SetPosRef(const Eigen::VectorXd& p_ref, const double& W_p)
    {
        _W_p = W_p; 
        _p_ref = p_ref;
    }
    
    
    double GetCost() const override
    {
        
            double value = 0;
                
            for(int i : {1, 2, 3, 4})
            {
                Eigen::Vector3d Fi = GetVariables()->GetComponent("F" + std::to_string(i))->GetValues();
                Eigen::Vector3d pi = GetVariables()->GetComponent("p" + std::to_string(i))->GetValues();
                
                value += 0.5*_W_p*(pi -_p_ref.segment(3*(i-1),3)).squaredNorm() + 0.5*Fi.squaredNorm();
            } 
            
            return value;
    };

    void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
    {
        jac.setZero();
        for(int i = 0; i < 4; i++)
        {
            if(var_set == ("F" + std::to_string(i+1)))
            {
                Eigen::Vector3d Fi = GetVariables()->GetComponent("F" + std::to_string(i+1))->GetValues();
                
                jac.coeffRef(0, 0) = Fi.x();
                jac.coeffRef(0, 1) = Fi.y();
                jac.coeffRef(0, 2) = Fi.z();
                
            }
            
            if(var_set == ("p" + std::to_string(i+1)))
            {
                Eigen::Vector3d pi = GetVariables()->GetComponent("p" + std::to_string(i+1))->GetValues();
                
                jac.coeffRef(0, 0) = _W_p*(pi.x()-_p_ref(3*i));
                jac.coeffRef(0, 1) = _W_p*(pi.y()-_p_ref(3*i+1));
                jac.coeffRef(0, 2) = _W_p*(pi.z()-_p_ref(3*i+2));
                
            }
            
        }
    }

private:

   double _W_p;
   Eigen::VectorXd _p_ref;

    
};



class StaticConstraint : public ConstraintSet {
    
public:

    StaticConstraint() : ConstraintSet(6, "StaticConstraint")
    {
        _wrench_ext.setZero();
        _mg << 0.0, 0.0, -100;
    }

    // The constraint value minus the constant value "1", moved to bounds.
    VectorXd GetValues() const override
    {
        Eigen::Vector6d value;
        value.setZero();
	
	Eigen::Vector3d com = GetVariables()->GetComponent("com")->GetValues();
        
        for(int i : {1, 2, 3, 4})
        {
            Eigen::Vector3d Fi = GetVariables()->GetComponent("F" + std::to_string(i))->GetValues();
            Eigen::Vector3d pi = GetVariables()->GetComponent("p" + std::to_string(i))->GetValues();
            value.head<3>() += Fi;
            value.tail<3>() += (pi-com).cross(Fi);
        }
        
        
        value -= _wrench_ext;
        value.head<3>() += _mg;
        
        return value;
        
    };
    
    void SetExternalWrench(const Eigen::Vector6d& w)
    {
        _wrench_ext = w;
    }
    
    // The only constraint in this set is an equality constraint to 1.
    // Constant values should always be put into GetBounds(), not GetValues().
    // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
    VecBound GetBounds() const override
    {
        VecBound b(GetRows());
        for(int i = 0; i < 6; i++)
        {            
            b.at(i) = Bounds(.0, .0);  
        }
           
        return b;
    }

    // This function provides the first derivative of the constraints.
    // In case this is too difficult to write, you can also tell the solvers to
    // approximate the derivatives by finite differences and not overwrite this
    // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
    {
        jac_block.setZero();
	
	Eigen::Vector3d com = GetVariables()->GetComponent("com")->GetValues();
	
        for(int i = 0; i < 4; i++)
        {
            if(var_set == ("F" + std::to_string(i+1)))
            {
                
                jac_block.coeffRef(0, 0) = 1.0;
                jac_block.coeffRef(1, 1) = 1.0;
                jac_block.coeffRef(2, 2) = 1.0;
                
                Eigen::Vector3d pi = GetVariables()->GetComponent("p" + std::to_string(i+1))->GetValues();
                jac_block.coeffRef(3, 1) = -(pi.z()-com.z());
                jac_block.coeffRef(3, 2) =   pi.y()-com.y();
                jac_block.coeffRef(4, 0) =   pi.z()-com.z();
                jac_block.coeffRef(4, 2) = -(pi.x()-com.x());
                jac_block.coeffRef(5, 0) = -(pi.y()-com.y());
                jac_block.coeffRef(5, 1) =   pi.x()-com.x();
            }
            
            if(var_set == ("p" + std::to_string(i+1)))
            {
                Eigen::Vector3d Fi = GetVariables()->GetComponent("F" + std::to_string(i+1))->GetValues();
                jac_block.coeffRef(3, 1) =  Fi.z();
                jac_block.coeffRef(3, 2) = -Fi.y();
                jac_block.coeffRef(4, 0) = -Fi.z();
                jac_block.coeffRef(4, 2) =  Fi.x();
                jac_block.coeffRef(5, 0) =  Fi.y();
                jac_block.coeffRef(5, 1) = -Fi.x();
            }           
        }
        
        if(var_set == "com")
        { 
	   for(int i : {1, 2, 3, 4})
	    {
		Eigen::Vector3d Fi = GetVariables()->GetComponent("F" + std::to_string(i))->GetValues();
		
		jac_block.coeffRef(3, 1) -=  Fi.z();
                jac_block.coeffRef(3, 2) -= -Fi.y();
                jac_block.coeffRef(4, 0) -= -Fi.z();
                jac_block.coeffRef(4, 2) -=  Fi.x();
                jac_block.coeffRef(5, 0) -=  Fi.y();
                jac_block.coeffRef(5, 1) -= -Fi.x();
		
	    }
        }
        
    }
    
private:
    
    Eigen::Vector6d _wrench_ext;
    Eigen::Vector3d _mg;
};



class SuperEllipsoidConstraint : public ConstraintSet {
    
public:

    SuperEllipsoidConstraint(const std::string& position_name) :
        ConstraintSet(1, "SuperEllipsoidConstraint " + position_name),
        pname_(position_name)
    {
        _C.setZero();
        _R.setOnes();
        _P << 8, 8, 4;
    }

    // The constraint value minus the constant value "1", moved to bounds.
    VectorXd GetValues() const override
    {
       
        Eigen::VectorXd value;
        value.setZero(1);
        
        Eigen::Vector3d p = GetVariables()->GetComponent(pname_)->GetValues(); 
             
        for(int i = 0; i < 3; i++)
        {            
           value(0) += pow((p(i)-_C(i))/_R(i),_P(i));
        }
        
        value(0) -= 1;
                
        return value;
        
    };
    
    void SetParam(const Eigen::Vector3d& C, const Eigen::Vector3d& R, const Eigen::Vector3d& P)
    {
        _C = C; // superellipsoid center
        _R = R; // radial axis
        _P = P; // r s t parameters (r=s super-ellipsoid)
    }

    // The only constraint in this set is an equality constraint to 1.
    // Constant values should always be put into GetBounds(), not GetValues().
    // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
    VecBound GetBounds() const override
    {
        VecBound b(GetRows());
         
        b.at(0) = Bounds(0.0, 0.0);  
          
        return b;
    }

    // This function provides the first derivative of the constraints.
    // In case this is too difficult to write, you can also tell the solvers to
    // approximate the derivatives by finite differences and not overwrite this
    // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
    {
        jac_block.setZero();
        
        if(var_set == pname_)
        {
            Eigen::Vector3d p = GetVariables()->GetComponent(pname_)->GetValues();
                          
            jac_block.coeffRef(0, 0) = _P.x()/pow(_R.x(),_P.x()) * pow(p.x()-_C.x(),_P.x()-1);
            jac_block.coeffRef(0, 1) = _P.y()/pow(_R.y(),_P.y()) * pow(p.y()-_C.y(),_P.y()-1);
            jac_block.coeffRef(0, 2) = _P.z()/pow(_R.z(),_P.z()) * pow(p.z()-_C.z(),_P.z()-1);
                               
        }
    }
    
private:
    
    Eigen::Vector3d _C, _R, _P;
    std::string pname_;
};



class FrictionConstraint : public ConstraintSet {
    
public:

    FrictionConstraint(const std::string& force_name) :
        ConstraintSet(2, "FrictionConstraint " + force_name),
        fname_(force_name)
    {
        mu_= 1; 

    }
    
    
    void set_mu(const double& mu)
    {
        mu_ = mu;

    }
    


    VectorXd GetValues() const override
    {
            Eigen::VectorXd value;
            
            value.setZero(2);
                    
            Eigen::Vector3d F = GetVariables()->GetComponent(fname_)->GetValues();  
  
            int k; 
            for(int i = 0; i < 4; i++)
            {
                if(fname_ == ("F" + std::to_string(i+1)))
                {
                    k = i + 1;
                }
            }                               
                       
            Eigen::Vector3d  n_SE = GetVariables()->GetComponent("n" + std::to_string(k))->GetValues();
            
            value(0) = -F.dot(n_SE); 
            value(1) = (F-(n_SE.dot(F))*n_SE).norm() - mu_*(F.dot(n_SE));            
                  
//             std::cout <<"value " << value.transpose() << std::endl;        
  
        return value;
        
    };

    // Constant values should always be put into GetBounds(), not GetValues().
    // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
    VecBound GetBounds() const override
    {
        VecBound b(GetRows());
        
        for(int i = 0; i < 2; i++)
        {            
             b.at(i) = BoundSmallerZero;               
        }        
                            
        return b;
    }

    // This function provides the first derivative of the constraints.
    // In case this is too difficult to write, you can also tell the solvers to
    // approximate the derivatives by finite differences and not overwrite this
    // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
    {
        
      jac_block.setZero();
      
      Eigen::Vector3d F = GetVariables()->GetComponent(fname_)->GetValues();  
      
       int k; 
       for(int i = 0; i < 4; i++)
       {
           if(fname_ == ("F" + std::to_string(i+1)))
           {
               k = i + 1;
           }
       }                               
                              
       Eigen::Vector3d  n_SE = GetVariables()->GetComponent("n" + std::to_string(k))->GetValues();
       
       
       double t1 = F.dot(n_SE);
       
       double t2 = F.x()-n_SE.x()*t1;
       double t3 = F.y()-n_SE.y()*t1;
       double t4 = F.z()-n_SE.z()*t1;
       
       double t5 = F.x()*n_SE.x();
       double t6 = F.y()*n_SE.y();
       double t7 = F.z()*n_SE.z();
       
       if(var_set == fname_)
       {     
           
            jac_block.coeffRef(0, 0) = -n_SE.x();
            jac_block.coeffRef(0, 1) = -n_SE.y();
            jac_block.coeffRef(0, 2) = -n_SE.z();

            jac_block.coeffRef(1, 0) = (t2*(n_SE.x()*n_SE.x()-1.0)*2.0+n_SE.x()*n_SE.y()*t3*2.0+n_SE.x()*n_SE.z()*t4*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - mu_*n_SE.x();
            jac_block.coeffRef(1, 1) = (t3*(n_SE.y()*n_SE.y()-1.0)*2.0+n_SE.x()*n_SE.y()*t2*2.0+n_SE.y()*n_SE.z()*t4*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - mu_*n_SE.y();
            jac_block.coeffRef(1, 2) = (t4*(n_SE.z()*n_SE.z()-1.0)*2.0+n_SE.x()*n_SE.z()*t2*2.0+n_SE.y()*n_SE.z()*t3*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - mu_*n_SE.z();

       }
       
  
       if(var_set == ("n" + std::to_string(k)))
       {     
            jac_block.coeffRef(0, 0) = -F.x();
            jac_block.coeffRef(0, 1) = -F.y();
            jac_block.coeffRef(0, 2) = -F.z();
            
            jac_block.coeffRef(1, 0) = (t2*(t6+t7+t5*2.0)*2.0+F.x()*n_SE.y()*t3*2.0+F.x()*n_SE.z()*t4*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - mu_*F.x();
            jac_block.coeffRef(1, 1) = (t3*(t5+t7+t6*2.0)*2.0+F.y()*n_SE.x()*t2*2.0+F.y()*n_SE.z()*t4*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - mu_*F.y();
            jac_block.coeffRef(1, 2) = (t4*(t5+t6+t7*2.0)*2.0+F.z()*n_SE.x()*t2*2.0+F.z()*n_SE.y()*t3*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - mu_*F.z();
                     
       }
        
    }
    
private:
    
  double mu_;
  std::string fname_;
    
};




class NormalConstraint : public ConstraintSet {
    
public:

     NormalConstraint(const std::string& position_name) :
        ConstraintSet(3, "NormalConstraint " + position_name),
        pname_(position_name)
    {
        _C.setZero();
        _R.setOnes();
        _P << 8, 8, 4;
    }

    // The constraint value minus the constant value "1", moved to bounds.
    VectorXd GetValues() const override
    {
       
        Eigen::VectorXd value;
        value.setZero(3);
        
        Eigen::Vector3d p = GetVariables()->GetComponent(pname_)->GetValues(); 
        
        int k; 
        for(int i = 0; i < 4; i++)
        {
            if(pname_ == ("p" + std::to_string(i+1)))
            {
                k = i + 1;
            }
        }
        
        Eigen::Vector3d  n_SE = GetVariables()->GetComponent("n" + std::to_string(k))->GetValues();
        
        Eigen::Vector3d grad;
        grad.setZero(3);
        
        grad.x() = _P.x()/pow(_R.x(),_P.x()) * pow(p.x()-_C.x(),_P.x()-1);
        grad.y() = _P.y()/pow(_R.y(),_P.y()) * pow(p.y()-_C.y(),_P.y()-1);
        grad.z() = _P.z()/pow(_R.z(),_P.z()) * pow(p.z()-_C.z(),_P.z()-1);   
             
        value(0)= n_SE.x() + grad.x()/grad.norm();
        value(1)= n_SE.y() + grad.y()/grad.norm();
        value(2)= n_SE.z() + grad.z()/grad.norm();   
                
//         std::cout <<"value " << value.transpose() << std::endl; 
        
        return value;
        
    };
    
    void SetParam(const Eigen::Vector3d& C, const Eigen::Vector3d& R, const Eigen::Vector3d& P)
    {
        _C = C; // superellipsoid center
        _R = R; // radial axis
        _P = P; // r s t parameters (r=s super-ellipsoid)
    }

    // The only constraint in this set is an equality constraint to 1.
    // Constant values should always be put into GetBounds(), not GetValues().
    // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
    VecBound GetBounds() const override
    {
        VecBound b(GetRows());
         
        for(int i = 0; i < 3; i++)
        {            
             b.at(i) = Bounds(.0, .0);               
        }
                            
        return b; 
        
    }

    // This function provides the first derivative of the constraints.
    // In case this is too difficult to write, you can also tell the solvers to
    // approximate the derivatives by finite differences and not overwrite this
    // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
    {
        jac_block.setZero();
        
        Eigen::Vector3d p = GetVariables()->GetComponent(pname_)->GetValues();  
      
        int k; 
        for(int i = 0; i < 4; i++)
        {
            if(pname_ == ("p" + std::to_string(i+1)))
            {
                k = i + 1;
            }
        }                               
         

        if(var_set == ("n" + std::to_string(k)))
        {
            
             jac_block.coeffRef(0, 0) = 1.0;
             jac_block.coeffRef(1, 1) = 1.0;
             jac_block.coeffRef(2, 2) = 1.0; 
             
        }
        
        if(var_set == pname_)
        {
                
             double t2 = -_C.x()+p.x();
             double t3 = _C.x()-p.x();
             double t4 = 1.0/(t3*t3);
             double t6 = _P.y()*2.0;
             double t5 = pow(_R.y(),-t6);
             double t7 = _C.y()-p.y();
             double t8 = 1.0/(t7*t7);
             double t10 = _P.z()*2.0;
             double t9 = pow(_R.z(),-t10);
             double t11 = _C.z()-p.z();
             double t12 = 1.0/(t11*t11);
             double t13 = _P.y()*_P.y();
             double t14 = -_C.y()+p.y();
             double t15 = pow(t14,t6);
             double t16 = _P.z()*_P.z();
             double t17 = -_C.z()+p.z();
             double t18 = pow(t17,t10);
             double t19 = pow(_R.z(),t10);
             double t20 = pow(_R.y(),t6);
            
             jac_block.coeffRef(0, 0) = _P.x()*pow(_R.x(),-_P.x())*pow(t2,_P.x())*t4*t5*t8*t9*t12*(_P.x()-1.0)*1.0/pow(t5*t8*t13*t15+t9*t12*t16*t18+(_P.x()*_P.x())*pow(_R.x(),_P.x()*-2.0)*pow(t2,_P.x()*2.0)*t4,3.0/2.0)*((_C.y()*_C.y())*t16*t18*t20+(_C.z()*_C.z())*t13*t15*t19+(p.y()*p.y())*t16*t18*t20
                                        +(p.z()*p.z())*t13*t15*t19-_C.y()*p.y()*t16*t18*t20*2.0-_C.z()*p.z()*t13*t15*t19*2.0);
             
             t2 = _P.y()*2.0;
             t3 = -_C.x()+p.x();
             t4 = _P.y()*_P.y();
             t5 = -_C.y()+p.y();
             t6 = t2-2.0;
             t7 = pow(_R.y(),-t2);
                 
             jac_block.coeffRef(0, 1) = _P.x()*pow(_R.x(),-_P.x())*pow(t3,_P.x()-1.0)*t4*pow(t5,t2-3.0)*t6*t7*1.0/pow((_P.z()*_P.z())*pow(_R.z(),_P.z()*-2.0)*pow(-_C.z()+p.z(),_P.z()*2.0-2.0)+(_P.x()*_P.x())*pow(_R.x(),_P.x()*-2.0)*pow(t3,_P.x()*2.0-2.0)+t4*pow(t5,t6)*t7,3.0/2.0)*(-1.0/2.0);
             
             t2 = _P.z()*2.0;   
             t3 = -_C.x()+p.x();
             t4 = _P.z()*_P.z();
             t5 = -_C.z()+p.z();   
             t6 = t2-2.0;   
             t7 = pow(_R.z(),-t2);  
             
             jac_block.coeffRef(0, 2) =  _P.x()*pow(_R.x(),-_P.x())*pow(t3,_P.x()-1.0)*t4*pow(t5,t2-3.0)*t6*t7*1.0/pow((_P.y()*_P.y())*pow(_R.y(),_P.y()*-2.0)*pow(-_C.y()+p.y(),_P.y()*2.0-2.0)+(_P.x()*_P.x())*pow(_R.x(),_P.x()*-2.0)*pow(t3,_P.x()*2.0-2.0)+t4*pow(t5,t6)*t7,3.0/2.0)*(-1.0/2.0);                              
             
             t2 = _P.x()*2.0;
             t3 = _P.x()*_P.x();
             t4 = -_C.x()+p.x();
             t5 = t2-2.0;
             t6 = -_C.y()+p.y();
             t7 = pow(_R.x(),-t2);
         
             jac_block.coeffRef(1, 0) = _P.y()*pow(_R.y(),-_P.y())*t3*pow(t4,t2-3.0)*t5*pow(t6,_P.y()-1.0)*t7*1.0/pow((_P.z()*_P.z())*pow(_R.z(),_P.z()*-2.0)*pow(-_C.z()+p.z(),_P.z()*2.0-2.0)+(_P.y()*_P.y())*pow(_R.y(),_P.y()*-2.0)*pow(t6,_P.y()*2.0-2.0)+t3*pow(t4,t5)*t7,3.0/2.0)*(-1.0/2.0);
            
             t3 = _P.x()*2.0;
             t2 = pow(_R.x(),-t3);
             t4 = _C.x()-p.x();
             t5 = 1.0/(t4*t4);
             t6 = -_C.y()+p.y();
             t7 = _C.y()-p.y();
             t8 = 1.0/(t7*t7);
             t10 = _P.z()*2.0;
             t9 = pow(_R.z(),-t10);
             t11 = _C.z()-p.z();
             t12 = 1.0/(t11*t11);
             t13 = _P.x()*_P.x();
             t14 = -_C.x()+p.x();
             t15 = pow(t14,t3);
             t16 = _P.z()*_P.z();
             t17 = -_C.z()+p.z();
             t18 = pow(t17,t10);
             t19 = pow(_R.z(),t10);
             t20 = pow(_R.x(),t3);
             
             jac_block.coeffRef(1, 1) = _P.y()*pow(_R.y(),-_P.y())*t2*t5*pow(t6,_P.y())*t8*t9*t12*(_P.y()-1.0)*1.0/pow(t2*t5*t13*t15+t9*t12*t16*t18+(_P.y()*_P.y())*pow(_R.y(),_P.y()*-2.0)*pow(t6,_P.y()*2.0)*t8,3.0/2.0)*((_C.x()*_C.x())*t16*t18*t20+(_C.z()*_C.z())*t13*t15*t19
                                        +(p.x()*p.x())*t16*t18*t20+(p.z()*p.z())*t13*t15*t19-_C.x()*p.x()*t16*t18*t20*2.0-_C.z()*p.z()*t13*t15*t19*2.0);

             t2 = _P.z()*2.0;
             t3 = -_C.y()+p.y();
             t4 = _P.z()*_P.z();
             t5 = -_C.z()+p.z();
             t6 = t2-2.0;
             t7 = pow(_R.z(),-t2);
             
             jac_block.coeffRef(1, 2) = _P.y()*pow(_R.y(),-_P.y())*pow(t3,_P.y()-1.0)*t4*pow(t5,t2-3.0)*t6*t7*1.0/pow((_P.x()*_P.x())*pow(_R.x(),_P.x()*-2.0)*pow(-_C.x()+p.x(),_P.x()*2.0-2.0)+(_P.y()*_P.y())*pow(_R.y(),_P.y()*-2.0)*pow(t3,_P.y()*2.0-2.0)+t4*pow(t5,t6)*t7,3.0/2.0)*(-1.0/2.0);

             t2 = _P.x()*2.0;
             t3 = _P.x()*_P.x();
             t4 = -_C.x()+p.x();
             t5 = t2-2.0;
             t6 = -_C.z()+p.z();
             t7 = pow(_R.x(),-t2);
             
             jac_block.coeffRef(2, 0) = _P.z()*pow(_R.z(),-_P.z())*t3*pow(t4,t2-3.0)*t5*pow(t6,_P.z()-1.0)*t7*1.0/pow((_P.y()*_P.y())*pow(_R.y(),_P.y()*-2.0)*pow(-_C.y()+p.y(),_P.y()*2.0-2.0)+(_P.z()*_P.z())*pow(_R.z(),_P.z()*-2.0)*pow(t6,_P.z()*2.0-2.0)+t3*pow(t4,t5)*t7,3.0/2.0)*(-1.0/2.0);

             t2 = _P.y()*2.0;
             t3 = _P.y()*_P.y();
             t4 = -_C.y()+p.y();
             t5 = t2-2.0;
             t6 = -_C.z()+p.z();
             t7 = pow(_R.y(),-t2);
 
             jac_block.coeffRef(2, 1) = _P.z()*pow(_R.z(),-_P.z())*t3*pow(t4,t2-3.0)*t5*pow(t6,_P.z()-1.0)*t7*1.0/pow((_P.x()*_P.x())*pow(_R.x(),_P.x()*-2.0)*pow(-_C.x()+p.x(),_P.x()*2.0-2.0)+(_P.z()*_P.z())*pow(_R.z(),_P.z()*-2.0)*pow(t6,_P.z()*2.0-2.0)+t3*pow(t4,t5)*t7,3.0/2.0)*(-1.0/2.0);

             t3 = _P.x()*2.0;
             t2 = pow(_R.x(),-t3);
             t4 = _C.x()-p.x();
             t5 = 1.0/(t4*t4);
             t7 = _P.y()*2.0;
             t6 = pow(_R.y(),-t7);
             t8 = _C.y()-p.y();
             t9 = 1.0/(t8*t8);
             t10 = -_C.z()+p.z();
             t11 = _C.z()-p.z();
             t12 = 1.0/(t11*t11);
             t13 = _P.x()*_P.x();
             t14 = -_C.x()+p.x();
             t15 = pow(t14,t3);
             t16 = _P.y()*_P.y();
             t17 = -_C.y()+p.y();
             t18 = pow(t17,t7);
             t19 = pow(_R.y(),t7);
             t20 = pow(_R.x(),t3);
              
             jac_block.coeffRef(2, 2) = _P.z()*pow(_R.z(),-_P.z())*t2*t5*t6*t9*pow(t10,_P.z())*t12*(_P.z()-1.0)*1.0/pow(t2*t5*t13*t15+t6*t9*t16*t18+(_P.z()*_P.z())*pow(_R.z(),_P.z()*-2.0)*pow(t10,_P.z()*2.0)*t12,3.0/2.0)*((_C.x()*_C.x())*t16*t18*t20+(_C.y()*_C.y())*t13*t15*t19
                                        +(p.x()*p.x())*t16*t18*t20+(p.y()*p.y())*t13*t15*t19-_C.x()*p.x()*t16*t18*t20*2.0-_C.y()*p.y()*t13*t15*t19*2.0);
             
                               
        }
    }
    
private:
    
    Eigen::Vector3d _C, _R, _P;
    std::string pname_;
};



} // namespace opt


