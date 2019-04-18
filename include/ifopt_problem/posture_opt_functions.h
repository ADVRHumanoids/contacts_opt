#include <eigen3/Eigen/Dense>


Eigen::MatrixXd get_jacobian(const Eigen::Vector3d& q, 
                             const Eigen::Vector3d& d)
{
    
    Eigen::MatrixXd jacob;
    jacob.setZero(2,3);
    
    double t2 = q[0]+q[1];
    double t3 = std::sin(t2);
    double t4 = q[0]+q[1]+q[2];
    double t5 = std::sin(t4);
    double t6 = std::cos(t2);
    double t7 = d[1]*t6;
    double t8 = std::cos(t4);
    double t9 = d[2]*t8;
    
    jacob(0,0) = -d[1]*t3-d[2]*t5-d[0]*std::sin(q[0]);
    jacob(0,1) = -d[1]*t3-d[2]*t5;
    jacob(0,2) = -d[2]*t5;
    jacob(1,0) = t7+t9+d[0]*std::cos(q[0]);
    jacob(1,1) = t7+t9;
    jacob(1,2) = t9;

    return jacob;
}

Eigen::MatrixXd get_manip_jacobian(const Eigen::Vector3d& q, 
                                   const Eigen::Vector3d& d)
{
    
    Eigen::Vector3d manip_jacob;

    double t2 = d[0]*d[0];
    double t3 = q[1]*2.0;
    double t4 = d[2]*d[2];
    double t5 = q[2]*2.0;
    double t6 = d[1]*d[1];
    double t7 = t3+t5;
    double t8 = q[1]+t5;
    double t9 = q[2]+t3;
    double t10 = std::sin(t7);
    double t11 = t2*t4*t10*2.0;
    double t12 = std::sin(t8);
    double t13 = std::sin(t9);
    double t14 = (t2*t6)/2.0;
    double t15 = t2*t4;
    double t16 = t4*t6;
    double t17 = std::cos(t3);
    double t18 = std::cos(t5);
    double t19 = std::cos(t7);
    double t20 = std::cos(q[1]);
    double t21 = d[0]*d[1]*t4*t20;
    double t22 = std::cos(q[2]);
    double t23 = d[1]*d[2]*t2*t22;
    double t24 = std::cos(t8);
    double t25 = std::cos(t9);
    double t26 = t14+t15+t16+t21+t23-t2*t4*t19-(t2*t6*t17)/2.0-t4*t6*t18-d[0]*d[1]*t4*t24-d[1]*d[2]*t2*t25;
    double t27 = 1.0/std::sqrt(t26);
    
    manip_jacob[0] = 0.0;
    manip_jacob[1] = (t27*(t11+t2*t6*std::sin(t3)+d[0]*d[1]*t4*t12+d[1]*d[2]*t2*t13*2.0-d[0]*d[1]*t4*std::sin(q[1])))/2.0;
    manip_jacob[2] = (t27*(t11+t4*t6*std::sin(t5)*2.0+d[0]*d[1]*t4*t12*2.0+d[1]*d[2]*t2*t13-d[1]*d[2]*t2*std::sin(q[2])))/2.0;

    return manip_jacob;
}


Eigen::Vector3d get_tau(const Eigen::Vector3d& q, 
                        const Eigen::Vector3d& d, 
                        const Eigen::Vector2d& F)
{
    
    Eigen::Vector3d tau;

    double t2 = q[0]+q[1];
    double t3 = q[0]+q[1]+q[2];
    double t4 = std::cos(t2);
    double t5 = d[1]*t4;
    double t6 = std::cos(t3);
    double t7 = d[2]*t6;
    double t8 = std::sin(t2);
    double t9 = d[1]*t8;
    double t10 = std::sin(t3);
    double t11 = d[2]*t10;
    tau[0] = F[1]*(t5+t7+d[0]*std::cos(q[0]))-F[0]*(t9+t11+d[0]*std::sin(q[0]));
    tau[1] = F[1]*(t5+t7)-F[0]*(t9+t11);
    tau[2] = F[1]*d[2]*t6-F[0]*d[2]*t10;

    return tau;
}

Eigen::Matrix3d get_dtau_dq(const Eigen::Vector3d& q, 
                        const Eigen::Vector3d& d, 
                        const Eigen::Vector2d& F)
{
    Eigen::Matrix3d dtau;
    
    double t2 = q[0]+q[1];
    double t3 = q[0]+q[1]+q[2];
    double t4 = std::cos(t2);
    double t5 = d[1]*t4;
    double t6 = std::cos(t3);
    double t7 = d[2]*t6;
    double t8 = std::sin(t2);
    double t9 = d[1]*t8;
    double t10 = std::sin(t3);
    double t11 = d[2]*t10;
    double t12 = t5+t7;
    double t13 = t9+t11;
    double t15 = F[0]*t12;
    double t16 = F[1]*t13;
    double t14 = -t15-t16;
    double t18 = F[0]*d[2]*t6;
    double t19 = F[1]*d[2]*t10;
    double t17 = -t18-t19;
    dtau(0,0) = -F[0]*(t5+t7+d[0]*std::cos(q[0]))-F[1]*(t9+t11+d[0]*std::sin(q[0]));
    dtau(0,1) = t14;
    dtau(0,2) = t17;
    dtau(1,0) = t14;
    dtau(1,1) = t14;
    dtau(1,2) = t17;
    dtau(2,0) = t17;
    dtau(2,1) = t17;
    dtau(2,2) = t17;
    
    return dtau;

}
