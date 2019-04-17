#include <eigen3/Eigen/Dense>


Eigen::MatrixXd get_jacobian(const Eigen::Vector3d& q, 
                        const Eigen::Vector3d& d)
{
    
    Eigen::MatrixXd jacob;
    jacob.setZero(2,3);

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

    jacob(0,0) = t5+t7+d[0]*std::cos(q[0]);
    jacob(0,1) = t5+t7;
    jacob(0,2) = d[2]*t6;
    jacob(1,0) = -(t9+t11+d[0]*std::sin(q[0]));
    jacob(1,1) = -(t9+t11);
    jacob(1,2) = -d[2]*t10;

    return jacob;
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
