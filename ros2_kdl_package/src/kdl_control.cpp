#include "kdl_control.h"
#include "utils.h"  
#include <Eigen/Dense>

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{

    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();


    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity();
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{

}

Eigen::VectorXd KDLController::velocity_ctrl_null(const Eigen::Vector3d &p_des,
                                                  double Kp,
                                                  const Eigen::VectorXd &q_current)
{

    KDL::Frame ee_frame = robot_->getEEFrame();
    Eigen::Vector3d p_curr(ee_frame.p.data);

    Eigen::Vector3d e_p = p_des - p_curr;

    Eigen::MatrixXd J = robot_->getEEJacobian().data;
    Eigen::MatrixXd J_pos = J.topRows(3);          
    Eigen::MatrixXd J_pos_pinv = pseudoinverse(J_pos); 

    Eigen::VectorXd qdot_task = J_pos_pinv * (Kp * e_p);  

    Eigen::VectorXd qdot0 = Eigen::VectorXd::Zero(q_current.size());
    double lambda = 12.0; 
    double eps = 1e-6;    
    double qdot0_max = 0.3; 

    Eigen::MatrixXd limits = robot_->getJntLimits();

    for (int i = 0; i < q_current.size(); i++)
    {
        double qi = q_current(i);
        double q_min = limits(i, 0);
        double q_max = limits(i, 1);

        double a = q_max - qi;
        double b = qi - q_min;
        if (std::abs(a) < eps) a = eps;
        if (std::abs(b) < eps) b = eps;    
       
        double numer = std::pow(q_max - q_min, 2);
        double denom = std::pow(a * b, 2);
        double dq0 = - (numer / lambda) * (q_max + q_min - 2.0 * qi) / denom;
    
        if (!std::isfinite(dq0)) dq0 = 0.0;
        if (dq0 > qdot0_max) dq0 = qdot0_max;
        if (dq0 < -qdot0_max) dq0 = -qdot0_max;

        qdot0(i) = dq0;
    }
    
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(q_current.size(), q_current.size()) - J_pos_pinv * J_pos;
  
    Eigen::VectorXd qdot = qdot_task + N * qdot0;
    
    return qdot;
}


Eigen::VectorXd KDLController::velocity_ctrl_vision(
    const Eigen::Vector3d &marker_pos_image,   
    const Eigen::VectorXd &q_curr,
    double K_gain)
{
   
    Eigen::Matrix3d R_image_to_camera;
    R_image_to_camera << -1,  0,  0,
                         0, -1,  0,
                         0,  0,  1;

    Eigen::Vector3d marker_pos_camera = R_image_to_camera * marker_pos_image;
  
    robot_->update(toStdVector(q_curr), std::vector<double>(robot_->getNrJnts(), 0.0));
  
    KDL::Frame ee_frame = robot_->getEEFrame();
    Eigen::Matrix3d R_world_to_ee = toEigen(ee_frame.M);
    Eigen::Vector3d p_ee(ee_frame.p.data);
   
    double dist = marker_pos_camera.norm();
    if (dist < 1e-6)
        dist = 1e-6;

    Eigen::Vector3d s = marker_pos_camera / dist;   
    Eigen::Vector3d s_d(0.0, 0.0, 1.0);            

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d skew_s;
    skew_s <<      0, -s(2),  s(1),
                s(2),     0, -s(0),
               -s(1),  s(0),     0;

    Eigen::MatrixXd L(3,6);
    L.block<3,3>(0,0) = -(I - s * s.transpose()) / dist;
    L.block<3,3>(0,3) = skew_s;

    Eigen::MatrixXd J_ee = robot_->getEEJacobian().data;
    int n_jnts = J_ee.cols();

    Eigen::MatrixXd J_c = J_ee;

    Eigen::MatrixXd LJ = L * J_c;
    Eigen::MatrixXd LJ_pinv = pseudoinverse(LJ);
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n_jnts, n_jnts) - LJ_pinv * LJ;

    Eigen::Vector3d s_error = s_d - s;

    Eigen::VectorXd q_dot = K_gain * (LJ_pinv * s_error);
 
    Eigen::VectorXd q_dot0 = Eigen::VectorXd::Zero(n_jnts);
    q_dot += N * q_dot0;

    std::cout << "s_error: " << s_error.transpose()
              << " | dist: " << dist << std::endl;

    return q_dot;
}
