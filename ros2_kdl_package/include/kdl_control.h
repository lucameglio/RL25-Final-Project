#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);

    Eigen::VectorXd velocity_ctrl_null(const Eigen::Vector3d &p_des, double Kp, const Eigen::VectorXd &q_current);

    Eigen::VectorXd velocity_ctrl_vision(
        const Eigen::Vector3d &marker_pos_image,      // posizione Aruco in frame immagine
        const Eigen::VectorXd &q_curr,
        double K_gain);


private:

    KDLRobot* robot_;

};

#endif
