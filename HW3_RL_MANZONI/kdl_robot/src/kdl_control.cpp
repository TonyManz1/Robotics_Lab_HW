#include "kdl_ros_control/kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // Read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // Calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
   // Retrieve initial ee pose
   KDL::Frame Fi =  robot_->getEEFrame();
   
   // Calculate gain matrices
   Eigen::Matrix<double,6,6> Kp, Kd;
   Kp=Eigen::MatrixXd::Zero(6,6);
   Kd=Eigen::MatrixXd::Zero(6,6);
   Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
   Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
   Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
   Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();

   // Read current state
   Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;
   Eigen::Matrix<double,7,7> I = Eigen::Matrix<double,7,7>::Identity();
   Eigen::Matrix<double,7,7> M = robot_->getJsim();
   //Eigen::Matrix<double,7,6> Jpinv = weightedPseudoInverse(M,J);
   Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);

   // Position
   Eigen::Vector3d p_d(_desPos.p.data);
   Eigen::Vector3d p_e(robot_->getEEFrame().p.data);
   Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data);
   Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
   R_d = matrixOrthonormalization(R_d);
   R_e = matrixOrthonormalization(R_e);

   // Velocity
   Eigen::Vector3d dot_p_d(_desVel.vel.data);
   Eigen::Vector3d dot_p_e(robot_->getEEVelocity().vel.data);
   Eigen::Vector3d omega_d(_desVel.rot.data);
   Eigen::Vector3d omega_e(robot_->getEEVelocity().rot.data);

   // Acceleration
   Eigen::Matrix<double,6,1> dot_dot_x_d;
   Eigen::Matrix<double,3,1> dot_dot_p_d(_desAcc.vel.data);
   Eigen::Matrix<double,3,1> dot_dot_r_d(_desAcc.rot.data);

   // Compute linear errors
   Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);
   Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_p_d,dot_p_e);

    // Shared control
    // Eigen::Vector3d lin_acc;
    // lin_acc << _desAcc.vel.x(), _desAcc.vel.y(), _desAcc.vel.z(); //use desired acceleration
    // lin_acc << dot_dot_p_d + _Kdp*(dot_e_p) + _Kpp*(e_p); // assuming no friction no loads
    // Eigen::Matrix<double,3,3> R_sh = shCntr(lin_acc);

   // Compute orientation errors
   Eigen::Matrix<double,3,1> e_o_w = computeOrientationError(toEigen(Fi.M), toEigen(robot_->getEEFrame().M)); 
   Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_d,R_e);
   Eigen::Matrix<double,3,1> dot_e_o = computeOrientationVelocityError(omega_d,
                                                                       omega_e,
                                                                       R_d,
                                                                       R_e);

   //Errors                                                                 
   Eigen::Matrix<double,6,1> x_tilde;
   Eigen::Matrix<double,6,1> dot_x_tilde;
   x_tilde << e_p, e_o_w[0],e_o[1],e_o[2];
   dot_x_tilde << dot_e_p, -omega_e;//dot_e_o;
   dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;

   // Null space control
   double cost;
   Eigen::VectorXd grad = gradientJointLimits(robot_->getJntValues(),robot_->getJntLimits(),cost);

////    std::cout << "---------------------" << std::endl;
////    std::cout << "p_d: " << std::endl << p_d << std::endl;
////    std::cout << "p_e: " << std::endl << p_e << std::endl;
////    std::cout << "dot_p_d: " << std::endl << dot_p_d << std::endl;
////    std::cout << "dot_p_e: " << std::endl << dot_p_e << std::endl;
////    std::cout << "R_sh*R_d: " << std::endl << R_d << std::endl;
////    std::cout << "R_e: " << std::endl << R_e << std::endl;
////    std::cout << "omega_d: " << std::endl << omega_d << std::endl;
////    std::cout << "omega_e: " << std::endl << omega_e << std::endl;
////    std::cout << "x_tilde: " << std::endl << x_tilde << std::endl;
////    std::cout << "dot_x_tilde: " << std::endl << dot_x_tilde << std::endl;
////    std::cout << "jacobian: " << std::endl << robot_->getJacobian() << std::endl;
////    std::cout << "jpinv: " << std::endl << Jpinv << std::endl;
////    std::cout << "jsim: " << std::endl << robot_->getJsim() << std::endl;
////    std::cout << "c: " << std::endl << robot_->getCoriolis().transpose() << std::endl;
////    std::cout << "g: " << std::endl << robot_->getGravity().transpose() << std::endl;
////    std::cout << "q: " << std::endl << robot_->getJntValues().transpose() << std::endl;
////    std::cout << "Jac Dot qDot: " << std::endl << robot_->getJacDotqDot().transpose() << std::endl;
////    std::cout << "Jnt lmt cost: " << std::endl << cost << std::endl;
////    std::cout << "Jnt lmt gradient: " << std::endl << grad.transpose() << std::endl;
////    std::cout << "---------------------" << std::endl;

   // Inverse dynamics
   //std::cout<<robot_->getJntVelocities()<<std::endl;         //TEST

   /*APPLYING FORMULAE 4 (HW_2)*/
   Eigen::Matrix<double,6,1> y;
   y << dot_dot_x_d - robot_->getEEJacDotqDot() + Kd*dot_x_tilde + Kp*x_tilde;

   //RETURN By + n
   return M * (Jpinv*y)+ robot_->getGravity() + robot_->getCoriolis();
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp,
                                      double _Kdp)
{
   
   // Calculate gain matrices
   Eigen::Matrix<double,3,3> Kp, Kd;
   Kp = _Kpp*Eigen::Matrix3d::Identity();       //3x3 proportional gain matrix of position error
   Kd = _Kdp*Eigen::Matrix3d::Identity();       //3x3 derivative gain matrix of position error
   
   // Read current state
   Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;
   Eigen::Matrix<double,3,7> J_red = J.topRows(3);
   Eigen::Matrix<double,7,7> M = robot_->getJsim();
   Eigen::Matrix<double,7,3> Jpinv = pseudoinverse(J_red);

   // Position
   Eigen::Vector3d p_d(_desPos.p.data);
   Eigen::Vector3d p_e(robot_->getEEFrame().p.data);

   // Velocity
   Eigen::Vector3d dot_p_d(_desVel.vel.data);
   Eigen::Vector3d dot_p_e(robot_->getEEVelocity().vel.data);

   // Acceleration
   Eigen::Matrix<double,3,1> dot_dot_x_d;
   Eigen::Matrix<double,3,1> dot_dot_p_d(_desAcc.vel.data);

   // Compute linear errors
   Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);
   Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_p_d,dot_p_e);

   // Errors                                                          
   Eigen::Matrix<double,3,1> x_tilde;
   Eigen::Matrix<double,3,1> dot_x_tilde;
   x_tilde << e_p;
   dot_x_tilde << dot_e_p;
   dot_dot_x_d << dot_dot_p_d;

   // Inverse dynamics
   Eigen::Matrix<double,3,1> y;

   /*APPLYING FORMULAE 4 (HW_2)*/
   y << dot_dot_x_d - robot_->getEEJacDotqDot_yellow() + Kd*dot_x_tilde + Kp*x_tilde;
   
   //RETURN By + n
   return M * (Jpinv*y)+ robot_->getGravity() + robot_->getCoriolis();

}

