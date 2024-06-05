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
    // Read current joint state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // Calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

/*FUNCTION TO IMPLEMENT INVERSE DYNAMICS OPERATIONAL SPACE CONTROLLER*/
Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
   
   // Calculate gain matrices 
   Eigen::Matrix<double,6,6> Kp, Kd;
   Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();    //3*3 MATRIX POSITION ERROR PROPORTIONAL GAIN 
   Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();    //3*3 MATRIX ORIENTATION ERROR PROPORTIONAL GAIN 
   Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();    //3*3 MATRIX POSITION ERROR DERIVATIVE GAIN 
   Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();    //3*3 MATRIX ORIENTATION ERROR DERIVATIVE GAIN 

   /*CONVERSION JACOBIAN TO MATRIX*/
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
   Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_d,R_e);
   Eigen::Matrix<double,3,1> dot_e_o = computeOrientationVelocityError(omega_d,
                                                                       omega_e,
                                                                       R_d,
                                                                       R_e);

   // ERROR                                                                    
   Eigen::Matrix<double,6,1> x_tilde;
   Eigen::Matrix<double,6,1> dot_x_tilde;
   x_tilde << e_p, e_o;
   dot_x_tilde << dot_e_p, - omega_e;//dot_e_o;
   dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;

   // Null space control
   double cost;
   Eigen::VectorXd grad = gradientJointLimits(robot_->getJntValues(),robot_->getJntLimits(),cost);

//    std::cout << "---------------------" << std::endl;
//    std::cout << "p_d: " << std::endl << p_d << std::endl;
//    std::cout << "p_e: " << std::endl << p_e << std::endl;
//    std::cout << "dot_p_d: " << std::endl << dot_p_d << std::endl;
//    std::cout << "dot_p_e: " << std::endl << dot_p_e << std::endl;
//    std::cout << "R_sh*R_d: " << std::endl << R_d << std::endl;
//    std::cout << "R_e: " << std::endl << R_e << std::endl;
//    std::cout << "omega_d: " << std::endl << omega_d << std::endl;
//    std::cout << "omega_e: " << std::endl << omega_e << std::endl;
//    std::cout << "x_tilde: " << std::endl << x_tilde << std::endl;
//    std::cout << "dot_x_tilde: " << std::endl << dot_x_tilde << std::endl;
//    std::cout << "jacobian: " << std::endl << robot_->getJacobian() << std::endl;
//    std::cout << "jpinv: " << std::endl << Jpinv << std::endl;
//    std::cout << "jsim: " << std::endl << robot_->getJsim() << std::endl;
//    std::cout << "c: " << std::endl << robot_->getCoriolis().transpose() << std::endl;
//    std::cout << "g: " << std::endl << robot_->getGravity().transpose() << std::endl;
//    std::cout << "q: " << std::endl << robot_->getJntValues().transpose() << std::endl;
//    std::cout << "Jac Dot qDot: " << std::endl << robot_->getJacDotqDot().transpose() << std::endl;
//    std::cout << "Jnt lmt cost: " << std::endl << cost << std::endl;
//    std::cout << "Jnt lmt gradient: " << std::endl << grad.transpose() << std::endl;
//    std::cout << "---------------------" << std::endl;

   /*INVERSE DYNAMICS*/
   //std::cout<<robot_->getJntVelocities()<<std::endl;         //TEST

   //DEFINE j_dot
   Eigen::Matrix<double,6,7> Jdot = robot_->getEEJacDotqDot().data;
   
   //Compute Analitycal Jacobian Ja=Ta^(-1)*J
    Eigen::Matrix<double,3,1> euler = computeEulerAngles(R_e);
    Eigen::Matrix<double,6,7> JA = AnalitycalJacobian(J,euler);
    Eigen::Matrix<double,7,6> JApinv = pseudoinverse(JA);

   // Compute TA_inv
   //Eigen::Matrix<double,6,> TA_inv = JA * J.inverse();
   Eigen::Matrix<double,6,6> TA;
   TA.block(0,0,3,3) = Eigen::Matrix3d::Identity();
   TA.block(3,3,3,3) = T_matrix(euler);
   Eigen::Matrix<double,6,6> TA_inv = TA.inverse();

   // Compute TA_dot
   Eigen::Matrix<double,6,6> TA_dot;
   TA_dot.block(0,0,3,3) = Eigen::Matrix3d::Identity();
   TA_dot.block(3,3,3,3) = Tdot_matrix(euler);

   // Compute JA_dot 
   Eigen::Matrix<double,6,7> JAdot = TA_inv*(Jdot - TA_dot*JA);

   /*APPLYING FORMULAE 4*/
   Eigen::Matrix<double,6,1> y;
   //y << dot_dot_x_d - Jdot*robot_->getJntVelocities() + Kd*dot_x_tilde + Kp*x_tilde;
   y << dot_dot_x_d - JAdot*robot_->getJntVelocities() + Kd*dot_x_tilde + Kp*x_tilde;

   //RETURN By + n
   //return M * (Jpinv*y )+ robot_->getGravity() + robot_->getCoriolis();
   return M * (JApinv*y )+ robot_->getGravity() + robot_->getCoriolis();
}

