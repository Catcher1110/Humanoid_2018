#include "Valkyrie_InvKinematics.hpp"
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <rbdl/urdfreader.h>
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <Eigen/QR>

using namespace RigidBodyDynamics;

Valkyrie_InvKinematics::Valkyrie_InvKinematics():max_iter_(1000){
    model_ = new Valkyrie_Model();
}

Valkyrie_InvKinematics::~Valkyrie_InvKinematics(){
    delete model_;
}

void Valkyrie_InvKinematics::getLegConfigAtVerticalPosture(
        int link_id, const dynacore::Vect3 & target_foot_pos,
        const dynacore::Vector & guess_Q, dynacore::Vector & config_sol){

    // Vertial Orientation
    config_sol = guess_Q;
    dynacore::Vector zero_qdot(valkyrie::num_qdot);
    zero_qdot.setZero();

    config_sol[valkyrie_joint::virtual_Rx] = 0.;
    config_sol[valkyrie_joint::virtual_Ry] = 0.;
    config_sol[valkyrie_joint::virtual_Rz] = 0.;
    config_sol[valkyrie_joint::virtual_Rw] = 1.;
    model_->UpdateSystem(guess_Q, zero_qdot);
    
    // Find Foot body id
    unsigned int bodyid;
    int leg_jidx;
    switch(link_id){
        case valkyrie_link::leftFoot:
            leg_jidx = valkyrie_joint::leftHipYaw;
            break;
        case valkyrie_link::rightFoot:
            leg_jidx = valkyrie_joint::rightHipYaw;
            break;
        default:
            printf("[Inverse Kinematics] Not valid link id\n");
            exit(0);
    }

    // Set parameters up
    int num_iter(0);
    double err(1000.0);
    double pre_err = err + 1000.;
    dynacore::Vect3 err_vec;
    dynacore::Vector delta_q;
    dynacore::Vect3 current_pos;
    dynacore::Matrix J, J_leg, Jinv;

    J = dynacore::Matrix::Zero(3, valkyrie::num_qdot);
    // search
    while((num_iter < max_iter_) && (err > 0.000001)){  // && (pre_err > err)){
        pre_err = err;
        model_->getPos(link_id, current_pos);
        err_vec = (target_foot_pos - current_pos);
        
        model_->getFullJacobian(link_id, J);
        J_leg = J.block(3, leg_jidx, 3, valkyrie::num_leg_joint);
        dynacore::pseudoInverse(J_leg, 0.0001, Jinv);
        
        delta_q = Jinv *  err_vec;
        config_sol.segment(leg_jidx, valkyrie::num_leg_joint) +=  delta_q;
        model_->UpdateSystem(config_sol, zero_qdot);

        ++num_iter;
        err = err_vec.norm();

        if(num_iter == max_iter_){
            printf("%d iter, err: %f\n", num_iter, err);
        }
    }
}

void Valkyrie_InvKinematics::getDoubleSupportLegConfig(
        const dynacore::Vector & current_Q,
        const dynacore::Quaternion & des_quat,
        const double & des_height, dynacore::Vector & config_sol){

    config_sol = current_Q;
    dynacore::Vector zero_qdot = dynacore::Vector::Zero(valkyrie::num_qdot);
    model_->UpdateSystem(config_sol, zero_qdot);

    // Get the Jacobians of the foot
    int body_id = valkyrie_link::torso;
    int lfoot_id = valkyrie_link::leftFoot;
    int rfoot_id = valkyrie_link::rightFoot;

    dynacore::Matrix full_J;
    dynacore::Matrix J_left_foot, J_right_foot;
    
    model_->getFullJacobian(lfoot_id, J_left_foot);
    model_->getFullJacobian(rfoot_id, J_right_foot);

    // Stack the Constraint Jacobians
    dynacore::Matrix J1 = dynacore::Matrix::Zero(12, valkyrie::num_qdot);
    J1.block(0,0, 6, valkyrie::num_qdot) = J_left_foot;
    J1.block(6,0, 6, valkyrie::num_qdot) = J_right_foot;
    //dynacore::pretty_print(J1, std::cout, "J1");


    // Create the Nullspace
    dynacore::Matrix J1_pinv;
    double threshold = 0.00001;
    dynacore::pseudoInverse(J1, threshold, J1_pinv);
    //dynacore::Matrix J1_pinv = J1.completeOrthogonalDecomposition().pseudoInverse();
    dynacore::Matrix N1 = 
        dynacore::Matrix::Identity(valkyrie::num_qdot, valkyrie::num_qdot) - 
        J1_pinv*J1;

    // Get the Jacobian of the body
    dynacore::Matrix J_body;
    model_->getFullJacobian(body_id, J_body);

    // Get the Jacobian rows for Body Roll, Pitch and Height
    int dim_ctrl(5);
    dynacore::Matrix J2 = dynacore::Matrix::Zero(dim_ctrl, valkyrie::num_qdot);
    //J2.block(0, 0, 2, valkyrie::num_qdot) = J_body.block(0, 0, 2, valkyrie::num_qdot);  // (Rx, Ry) Roll and Pitch 
    //J2.block(2, 0, 1, valkyrie::num_qdot) = J_body.block(5, 0, 1, valkyrie::num_qdot);  //  Z - body height
    J2.block(0,0, 2, valkyrie::num_qdot) = J_body.block(0, 0, 2, valkyrie::num_qdot);
    J2(2, 2) = 1.;
    //Compute Orientation Error
    // Orientation
    dynacore::Quaternion curr_quat;
    model_->getOri(body_id, curr_quat);

    dynacore::Quaternion err_quat = dynacore::QuatMultiply(des_quat, curr_quat.inverse());
    dynacore::Vect3 ori_err;
    dynacore::convert(err_quat, ori_err);

    // Compute Height Error
    double height_error = des_height - current_Q[2];
    
    // Construct the operational space error
    dynacore::Vector delta_x = dynacore::Vector::Zero(dim_ctrl);
    delta_x[0] = ori_err[0];   
    delta_x[1] = ori_err[1];
    delta_x[2] = height_error;

    dynacore::Matrix J2N1 = J2*N1;
    dynacore::Matrix J2N1_pinv;
    dynacore::pseudoInverse(J2N1, threshold, J2N1_pinv);
    dynacore::Vector delta_q(valkyrie::num_qdot); delta_q.setZero();
    delta_q = J2N1_pinv*delta_x;
    
    config_sol.segment(valkyrie::num_virtual, valkyrie::num_act_joint) 
        += delta_q.segment(valkyrie::num_virtual, valkyrie::num_act_joint);
    
    //dynacore::pretty_print(config_sol, std::cout, "config_sol");
    //dynacore::pretty_print(J1, std::cout, "J1");
    //dynacore::pretty_print(J2N1, std::cout, "J2N1");
    //dynacore::pretty_print(J2N1_pinv, std::cout, "J2N1_pinv");
    //dynacore::pretty_print(delta_x, std::cout, "delta x");
    //dynacore::pretty_print(delta_q, std::cout, "delta q");
}

void Valkyrie_InvKinematics::getSingleSupportFullConfigSeperation(
        const dynacore::Vector & current_Q,
        const dynacore::Quaternion & des_quat,
        const double & des_height, 
        int swing_foot_,
        const dynacore::Vect3 & foot_pos,
        const dynacore::Vect3 & foot_vel,
        const dynacore::Vect3 & foot_acc,
        dynacore::Vector & config_sol,
        dynacore::Vector & qdot_cmd, 
        dynacore::Vector & qddot_cmd){

    int stance_bodyid = valkyrie_link::leftFoot;
    int swing_bodyid = valkyrie_link::rightFoot;
    int body_id = valkyrie_link::torso;

    if(swing_foot_ == valkyrie_link::leftFoot) {
        stance_bodyid = valkyrie_link::rightFoot;
        swing_bodyid = valkyrie_link::leftFoot;;
    }
    config_sol = current_Q;
    qdot_cmd = dynacore::Vector::Zero(valkyrie::num_qdot);
    qddot_cmd = dynacore::Vector::Zero(valkyrie::num_qdot);
    dynacore::Vector zero_qdot = dynacore::Vector::Zero(valkyrie::num_qdot);
    dynacore::Vect3 zero_vec; zero_vec.setZero();
    model_->UpdateSystem(config_sol, zero_qdot);

    // contact jacobian and null space
    dynacore::Matrix Jc, J_full;
    model_->getFullJacobian(stance_bodyid, Jc);
    
    dynacore::Matrix Jc_pinv;
    double threshold = 0.0000001;
    dynacore::pseudoInverse(Jc, threshold, Jc_pinv);
    
    dynacore::Matrix Nc = 
        dynacore::Matrix::Identity(valkyrie::num_qdot, valkyrie::num_qdot) -
        Jc_pinv * Jc;

    //// Operational Space error (height, Rx, Ry, foot_x, foot_y, foot_z) *****
    dynacore::Vector body_err(5);
    dynacore::Matrix Jop(5, valkyrie::num_qdot); Jop.setZero();

    // Height
    body_err[0] = des_height - current_Q[valkyrie_joint::virtual_Z];
    
    // Orientation
    dynacore::Quaternion curr_quat;
    model_->getOri(body_id, curr_quat);

    dynacore::Quaternion err_quat = dynacore::QuatMultiply(des_quat, curr_quat.inverse());
    dynacore::Vect3 ori_err;
    dynacore::convert(err_quat, ori_err);
    body_err[1] = ori_err[0];
    body_err[2] = ori_err[1];

    model_->getOri(valkyrie_link::pelvis, curr_quat);
    err_quat = dynacore::QuatMultiply(des_quat, curr_quat.inverse());
    dynacore::convert(err_quat, ori_err);
    body_err[3] = ori_err[0];
    body_err[4] = ori_err[1];
  
    // Jacobian 
    dynacore::Matrix J_body, J_pelvis;
    model_->getFullJacobian(body_id, J_body);
    model_->getFullJacobian(valkyrie_link::pelvis, J_pelvis);
    
    Jop(0, 2) = 1.; // Height
    Jop.block(1, 0, 2, valkyrie::num_qdot) = J_body.block(0,0, 2, valkyrie::num_qdot);
    Jop.block(3, 3, 2, 2) = dynacore::Matrix::Identity(2,2);
    
    dynacore::Matrix JNc = Jop * Nc;
    dynacore::Matrix JNc_pinv;
    dynacore::pseudoInverse(JNc, threshold, JNc_pinv);
    dynacore::Vector qdelta = JNc_pinv * body_err;
    
    //dynacore::pretty_print(J_pelvis, std::cout, "Jacob pelvis");
    //dynacore::pretty_print(curr_quat, std::cout, "pelvis quat");
    //dynacore::pretty_print(current_Q, std::cout, "curr q");
 
   //Jop.block(3, 0, 3, valkyrie::num_qdot) = Jfoot;

    // Foot
    dynacore::Vect3 swingfoot_pos;
    dynacore::Vect3 foot_err, xdot, xddot;

    model_->getPos(swing_bodyid, swingfoot_pos);
    for(int i(0); i<3; ++i) {
        foot_err[i] = foot_pos[i] - swingfoot_pos[i];
        xdot[i] = foot_vel[i];
        xddot[i] = foot_acc[i];
    }
 
    dynacore::Matrix Jfoot;
    model_->getFullJacobian(swing_bodyid, J_full);
    Jfoot = J_full.block(3, 0, 3, valkyrie::num_qdot);
    Jfoot.block(0,0, 3,6) = dynacore::Matrix::Zero(3,6);

    dynacore::Matrix Jfoot_pinv;
    dynacore::pseudoInverse(Jfoot, threshold, Jfoot_pinv);

    qdelta += Jfoot_pinv * foot_err;
    qdot_cmd += Jfoot_pinv * xdot;
    qddot_cmd += Jfoot_pinv * xddot;

    config_sol.segment(valkyrie::num_virtual, valkyrie::num_act_joint) += 
        qdelta.segment(valkyrie::num_virtual, valkyrie::num_act_joint);

    //dynacore::pretty_print(qdelta, std::cout, "delta");
    //dynacore::pretty_print(current_Q, std::cout, "Current Q");
    //dynacore::pretty_print(Jop, std::cout, "Jop");
    //dynacore::pretty_print(Jfoot, std::cout, "Jfoot");
    if(isnan(current_Q[1])){ exit(0); }
}

