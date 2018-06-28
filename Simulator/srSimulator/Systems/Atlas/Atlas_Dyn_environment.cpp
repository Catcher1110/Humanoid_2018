#include "Atlas_Dyn_environment.h"
#include <iostream>
#include "srDyn/srSpace.h"
#include <stdio.h>

#include "common/utils.h"

//#include <ControlSystem/Atlas/Atlas_Controller/interface.hpp>
//#include <ControlSystem/Atlas/Atlas_Controller/StateProvider.hpp>
#include <srTerrain/Ground.h>
#include <RobotSystems/Atlas/Atlas_Definition.h>

#ifdef Measure_Time
#include <chrono>
using namespace std::chrono;
#endif

#include <srConfiguration.h>

// #define SENSOR_NOISE
#define SENSOR_DELAY 0 // Sensor_delay* atlas::servo_rate (sec) = time delay
#define ENVIRONMENT_SETUP 0

Atlas_Dyn_environment::Atlas_Dyn_environment():
    ang_vel_(3)
{
    /********** Space Setup **********/
    m_Space = new srSpace();
    m_ground = new Ground();

    m_Space->AddSystem(m_ground->BuildGround());
    /********** Robot Set  **********/
    new_robot_ = new Atlas();
    new_robot_->BuildRobot(Vec3 (0., 0., 0.), 
            srSystem::FIXED, srJoint::TORQUE, ModelPath"Atlas/atlas_v3_no_head.urdf");
    m_Space->AddSystem((srSystem*)new_robot_);

    /******** Interface set ********/
    //interface_ = new interface();

    m_Space->DYN_MODE_PRESTEP();
    m_Space->SET_USER_CONTROL_FUNCTION_2(ControlFunction);
    m_Space->SetTimestep(atlas::servo_rate);
    m_Space->SetGravity(0.0,0.0,-9.81);

    m_Space->SetNumberofSubstepForRendering(30);

    printf("[Atlas Dynamic Environment] Build Dynamic Environment\n");
}

void Atlas_Dyn_environment::ControlFunction( void* _data ) {
    static int count(0);
    ++count;

    Atlas_Dyn_environment* pDyn_env = (Atlas_Dyn_environment*)_data;
    Atlas* robot = (Atlas*)(pDyn_env->new_robot_);

    std::vector<double> jpos(robot->num_act_joint_);
    std::vector<double> jvel(robot->num_act_joint_);
    std::vector<double> jtorque(robot->num_act_joint_);
    dynacore::Vect3 pos;
    dynacore::Quaternion rot;
    dynacore::Vect3 body_vel;
    dynacore::Vect3 ang_vel;
    std::vector<double> torque_command(robot->num_act_joint_);

    for(int i(0); i<robot->num_act_joint_; ++i){
        jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
        jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
        jtorque[i] = robot->r_joint_[i]->m_State.m_rValue[3];
    }

    //pDyn_env->_CheckFootContact();

    for (int i(0); i<3; ++i){
        pos[i] = robot->vp_joint_[i]->m_State.m_rValue[0];
        body_vel[i] = robot->vp_joint_[i]->m_State.m_rValue[1];
        ang_vel[i] = robot->link_[robot->link_idx_map_.find("pelvis")->second]->GetVel()[i];
    }
    pDyn_env->_Get_Orientation(rot);
    //pDyn_env->interface_->GetCommand(alternate_time, jpos, jvel, jtorque, pos, rot, body_vel, ang_vel, torque_command);

    for(int i(0); i<3; ++i){
        robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
        robot->vr_joint_[i]->m_State.m_rCommand = 0.0;
    }

    for(int i(0); i<robot->num_act_joint_; ++i){
        robot->r_joint_[i]->m_State.m_rCommand = torque_command[i];
    }
    // pDyn_env->_ExternalDisturbance(count);
    //pDyn_env->_ListReactionForce();
    //pDyn_env->_ListCommandedReactionForce(StateProvider::GetStateProvider()->Fr_);
    //pDyn_env->_SaveStanceFoot();
}


  void Atlas_Dyn_environment::Rendering_Fnc(){
  }
    void Atlas_Dyn_environment::_Get_Orientation(dynacore::Quaternion & rot){
        SO3 so3_body =  new_robot_->link_[new_robot_->link_idx_map_.find("pelvis")->second]->GetOrientation();

        Eigen::Matrix3d ori_mtx;
        for (int i(0); i<3; ++i){
            ori_mtx(i, 0) = so3_body[0+i];
            ori_mtx(i, 1) = so3_body[3+i];
            ori_mtx(i, 2) = so3_body[6+i];
        }
        dynacore::Quaternion ori_quat(ori_mtx);
        rot = ori_quat;
    }
    Atlas_Dyn_environment::~Atlas_Dyn_environment()
    {
        //SR_SAFE_DELETE(interface_);
        SR_SAFE_DELETE(new_robot_);
        SR_SAFE_DELETE(m_Space);
        SR_SAFE_DELETE(m_ground);
    }


