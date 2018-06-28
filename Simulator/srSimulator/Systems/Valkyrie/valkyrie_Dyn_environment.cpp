#include "valkyrie_Dyn_environment.h"
#include <iostream>
#include <stdio.h>
#include "srDyn/srSpace.h"
#include "common/utils.h"

#include <DynaController/Valkyrie_Controller/Valkyrie_interface.hpp>
#include <DynaController/Valkyrie_Controller/Valkyrie_StateProvider.hpp>
#include <DynaController/Valkyrie_Controller/Valkyrie_DynaControl_Definition.h>

#include <srTerrain/Ground.h>
#include <srConfiguration.h>

Valkyrie_Dyn_environment::Valkyrie_Dyn_environment()
{
    /********** Space Setup **********/
    m_Space = new srSpace();
    m_ground = new Ground();

    m_Space->AddSystem(m_ground->BuildGround());

    /********** Robot Set  **********/
    new_robot_ = new New_Valkyrie();
    //new_robot_->BuildRobot(Vec3 (0., 0., 0.), srSystem::FIXED, 
    //srJoint::TORQUE, ModelPath"Valkyrie_Model/r5_urdf.urdf");
    new_robot_->BuildRobot(Vec3 (0., 0., 0.), srSystem::FIXED, 
            srJoint::TORQUE, ModelPath"Valkyrie/valkyrie_simple.urdf");
    m_Space->AddSystem((srSystem*)new_robot_);

    /******** Interface set ********/
    interface_ = new Valkyrie_interface();
    data_ = new Valkyrie_SensorData();
    cmd_ = new Valkyrie_Command();

    m_Space->DYN_MODE_PRESTEP();
    m_Space->SET_USER_CONTROL_FUNCTION_2(ControlFunction);
    m_Space->SetTimestep(valkyrie::servo_rate);
    m_Space->SetGravity(0.0,0.0,-9.81);

    m_Space->SetNumberofSubstepForRendering(10);

    printf("[Valkyrie Dynamic Environment] Build Dynamic Environment\n");
}

void Valkyrie_Dyn_environment::ControlFunction( void* _data ) {
    static int count(0);
    ++count;

    Valkyrie_Dyn_environment* pDyn_env = (Valkyrie_Dyn_environment*)_data;
    New_Valkyrie* robot = (New_Valkyrie*)(pDyn_env->new_robot_);
    Valkyrie_SensorData* p_data = pDyn_env->data_;

    //printf("num act joint: %d \n", robot->num_act_joint_);
    std::vector<double> torque_command(robot->num_act_joint_);

    for(int i(0); i<robot->num_act_joint_; ++i){
        p_data->jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
        p_data->jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
    }

    pDyn_env->_CheckFootContact();

    for (int i(0); i<3; ++i){
        p_data->imu_ang_vel[i] = 
            robot->link_[robot->link_idx_map_.find("pelvis")->second]->GetVel()[i];
    }
    pDyn_env->interface_->GetCommand(p_data, pDyn_env->cmd_); 

    // Set Command
    for(int i(0); i<3; ++i){
        robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
        robot->vr_joint_[i]->m_State.m_rCommand = 0.0;
    }

    double Kp(500.);
    double Kd(5.);
     //double Kp(30.);
     //double Kd(0.5);
    // Right
    double ramp(1.);
    if( count < 10 ){
        ramp = ((double)count)/10.;
    }
    for(int i(0); i<robot->num_act_joint_; ++i){
        robot->r_joint_[i]->m_State.m_rCommand = pDyn_env->cmd_->jtorque_cmd[i] + 
            Kp * (pDyn_env->cmd_->jpos_cmd[i] - p_data->jpos[i]) + 
            Kd * (pDyn_env->cmd_->jvel_cmd[i] - p_data->jvel[i]);
        
        //robot->r_joint_[i]->m_State.m_rCommand *= ramp;
    }
}

void Valkyrie_Dyn_environment::Rendering_Fnc()
{
}


void Valkyrie_Dyn_environment::_DrawHollowCircle(GLfloat x, GLfloat y, GLfloat z, GLfloat radius){
    int i;
    int lineAmount = 200; //# of triangles used to draw circle

    //GLfloat radius = 0.8f; //radius
    GLfloat twicePi = 2.0f * M_PI;

    for (int i(0); i< lineAmount-1; ++i){
        glLineWidth(2.0);
        glBegin(GL_LINES);
        glVertex3f(x + radius * cos(i * twicePi / lineAmount),
                y + radius * sin(i * twicePi / lineAmount), z);
        glVertex3f(x + radius * cos((i+1) * twicePi / lineAmount),
                y + radius * sin((i+1) * twicePi / lineAmount), z);
        glEnd();
    }
}

void Valkyrie_Dyn_environment::_Copy_Array(double * subject, double * data, int num_element){
    // for(int i(0); i< num_element; ++i){
    //     subject[i] = data[i];
    // }
}

Valkyrie_Dyn_environment::~Valkyrie_Dyn_environment()
{
    SR_SAFE_DELETE(interface_);
    SR_SAFE_DELETE(new_robot_);
    SR_SAFE_DELETE(m_Space);
    SR_SAFE_DELETE(m_ground);
}

void Valkyrie_Dyn_environment::_CheckFootContact(){
    //Valkyrie_StateProvider::getStateProvider()->b_both_contact_ = false;

    // Vec3 lfoot_pos = new_robot_->link_[new_robot_->link_idx_map_.find("leftCOP_Frame")->second]->GetPosition();
    // Vec3 rfoot_pos = new_robot_->link_[new_robot_->link_idx_map_.find("rightCOP_Frame")->second]->GetPosition();

    // double lheight(0.);
    // double rheight(0.);
    // if(lfoot_pos[0] > loc_x_){
    //   lheight = (lfoot_pos[0] - loc_x_)*tan(slope_) + 0.003;
    // }
    // if(rfoot_pos[0] > loc_x_){
    //   rheight = (rfoot_pos[0] - loc_x_)*tan(slope_) + 0.005;
    // }

    // if(  fabs(lheight - lfoot_pos[2]) < 0.005 && fabs(rheight -rfoot_pos[2])<0.005  ){
    //   std::cout<<"left"<<std::endl;
    //   std::cout<<lfoot_pos;
    //   printf("left height: %f\n", lheight);


    //   std::cout<<"right"<<std::endl;
    //   std::cout<<rfoot_pos;
    //   printf("right height: %f\n", rheight);

    //   Valkyrie_StateProvider::getStateProvider()->b_both_contact_ = true;
    // } else {
    //   Valkyrie_StateProvider::getStateProvider()->b_both_contact_ = false;
    // }
}
