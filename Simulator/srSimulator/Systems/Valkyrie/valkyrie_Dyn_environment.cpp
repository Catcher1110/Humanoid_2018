#include "valkyrie_Dyn_environment.h"
#include <iostream>
#include "srDyn/srSpace.h"
#include <stdio.h>

#include "common/utils.h"

#include <DynaController/Valkyrie_Controller/Valkyrie_interface.hpp>
#include <DynaController/Valkyrie_Controller/Valkyrie_StateProvider.hpp>
#include <srTerrain/Ground.h>

#ifdef Measure_Time
#include <chrono>
using namespace std::chrono;
#endif

#include <srConfiguration.h>

// #define SENSOR_NOISE
#define SENSOR_DELAY 0 // Sensor_delay* SERVO_RATE (sec) = time delay
#define ENVIRONMENT_SETUP 0

Valkyrie_Dyn_environment::Valkyrie_Dyn_environment():
    indicated_contact_pt_list_(8),
    commanded_contact_force_list_(8),
    ang_vel_(3)
{
    double box_size = 0.05;	
    /********** Space Setup **********/
    m_Space = new srSpace();
    m_ground = new Ground();

    m_Space->AddSystem(m_ground->BuildGround());

    /********** Robot Set  **********/
    new_robot_ = new New_Valkyrie();
     //new_robot_->BuildRobot(Vec3 (0., 0., 0.), srSystem::FIXED, 
            //srJoint::TORQUE, ModelPath"Valkyrie_Model/r5_urdf.urdf");
    new_robot_->BuildRobot(Vec3 (0., 0., 0.), srSystem::FIXED, 
            srJoint::TORQUE, ModelPath"Valkyrie/valkyrie_no_collision.urdf");
    m_Space->AddSystem((srSystem*)new_robot_);

    /******** Interface set ********/
    //interface_ = new Valkyrie_interface();
    contact_pt_list_.clear();
    contact_force_list_.clear();

    m_Space->DYN_MODE_PRESTEP();
    m_Space->SET_USER_CONTROL_FUNCTION_2(ControlFunction);
    m_Space->SetTimestep(valkyrie::servo_rate);
    m_Space->SetGravity(0.0,0.0,-9.81);

    m_Space->SetNumberofSubstepForRendering(1);

    printf("[Valkyrie Dynamic Environment] Build Dynamic Environment\n");
}

void Valkyrie_Dyn_environment::ControlFunction( void* _data ) {
  static int count(0);
  ++count;

  Valkyrie_Dyn_environment* pDyn_env = (Valkyrie_Dyn_environment*)_data;
  New_Valkyrie* robot = (New_Valkyrie*)(pDyn_env->new_robot_);

  double alternate_time = valkyrie::servo_rate * count;
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

  pDyn_env->_CheckFootContact();

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
  pDyn_env->_ListReactionForce();
  pDyn_env->_ListCommandedReactionForce(Valkyrie_StateProvider::getStateProvider()->Fr_);
  pDyn_env->_SaveStanceFoot();
}

void Valkyrie_Dyn_environment::_ExternalDisturbance(int count){
  int interrupt_count(500);
  int interrupt_duration(2500); // 0.2 sec

  double impulse_force(80.);
  double theta (0. * M_PI/180.);

  double force_x = impulse_force * cos(theta);
  double force_y = impulse_force * sin(theta);
  if(interrupt_count< count && count < (interrupt_count + interrupt_duration)){
    dse3 ext_force(0., 0., 0., force_x, force_y, 0.);
    // new_robot_->link_[new_robot_->link_idx_map_.find("pelvis")->second]->AddUserExternalForce(ext_force);
    // new_robot_->link_[new_robot_->link_idx_map_.find("torso")->second]->AddUserExternalForce(ext_force);
    new_robot_->link_[new_robot_->link_idx_map_.find("leftFoot")->second]->AddUserExternalForce(ext_force);
    new_robot_->link_[new_robot_->link_idx_map_.find("rightFoot")->second]->AddUserExternalForce(ext_force);

  }
}

void Valkyrie_Dyn_environment::_SaveStanceFoot(){
    Vec3 foot_pos;
    if(Valkyrie_StateProvider::getStateProvider()->stance_foot_ == valkyrie_link::rightFoot ||
            Valkyrie_StateProvider::getStateProvider()->stance_foot_ == valkyrie_link::rightCOP_Frame){
        foot_pos = new_robot_->link_[new_robot_->link_idx_map_.find("rightCOP_Frame")->second]->GetPosition();
    } else{
        foot_pos = new_robot_->link_[new_robot_->link_idx_map_.find("leftCOP_Frame")->second]->GetPosition();
    }
    Valkyrie_StateProvider::getStateProvider()->global_stance_foot_pos_<<foot_pos[0], foot_pos[1], foot_pos[2];
}


void Valkyrie_Dyn_environment::_ListCommandedReactionForce(const dynacore::Vector & Fr){
    dynacore::Vect3 vec3_fr;
    dynacore::Vect3 vec3_cp;
    Vec3 contact_point;
    for (int i(0); i<8; ++i){
        if(i < 4){ //Left
            contact_point = new_robot_->link_[new_robot_->link_idx_map_.find("leftFootOutFront")->second+ i]->GetPosition();
        } else { // Right
            contact_point = new_robot_->link_[new_robot_->link_idx_map_.find("rightFootOutFront")->second+ i-4]->GetPosition();
        }

        for (int j(0); j<3; ++j){
            vec3_cp[j] = contact_point[j];
            vec3_fr[j] = Fr[3*i + j];
        }
        indicated_contact_pt_list_[i] = vec3_cp;
        commanded_contact_force_list_[i] = vec3_fr;
    }
}

void Valkyrie_Dyn_environment::_ListReactionForce(){
    contact_pt_list_.clear();
    contact_force_list_.clear();

    int iCount = (m_Space->m_srDYN).m_Baked_ContactConstraints.get_size();
    for (int i(0) ; i <  iCount ; i++){
        Contact* contact_pt = (m_Space->m_srDYN).m_Baked_ContactConstraints[i].pContactPts[0];
        ContactConstraint* cc_array = &((m_Space->m_srDYN).m_Baked_ContactConstraints[i]);
        // printf("num contact point: %i\n", cc_array->nContactPts);
        // if (contact_pt->bActive && (Norm(contact_pt->globalpoint) > 0) ){
        if (cc_array->nContactPts > 0 ){
            // Left: Ground, Right: Link
            dynacore::Vect3 contact_point_loc;
            dynacore::Vect3 contact_force;

            dse3 impulse(0.0);
            for (int k(0); k < 3; ++k){
                contact_point_loc[k] = contact_pt->globalpoint[k];
                impulse += cc_array->JacobianRight[k] * ((sr_real)contact_pt->lambda[k]);
                // std::cout<<"k right jacobian:"<<cc_array->JacobianRight[k];
            }
            // printf("%i the contact point lambda: %f, %f, %f \n", i,
            //        ((sr_real)contact_pt->lambda[0]),
            //        ((sr_real)contact_pt->lambda[1]),
            //        ((sr_real)contact_pt->lambda[2])
            //        );

            // impulse += cc_array->JacobianRight[0] * ((sr_real)contact_pt->lambda[0]);
            contact_force[0] = impulse[3];
            contact_force[1] = impulse[4];
            contact_force[2] = impulse[5];
            contact_pt_list_.push_back(contact_point_loc);
            contact_force_list_.push_back(contact_force);
        }
    }
    }

    void Valkyrie_Dyn_environment::Rendering_Fnc()
    {
        //_Draw_Contact_Point();
        // _Draw_Contact_Force();
        _Draw_Commanded_Force();
        //_Draw_FootPlacement();
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

    void Valkyrie_Dyn_environment::_Draw_Contact_Point(){
        double radi(0.02);

        double theta(0.0);
        dynacore::Vect3 contact_loc;
        for (int j(0); j<contact_pt_list_.size(); ++j){
            contact_loc = contact_pt_list_[j];
            glBegin(GL_LINE_LOOP);
            for (int i(0); i<3600; ++i){
                theta += DEG2RAD(i*0.1);
                glColor4f(0.5f, 0.1f, 0.5f, 0.1f);

                glVertex3f(contact_loc[0] + cos(theta)*radi, contact_loc[1] + sin(theta)*radi, contact_loc[2]);
            }
            glEnd();
        }
    }

    void Valkyrie_Dyn_environment::_Draw_Contact_Force(){
        dynacore::Vect3 contact_loc;
        dynacore::Vect3 contact_force;
        double reduce_ratio(1.0);

        // GLfloat LineRange[2];
        // glGetFloatv(GL_LINE_WIDTH_RANGE,LineRange);
        // std::cout << "Minimum Line Width " << LineRange[0] << " -- ";
        // std::cout << "Maximum Line Width " << LineRange[1] << std::endl;


        for (int j(0); j<contact_pt_list_.size(); ++j){
            glLineWidth(2.5);
            glColor4f(0.01f, 0.5f, 0.05f, 0.5f);
            glBegin(GL_LINES);

            contact_loc = contact_pt_list_[j];
            contact_force = reduce_ratio * contact_force_list_[j];
            contact_force += contact_loc;

            glVertex3f(contact_loc[0],  contact_loc[1],  contact_loc[2]);
            glVertex3f(contact_force[0], contact_force[1], contact_force[2]);
            glEnd();
        }

    }

    void Valkyrie_Dyn_environment::_Draw_Commanded_Force(){
        dynacore::Vect3 contact_loc;
        dynacore::Vect3 contact_force;
        double reduce_ratio(0.001);

        for (int j(0); j<indicated_contact_pt_list_.size(); ++j){
            glLineWidth(2.5);
            glColor4f(0.9f, 0.0f, 0.0f, 0.8f);
            glBegin(GL_LINES);

            contact_loc = indicated_contact_pt_list_[j];
            contact_force = reduce_ratio * commanded_contact_force_list_[j];
            contact_force += contact_loc;

            glVertex3f(contact_loc[0],  contact_loc[1],  contact_loc[2]);
            glVertex3f(contact_force[0], contact_force[1], contact_force[2]);
            glEnd();
        }

    }


    void Valkyrie_Dyn_environment::SetCurrentState_All(){

        //     Vec3 ang_noise;
        //     Vec3 ang_vel_noise;

        // #ifdef SENSOR_NOISE
        //     for (int i(0); i< 3; ++i){
        //         ang_noise[i] = 0.005*generator_white_noise(0.0, 1.0);
        //         ang_vel_noise[i] = 0.005*generator_white_noise(0.0, 1.0);
        //     }
        // #endif

        //     for (int i(0); i< SIM_NUM_RJOINT; ++i){
        //         curr_conf_[i] = robot_->Full_Joint_State_[i]->m_rValue[0];
        //         curr_jvel_[i] = robot_->Full_Joint_State_[i]->m_rValue[1];
        //         torque_ [i]   = robot_->Full_Joint_State_[i]->m_rValue[3];
        //     }
        //     _Save_Orientation_Matrix();
        //     // R x, y, z 
        //     for (int i(0); i<3; ++i){ 
        //         ang_vel_[i] = robot_->m_Link[0].GetVel()[i] + ang_vel_noise[i]; 
        //     } 

        //     state curr_state; 
        //     curr_state.conf               =   curr_conf_; 
        //     curr_state.jvel               =   curr_jvel_; 
        //     curr_state.torque             =   torque_   ;
        //     _Copy_Array(curr_state.ori_mtx, ori_mtx_, 9);
        //     curr_state.ang_vel            =   ang_vel_; 

        //     history_state_.push_back(curr_state); 
    } 


    void Valkyrie_Dyn_environment::_Save_Orientation_Matrix(){
        SO3 so3_body =  new_robot_->link_[0]->GetOrientation();

        for (int i(0); i<3; ++i){
            ori_mtx_[0 + 3*i] = so3_body[0+i];
            ori_mtx_[1 + 3*i] = so3_body[3+i];
            ori_mtx_[2 + 3*i] = so3_body[6+i];
        }
    }

    void Valkyrie_Dyn_environment::_Get_Orientation(dynacore::Quaternion & rot){
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
