#include "Atlas.h"

Atlas::Atlas():SystemGenerator()
{
  printf("[Atlas] ASSEMBLED\n");
}

Atlas::~Atlas(){
}

void Atlas::_SetJointLimit(){
  // r_joint_[r_joint_idx_map_.find("leftShoulderRoll" )->second]->SetPositionLimit(-90, 0);
  // r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->SetPositionLimit(0, 90);

  // r_joint_[r_joint_idx_map_.find("leftElbowPitch" )->second]->SetPositionLimit(-60., 0.);
  // r_joint_[r_joint_idx_map_.find("rightElbowPitch" )->second]->SetPositionLimit(-0., 60.);

  // r_joint_[r_joint_idx_map_.find("leftShoulderPitch" )->second]->SetPositionLimit(-45, 75);
  // r_joint_[r_joint_idx_map_.find("rightShoulderPitch" )->second]->SetPositionLimit(-75, 45);

  // r_joint_[r_joint_idx_map_.find("torsoYaw" )->second]->SetPositionLimit(-140., 140.);


  // for(int i(0);i <3; ++i){
  //   vp_joint_[i]->MakePositionLimit(false);
  //   vr_joint_[i]->MakePositionLimit(false);
  // }
}

void Atlas::_SetCollision(){
  collision_.resize(2);
  for (int i = 0; i < collision_.size(); ++i) {
    collision_[i] = new srCollision();
  }

  collision_[0]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  collision_[0]->GetGeomInfo().SetDimension(0.27, 0.16, 0.09);
  collision_[1]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  collision_[1]->GetGeomInfo().SetDimension(0.27, 0.16, 0.09);

  link_[link_idx_map_.find("r_foot")->second]->AddCollision(collision_[0]);
  link_[link_idx_map_.find("l_foot")->second]->AddCollision(collision_[1]);

  double fric(0.8);
  link_[link_idx_map_.find("r_foot")->second]->SetFriction(fric);
  link_[link_idx_map_.find("l_foot")->second]->SetFriction(fric);

  double damp(0.01);
  link_[link_idx_map_.find("r_foot")->second]->SetDamping(damp);
  link_[link_idx_map_.find("l_foot")->second]->SetDamping(damp);

  double restit(0.0);
  link_[link_idx_map_.find("r_foot")->second]->SetRestitution(restit);
  link_[link_idx_map_.find("l_foot")->second]->SetRestitution(restit);
}

void Atlas::_SetInitialConf(){
  //case 0 : just stand
  //case 1 : left leg up
  //case 2 : lower stand
  //case 3: walking ready
  //case 4 : ICRA 2018

  int pose(2);

  vp_joint_[0]->m_State.m_rValue[0] = 0.0;
  vp_joint_[1]->m_State.m_rValue[0] = 0.0;
  vp_joint_[2]->m_State.m_rValue[0] = 1.135;// + 0.3;
  vr_joint_[0]->m_State.m_rValue[0] = 0.0;
  vr_joint_[1]->m_State.m_rValue[0] = 0.0;
  vr_joint_[2]->m_State.m_rValue[0] = 0.0;

  //r_joint_[r_joint_idx_map_.find("leftHipPitch"  )->second]->m_State.m_rValue[0] = -0.3;
  //r_joint_[r_joint_idx_map_.find("leftHipPitch"  )->second]->m_State.m_rValue[0] = -0.3;
  //r_joint_[r_joint_idx_map_.find("rightHipPitch" )->second]->m_State.m_rValue[0] = -0.3;
  //r_joint_[r_joint_idx_map_.find("leftKneePitch" )->second]->m_State.m_rValue[0] = 0.6;
  //r_joint_[r_joint_idx_map_.find("rightKneePitch")->second]->m_State.m_rValue[0] = 0.6;
  //r_joint_[r_joint_idx_map_.find("leftAnklePitch")->second]->m_State.m_rValue[0] = -0.3;
  //r_joint_[r_joint_idx_map_.find("rightAnklePitch")->second]->m_State.m_rValue[0] = -0.3;

  //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;
  //r_joint_[r_joint_idx_map_.find("leftShoulderPitch" )->second]->m_State.m_rValue[0] = -0.2;
  //r_joint_[r_joint_idx_map_.find("leftShoulderRoll"  )->second]->m_State.m_rValue[0] = -1.1;
  //r_joint_[r_joint_idx_map_.find("leftElbowPitch"    )->second]->m_State.m_rValue[0] = -0.4;
  //r_joint_[r_joint_idx_map_.find("leftForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;

  switch(pose){
  case 0:

    break;

  case 1:
    vp_joint_[2]->m_State.m_rValue[0] = 1.131;
    break;

  case 2:
    vp_joint_[0]->m_State.m_rValue[0] =  -0.0183;
    vp_joint_[2]->m_State.m_rValue[0] =  1.079;
    break;

  case 3:
    vp_joint_[0]->m_State.m_rValue[0] =  -0.032720;
    vp_joint_[2]->m_State.m_rValue[0] =  1.050418;
    break;

  case 4:
    vp_joint_[0]->m_State.m_rValue[0] =  -0.0183;
    vp_joint_[2]->m_State.m_rValue[0] =  1.079;

    break;
  }
  KIN_UpdateFrame_All_The_Entity();
}
