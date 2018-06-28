#ifndef  DYN_ENVIRONMENT_Atlas
#define  DYN_ENVIRONMENT_Atlas

#include "LieGroup/LieGroup.h"
#include <vector>
#include <Utils/wrap_eigen.hpp>

//TEST JUNHYEOK
#include "Atlas.h"

//TEST
////////////////////////////////////////////////
#ifdef __APPLE__
#include <GLUT/glut.h>
#endif

//#ifdef linux
#ifdef __linux__
#include <GL/glut.h>
#endif
////////////////////////////////////////////////


class interface;
class srSpace;
class Ground;

struct state
{
  std::vector<double> conf;
  std::vector<double> jvel;
  std::vector<double> torque   ;
  // std::vector<double> euler_ang;
  double ori_mtx[9];
  std::vector<double> ang_vel;
};

class Atlas_Dyn_environment
{
public:
  Atlas_Dyn_environment();
  ~Atlas_Dyn_environment();

  static void ControlFunction(void* _data);
  void Rendering_Fnc();

  void SetCurrentState_All();
  void saveLandingLocation();
public:
  interface* interface_;
  Atlas* new_robot_;

  srSpace*	m_Space;
  Ground*	m_ground;

  double ori_mtx_[9];
  std::vector<double> ang_vel_  ;
protected:
  void _Get_Orientation(dynacore::Quaternion & rot);
  void _Copy_Array(double * , double *, int);
};

#endif
