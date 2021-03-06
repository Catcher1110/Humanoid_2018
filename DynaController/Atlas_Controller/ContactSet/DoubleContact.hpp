#ifndef ATLAS_DOUBLE_CONTACT
#define ATLAS_DOUBLE_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>

class RobotSystem;
class Atlas_StateProvider;

class DoubleContact: public WBDC_ContactSpec{
public:
  DoubleContact(RobotSystem*);
  virtual ~DoubleContact();

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  void _setU(double x, double y, double mu, dynacore::Matrix & U);

  const RobotSystem* robot_sys_;
  Atlas_StateProvider* sp_;
};


#endif
