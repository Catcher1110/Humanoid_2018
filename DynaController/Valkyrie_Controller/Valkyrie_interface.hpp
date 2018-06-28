#ifndef VALKYRIE_INTERFACE_H
#define VALKYRIE_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include "Valkyrie_DynaControl_Definition.h"
#include <Filter/filters.hpp>

class Valkyrie_StateEstimator;
class Valkyrie_StateProvider;

class Valkyrie_interface: public interface{
public:
  Valkyrie_interface();
  virtual ~Valkyrie_interface();

public:
  virtual void GetCommand(void * data, void * command);
  dynacore::Quaternion global_ori_;

private:
  int waiting_count_;
  
  std::vector<double> torque_limit_min_;
  std::vector<double> torque_limit_max_;
  std::vector<double> jpos_limit_min_;
  std::vector<double> jpos_limit_max_;
  std::vector<filter*> filter_jtorque_cmd_;
  
  void _ParameterSetting();
  bool _Initialization(Valkyrie_SensorData* );

  Valkyrie_Command* test_cmd_;
  dynacore::Vector filtered_torque_command_;
  dynacore::Vector torque_command_;
  dynacore::Vector jpos_command_;
  dynacore::Vector jvel_command_;
  
  dynacore::Vector sensed_torque_;
  
  dynacore::Vector motor_current_;
  dynacore::Vector bus_current_;
  dynacore::Vector bus_voltage_;
  dynacore::Vector last_config_;
  bool b_last_config_update_;

  dynacore::Vector jjvel_;
  dynacore::Vector jjpos_;
  
  dynacore::Vector initial_jpos_;
  Valkyrie_StateEstimator* state_estimator_;
  Valkyrie_StateProvider* sp_;
};

#endif
