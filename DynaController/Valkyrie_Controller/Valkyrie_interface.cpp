#include "Valkyrie_interface.hpp"
#include <stdio.h>
#include <math.h>
#include <string>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/wrap_eigen.hpp>
#include "Valkyrie_StateProvider.hpp"
#include "Valkyrie_StateEstimator.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <Valkyrie/Valkyrie_Model.hpp>

// Test SET LIST
// Basic Test
#include <Valkyrie_Controller/TestSet/JointCtrlTest.hpp>

// Walking Test
#include <Valkyrie_Controller/TestSet/WalkingConfigTest.hpp>

// Body Ctrl Test
#include <Valkyrie_Controller/TestSet/BodyConfigTest.hpp>

Valkyrie_interface::Valkyrie_interface():
    interface(),
    torque_command_(valkyrie::num_act_joint),
    jpos_command_(valkyrie::num_act_joint),
    jvel_command_(valkyrie::num_act_joint),
    jjvel_(valkyrie::num_act_joint),
    jjpos_(valkyrie::num_act_joint),
    waiting_count_(2)
{

    robot_sys_ = new Valkyrie_Model();
    torque_command_.setZero();
    jpos_command_.setZero();
    jvel_command_.setZero();
    jjvel_.setZero();
    jjpos_.setZero();

    test_cmd_ = new Valkyrie_Command();
    sp_ = Valkyrie_StateProvider::getStateProvider();
    state_estimator_ = new Valkyrie_StateEstimator(robot_sys_);  
    
    DataManager::GetDataManager()->RegisterData(
            &jpos_command_, DYN_VEC, "jpos_des", valkyrie::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jvel_command_, DYN_VEC, "jvel_des", valkyrie::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &running_time_, DOUBLE, "running_time");
    DataManager::GetDataManager()->RegisterData(
            &torque_command_, DYN_VEC, "command", valkyrie::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jjpos_, DYN_VEC, "jjpos", valkyrie::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jjvel_, DYN_VEC, "jjvel", valkyrie::num_act_joint);
    
    _ParameterSetting();
    
    printf("[Valkyrie_interface] Contruct\n");
}

Valkyrie_interface::~Valkyrie_interface(){
    delete test_;
}

void Valkyrie_interface::GetCommand( void* _data, void* _command){
    Valkyrie_Command* cmd = ((Valkyrie_Command*)_command);
    Valkyrie_SensorData* data = ((Valkyrie_SensorData*)_data);

    if(!_Initialization(data)){
        state_estimator_->Update(data);
        test_->getCommand(test_cmd_);
    }
    
    // Update Command (and Data)
    for(int i(0); i<valkyrie::num_act_joint; ++i){
        cmd->jtorque_cmd[i] = test_cmd_->jtorque_cmd[i];
        cmd->jpos_cmd[i] = test_cmd_->jpos_cmd[i];
        cmd->jvel_cmd[i] = test_cmd_->jvel_cmd[i];

        jjvel_[i] = data->jvel[i];
        jjpos_[i] = data->jpos[i];
    }
    running_time_ = (double)(count_) * valkyrie::servo_rate;
    ++count_;
    // When there is sensed time
    sp_->curr_time_ = running_time_;
}

bool Valkyrie_interface::_Initialization(Valkyrie_SensorData* data){
    static bool test_initialized(false);
    if(!test_initialized) {
        test_->TestInitialization();
        test_initialized = true;
        printf("[Valkyrie Interface] Test initialization is done\n");
    }
    if(count_ < waiting_count_){
        for(int i(0); i<valkyrie::num_act_joint; ++i){
            test_cmd_->jtorque_cmd[i] = 0.;
            test_cmd_->jpos_cmd[i] = data->jpos[i];
            test_cmd_->jvel_cmd[i] = 0.;
        }
        state_estimator_->Initialization(data);

        DataManager::GetDataManager()->start();
        return true;
    }
    return false;
}

void Valkyrie_interface::_ParameterSetting(){
    ParamHandler handler(ValkyrieConfigPath"INTERFACE_setup.yaml");
    std::string tmp_string;
    bool b_tmp;
    // Test SETUP
    handler.getString("test_name", tmp_string);
    // Basic Test ***********************************
    if(tmp_string == "joint_ctrl_test"){
        test_ = new JointCtrlTest(robot_sys_);
        // Walking Test ***********************************
    }else if(tmp_string == "walking_config_test"){
        test_ = new WalkingConfigTest(robot_sys_);
        // Body Ctrl Test ***********************************
    }else if(tmp_string == "body_ctrl_test"){
        test_ = new BodyConfigTest(robot_sys_);    
        // Stance and Swing Test ***********************************
    }else {
        printf("[Interfacce] There is no test matching with the name\n");
        exit(0);
    }
    printf("[Valkyrie_interface] Parameter setup is done\n");
}
