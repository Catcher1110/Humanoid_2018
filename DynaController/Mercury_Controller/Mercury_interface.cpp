#include "Mercury_interface.hpp"
#include <stdio.h>
#include <math.h>
#include <string>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/wrap_eigen.hpp>
#include "Mercury_StateProvider.hpp"
#include "Mercury_StateEstimator.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <Mercury/Mercury_Model.hpp>

// Test SET LIST
// Basic Test
#include <Mercury_Controller/TestSet/JointCtrlTest.hpp>

// Walking Test
#include <Mercury_Controller/TestSet/WalkingConfigTest.hpp>

// Body Ctrl Test
#include <Mercury_Controller/TestSet/BodyConfigTest.hpp>

#define MEASURE_TIME 1
#if MEASURE_TIME
#include <chrono>
#endif

Mercury_interface::Mercury_interface():
    interface(),
    filtered_torque_command_(mercury::num_act_joint),
    torque_command_(mercury::num_act_joint),
    jpos_command_(mercury::num_act_joint),
    jvel_command_(mercury::num_act_joint),
    sensed_torque_(mercury::num_act_joint),
    torque_limit_max_(mercury::num_act_joint),
    torque_limit_min_(mercury::num_act_joint),    
    jpos_limit_max_(mercury::num_act_joint),
    jpos_limit_min_(mercury::num_act_joint),
    motor_current_(mercury::num_act_joint),
    bus_current_(mercury::num_act_joint),
    bus_voltage_(mercury::num_act_joint),
    jjvel_(mercury::num_act_joint),
    jjpos_(mercury::num_act_joint),
    b_last_config_update_(true),
    waiting_count_(1000),
    ramp_time_(0.5)
{

    double cut_freq = 2* M_PI * 100.;
    filter_jtorque_cmd_.resize(mercury::num_act_joint);
    for(int i(0); i<mercury::num_act_joint; ++i){
        filter_jtorque_cmd_[i] = new digital_lp_filter(cut_freq, mercury::servo_rate);
    }

    filtered_torque_command_.setZero();
    robot_sys_ = new Mercury_Model();
    sensed_torque_.setZero();
    torque_command_.setZero();
    jpos_command_.setZero();
    jvel_command_.setZero();
    motor_current_.setZero();
    bus_current_.setZero();
    bus_voltage_.setZero();
    jjvel_.setZero();
    jjpos_.setZero();

    test_cmd_ = new Mercury_Command();

    sp_ = Mercury_StateProvider::getStateProvider();
    state_estimator_ = new Mercury_StateEstimator(robot_sys_);  
    DataManager::GetDataManager()->RegisterData(
            &jpos_command_, DYN_VEC, "jpos_des", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jvel_command_, DYN_VEC, "jvel_des", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &filtered_torque_command_, DYN_VEC, "filtered_cmd", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &running_time_, DOUBLE, "running_time");
    DataManager::GetDataManager()->RegisterData(
            &sensed_torque_, DYN_VEC, "torque", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &torque_command_, DYN_VEC, "command", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &motor_current_, DYN_VEC, "motor_current", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jjpos_, DYN_VEC, "jjpos", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &bus_current_, DYN_VEC, "bus_current", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &bus_voltage_, DYN_VEC, "bus_voltage", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jjvel_, DYN_VEC, "jjvel", mercury::num_act_joint);
    _ParameterSetting();
    //printf("[Mercury_interface] Contruct\n");
}

Mercury_interface::~Mercury_interface(){
    delete test_;
}

void Mercury_interface::GetCommand( void* _data, void* _command){
    Mercury_Command* cmd = ((Mercury_Command*)_command);
    Mercury_SensorData* data = ((Mercury_SensorData*)_data);

#if MEASURE_TIME
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif
 
    if(!_Initialization(data)){
       state_estimator_->Update(data);
        test_->getCommand(test_cmd_);
    }
    // Ramp up the command
    if(sp_->curr_time_ < ramp_time_){
        double initialization_time = waiting_count_ * mercury::servo_rate;
        for(int i(0); i<mercury::num_act_joint; ++i){
            test_cmd_->jtorque_cmd[i] = test_cmd_->jtorque_cmd[i] *
             (sp_->curr_time_ - initialization_time)/(ramp_time_-initialization_time);
        }
    }
#if MEASURE_TIME
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
        if(count_%1000 == 1){
            std::cout << "[Mercury_interface] All process took me " << time_span1.count()*1000.0 << "ms."<<std::endl;
        }
#endif
 
    /// Begin of Torque Limit && NAN command (decide torque & jpos command)
    static bool isTurnoff_forever(false);

    if(isTurnoff_forever){
        torque_command_.setZero();
        jvel_command_.setZero();
        jpos_command_ = last_config_;
        b_last_config_update_ = false;
    } else{
        for(int i(0); i<mercury::num_act_joint; ++i){
            // NAN Test
            if(std::isnan(test_cmd_->jtorque_cmd[i])){ 
                isTurnoff_forever = true;
                printf("[Interface] There is nan value in command\n");
                for(int i(0); i<mercury::num_act_joint; ++i){
                    torque_command_[i] = 0.;
                    jpos_command_[i] = last_config_[i];
                    b_last_config_update_ = false;
                }
            }
            // Torque Limit Truncation
            if( (test_cmd_->jtorque_cmd[i]) > torque_limit_max_[i] ){
                torque_command_[i] = torque_limit_max_[i];
            }else if((test_cmd_->jtorque_cmd[i]) < torque_limit_min_[i]) {
                torque_command_[i] = torque_limit_min_[i];
            } else{
                torque_command_[i] = test_cmd_->jtorque_cmd[i];
            }
            // JPos Limit Truncation
            if( (test_cmd_->jpos_cmd[i]) > jpos_limit_max_[i] ){
                jpos_command_[i] = jpos_limit_max_[i];
                jvel_command_[i] = 0.;
            }else if((test_cmd_->jpos_cmd[i]) < jpos_limit_min_[i]) {
                jpos_command_[i] = jpos_limit_min_[i];
                jvel_command_[i] = 0.;
            } else{
                jpos_command_[i] = test_cmd_->jpos_cmd[i];
                jvel_command_[i] = test_cmd_->jvel_cmd[i];
            }
            filter_jtorque_cmd_[i]->input(torque_command_[i]);
            filtered_torque_command_[i] = filter_jtorque_cmd_[i]->output();
       }
    }

    // Update Command (and Data)
    for(int i(0); i<mercury::num_act_joint; ++i){
        cmd->jtorque_cmd[i] = torque_command_[i];
        cmd->jpos_cmd[i] = jpos_command_[i];
        cmd->jvel_cmd[i] = jvel_command_[i];

        sensed_torque_[i] = data->jtorque[i];
        motor_current_[i] = data->motor_current[i];
        bus_current_[i] = data->bus_current[i];
        bus_voltage_[i] = data->bus_voltage[i];
        jjvel_[i] = data->joint_jvel[i];
        jjpos_[i] = data->joint_jpos[i];

        if(b_last_config_update_) last_config_ = jpos_command_;
    }
    running_time_ = (double)(count_) * mercury::servo_rate;
    ++count_;
    // When there is sensed time
    sp_->curr_time_ = running_time_;

    sp_->phase_copy_ = test_->getPhase();

    // Stepping forward
    double walking_start(10.);
    double walking_duration(7.);
    double walking_distance(2.5);
    if(sp_->curr_time_ > walking_start){
        double walking_time = sp_->curr_time_ - walking_start;
        sp_->des_location_[0] = walking_distance * 
            (1-cos(walking_time/walking_duration * M_PI))/2.;
    }
    if(sp_->curr_time_ > walking_start + walking_duration){
        sp_->des_location_[0] = walking_distance;
    }
}
void Mercury_interface::GetReactionForce(std::vector<dynacore::Vect3> & reaction_force ){
    reaction_force.resize(2);
    for(int i(0); i<2; ++i){
        for(int j(0); j<3; ++j){
            reaction_force[i][j] = sp_->reaction_forces_[j];
        }
    }
}

bool Mercury_interface::_Initialization(Mercury_SensorData* data){
    static bool test_initialized(false);
    if(!test_initialized) {
        test_->TestInitialization();
        test_initialized = true;
        printf("[Mercury Interface] Test initialization is done\n");
    }
    //printf("count: %d/ %d\n", count_, waiting_count_);
    if(count_ < waiting_count_){
        for(int i(0); i<mercury::num_act_joint; ++i){
            test_cmd_->jtorque_cmd[i] = 0.;
            test_cmd_->jpos_cmd[i] = data->joint_jpos[i];
            test_cmd_->jvel_cmd[i] = 0.;
        }
        state_estimator_->Initialization(data);


        if(fabs(data->imu_acc[2]) < 0.00001){
            waiting_count_ = 10000000;
            if(count_%1000 ==1)
                dynacore::pretty_print(data->imu_acc, "data->imu_acc", 3);            
        }

        DataManager::GetDataManager()->start();
        //printf("[Mercury Interface] Data logging starts\n");
        return true;
    }
    return false;
}

void Mercury_interface::_ParameterSetting(){
    ParamHandler handler(MercuryConfigPath"INTERFACE_setup.yaml");

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
    }else {
        printf("[Interfacce] There is no test matching with the name\n");
        exit(0);
    }

     // Torque limit
    handler.getVector("torque_max", torque_limit_max_);
    handler.getVector("torque_min", torque_limit_min_);
    // JPos limit
    handler.getVector("joint_max", jpos_limit_max_);
    handler.getVector("joint_min", jpos_limit_min_);

    // Joint pos based model update
    handler.getBoolean("jpos_model_update", b_tmp);
    state_estimator_->setJPosModelUpdate(b_tmp);

    printf("[Mercury_interface] Parameter setup is done\n");
}
