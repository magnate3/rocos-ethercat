/**
 * Copyright (c) 2021 Yang Luo, luoyang@sia.cn
 * 
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

/*-----------------------------------------------------------------------------
 * ecat_config.hpp
 * Description              EtherCAT Configurations
 * Author                   Yang Luo , luoyang@sia.cn
 * 
 *---------------------------------------------------------------------------*/

#ifndef ECAT_CONFIG_HPP_INCLUDED
#define ECAT_CONFIG_HPP_INCLUDED

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <sstream>

#include <boost/format.hpp>
#include <boost/timer/timer.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>

#include <string>
#include <cstring>
#include <cstdlib>
#include <unordered_map>

#include <sys/mman.h> //shm_open() mmap()
#include <unistd.h>   // ftruncate()
#include <fcntl.h>
#include <semaphore.h> //sem
#include <sys/stat.h>  //umask

#include <math.h>

#include "ecat_info.hpp"

#define ROBOT_CONFIG_FILE "ecat_config.yaml"

/*-SHARED MEMORY ------------------------------------------------------------*/
#define EC_SHM "ecm"
#define EC_SHM_MAX_SIZE 2048

//#define EC_SEM_SYNC "sync"
#define EC_SEM_MUTEX "sync"

#define MAX_NAME_LEN 20
#define MAX_JOINT_NUM 10


enum INPUTS //
{
    STATUS_WORD,
    POSITION_ACTUAL_VALUE,
    VELOCITY_ACTUAL_VALUE,
    TORQUE_ACTUAL_VALUE,
    LOAD_TORQUE_VALUE
};
enum OUTPUTS {
    MODE_OF_OPERATION,
    CONTROL_WORD,
    TARGET_POSITION,
    TARGET_VELOCITY,
    TARGET_TORQUE
};

//Terminal Color Show
enum Color {
    BLACK = 0,
    RED = 1,
    GREEN = 2,
    YELLOW = 3,
    BLUE = 4,
    MAGENTA = 5,
    CYAN = 6,
    WHITE = 7,
    DEFAULT = 9
};

enum MessageLevel {
    NORMAL = 0,
    WARNING = 1,
    ERROR = 2
};



//class EcatConfig {
//public:
//    EcatConfig(const EcatConfig&) = delete;
//    EcatConfig& operator=(const EcatConfig&) = delete;
//
//    static EcatConfig* p_instance;
//public:
//    static EcatConfig*  getInstance() {
//        if(p_instance == nullptr) {
//
//        }
//
//        return p_instance;
//    }
//
//private:
//    EcatConfig() {
//
//    };
//};
//
//EcatConfig* EcatConfig::p_instance = nullptr;  // static memeber initialize



/** Class SlaveConfig contains all configurations of one joint
 * 
 */
class SlaveConfig {
public:
    SlaveConfig() {
        // EtherCAT Process Data Input default Name Mapping
        ecInpMap[STATUS_WORD] = "Status word";
        ecInpMap[POSITION_ACTUAL_VALUE] = "Position actual value";
        ecInpMap[VELOCITY_ACTUAL_VALUE] = "Velocity actual value";
        ecInpMap[TORQUE_ACTUAL_VALUE] = "Torque actual value";
        ecInpMap[LOAD_TORQUE_VALUE] = "Analog Input 1";

        // EtherCAT Process Data Output default Name Mapping
        ecOutpMap[MODE_OF_OPERATION] = "Mode of operation";
        ecOutpMap[CONTROL_WORD] = "Control word";
        ecOutpMap[TARGET_POSITION] = "Target Position";
        ecOutpMap[TARGET_VELOCITY] = "Target Velocity";
        ecOutpMap[TARGET_TORQUE] = "Target Torque";
    }

    ~SlaveConfig() {

    }

    int id{0};

//    std::string jntName;

    std::string name{"Slave_1001 [Elmo Drive ]"};
    std::map<INPUTS, std::string> ecInpMap;
    std::map<OUTPUTS, std::string> ecOutpMap;

//    T_JOINT_EC_INPUT *jntEcInpPtr = nullptr;
//    T_JOINT_EC_OUTPUT *jntEcOutpPtr = nullptr;
};


/** Class RobotConfig contains all configurations of the robot
 * 
 */
class EcatConfig {
public:
    explicit EcatConfig(std::string configFile = ROBOT_CONFIG_FILE) : configFileName(configFile) {
    }

    ~EcatConfig() {

    }

    std::string configFileName{};

    std::string name{"default_robot"};

    uint32_t loop_hz{1000};

    int slave_number {0};

    std::vector<SlaveConfig> slaveCfg;


    int ec_in_fd;
    int ec_out_fd;

    sem_t *sem_mutex;
    sem_t *sem_sync;

    EcatInfo* ecatInfo = nullptr;

public:
    bool parserYamlFile(const std::string &configFile) {
        if (!boost::filesystem::exists(configFile)) {
            print_message("[YAML] Can not find the config file.", MessageLevel::ERROR);
            return false;
        }

        YAML::Node config = YAML::LoadFile(configFile);

        if (!config["robot"]) {
            print_message("[YAML] Can not find the robot label.", MessageLevel::ERROR);
            return false;
        }

        YAML::Node robot = config["robot"];

        name = robot["name"].as<std::string>();
        print_message("[YAML] Robot name is: " + name, MessageLevel::NORMAL);

        loop_hz = robot["loop_hz"].as<uint32_t>();

        slave_number = robot["number_of_joints"].as<int>();
        if (slave_number == robot["joints"].size()) {
            print_message((boost::format("[YAML] Robot has %d joints") % slave_number).str(), MessageLevel::NORMAL);
        } else {
            print_message("[YAML] Robot has bad number of joints.", MessageLevel::ERROR);
            return false;
        }

        YAML::Node joints = robot["joints"];
        slaveCfg.resize(slave_number);
        std::set<int> isJntOK;
        for (int i = 0; i < slave_number; i++) {

            /// joints.id
            int id = joints[i]["id"].as<int>();
            if (id < slave_number && id >= 0) {
                print_message((boost::format("[YAML] -- Joint ID: %d. ") % id).str(), MessageLevel::NORMAL);
            } else {
                print_message("[YAML] Bad joint ID!! ", MessageLevel::ERROR);
                return false;
            }
            auto res = isJntOK.insert(id);
            if (!res.second) { //Found duplicate elements
                print_message("[YAML] Bad joint ID is DUPLICATE!! ", MessageLevel::ERROR);
                return false;
            }

//            //name
//            slaveCfg[id].jntName = joints[i]["name"].as<std::string>();
//            print_message((boost::format("[YAML] -- Joint name: %s. ") % slaveCfg[id].jntName).str(),
//                          MessageLevel::NORMAL);

            /// joints[].name
            slaveCfg[id].name = joints[i]["ec_slave_name"].as<std::string>();
            print_message((boost::format("[YAML] -- Joint ec slave name: %s .") % slaveCfg[id].name).str(),
                          MessageLevel::NORMAL);

            /// Process Data Input Mapping
            if (joints[i]["inputs"]["status_word"])
                slaveCfg[id].ecInpMap[STATUS_WORD] = joints[i]["inputs"]["status_word"].as<std::string>();
            if (joints[i]["inputs"]["position_actual_value"])
                slaveCfg[id].ecInpMap[POSITION_ACTUAL_VALUE] = joints[i]["inputs"]["position_actual_value"].as<std::string>();
            if (joints[i]["inputs"]["velocity_actual_value"])
                slaveCfg[id].ecInpMap[VELOCITY_ACTUAL_VALUE] = joints[i]["inputs"]["velocity_actual_value"].as<std::string>();
            if (joints[i]["inputs"]["torque_actual_value"])
                slaveCfg[id].ecInpMap[TORQUE_ACTUAL_VALUE] = joints[i]["inputs"]["torque_actual_value"].as<std::string>();
            if (joints[i]["inputs"]["load_torque_value"])
                slaveCfg[id].ecInpMap[LOAD_TORQUE_VALUE] = joints[i]["inputs"]["load_torque_value"].as<std::string>();

            /// Process Data Out Mapping
            if (joints[i]["outputs"]["mode_of_operation"])
                slaveCfg[id].ecOutpMap[MODE_OF_OPERATION] = joints[i]["outputs"]["mode_of_operation"].as<std::string>();
            if (joints[i]["outputs"]["control_word"])
                slaveCfg[id].ecOutpMap[CONTROL_WORD] = joints[i]["outputs"]["control_word"].as<std::string>();
            if (joints[i]["outputs"]["target_position"])
                slaveCfg[id].ecOutpMap[TARGET_POSITION] = joints[i]["outputs"]["target_position"].as<std::string>();
            if (joints[i]["outputs"]["target_velocity"])
                slaveCfg[id].ecOutpMap[TARGET_VELOCITY] = joints[i]["outputs"]["target_velocity"].as<std::string>();
            if (joints[i]["outputs"]["target_torque"])
                slaveCfg[id].ecOutpMap[TARGET_TORQUE] = joints[i]["outputs"]["target_torque"].as<std::string>();

        }

        if (isJntOK.size() != slave_number) {
            print_message("[YAML] Bad joint IDs. Please check the ID of each joint. ", MessageLevel::ERROR);
            return false;
        }

        return true;
    }

    inline bool parserYamlFile() { return parserYamlFile(configFileName); }

    std::string getEcInpVarName(int jntId, INPUTS enumEcInp) {
        return slaveCfg[jntId].name + ".Inputs." + slaveCfg[jntId].ecInpMap[enumEcInp];
    }

    std::string getEcOutpVarName(int jntId, OUTPUTS enumEcOutp) {
        return slaveCfg[jntId].name + ".Outputs." + slaveCfg[jntId].ecOutpMap[enumEcOutp];
    }

    bool createSharedMemory() {
        mode_t mask = umask(0); // 取消屏蔽的权限位

        sem_mutex = sem_open(EC_SEM_MUTEX, O_CREAT | O_RDWR, 0777, 1);
        if (sem_mutex == SEM_FAILED) {
            print_message("[SHM] Can not open or create semaphore mutex.", MessageLevel::ERROR);
            return false;
        }

        int val = 0;
        sem_getvalue(sem_mutex, &val);
        std::cout << "value of sem_mutex is: " << val << std::endl;
        if (val != 1) {
            sem_destroy(sem_mutex);
            sem_unlink(EC_SEM_MUTEX);
            sem_mutex = sem_open(EC_SEM_MUTEX, O_CREAT | O_RDWR, 0777, 1);
        }

        sem_getvalue(sem_mutex, &val);
        if (val != 1) {
            print_message("[SHM] Can not set semaphore mutex to 1.", MessageLevel::ERROR);
            return false;
        }

        using namespace boost::interprocess;
        shared_memory_object::remove(EC_SHM);
        managed_shared_memory managedSharedMemory {open_or_create, EC_SHM, EC_SHM_MAX_SIZE};
        ecatInfo = managedSharedMemory.find_or_construct<EcatInfo>("ecat")();

        umask(mask); // 恢复umask的值

        return true;
    }

    /// \brief
    /// \return
    bool getSharedMemory() {
        mode_t mask = umask(0); // 取消屏蔽的权限位

        sem_mutex = sem_open(EC_SEM_MUTEX, O_CREAT, 0777, 1);
        if (sem_mutex == SEM_FAILED) {
            print_message("[SHM] Can not open or create semaphore mutex.", MessageLevel::ERROR);
            return false;
        }

        using namespace boost::interprocess;
        managed_shared_memory managedSharedMemory {open_or_create, EC_SHM, EC_SHM_MAX_SIZE};
        ecatInfo = managedSharedMemory.find_or_construct<EcatInfo>("ecat")();

        umask(mask); // 恢复umask的值

        return true;
    }

    ////////////// Get joints info for Ec Input /////////////////////

    inline int32_t getActualPositionEC(int id) const { return ecatInfo->slaves[id].inputs.position_actual_value; }

    inline int32_t getActualVelocityEC(int id) const { return ecatInfo->slaves[id].inputs.velocity_actual_value; }

    inline int16_t getActualTorqueEC(int id) const { return ecatInfo->slaves[id].inputs.torque_actual_value; }

    inline int32_t getLoadTorqueEC(int id) const { return ecatInfo->slaves[id].inputs.load_torque_value; }

    inline uint16_t getStatusWord(int id) const { return ecatInfo->slaves[id].inputs.status_word; }

    ////////////// Get joints info for Ec Input /////////////////////

    inline void setTargetPositionEC(int id, int32_t pos) { ecatInfo->slaves[id].outputs.target_position = pos; }

    inline void setTargetVelocityEC(int id, int32_t vel) { ecatInfo->slaves[id].outputs.target_velocity = vel; }

    inline void setTargetTorqueEC(int id, int32_t tor) { ecatInfo->slaves[id].outputs.target_torque = tor; }

    inline void setJointMode(int id, int32_t mode) { ecatInfo->slaves[id].outputs.mode_of_operation = mode; }

    inline void
    setControlWord(int id, int32_t ctrlword) { ecatInfo->slaves[id].outputs.control_word = ctrlword; }

    inline void waitForSignal() { sem_wait(sem_mutex); }

//    inline void unlock() { sem_post(sem_mutex); }

    ///////////// Format robot info /////////////////
    std::string to_string() {
        std::stringstream ss;
        for (int i = 0; i < slave_number; i++) {
            ss << "[Joint " << i << "] "
               << " stat: " << getStatusWord(i) << "; pos: " << getActualPositionEC(i)
               << "; vel: " << getActualVelocityEC(i) << "; tor: " << getActualTorqueEC(i) << ";";
        }
        return ss.str();
    }

private:
    //////////// OUTPUT FORMAT SETTINGS ////////////////////
    boost::format _f{"\033[1;3%1%m "};       //设置前景色
    boost::format _b{"\033[1;4%1%m "};       //设置背景色
    boost::format _fb{"\033[1;3%1%;4%2%m "}; //前景背景都设置
    boost::format _def{"\033[0m "};          //恢复默认

    void print_message(const std::string &msg, MessageLevel msgLvl = MessageLevel::NORMAL) {
        switch (msgLvl) {
            case MessageLevel::NORMAL:
                std::cout << _f % Color::GREEN << "[INFO]";
                break;
            case MessageLevel::WARNING:
                std::cout << _f % Color::YELLOW << "[WARNING]";
                break;
            case MessageLevel::ERROR:
                std::cout << _f % Color::RED << "[ERROR]";
                break;
            default:
                break;
        }

        std::cout << msg << _def << std::endl;
    }
};

#endif //ifndef ECAT_CONFIG_HPP_INCLUDED
