/**
 * Copyright (c) 2021 Yang Luo, luoyang@sia.cn
 * 
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

/*-----------------------------------------------------------------------------
 * robot_config.hpp
 * Description              Robot Configurations
 * Author                   Yang Luo , luoyang@sia.cn
 * 
 *---------------------------------------------------------------------------*/

#ifndef ROBOT_CONFIG_HPP_INCLUDED
#define ROBOT_CONFIG_HPP_INCLUDED

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <sstream>

#include <boost/format.hpp>
#include <boost/timer/timer.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>  

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

#define ROBOT_CONFIG_FILE "robot_config.yaml"

/*-SHARED MEMORY ------------------------------------------------------------*/
#define EC_SHM_IN "ecat_in"
#define EC_SHM_OUT "ecat_out"

#define EC_SEM_MUTEX "mutex"
#define EC_SEM_SYNC "sync"

#define MAX_NAME_LEN 20
#define MAX_JOINT_NUM 10

//static int ec_in_fd;
//static int ec_out_fd;
//
//static sem_t *sem_mutex;
//static sem_t *sem_sync;

enum Inputs
{
    ec_status_word,
    ec_position_actual_value,
    ec_velocity_actual_value,
    ec_torque_actual_value,
    ec_load_torque_value
};
enum Outputs
{
    ec_mode_of_operation,
    ec_control_word,
    ec_target_position,
    ec_target_velocity,
    ec_target_torque,
};

//Terminal Color Show
enum Color
{
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

enum MessageLevel
{
    NORMAL = 0,
    WARNING = 1,
    ERROR = 2
};

/** Struct store the EtherCAT process data input  **/
typedef struct _T_JOINT_EC_INPUT
{
    uint16_t ec_status_word;          //  Size: 2.0 unsigned
    int32_t ec_position_actual_value; // Size: 4.0 signed
    int32_t ec_velocity_actual_value; // Size: 4.0 signed
    int16_t ec_torque_actual_value;   // Size: 2.0 signed
    int16_t ec_load_torque_value;     // Size: 2.0 signed
} T_JOINT_EC_INPUT;

/** Struct store the EtherCAT process data output  **/
typedef struct _T_JOINT_EC_OUTPUT
{
    int8_t ec_mode_of_operation = 8; // Size: 1.0 signed
    uint16_t ec_control_word = 0;    // Size: 2.0 unsigned
    int32_t ec_target_position = 0;  // Size: 4.0 signed
    int32_t ec_target_velocity = 0;  // Size: 4.0 signed
    int16_t ec_target_torque = 0;    // Size: 2.0 signed
} T_JOINT_EC_OUTPUT;

/** Struct store the robot of MAX_JOINT_NUM joints' inputs **/
typedef struct _T_ROBOT_EC_INPUT
{
    T_JOINT_EC_INPUT j[MAX_JOINT_NUM];
} T_ROBOT_EC_INPUT;

/** Struct store the robot of MAX_JOINT_NUM joints' outputs **/
typedef struct _T_ROBOT_EC_OUTPUT
{
    T_JOINT_EC_OUTPUT j[MAX_JOINT_NUM];
} T_ROBOT_EC_OUTPUT;

/** Class SlaveConfig contains all configurations of one joint
 * 
 */
class SlaveConfig
{
public:
    SlaveConfig()
    {
        // EtherCAT Process Data Input default Name Mapping
        ecInpMap[ec_status_word] = "Status word";
        ecInpMap[ec_position_actual_value] = "Position actual value";
        ecInpMap[ec_velocity_actual_value] = "Velocity actual value";
        ecInpMap[ec_torque_actual_value] = "Torque actual value";
        ecInpMap[ec_load_torque_value] = "Analog Input 1";

        // EtherCAT Process Data Output default Name Mapping
        ecOutpMap[ec_mode_of_operation] = "Mode of operation";
        ecOutpMap[ec_control_word] = "Control word";
        ecOutpMap[ec_target_position] = "Target Position";
        ecOutpMap[ec_target_velocity] = "Target Velocity";
        ecOutpMap[ec_target_torque] = "Target Torque";
    }

    ~SlaveConfig()
    {
//        munmap(jntEcInpPtr, sizeof(T_JOINT_EC_INPUT));
//        munmap(jntEcOutpPtr, sizeof(T_JOINT_EC_OUTPUT));
    }

    int id{0};

    std::string jntName;

    int mode_of_operation = 8;

    double minPosition = -100.0; // rad [required]
    double maxPosition = 100.0;  // rad [required]

    double maxVelocity = 2.0; // rad/s -> radius per second [required]

    double maxAcceleration = 600.0; // rad/s^2 [required]

    int countPerRound = 524288; // 2^19

    double torque_range = 100; // range of the torque sensor

    std::string name{"Slave_1001 [Elmo Drive ]"};
    std::map<Inputs, std::string> ecInpMap;
    std::map<Outputs, std::string> ecOutpMap;

    T_JOINT_EC_INPUT *jntEcInpPtr = nullptr;
    T_JOINT_EC_OUTPUT *jntEcOutpPtr = nullptr;
};


/** Class RobotConfig contains all configurations of the robot
 * 
 */
class RobotConfig
{
public:
    explicit RobotConfig(std::string configFile = ROBOT_CONFIG_FILE) : configFileName(configFile)
    {
    }

    ~RobotConfig()
    {
        // sem_destroy(sem_mutex);
        // sem_destroy(sem_sync);

        // sem_unlink(EC_SEM_MUTEX);
        // sem_unlink(EC_SEM_SYNC);

        // shm_unlink(EC_SHM_IN);
        // shm_unlink(EC_SHM_OUT);
    }

    std::string configFileName{};

    std::string robotName{"default_robot"};

    uint32_t loop_hz{1000};

    int jntNum{6};

    std::vector<SlaveConfig> jntCfg;
    //    SlaveConfig slaveCfg[MAX_JOINT_NUM];

    T_ROBOT_EC_INPUT *ecInpPtr{nullptr};
    T_ROBOT_EC_OUTPUT *ecOutpPtr{nullptr};

    int ec_in_fd;
    int ec_out_fd;

    sem_t *sem_mutex;
    sem_t *sem_sync;

public:
    bool parserYamlFile(const std::string &configFile)
    {
        if (!boost::filesystem::exists(configFile))
        {
            print_message("[YAML] Can not find the config file.", MessageLevel::ERROR);
            return false;
        }

        YAML::Node config = YAML::LoadFile(configFile);

        if (!config["robot"])
        {
            print_message("[YAML] Can not find the robot label.", MessageLevel::ERROR);
            return false;
        }

        YAML::Node robot = config["robot"];

        robotName = robot["name"].as<std::string>();
        print_message("[YAML] Robot name is: " + robotName, MessageLevel::NORMAL);

        loop_hz = robot["loop_hz"].as<uint32_t>();

        jntNum = robot["number_of_joints"].as<int>();
        if (jntNum == robot["joints"].size())
        {
            print_message((boost::format("[YAML] Robot has %d joints") % jntNum).str(), MessageLevel::NORMAL);
        }
        else
        {
            print_message("[YAML] Robot has bad number of joints.", MessageLevel::ERROR);
            return false;
        }

        YAML::Node joints = robot["joints"];
        jntCfg.resize(jntNum);
        std::set<int> isJntOK;
        for (int i = 0; i < jntNum; i++)
        {

            /// joints.id
            int id = joints[i]["id"].as<int>();
            if (id < jntNum && id >= 0)
            {
                print_message((boost::format("[YAML] -- Joint ID: %d. ") % id).str(), MessageLevel::NORMAL);
            }
            else
            {
                print_message("[YAML] Bad joint ID!! ", MessageLevel::ERROR);
                return false;
            }
            auto res = isJntOK.insert(id);
            if (!res.second)
            { //Found duplicate elements
                print_message("[YAML] Bad joint ID is DUPLICATE!! ", MessageLevel::ERROR);
                return false;
            }

            //name
            jntCfg[id].jntName = joints[i]["name"].as<std::string>();
            print_message((boost::format("[YAML] -- Joint name: %s. ") % jntCfg[id].jntName).str(), MessageLevel::NORMAL);

            //mode_of_operation
            std::string mp = joints[i]["mode_of_operation"].as<std::string>();

            if (boost::algorithm::to_lower_copy(mp) == "csp")
            {
                jntCfg[id].mode_of_operation = 8;
            }
            else if (boost::algorithm::to_lower_copy(mp) == "csv")
            {
                jntCfg[id].mode_of_operation = 9;
            }
            else if (boost::algorithm::to_lower_copy(mp) == "cst")
            {
                jntCfg[id].mode_of_operation = 10;
            }

            /// joints[].min_position
            if (!joints[i]["min_position"])
            {
                print_message("[YAML] min_position is REQUIRED!! ", MessageLevel::ERROR);
                return false;
            }
            jntCfg[id].minPosition = joints[i]["min_position"].as<double>();
            print_message((boost::format("[YAML] -- Joint minimum position: %f .") % jntCfg[id].minPosition).str(),
                          MessageLevel::NORMAL);

            /// joints[].maxPosition
            if (!joints[i]["max_position"])
            {
                print_message("[YAML] max_position is REQUIRED!! ", MessageLevel::ERROR);
                return false;
            }
            jntCfg[id].maxPosition = joints[i]["max_position"].as<double>();
            print_message((boost::format("[YAML] -- Joint maximum position: %f .") % jntCfg[id].maxPosition).str(),
                          MessageLevel::NORMAL);

            /// joints[].max_velocity
            if (!joints[i]["max_velocity"])
            {
                print_message("[YAML] max_velocity is REQUIRED!! ", MessageLevel::ERROR);
                return false;
            }
            jntCfg[id].maxVelocity = joints[i]["max_velocity"].as<double>();
            print_message((boost::format("[YAML] -- Joint maximum velocity: %f .") % jntCfg[id].maxVelocity).str(),
                          MessageLevel::NORMAL);

            /// joints[].max_acceleration
            if (!joints[i]["max_acceleration"])
            {
                print_message("[YAML] max_acceleration is REQUIRED!! ", MessageLevel::ERROR);
                return false;
            }
            jntCfg[id].maxAcceleration = joints[i]["max_acceleration"].as<double>();
            print_message((boost::format("[YAML] -- Joint maximum acceleration: %f .") % jntCfg[id].maxAcceleration).str(),
                          MessageLevel::NORMAL);

            /// joints[].countsPerRound
            if (!joints[i]["counts_per_round"])
            {
                print_message("[YAML] counts_per_round is REQUIRED!! ", MessageLevel::ERROR);
                return false;
            }
            jntCfg[id].countPerRound = joints[i]["counts_per_round"].as<double>();
            print_message((boost::format("[YAML] -- Joint counts per round: %f .") % jntCfg[id].countPerRound).str(),
                          MessageLevel::NORMAL);

            if (joints[i]["torque_range"])
            {
                jntCfg[id].torque_range = joints[i]["torque_range"].as<double>();
            }

            /// joints[].name
            jntCfg[id].name = joints[i]["ec_slave_name"].as<std::string>();
            print_message((boost::format("[YAML] -- Joint ec slave name: %s .") % jntCfg[id].name).str(),
                          MessageLevel::NORMAL);

            //Process Data Input Mapping
            if (joints[i]["inputs"]["ec_status_word"])
                jntCfg[id].ecInpMap[ec_status_word] = joints[i]["inputs"]["ec_status_word"].as<std::string>();
            if (joints[i]["inputs"]["ec_position_actual_value"])
                jntCfg[id].ecInpMap[ec_position_actual_value] = joints[i]["inputs"]["ec_position_actual_value"].as<std::string>();
            if (joints[i]["inputs"]["ec_velocity_actual_value"])
                jntCfg[id].ecInpMap[ec_velocity_actual_value] = joints[i]["inputs"]["ec_velocity_actual_value"].as<std::string>();
            if (joints[i]["inputs"]["ec_torque_actual_value"])
                jntCfg[id].ecInpMap[ec_torque_actual_value] = joints[i]["inputs"]["ec_torque_actual_value"].as<std::string>();
            if (joints[i]["inputs"]["ec_load_torque_value"])
                jntCfg[id].ecInpMap[ec_load_torque_value] = joints[i]["inputs"]["ec_load_torque_value"].as<std::string>();

            //Process Data Out Mapping
            if (joints[i]["outputs"]["ec_mode_of_operation"])
                jntCfg[id].ecOutpMap[ec_mode_of_operation] = joints[i]["outputs"]["ec_mode_of_operation"].as<std::string>();
            if (joints[i]["outputs"]["ec_control_word"])
                jntCfg[id].ecOutpMap[ec_control_word] = joints[i]["outputs"]["ec_control_word"].as<std::string>();
            if (joints[i]["outputs"]["ec_target_position"])
                jntCfg[id].ecOutpMap[ec_target_position] = joints[i]["outputs"]["ec_target_position"].as<std::string>();
            if (joints[i]["outputs"]["ec_target_velocity"])
                jntCfg[id].ecOutpMap[ec_target_velocity] = joints[i]["outputs"]["ec_target_velocity"].as<std::string>();
            if (joints[i]["outputs"]["ec_target_torque"])
                jntCfg[id].ecOutpMap[ec_target_torque] = joints[i]["outputs"]["ec_target_torque"].as<std::string>();
        }

        if (isJntOK.size() != jntNum)
        {
            print_message("[YAML] Bad joint IDs. Please check the ID of each joint. ", MessageLevel::ERROR);
            return false;
        }

        return true;
    }

    inline bool parserYamlFile() { return parserYamlFile(configFileName); }

    std::string getEcInpVarName(int jntId, Inputs enumEcInp)
    {
        return jntCfg[jntId].name + ".Inputs." + jntCfg[jntId].ecInpMap[enumEcInp];
    }

    std::string getEcOutpVarName(int jntId, Outputs enumEcOutp)
    {
        return jntCfg[jntId].name + ".Outputs." + jntCfg[jntId].ecOutpMap[enumEcOutp];
    }

    bool createSharedMemory()
    {
        mode_t mask = umask(0); // 取消屏蔽的权限位

        sem_mutex = sem_open(EC_SEM_MUTEX, O_CREAT | O_RDWR, 0777, 1);
        if (sem_mutex == SEM_FAILED)
        {
            print_message("[SHM] Can not open or create semaphore mutex.", MessageLevel::ERROR);
            return false;
        }

        int val = 0;
        sem_getvalue(sem_mutex, &val);
        std::cout << "value of sem_mutex is: " << val << std::endl;
        if (val != 1)
        {
            sem_destroy(sem_mutex);
            sem_unlink(EC_SEM_MUTEX);
            sem_mutex = sem_open(EC_SEM_MUTEX, O_CREAT | O_RDWR, 0777, 1);
        }

        sem_getvalue(sem_mutex, &val);
        if (val != 1)
        {
            print_message("[SHM] Can not set semaphore mutex to 1.", MessageLevel::ERROR);
            return false;
        }

        sem_sync = sem_open(EC_SEM_SYNC, O_CREAT | O_RDWR, 0777, 0);
        if (sem_sync == SEM_FAILED)
        {
            print_message("[SHM] Can not open or create semaphore sync.", MessageLevel::ERROR);
            return false;
        }

        sem_getvalue(sem_sync, &val);
        if (val != 0)
        {
            sem_destroy(sem_sync);
            sem_unlink(EC_SEM_SYNC);
            sem_sync = sem_open(EC_SEM_SYNC, O_CREAT | O_RDWR, 0777, 0);
        }

        sem_getvalue(sem_sync, &val);
        if (val != 0)
        {
            print_message("[SHM] Can not set semaphore sync to 0.", MessageLevel::ERROR);
            return false;
        }

        // ecat_in shared memory
        ec_in_fd = shm_open(EC_SHM_IN, O_RDWR | O_CREAT, 0777);
        if (ec_in_fd == -1)
        {
            print_message("[SHM] Can not open or create the shared memory for EcInput.", MessageLevel::ERROR);
            return false;
        }
        if (ftruncate(ec_in_fd, sizeof(T_JOINT_EC_INPUT) * jntNum) == -1)
        {
            print_message("[SHM] Can not allocate the shared memory for EcInput.", MessageLevel::ERROR);
            return false;
        }

        // ecat_out shared memory
        ec_out_fd = shm_open(EC_SHM_OUT, O_RDWR | O_CREAT, 0777);
        if (ec_out_fd == -1)
        {
            print_message("[SHM] Can not open or create the shared memory for EcOutput.", MessageLevel::ERROR);
            return false;
        }
        if (ftruncate(ec_out_fd, sizeof(T_JOINT_EC_OUTPUT) * jntNum) == -1)
        {
            print_message("[SHM] Can not allocate the shared memory for EcOutput.", MessageLevel::ERROR);
            return false;
        }

        for (int i = 0; i < jntCfg.size(); i++)
        {
            jntCfg[i].jntEcInpPtr = (T_JOINT_EC_INPUT *)mmap(NULL, sizeof(T_JOINT_EC_INPUT), PROT_READ | PROT_WRITE,
                                                             MAP_SHARED, ec_in_fd,
                                                             i * sizeof(T_JOINT_EC_INPUT));
            if (jntCfg[i].jntEcInpPtr == nullptr)
                return false;

            jntCfg[i].jntEcOutpPtr = (T_JOINT_EC_OUTPUT *)mmap(NULL, sizeof(T_JOINT_EC_OUTPUT), PROT_READ | PROT_WRITE,
                                                               MAP_SHARED, ec_out_fd,
                                                               i * sizeof(T_JOINT_EC_OUTPUT));
            if (jntCfg[i].jntEcOutpPtr == nullptr)
                return false;
        }

        umask(mask); // 恢复umask的值

        return true;
    }

    bool getSharedMemory()
    {
        mode_t mask = umask(0); // 取消屏蔽的权限位

        sem_mutex = sem_open(EC_SEM_MUTEX, O_CREAT, 0777, 1);
        if (sem_mutex == SEM_FAILED)
        {
            print_message("[SHM] Can not open or create semaphore mutex.", MessageLevel::ERROR);
            return false;
        }

        sem_sync = sem_open(EC_SEM_SYNC, O_CREAT, 0777, 0);
        if (sem_sync == SEM_FAILED)
        {
            print_message("[SHM] Can not open or create semaphore sync.", MessageLevel::ERROR);
            return false;
        }

        // ecat_in shared memory
        ec_in_fd = shm_open(EC_SHM_IN, O_RDWR, 0777);
        if (ec_in_fd == -1)
        {
            print_message("[SHM] Can not open or create the shared memory for EcInput.", MessageLevel::ERROR);
            return false;
        }

        // ecat_out shared memory
        ec_out_fd = shm_open(EC_SHM_OUT, O_RDWR, 0777);
        if (ec_out_fd == -1)
        {
            print_message("[SHM] Can not open or create the shared memory for EcOutput.", MessageLevel::ERROR);
            return false;
        }

        for (int i = 0; i < jntCfg.size(); i++)
        {
            jntCfg[i].jntEcInpPtr = (T_JOINT_EC_INPUT *)mmap(NULL, sizeof(T_JOINT_EC_INPUT), PROT_READ | PROT_WRITE,
                                                             MAP_SHARED, ec_in_fd,
                                                             i * sizeof(T_JOINT_EC_INPUT));
            if (jntCfg[i].jntEcInpPtr == nullptr)
                return false;

            jntCfg[i].jntEcOutpPtr = (T_JOINT_EC_OUTPUT *)mmap(NULL, sizeof(T_JOINT_EC_OUTPUT), PROT_READ | PROT_WRITE,
                                                               MAP_SHARED, ec_out_fd,
                                                               i * sizeof(T_JOINT_EC_OUTPUT));
            if (jntCfg[i].jntEcOutpPtr == nullptr)
                return false;
        }

        umask(mask); // 恢复umask的值

        return true;
    }

    ////////////// Get joints info for Ec Input /////////////////////

    inline int32_t getJointPositionEC(int jntId) { return jntCfg[jntId].jntEcInpPtr->ec_position_actual_value; }
    inline int32_t getJointVelocityEC(int jntId) { return jntCfg[jntId].jntEcInpPtr->ec_velocity_actual_value; }
    inline int16_t getJointTorqueEC(int jntId) { return jntCfg[jntId].jntEcInpPtr->ec_torque_actual_value; }
    inline int32_t getJointLoadTorqueEC(int jntId) { return jntCfg[jntId].jntEcInpPtr->ec_load_torque_value; }

     inline double getJointPosition(int jntId) { return getJointPositionEC(jntId)  * 2.0 * M_PI / jntCfg[jntId].countPerRound; }
     inline double getJointVelocity(int jntId) { return getJointVelocityEC(jntId) * 2.0 * M_PI / jntCfg[jntId].countPerRound; }
     inline double getJointTorque(int jntId) { return getJointTorqueEC(jntId); }
     inline double getJointLoadTorque(int jntId) { return getJointLoadTorqueEC(jntId); }
//     inline double getJointLoadTorque(int jntId) { return (double)slaveCfg[jntId].jntEcInpPtr->ec_load_torque_value * slaveCfg[jntId].torque_range / 1000.0; }


    inline uint16_t getJointStatus(int jntId) { return jntCfg[jntId].jntEcInpPtr->ec_status_word; }

    ////////////// Get joints info for Ec Input /////////////////////
    
    inline void setJointPositionEC(int jntId, int32_t pos) { jntCfg[jntId].jntEcOutpPtr->ec_target_position = pos; }
    inline void setJointVelocityEC(int jntId, int32_t vel) { jntCfg[jntId].jntEcOutpPtr->ec_target_velocity = vel; }
    inline void setJointTorqueEC(int jntId, int32_t tor) { jntCfg[jntId].jntEcOutpPtr->ec_target_torque = tor; }

    // inline void setJointPositionEC(int jntId, double pos_rad) { slaveCfg[jntId].jntEcOutpPtr->ec_target_position = pos_rad * slaveCfg[jntId].countPerRound / (2 * M_PI) ; }
    // inline void setJointVelocityEC(int jntId, double vel_rad) { slaveCfg[jntId].jntEcOutpPtr->ec_target_velocity = vel_rad * slaveCfg[jntId].countPerRound / (2 * M_PI) ; }
    // inline void setJointTorqueEC(int jntId, double tor) { slaveCfg[jntId].jntEcOutpPtr->ec_target_torque = tor; }


    inline void setJointMode(int jntId, int32_t mode) { jntCfg[jntId].jntEcOutpPtr->ec_mode_of_operation = mode; }

    inline void
    setJointCtrlWord(int jntId, int32_t ctrlword) { jntCfg[jntId].jntEcOutpPtr->ec_control_word = ctrlword; }

    inline void lock() { sem_wait(sem_mutex); }

    inline void unlock() { sem_post(sem_mutex); }

    ///////////// Format robot info /////////////////
    std::string to_string()
    {
        std::stringstream ss;
        for (int i = 0; i < jntNum; i++)
        {
            ss << "[Joint " << i << "] "
               << " stat: " << getJointStatus(i) << "; pos: " << getJointPositionEC(i)
               << "; vel: " << getJointVelocityEC(i) << "; tor: " << getJointTorqueEC(i) << ";";
        }
        return ss.str();
    }

private:
    //////////// OUTPUT FORMAT SETTINGS ////////////////////
    boost::format _f{"\033[1;3%1%m "};       //设置前景色
    boost::format _b{"\033[1;4%1%m "};       //设置背景色
    boost::format _fb{"\033[1;3%1%;4%2%m "}; //前景背景都设置
    boost::format _def{"\033[0m "};          //恢复默认

    void print_message(const std::string &msg, MessageLevel msgLvl = MessageLevel::NORMAL)
    {
        switch (msgLvl)
        {
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

#endif //ifndef ROBOT_CONFIG_HPP_INCLUDED
