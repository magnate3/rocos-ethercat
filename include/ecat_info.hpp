//
// Created by think on 12/1/21.
//

#ifndef ROCOS_ECM_ECAT_INFO_HPP
#define ROCOS_ECM_ECAT_INFO_HPP

#include <boost/chrono.hpp>

/////////////////////   STRUCT DEFINITION   /////////////////////////////////
struct EcatInfo {

    boost::chrono::time_point<boost::chrono::system_clock> timestamp;  // Timestamp

    enum EcatState {
        UNKNOWN = 0,
        INIT = 1,
        PREOP = 2,
        SAFEOP = 4,
        OP = 8,

        BOOTSTRAP = 3
    };

    EcatState ecatState;    // State of Ec-Master

    struct EcatSlaveInfo {

        std::string slave_name{""};

        /** Struct store the EtherCAT process data input  **/
        struct PDInput {
            uint16_t status_word{0};          //  Size: 2.0 unsigned
            int32_t position_actual_value{0}; // Size: 4.0 signed
            int32_t velocity_actual_value{0}; // Size: 4.0 signed
            int16_t torque_actual_value{0};   // Size: 2.0 signed
            int16_t load_torque_value{0};     // Size: 2.0 signed
        };

        PDInput inputs;

        /** Struct store the EtherCAT process data output  **/
        struct PDOutput {
            int8_t mode_of_operation{8}; // Size: 1.0 signed
            uint16_t control_word{0};    // Size: 2.0 unsigned
            int32_t target_position{0};  // Size: 4.0 signed
            int32_t target_velocity{0};  // Size: 4.0 signed
            int16_t target_torque{0};    // Size: 2.0 signed
        };

        PDOutput outputs;
    };


    std::vector<EcatSlaveInfo> slaves; // all the slaves data

};

#endif //ROCOS_ECM_ECAT_INFO_HPP
