//
// Created by think on 2021/11/19.
//

//#define DOCTEST_CONFIG_IMPLEMENT
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <test/doctest.h>

#include <ecat_config.hpp>
#include <iostream>

TEST_CASE("Shared memory test") {
    EcatConfig ecatConfig;
    ecatConfig.getSharedMemory();
    for(int i = 0; i < 1000; i ++) {
        std::cout <<"Timestamp: " << ecatConfig.ecatInfo->timestamp << "; Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;
        usleep(10000);
    }
}

TEST_CASE("Shared memory test 2") {
    EcatConfig ecatConfig;
    ecatConfig.getSharedMemory();
    for(int i = 0; i < 10; i ++) {
        std::cout <<"Timestamp: " << ecatConfig.ecatInfo->timestamp << std::endl;
        std::cout << "  Slave name:  " << ecatConfig.ecatSlaveNameVec->at(0) << std::endl;
        std::cout << "  Ethercat State: " << ecatConfig.ecatInfo->ecatState << std::endl;
        std::cout << "  " << ecatConfig.ecatSlaveVec->at(0).inputs.position_actual_value << std::endl;

        ecatConfig.ecatSlaveVec->at(0).outputs.mode_of_operation = 8;
        ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 128;
        ecatConfig.ecatSlaveVec->at(0).outputs.target_position = ecatConfig.ecatSlaveVec->at(0).inputs.position_actual_value;
        usleep(100000);

        ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 6;
        usleep(100000);
        std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;


        ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 7;
        usleep(100000);
        std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;

        ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 15;
        usleep(100000);
        std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;

        usleep(1000000);

        ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 0;
        usleep(100000);
        std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;
    }
}

TEST_CASE("csp") {
    EcatConfig ecatConfig;
    ecatConfig.getSharedMemory();
    std::cout <<"Timestamp: " << ecatConfig.ecatInfo->timestamp << std::endl;
    std::cout << "  Slave name:  " << ecatConfig.ecatSlaveNameVec->at(0) << std::endl;
    std::cout << "  Ethercat State: " << ecatConfig.ecatInfo->ecatState << std::endl;
    std::cout << "  " << ecatConfig.ecatSlaveVec->at(0).inputs.position_actual_value << std::endl;

    ecatConfig.ecatSlaveVec->at(0).outputs.mode_of_operation = 8;
    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 128;
    ecatConfig.ecatSlaveVec->at(0).outputs.target_position = ecatConfig.ecatSlaveVec->at(0).inputs.position_actual_value ;
    usleep(1000000);
    std::cout << "Target Position is: " << ecatConfig.ecatSlaveVec->at(0).outputs.target_position << std::endl;
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;


    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 6;
    usleep(1000000);
    std::cout << "Target Position is: " << ecatConfig.ecatSlaveVec->at(0).outputs.target_position << std::endl;
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;


    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 7;
    usleep(1000000);
    std::cout << "Target Position is: " << ecatConfig.ecatSlaveVec->at(0).outputs.target_position << std::endl;
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;

    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 15;
    usleep(1000000);
    std::cout << "Target Position is: " << ecatConfig.ecatSlaveVec->at(0).outputs.target_position << std::endl;
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;
//    ecatConfig.ecatSlaveVec->at(0).outputs.target_velocity = 10000;

    usleep(10000000);

    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 0;
    usleep(1000000);
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;
}

TEST_CASE("csv") {
    EcatConfig ecatConfig;
    ecatConfig.getSharedMemory();
    std::cout <<"Timestamp: " << ecatConfig.ecatInfo->timestamp << std::endl;
    std::cout << "  Slave name:  " << ecatConfig.ecatSlaveNameVec->at(0) << std::endl;
    std::cout << "  Ethercat State: " << ecatConfig.ecatInfo->ecatState << std::endl;
    std::cout << "  " << ecatConfig.ecatSlaveVec->at(0).inputs.position_actual_value << std::endl;

    ecatConfig.ecatSlaveVec->at(0).outputs.mode_of_operation = 9;
    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 128;
    usleep(1000000);
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;


    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 6;
    usleep(1000000);
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;


    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 7;
    usleep(1000000);
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;

    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 15;
    usleep(1000000);
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;
    ecatConfig.ecatSlaveVec->at(0).outputs.target_velocity = 10000;

    usleep(10000000);

    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 0;
    usleep(1000000);
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;
}

TEST_CASE("cst") {
    EcatConfig ecatConfig;
    ecatConfig.getSharedMemory();
    std::cout <<"Timestamp: " << ecatConfig.ecatInfo->timestamp << std::endl;
    std::cout << "  Slave name:  " << ecatConfig.ecatSlaveNameVec->at(0) << std::endl;
    std::cout << "  Ethercat State: " << ecatConfig.ecatInfo->ecatState << std::endl;
    std::cout << "  " << ecatConfig.ecatSlaveVec->at(0).inputs.position_actual_value << std::endl;

    ecatConfig.ecatSlaveVec->at(0).outputs.mode_of_operation = 10;
    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 128;
    usleep(1000000);
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;


    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 6;
    usleep(1000000);
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;


    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 7;
    usleep(1000000);
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;

    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 15;
    usleep(1000000);
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;
    ecatConfig.ecatSlaveVec->at(0).outputs.target_torque = 70;

    usleep(20000000);

    ecatConfig.ecatSlaveVec->at(0).outputs.control_word = 0;
    usleep(1000000);
    std::cout << "Status is:  " << ecatConfig.ecatSlaveVec->at(0).inputs.status_word << std::endl;
}


//int main(int argc, char** argv) {
//    doctest::Context context;
//
//    // !!! THIS IS JUST AN EXAMPLE SHOWING HOW DEFAULTS/OVERRIDES ARE SET !!!
//
//    // defaults
//    context.addFilter("test-case-exclude", "*math*"); // exclude test cases with "math" in their name
//    context.setOption("abort-after", 5);              // stop test execution after 5 failed assertions
//    context.setOption("order-by", "name");            // sort the test cases by their name
//
//    context.applyCommandLine(argc, argv);
//
//    // overrides
//    context.setOption("no-breaks", true);             // don't break in the debugger when assertions fail
//
//    //---------------------八室初始化----------------------------
////    int err = 0;
////    command_arg arg;
////    int ac = 3;
////    char av1[] = "--EtherCATonly"; char av2[] = "on";
////    char* av[3] = {argv[0], av1 , av2};
////    err = commandLineParser(argc, av, &arg);
////    if (0 != err) {
////        return -1;
////    }
////    err = system_initialize(&arg);
////
////    if (0 != err) {
////        return err;
////    }
//    //--------------------八室初始化完毕------------------------
//
//    int res = context.run(); // run
//
//    if(context.shouldExit()) // important - query flags (and --exit) rely on the user doing this
//        return res;          // propagate the result of the tests
//
//    int client_stuff_return_code = 0;
//    // your program - if the testing framework is integrated in your production code
//
//    return res + client_stuff_return_code; // the result from doctest is propagated here as well
//}
