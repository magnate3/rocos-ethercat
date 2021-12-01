//
// Created by think on 2021/11/19.
//

//#define DOCTEST_CONFIG_IMPLEMENT
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <test/doctest.h>

#include <ecat_config.hpp>

TEST_CASE("Shared memory test") {

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
