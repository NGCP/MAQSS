#include "log.hpp"
#include <iostream>
#include <regex>
#include <unistd.h>

Log::Log() {
    // run create_log.sh
    FILE *pipe = popen(SCRIPT_PATH, "r");
    if(!pipe) {
        //throw exception
        std::cerr << "Failed to create log file\n";
    }

    // obtain its result from pipe
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);

    // result is of formate "DATE TIME"
    int delim = result.find(" ");
    std::string date = "/" + result.substr(0, delim);
    // omit \n at the end of result
    std::string curTime = "/" + result.substr(delim + 1, result.size() - delim - 2);

    std::cout << "logging to " LOG_DIR + date + curTime + "\n";

    // TODO remove?
    usleep(2000000);

    logger1 = spdlog::basic_logger_mt("logger1", LOG_DIR + date + curTime + LOG_1);
    logger2 = spdlog::basic_logger_mt("logger2", LOG_DIR + date + curTime + LOG_2);
    logger3 = spdlog::basic_logger_mt("logger3", LOG_DIR + date + curTime + LOG_3);
}

void Log::level1(std::string msg) {
    logger1->info(msg);
}

void Log::level2(std::string msg) {
    logger2->info(msg);
    logger1->info(msg);
}

void Log::level3(std::string msg) {
    logger3->info(msg);
    logger1->info(msg);
}