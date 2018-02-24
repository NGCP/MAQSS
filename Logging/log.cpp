/*
    Creates logging system.
    Upon construction, runs a shell script to set up log directory in
    build/logs/DATE/TIME.
    Creates log files for level1, level2, and level3 log messages.
    Declares individual methods to log to each file. 
*/

#include "log.hpp"
#include <iostream>
#include <regex>
#include <unistd.h>
#include <stdexcept>

// Initializes loggers 

Log::Log(int input_level) {

    this->input_level = input_level;
    // run create_log.sh
    FILE *pipe = popen(SCRIPT_PATH, "r");
    if(!pipe) {
        throw std::runtime_error("Failed to create log file");
    }

    // obtain script result from pipe
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);

    // script result is of format "DATE TIME"
    int delim = result.find(" ");
    std::string date = "/" + result.substr(0, delim);
    // omit \n at the end of result
    std::string curTime = "/" + result.substr(delim + 1, result.size() - delim - 2);

    std::cout << "logging to " LOG_DIR + date + curTime + "\n";

    std::string dirName = LOG_DIR + date + curTime;
    // Shared file sink for all log messages
    auto all_sink = std::make_shared<spdlog::sinks::simple_file_sink_mt>(dirName);
    logger = std::make_shared<spdlog::logger>(all_sink);
}

void Log::log(int log_level, std::string msg) {
    if (log_level == input_level || input_level == 3) {
        logger->info(msg);
    }
}
