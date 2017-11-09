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
Log::Log() {
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
    auto all_sink = std::make_shared<spdlog::sinks::simple_file_sink_mt>(dirName + LOG_1);
    logger1 = std::make_shared<spdlog::logger>(LOG_1_NAME, all_sink);

    // Create sink for level 2 and group it with all_sink
    std::vector<spdlog::sink_ptr> log2SinkVect;
    auto log2Sink = std::make_shared<spdlog::sinks::simple_file_sink_mt>(dirName + LOG_2);
    log2SinkVect.push_back(all_sink);
    log2SinkVect.push_back(log2Sink);

    logger2 = std::make_shared<spdlog::logger>(LOG_2_NAME, begin(log2SinkVect), end(log2SinkVect));

    // Create sink for level 3 and group it with all_sink
    std::vector<spdlog::sink_ptr> log3SinkVect;
    auto log3Sink = std::make_shared<spdlog::sinks::simple_file_sink_mt>(dirName + LOG_3);
    log3SinkVect.push_back(all_sink);
    log3SinkVect.push_back(log3Sink);

    logger3 = std::make_shared<spdlog::logger>(LOG_3_NAME, begin(log3SinkVect), end(log3SinkVect));
}

void Log::level1(std::string msg) {
    logger1->info(msg);
}

void Log::level2(std::string msg) {
    logger2->info(msg);
}

void Log::level3(std::string msg) {
    logger3->info(msg);
}