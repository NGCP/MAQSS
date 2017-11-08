#include "log.hpp"
#include <iostream>
#include <unistd.h>

Log::Log() {
    FILE *pipe = popen(SCRIPT_PATH, "r");
    if(!pipe) {
        //throw exception
        std::cerr << "Failed to create log file\n";
    }
    
    char buffer[128];
    std::string dirName = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
            dirName += buffer;
    }
    pclose(pipe);

    dirName.resize(dirName.size()-1);
    // TODO replace spaces with \ 
    dirName = "/\"" + dirName + "\"";
    std::cout << dirName + LOG_1 + "\n";

    usleep(2000000);

    logger1 = spdlog::basic_logger_mt("logger1", "logs/basic");
    logger2 = spdlog::basic_logger_mt("logger2", LOG_DIR + dirName + LOG_2);
    logger3 = spdlog::basic_logger_mt("logger3", LOG_DIR + dirName + LOG_3);
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