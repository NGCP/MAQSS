// C++ headers
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cstring>
#include <ctime>

// C headers
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "processInterface.hpp"
#include "fileIO.hpp"

#define MAX_IMGS 2000

processInterface *Cv_quit;
configContainer *configs_quit;
raspicam::RaspiCam_Cv *cam_quit;

unsigned int nCaptures(0);

void quit_handler(int sig) {
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    // Close camera interface
    cam_quit->release();

    // pipes
    Cv_quit->cleanup(configs_quit);

    // end program here
    exit(0);

}

int mainLoop(processInterface *Cv, configContainer *configs) {
    raspicam::RaspiCam_Cv cam;

    // Setup interrupt handlers so all interfaces get closed
    Cv_quit = Cv;
    configs_quit = configs;
    cam_quit = &cam;
    signal(SIGINT, quit_handler);

    // set up camera interface
    cam.set(CV_CAP_PROP_FRAME_WIDTH, configs->cam_Width);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, configs->cam_Height);
    cam.set(CV_CAP_PROP_FORMAT, CV_8UC3);

    // TODO: Is there support for video_stabilization with Raspicam_CV API?
    //    cam.setISO(0);
    //    cam.setExposureCompensation(0);
    //    cam.setFormat(raspicam::RASPICAM_FORMAT_BGR); // FORMAT MUST BE BGR or colors will be reversed
    //    cam.setVideoStabilization(configs->video_Stabilization);

    if (!cam.open()) {
        std::cerr << "Error opening camera" << std::endl;
        exit(1);
    }
    std::cerr << "Cam Opened, Sleeping... " << std::endl;
    sleep(2);
    std::cerr << "Awake, progressing... " << std::endl;

    cv::Mat img;
    char tmp[BUF_LEN];
    int ndx;
    int ctr(1);
    std::string done = "Done";
    std::string str = "image";
    
    // capture 5 frames when a start signal is received
    while (true) {

        // TODO: Fix raspicam photo capture, its off color
        read(configs->fd_PNav_to_CV, tmp, BUF_LEN);
        std::cerr << "Read msg from PNav: " << tmp << std::endl;
        sleep(1);
        if (!strcmp(tmp, "Start")) {

            // TODO: Figure out how to send a kill message from PNAV to CV
            for (ndx = 0; ndx < 5; ndx++) {
                cam.grab();
                cam.retrieve(img);
                cv::imwrite(str + std::to_string(ctr) + "_" + std::to_string(ndx) + ".jpg", img);
                nCaptures++;

            }

            Cv->writePipe(configs->fd_CV_to_PNav, done);
            ctr++;
        } else if (!strcmp(tmp, "Exit")) {
            cam.release();
            Cv->cleanup(configs);
            exit(0);
        }
        if (nCaptures > MAX_IMGS) break;
    }

    return 0;
}

int testLoop(processInterface *Cv, configContainer *configs) {
    // run CV process in test mode to continually take up to 2000 images

    raspicam::RaspiCam_Cv cam;
    //    unsigned char *data = new unsigned char[ cam.getImageBufferSize()];
    cv::Mat img;

    float cap_freq = configs->cap_Freq;
    std::string str = "image";

    // Setup interrupt handlers so all interfaces get closed
    Cv_quit = Cv;
    configs_quit = configs;
    cam_quit = &cam;
    signal(SIGINT, quit_handler);

    // set up camera interface
    cam.set(CV_CAP_PROP_FRAME_WIDTH, configs->cam_Width);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, configs->cam_Height);
    cam.set(CV_CAP_PROP_FORMAT, CV_8UC3);

    // set up camera interface
    //    cam.setISO(0);
    //    cam.setExposureCompensation(0);
    //    cam.setFormat(raspicam::RASPICAM_FORMAT_BGR); // FORMAT MUST BE BGR or colors will be reversed

    //    std::cerr << "Setting Video Stabilization: " << configs->video_Stabilization << std::endl;
    //    cam.setVideoStabilization(configs->video_Stabilization);

    if (!cam.open()) {
        std::cerr << "Error opening camera" << std::endl;
        exit(1);
    }
    std::cerr << "Cam Opened, Sleeping... " << std::endl;
    sleep(2);
    std::cerr << "Awake, progressing... " << std::endl;

    // start capturing upto MAX_IMGS images
    while (nCaptures < MAX_IMGS) {
        std::cerr << "In Loop, image: " << nCaptures << " @freq:" << 1 / cap_freq  << "/s"  << std::endl;
        cam.grab();
        cam.retrieve(img);
        cv::imwrite(str + std::to_string(nCaptures) + ".jpg", img);
        nCaptures++;
        sleep(1.0 / cap_freq);
    }

    return 0;
}

int main(int argc, char** argv) {
    configContainer configs;
    if (argc > 1) configs = fileIO::getConfig(argc, argv);
    else {
        // TODO: Add a print help function and figure out where to put it
        std::cerr << "Must specify command line arguments" << std::endl;
        exit(1); // TODO: Perform closeout/ clean up function when exiting
    }
    processInterface Cv(&configs, CV);
    fileIO::printConfig(&configs);
    if (configs.cam_Test) testLoop(&Cv, &configs);
    else mainLoop(&Cv, &configs);

    return 0;
}