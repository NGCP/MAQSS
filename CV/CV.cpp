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
#include <raspicam/raspicam.h>

#include "processInterface.hpp"
#include "fileIO.hpp"

#define MAX_IMGS 2000

processInterface *Cv_quit;
configContainer *configs_quit;
raspicam::RaspiCam *cam_quit;

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

void saveImage(std::string filepath, unsigned char *data, raspicam::RaspiCam &Camera) {
    std::ofstream outFile(filepath.c_str(), std::ios::binary);
    if (Camera.getFormat() == raspicam::RASPICAM_FORMAT_BGR || Camera.getFormat() == raspicam::RASPICAM_FORMAT_RGB) {
        outFile << "P6\n";
    } else if (Camera.getFormat() == raspicam::RASPICAM_FORMAT_GRAY) {
        outFile << "P5\n";
    } else if (Camera.getFormat() == raspicam::RASPICAM_FORMAT_YUV420) { //made up format
        outFile << "P7\n";
    }
    outFile << Camera.getWidth() << " " << Camera.getHeight() << " 255\n";
    outFile.write((char*) data, Camera.getImageBufferSize());
}

void printCameraParameters(raspicam::RaspiCam *cam) {
    // Print out the cam parameters for debugging
    std::cerr << "\nCAM PARAMS" << std::endl;
    std::cerr << "Format: " << cam->getFormat() << std::endl;
    std::cerr << "Width: " << cam->getWidth() << std::endl;
    std::cerr << "Height: " << cam->getHeight() << std::endl;
    std::cerr << "Brightness: " << cam->getBrightness() << std::endl;
    std::cerr << "Rotation: " << cam->getRotation() << std::endl;
    std::cerr << "ISO: " << cam->getISO() << std::endl;
    std::cerr << "Sharpness: " << cam->getSharpness() << std::endl;
    std::cerr << "Contrast: " << cam->getContrast() << std::endl;
    std::cerr << "Saturation: " << cam->getSaturation() << std::endl;
    std::cerr << "Shutter Speed: " << cam->getShutterSpeed() << std::endl;
    std::cerr << "Exposure: " << cam->getExposure() << std::endl;
    std::cerr << "AWB: " << cam->getAWB() << std::endl;
    std::cerr << "AWBG_red: " << cam->getAWBG_red() << std::endl;
    std::cerr << "AWBG_blue: " << cam->getAWBG_blue() << std::endl;
    std::cerr << "Image Effect: " << cam->getImageEffect() << std::endl;
    std::cerr << "Metering: " << cam->getMetering() << std::endl;
    std::cerr << "HFlip: " << cam->isHorizontallyFlipped() << std::endl;
    std::cerr << "VFlip " << cam->isVerticallyFlipped() << std::endl;
    std::cerr << "Opened: " << cam->isOpened() << std::endl;
}

int mainLoop(processInterface *Cv, configContainer *configs) {
    raspicam::RaspiCam cam;

    // Setup interrupt handlers so all interfaces get closed
    Cv_quit = Cv;
    configs_quit = configs;
    cam_quit = &cam;
    signal(SIGINT, quit_handler);

    // set up camera interface
    cam.setISO(0);
    cam.setExposureCompensation(0);
    cam.setFormat(raspicam::RASPICAM_FORMAT_BGR); // FORMAT MUST BE BGR or colors will be reversed
    cam.setVideoStabilization(configs->videoStabilization);

    if (!cam.open()) {
        std::cerr << "Error opening camera" << std::endl;
        exit(1);
    }
    std::cerr << "Cam Opened, Sleeping... " << std::endl;
    sleep(2);
    std::cerr << "Awake, progressing... " << std::endl;

    unsigned char *data = new unsigned char[ cam.getImageBufferSize()];
    char tmp[BUF_LEN];
    int ndx;
    int ctr(1);
    std::string done = "Done";
    std::string img = "image";
    // capture 5 frames when a start signal is received
    while (true) {

        // TODO: Fix raspicam photo capture, its off color
        read(configs->cv_fd2, tmp, BUF_LEN);
        std::cerr << "Read msg from PNav: " << tmp << std::endl;
        sleep(1);
        if (!strcmp(tmp, "Start")) {

            // TODO: Figure out how to send a kill message from PNAV to CV
            for (ndx = 0; ndx < 5; ndx++) {
                cam.grab();
                cam.retrieve(data);
                saveImage(img + std::to_string(ctr) + "_" + std::to_string(ndx) + ".ppm", data, cam);
            }

            Cv->writePipe(configs->cv_fd, done);
            ctr++;
        } else if (!strcmp(tmp, "Exit")) {
            cam.release();
            Cv->cleanup(configs);
            exit(0);
        }
    }

    return 0;
}

int testLoop(processInterface *Cv, configContainer *configs) {
    // run CV process in test mode to continually take up to 2000 images

    raspicam::RaspiCam cam;
    unsigned char *data = new unsigned char[ cam.getImageBufferSize()];
    unsigned int nCaptures(0);
    float cap_freq = 1.0/3.0; // Hz
    std::string img = "image";

    // Setup interrupt handlers so all interfaces get closed
    Cv_quit = Cv;
    configs_quit = configs;
    cam_quit = &cam;
    signal(SIGINT, quit_handler);

    // set up camera interface
    cam.setISO(0);
    cam.setExposureCompensation(0);
    cam.setFormat(raspicam::RASPICAM_FORMAT_BGR); // FORMAT MUST BE BGR or colors will be reversed
    
    std::cerr << "Setting Video Stabilization: " << configs->videoStabilization << std::endl;
    cam.setVideoStabilization(configs->videoStabilization);

    if (!cam.open()) {
        std::cerr << "Error opening camera" << std::endl;
        exit(1);
    }
    std::cerr << "Cam Opened, Sleeping... " << std::endl;
    sleep(2);
    std::cerr << "Awake, progressing... " << std::endl;

    // start capturing upto MAX_IMGS images
    while (nCaptures < MAX_IMGS) {
        std::cerr << "In Loop, image: " << nCaptures << " @freq:" << 1/cap_freq << std::endl;
        cam.grab();
        cam.retrieve(data);
        saveImage(img + std::to_string(nCaptures) + ".ppm", data, cam);
        nCaptures++;
        sleep(1.0/cap_freq);
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

    if (configs.camTest) testLoop(&Cv, &configs);
    else mainLoop(&Cv, &configs);

    return 0;
}