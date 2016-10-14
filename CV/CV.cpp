// C++ headers
#include <iostream>
#include <fstream>
#include <ctime>
#include <stdexcept>
#include <string>
#include <vector>

// C headers
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>

// Raspicam headers
#include <raspicam/raspicam_cv.h>

// OpenCV headers
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

// Headers
#include "CV.hpp"
#include "fileIO.hpp"
#include "processInterface.hpp"

processInterface *Cv_quit;
configContainer *configs_quit;
raspicam::RaspiCam_Cv *cam_quit;


void quit_handler(int signal) {

    printf("\nTERMINATING AT USER REQUEST\n\n");

    // Close camera interface
    cam_quit->release();
    // Close pipes
    Cv_quit->cleanup(configs_quit);
    // End the program
    exit(EXIT_SUCCESS);
}

static void setupCamera(raspicam::RaspiCam_Cv& cam, configContainer *configs) {

    cam.set(cv::CV_CAP_PROP_FRAME_WIDTH, configs->cam_Width);
    cam.set(cv::CV_CAP_PROP_FRAME_HEIGHT, configs->cam_Height);
    cam.set(cv::CV_CAP_PROP_FORMAT, cv::CV_8UC3);
    cam.set(cv::CV_CAP_PROP_BRIGHTNESS, BRIGHTNESS);
    cam.set(cv::CV_CAP_PROP_CONTRAST, CONTRAST);
    cam.set(cv::CV_CAP_PROP_SATURATION, SATURATION);
    cam.set(cv::CV_CAP_PROP_GAIN, GAIN);

    if (!cam.open()) {
        fprintf(stderr, "\nCamera has not been opened. Exiting...\n");
	throw std::runtime_error("Camera failed to open.\n");
	exit(EXIT_FAILURE);
    }

    sleep(2);
}

static void detectBall(cv::Mat& image, cv::Mat& output, std::vector<cv::Vecsf>& circles) {
    cv::Mat channels[3];
    bool drawCircles;
    size_t i;

    #ifdef DEBUG
        drawCircles = true;
    #elif
        drawCircles = false;
    #endif

    // Split the image into 3 channels: red, green, and blue
    cv::split(image, channels);

    // Threshold the image, keeping frames that are closer to white (a value of 255)
    cv::threshold(channels[0], output, 155, 255, cv::THRESH_BINARY);
    // Apply a Guassian Blur to smooth out the edges of the image
    cv::GaussianBlur(output, output, cv::Size(7, 7,), 8, 8);
    // Use a Hough Transform to find the circles, and store their coordinates relative
    // to the frame in a 3-D vector formatted as (x, y, radius)
    cv::HoughCircles(output, circles, cv::HOUGH_GRADIENT, 1.0, output.rows/4, 100, 10, 0, 0);

    // If we want to, draw the circles (mostly for debugging purposes)
    if (drawCircles) {

	for (i = 0; i < circles.size(); i++) {
            int radius;

	    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	    radius = cvRound(circles[i][2]);
	    // Draw the circle center
	    cv::circle(image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
	    // Draw the circle outline
	    cv::circle(image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
	}
    }
}

static void grabFrame(raspicam::RaspiCam_Cv& cam, unsigned int& nCaputres, cv::Mat& image, cv::Mat& output, std::vector<cv::Vec3f>& circles) {
    int i;
    std::string imageHeader;

    
    imageHeader = "image";

    for (i = 0; i < 5; i++) {
        
	cam.grab();
	cam.retrieve(image);

        detectBall(image, output, circles, drawCircles);

	cv::imwrite(str + std::to_string(ctr) + "_" + std::to_string(ndx) + ".jpg", img);
        nCaptures++;
    }
}

void frameLoop(unsigned int& nCaptures, processInterface *Cv, configContainer *configs) {
    bool grabFrame;
    char grabMsg[BUF_LEN];
    int ctr;
    int message;

    raspicam::RaspiCam_Cv cam;
    cv::Mat image, output;
    std::vector<cv::Vec3f> circles;

    // Setup interrupt handlers so all interfaces get closed
    Cv_quit = Cv;
    configs_quit = configs;
    cam_quit = &cam;
    signal(SIGINT, quit_handler);

    // Setup camera interface
    setupCamera(cam, &configs);

    grabFrame = true;
    ctr = 1;

    while (grabFrame) {
       
        read(configs->fd_PNav_to_CV, grabMsg, BUF_LEN);
        fprintf(stderr, "Read message from PNav: %s\n", grabMsg);
        sleep(1);

        message

        if (!strcmp(grabMsg, START_STR)) {

            grabFrame(cam, image, output, circles);
            Cv->writePipe(configs->fd_CV_to_PNav, DONE_STR);
            ctr++;
        }
	else if (!strcmp(getMsg, EXIT_STR)) {
            
	    can.release();
	    Cv->cleanup(configs);
	    exit(EXIT_SUCCESS);
	}
        
	grabframe = nCaptures > MAX_IMGS ? false : true;
    }
}

// Run CV process in test mode to continually take up to 2000 images
void testLoop(processInterface *Cv, configContainer *configs) {

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

    // set up camera interfac
    setupCamera(cam, &configs);


    // start capturing upto MAX_IMGS images
    while (nCaptures < MAX_IMGS) {
        std::cerr << "In Loop, image: " << nCaptures << " @freq:" << 1 / cap_freq << "/s" << std::endl;
        cam.grab();
        cam.retrieve(img);
        cv::imwrite(str + std::to_string(nCaptures) + ".jpg", img);
        nCaptures++;
        sleep(1.0 / cap_freq);
    }
}

int main(int argc, char** argv) {
    configContainer configs;
    unsigned int nCaptures;

    if (argc > 1)
        configs = fileIO::getConfig(argc, argv);
    else {
        // TODO: Add a print help function and figure out where to put it
        std::cerr << "Must specify command line arguments" << std::endl;
        exit(EXIT_FAILURE);
	// TODO: Perform closeout / clean up function when exiting
    }

    processInterface Cv(&configs, CV);
    fileIO::printConfig(&configs);
    nCaptures = 0;

    if (configs.cam_Test)
        testLoop(nCaptures, &Cv, &configs);
    else
        frameLoop(nCaptures, &Cv, &configs);

    exit(EXIT_SUCCESS);
}
