// C++ headers
#include <iostream>
#include <fstream>
#include <ctime>
#include <stdexcept>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

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
#include "offboard.hpp"
#include "CV.hpp"
#include "fileIO.hpp"

#define N_TEN_TO_SEVENTH 1E-7
#define N_TEN_TO_THIRD 1E-3
#define N_TEN_TO_SECOND 1E-2
#define TEN_TO_SEVENTH 1E7
#define TEN_TO_THIRD 1E3
#define TEN_TO_SECOND 1E2

#define BALL_DIAMETER 0.55
#define EARTH_RADIUS 63780000
#define DEGREE_180 180.0
#define RADIANS_270 4.71239
#define RADIANS_180 3.14159
#define RADIANS_90 1.5708

using namespace cv;

typedef struct {
    float lat;
    float lon;
    float height;
    float pitch;
    float roll;
    float yaw;
} GPS;

raspicam::RaspiCam_Cv *cam_quit;

void CV_call_stop() {
  cam_quit->release();
  exit(1);
}

/*
 * Helper function to convert Degrees to Radians
 */
double degreesToRadian(double deg) {
    return deg * M_PI / DEGREE_180;
}

/*
 * Helper function to convert Radians to Degrees
 */
double radiansToDegrees(double rad) {
    return rad * (DEGREE_180 / M_PI);
}

/*
 * Initializes the camera module with set properties
 */
static void setupCamera(raspicam::RaspiCam_Cv &cam, configContainer *configs) {
    cam.set(CV_CAP_PROP_FRAME_WIDTH, configs->cam_Width);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, configs->cam_Height);
    cam.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    cam.set(CV_CAP_PROP_BRIGHTNESS, BRIGHTNESS);
    cam.set(CV_CAP_PROP_CONTRAST, CONTRAST);
    cam.set(CV_CAP_PROP_SATURATION, SATURATION);
    cam.set(CV_CAP_PROP_GAIN, GAIN);

    if (!cam.open()) {
        fprintf(stderr, "\nCamera has not been opened. Exiting...\n");
        throw std::runtime_error("Camera failed to open.\n");
        exit(EXIT_FAILURE);
    }
    sleep(2);
}

/*
 * Takes an image and vectors (edges from HoughCircles algo) and draw those
 * vectors onto the image for debugging purposes
 */
static void drawCircles(cv::Mat *image, std::vector<cv::Vec3f> &circles) {
    for (int i = 0; i < circles.size(); i++) {
        int radius;
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        radius = cvRound(circles[i][2]);
        // Draw the circle center
        cv::circle(image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // Draw the circle outline
        cv::circle(image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
    }
}

/*
 * Calculates the GPS of the target relative to that of the drone
 * Uses data from Autopilot_Interface.current_messages.global_position_int and .attitude
 * 
 */
void calculateGPS(cv::Mat image, std::vector<Vec3f> &circles, GPS droneGPS, GPS *ballGPS) {
    double height, north, east, mpLat, mpLon, mpp, z, radius, adjX, adjY, angle, yawRads;
    int centerX, centerY;
    height = 7.62;
    radius = circles[0][2];
    mpp = BALL_DIAMETER / (radius * 2);
    centerX = image.cols / 2;
    centerY = image.rows / 2;
    adjX = circles[0][0] + height * tan(droneGPS.pitch) - centerX;
    adjY = centerY - circles[0][1] + height * tan(droneGPS.roll);
    z = sqrt(pow(adjX, 2) + pow(adjY, 2));
    angle = atan(adjX/adjY);
    yawRads = degreesToRadian(droneGPS.yaw);
    if (adjX >= 0 && adjY >= 0) {
        north = z * cos(yawRads + angle);
        east = z * sin(yawRads + angle);
    } else if (adjX > 0 && adjY < 0) {
        north = z * cos(RADIANS_90 - yawRads + angle);
        east = z * sin(RADIANS_90 - yawRads + angle);
    } else if (adjX < 0 && adjY > 0) {
        north = z * cos(RADIANS_180 - yawRads + angle);
        east = z * sin(RADIANS_180 - yawRads + angle);
    } else {
        north = z * cos(RADIANS_270 - yawRads + angle);
        east = z * sin(RADIANS_270 - yawRads + angle);
    }
    ballGPS->lat = droneGPS.lat  + ((north * mpp) / EARTH_RADIUS) * (DEGREE_180 / M_PI);
    ballGPS->lon = droneGPS.lon + ((east * mpp) / EARTH_RADIUS) * (DEGREE_180 / M_PI) / cos(droneGPS.lat * M_PI/DEGREE_180);
    
    #ifdef DEBUG
        printf("\tRadius: %f, centerX %d, centerY: %d, ballX: %f, ballY: %f\n", radius, centerX, centerY, circles[0][0], circles[0][1]);
        printf("\tAngle: %f\n", radiansToDegrees(angle));
        printf("\tadjX: %f [m], adjY: %f [m], z: %f [m]\n", adjX * mpp, adjY * mpp, z * mpp);
        printf("\tEast diff: %f [m], North diff: %f [m]\n", mpp*east, mpp*north);
        printf("\tballGPS Lat: %f, ballGPS lon: %f\n", ballGPS->lat, ballGPS->lon);
    #endif
}

/*
 * Applies Morphological operations onto a binary image for more accurate radius detection.
 * 1) Applies an Open (erode -> dilate) to remove any extraneous white pixels
 * 2) Applies a Close Operation (dilate -> erode) to fill in any gaps
 */
void morphingImage(cv::Mat input, cv::Mat *output) {
    Mat element, temp1, temp2;
    int morph_size = 3;

    element = cv::getStructuringElement(MORPH_ELLIPSE, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
    cv::morphologyEx(input, temp1, MORPH_OPEN, element, Point(-1,-1), 3);
    cv::morphologyEx(temp1, *output, MORPH_CLOSE, element, Point(-1,-1), 60);
}

/*
 * Runs the computer vision algorithms onto the input "image" matrix.
 * Any image resizing needs to happen to the input image before this function
 *    is call
 * Applies median blue -> converts image to HSV -> thresholds the upper & lower
 *    red hues -> Applies GaussianBlur to normalize colors -> Apply Morphologial Operations
 *    -> Apply Hough Circles Algorithm
 */
static bool runCV(int role, cv::Mat &image, cv::Mat &output, std::vector<cv::Vec3f> &circles) {
    Mat original, rgbImage, ogImage, hsvImage, lowerRed, upperRed, redImage, blurImage, morphImage, temp;
    bool drawCircles = false, found_ball = false;
    int minRadius = 0;
    size_t i;

    ogImage = image.clone();

    cv::medianBlur(image, image, 3);

    // Convert image to HSV
    cv::cvtColor(image, hsvImage, COLOR_BGR2HSV);

    // Threshold the image, keeping only red pixels: OpenCV red hue range 0-10
    // and 160-179
    cv::inRange(hsvImage, Scalar(0,100,100), Scalar(10, 255, 255), lowerRed);
    cv::inRange(hsvImage, Scalar(160,100,100), Scalar(179, 255, 255), upperRed);

    // Combine lower and upper red matrices
    cv::addWeighted(upperRed, 1.0, upperRed, 1.0, 0.0, redImage);

    // Apply guassian blur to red hue Image
    cv::GaussianBlur(redImage, blurImage, Size(9,9), 2, 2);

    //Apply Morphological operations
    morphingImage(blurImage, &morphImage);

    // Apply guassian blur again
    cv::GaussianBlur(morphImage, temp, Size(9,9), 2, 2);


    //Change min radius size if image is not resized
    if (role) {
        minRadius = 25;
    }
    // Apply Hough Transform to detect circles in redImage
    cv::HoughCircles(morphImage, circles, CV_HOUGH_GRADIENT, 1, morphImage.rows/8, 100, 20, minRadius, 200);

    if (circles.size() > 0) {
        #ifdef DEBUG
            drawCircles(image, circles);
        #endif
        std::cerr << "Found Ball" << std::endl;
        found_ball = true;
    }

    return found_ball;
}

/*
 * Runs the specified search mode based upon the PNav's role property
 * Resizes the quick search image before running the cv function
 * Detailed search locks the CeeToPee thread before beggining CV
 */
static bool findBall(int role, cv::Mat &image, cv::Mat &output, std::vector<cv::Vec3f> &circles) {
    bool foundBall = false;
    GPS droneGPS, ballGPS;
    ballGPS.pitch = 0;
    ballGPS.yaw = 0;
    ballGPS.roll = 0;
    ballGPS.height = 0;


    if (role == QUICK_SEARCH) {
        //Resize the image for faster processing
        cv::resize(image, image, cv::Size(), 0.25, 0.25, INTER_LINEAR);
        // Call CV function
        found_ball = runCV(role, image, output, circles);
        CeeToPee.set_ball_lat(PeeToCee.get_lat());
        CeeToPee.set_ball_lon(PeeToCee.get_lon());
        //Add GPS Calc and replace boolean return
    } else if (role == DEPTH_SEARCH) {
        //Lock thread access: Stops the PNav system from accessing the foundBall boolean
        CeeToPee.CV_lock();
        //Get current drone positional information
        droneGPS.lat = float(PeeToCee.get_lat()) * N_TEN_TO_SEVENTH;
        droneGPS.lon = float(PeeToCee.get_lon()) * N_TEN_TO_SEVENTH;
        droneGPS.height = float(PeeToCee.get_height()) * N_TEN_TO_THIRD;
        droneGPS.yaw = float(PeeToCee.get_yaw()) * N_TEN_TO_SECOND;
        droneGPS.pitch = PeeToCee.get_pitch();
        droneGPS.roll = PeeToCee.get_role();

        // Call CV function:
        found_ball = runCV(image, output, circles);
        //Add GPS Calc and replace boolean return
        calculateGPS(image, circles, droneGPS, &ballGPS);
        //Set Ball GPS information in CeeToPee
        CeeToPee.set_ball_lat(int(ballGPS.lat * TEN_TO_SEVENTH));
        CeeToPee.set_ball_lon(int(ballGPS.lon * TEN_TO_SEVENTH));
        //Unlock Thread access: PNav system can now access bool and continue on
        CeeToPee.CV_unlock();
        //Stop it from running
        PeeToCee.set_CV_start(false);
    }
    #ifdef DEBUG
        Print output image for debugging
        cv::imwrite("output" + std::to_string(ctr) + ".jpg",output);
    #endif
    return foundBall;
}

/*
 * Grabs the most current image from the Raspicam camera
 */
static bool grabFrame(raspicam::RaspiCam_Cv &cam, int &ctr, cv::Mat &image) {
    std::string imageHeader = "image";

    cam.grab();
    cam.retrieve(image);
    // Write the full size image to file
    
    cv::imwrite(imageHeader + std::to_string(ctr) + ".jpg", image);
}

void frameLoop(unsigned int &nCaptures, configContainer *configs) {
    bool nextFrame, CV_found;
    int ctr = 1;

    raspicam::RaspiCam_Cv cam;
    cv::Mat image, output;
    std::vector<cv::Vec3f> circles;
    std::cerr << "Before camera setup\n";
    // Setup camera interface
    cam_quit = &cam;
    setupCamera(cam, configs);
    std::cerr << "CV thread is made\n";

    nextFrame = true;
    while (nextFrame) {
        if (PeeToCee.CV_start()) {
            grabFrame(cam, ctr, image);
            if ((CV_found = findBall(PeeToCee.get_role(), image, output, circles))) {
                CeeToPee.set_CV_found(CV_found);
            }
            ctr++;
            nextFrame = nCaptures++ > MAX_IMGS ? false : true;
        }
    }
    cam.release();
}

/*
 * Run CV process in test mode to continually take up to 2000 images
 */
void testLoop(unsigned int &nCaptures, configContainer *configs) {
    int ctr;
    raspicam::RaspiCam_Cv cam;
    cv::Mat image, output;
    std::vector<cv::Vec3f> circles;

    float cap_freq = configs->cap_Freq;

    // set up camera interface
    setupCamera(cam, configs);

    ctr = 1;

    // start capturing upto MAX_IMGS images
    while (nCaptures < MAX_IMGS && PeeToCee.CV_start()) {
        std::cerr << "In Loop, image: " << nCaptures << " @freq:" << 1 / cap_freq << "/s" << std::endl;
        grabFrame(cam, nCaptures, ctr, image, output, circles);
        sleep(1.0 / cap_freq);
    }
    cam.release();
}

