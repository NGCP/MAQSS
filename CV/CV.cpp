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

raspicam::RaspiCam_Cv *cam_quit;

void CV_call_stop() {
  cam_quit->release();
  exit(1);
}

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

static bool detectBall(const unsigned int &nCaptures, cv::Mat &image, cv::Mat &output, std::vector<cv::Vec3f> &circles) {
    cv::Mat channels[3], bw;
    bool drawCircles, found_ball = false;
    size_t i;
    //clock_t t;

    //t = clock();

    //Threshold the image, keeping only red pixels
    Mat lowerRed, upperRed;
    cv::inRange(hsvImage, cv::Scalar(0,100,100), cv::Scalar(10, 255, 255), lowerRed);
    cv::inRange(hsvImage, cv::Scalar(160,100,100), cv::Scalar(179, 255, 255), upperRed);

    //Combine lower and upper red matrices
    cv::addWeighted(lowerRed, 1.0, upperRed, 1.0, 0.0, output);

    //Apply guassian blur to red hue Image
    cv::GaussianBlur(output, output, cv::Size(9,9), 2, 2);

    //Apply Hough Transform to detect circles in redImage
    cv::HoughCircles(output, circles, cv::CV_HOUGH_GRADIENT, 1, output.rows/8, 100, 20, 0, 0);

    //t = clock() - t;

    //fprintf(stderr, "\n\nImage %d took %f seconds.\n\n", nCaptures, ((float) t) / CLOCKS_PER_SEC);

    if (circles.size() > 0) {
        std::cerr << "Found Ball" << std::endl;
        found_ball = true;
    }

    //#ifdef DEBUG
        //drawCircles(input, circles)
    //#endif
    return found_ball;
}

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

static bool grabFrame(raspicam::RaspiCam_Cv &cam, unsigned int &nCaptures, int &ctr, cv::Mat &image, cv::Mat &output, std::vector<cv::Vec3f> &circles) {
    int role = 0;
    bool found_ball = false;
    std::string imageHeader = "image";

    cam.grab();
    cam.retrieve(image);

    // Reduce image noise
    cv::medianBlur(image, image, 3);

    // Convert image to HSV color space from RGB
    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);

    // Write the full size image to file
    cv::imwrite(imageHeader + std::to_string(ctr) + ".jpg", image);

    // Get what type of search the drone is running from the PNav thread
    role = PeeToCee.get_role();

    if (role == QUICK_SEARCH) {
        found_ball = detectBall(nCaptures, image, output, circles);
    }
    else if (role == DEPTH_SEARCH) {

        CeeToPee.CV_lock();
        found_ball = detectBall(nCaptures, image, output, circles);
        CeeToPee.CV_unlock();
        //Stop it from running
        PeeToCee.set_CV_start(false);
    }
    //#ifdef DEBUG
        //Print output image for debugging
        //cv::imwrite("output" + std::to_string(ctr) + ".jpg",output);
    //#endif
    return found_ball;
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
            if (grabFrame(cam, nCaptures, ctr, image, output, circles)) {
                CeeToPee.set_CV_found(CV_found);
            }
            ctr++;
            nextFrame = nCaptures++ > MAX_IMGS ? false : true;
        }
    }
    cam.release();
}

// Run CV process in test mode to continually take up to 2000 images
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

/*
static bool detailedSearch(cv::Mat &image, cv::Mat &output, std::vector<cv::Vec3f> circles)
{
  cv::Mat channels[3];
  cv::Mat stats;
  cv::Mat centroids;
  cv::Mat bw;
  cv::Mat labels;
  cv::Point imCenter;
  std::vector<cv::Point> cc_centers;
  std::vector<cv::Vec3f> hough_centers;
  std::vector<cv::Point> centers;
  int nLabels;
  bool drawCircles;
  bool found_ball = false;

//#ifdef DEBUG
  drawCircles = true;
//#else
//  drawCircles = false;
//#endif

  // Split the image into the three seperate color channels.
  cv::split(image, channels);
  // Create a single channel Mat object with every pixel having the value 255;
  // this allows us to subtract the chosen channel and get the negative image of it.
  //bw = cv::Mat(image.rows, image.cols, channels[BLUE].type(), cv::Scalar(1, 1, 1) * 255);
  // Create our negative of the wanted channel.
  //cv::subtract(bw, channels[BLUE], bw);
  // Threshold the image, keeping frames that are closer to white (a value of 255)
  cv::threshold(channels[RED], bw, 200, 255, cv::THRESH_BINARY);
  // Use a median blur to remove any binary noise
  cv::medianBlur(bw, bw, 15);
  // Create a Mat object that is a single channel and 32-bit for creating labels;
  // these labels will represent objects found by the connected components algorithm.
  labels = cv::Mat(image.size(), CV_32S);
  // Run the connected component algorithm. This will find groups of pixels that can
  // be classified as belonging to the same object. A few useful things this lets learn
  // is the rough area of the found object and the center of the object.
  // The function returns the number of labeled areas, and we store that for later use.
  // Labels for an object start at 1, since objects starting at 0 would just be considered
  // background pixels.
  nLabels = cv::connectedComponentsWithStats(bw, labels, stats, centroids, 8);
  // Determine where the center of the image is, to calculate the offset of the found object.
  imCenter = cv::Point(image.cols / 2, image.rows / 2);
  // Loop through the different labeled objects, checking the areas and finding the centers.
  for (int i = 1; i < nLabels; i++)
  {
    int area;
    // First check the area of the object.
    area = stats.at<int>(i, cv::CC_STAT_AREA);
    std::cerr << "Area of label " << i << " :" << area << std::endl;
    // We have a rought idea of what the area of the object is, so check within
    // those limits.
    if (area > CC_MIN_AREA && area < CC_MAX_AREA)
    {
      cv::Point center(cvRound(centroids.at<double>(i, 0)), cvRound(centroids.at<double>(i, 1)));
	  centers.push_back(center);
    }
  }
  // Perform a Hough Circle Transform to detect circles within the image.
  cv::HoughCircles(bw, hough_centers, cv::HOUGH_GRADIENT, 2.0, image.rows / 4, 100, 10, 20, 100);
  // Loop through the center points for the circles we found.
  for (unsigned int i = 0; i < hough_centers.size(); i++)
  {

    cv::Point center(cvRound(hough_centers[i][0]), cvRound(hough_centers[i][1]));
    // Loop through the labeled points we stored, checking to see if they are
    // roughly equivalent to our circle center points.
    for (unsigned int j = 0; j < centers.size(); j++)
    {
      cv::Point check = centers.at(j);

      if (center.x > check.x - POINT_TOLERANCE && center.x < check.x + POINT_TOLERANCE && center.y > check.y - POINT_TOLERANCE && center.y < check.y + POINT_TOLERANCE)
      {
        // Add the point to our output vector.
        circles.push_back(hough_centers.at(i));
        // Find the offset from the center of the image.
        cv::Point offset = cv::Point(imCenter.x - center.x, imCenter.y - center.y);
        // Add to the vector
        //circles.push_back(offset);
        found_ball = true;
        // Draw a circle for debugging purposes if we want to.
        if (drawCircles)
        {
          int radius;

          radius = cvRound(circles[i][2]);
          // Draw the circle center
          cv::circle(image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
          // Draw the circle outline
          cv::circle(image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
        }
      }
    }
  }
  if (circles.size())
    found_ball = true;

  return found_ball;
}
*/

/*
static void convertPixelsToMeters(Point center, double height, cv::Mat& image) {
  int xPixels, yPixels;
  Size imgSize = image.size();
  double GSD = (SENSOR_WIDTH * height) / (FOCAL_LENGTH * imgSize.width); //Calculate Ground Sampling Distance

  xPixels = center.x - s.width/2; //How far from center?
  yPixels = center.y - s.width/2;

  cout << GSD * xPixels <<endl; //x distance in meters
  cout << GSD * yPixels <<endl; //y distance in meters
}*/
