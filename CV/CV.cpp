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

static void getPixels(cv::Point center) { //Multiply this by height
   std::cerr << (center.x - MID_WIDTH) * PIXEL_RATIO; //return pixel offset X coord * PIXEL_RATIO
   std::cerr << (center.y - MID_HEIGHT) * PIXEL_RATIO; //return pixel offset y coord * PIXEL_RATIO
}
/*
static void convertPixelsToMeters(Point center, double height, cv::Mat& image) {
  int xPixels, yPixels;
  Size imgSize = image.size();
  double GSD = (SENSOR_WIDTH * height) / (FOCAL_LENGTH * imgSize.width); //Calculate Ground Sampling Distance
  
  xPixels = center.x - s.width/2; //How far from center?
  yPixels = center.y - s.height/2;
  
  cout << GSD * xPixels <<endl; //x distance in meters
  cout << GSD * yPixels <<endl; //y distance in meters
}*/

static bool detectBall(const unsigned int& nCaptures, cv::Mat& image, cv::Mat& output, std::vector<cv::Vec3f>& circles) {
  //cv::Mat channels[3];
  bool drawCircles;
  bool found_ball = false;
  size_t i;
  clock_t t;


#ifdef DEBUG
  drawCircles = true;
#else
  drawCircles = false;
#endif

  t = clock();

  // Resize the image to a quarter of its original dimensions
  cv::resize(image, image, cv::Size(), 0.25, 0.25, cv::INTER_LINEAR);

  // Split the image into 3 channels: red, green, and blue
  //cv::split(image, channels);

  // Threshold the image, keeping frames that are closer to white (a value of 255)
  //cv::threshold(channels[RED], output, 155, 255, cv::THRESH_BINARY);
  cv::inRange(image, cv::Scalar(0,0,0), cv::Scalar(50,25,255), output);
  // Apply a Guassian Blur to smooth out the edges of the image
  cv::GaussianBlur(output, output, cv::Size(7, 7), 8, 8);
  // Use a Hough Transform to find the circles, and store their coordinates relative
  // to the frame in a 3-D vector formatted as (x, y, radius)
  cv::HoughCircles(output, circles, cv::HOUGH_GRADIENT, 1.0, output.rows / 4, 100, 10, 0, 0);

  t = clock() - t;

  fprintf(stderr, "\n\nImage %d took %f seconds.\n\n", nCaptures, ((float) t) / CLOCKS_PER_SEC);

  if (circles.size() > 0) {
    std::cerr << "Found Ball" << std::endl;
    found_ball = true;
  }

  // If we want to, draw the circles (mostly for debugging purposes)
  if (drawCircles) {

    for (i = 0; i < circles.size(); i++) {
      int radius;

      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      getPixels(center); //Prints pixels to cerr, Just multiply this by height 
      radius = cvRound(circles[i][2]);
      // Draw the circle center
      cv::circle(image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
      // Draw the circle outline
      cv::circle(image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
    }
  }
  return found_ball;
}

static bool grabFrame(raspicam::RaspiCam_Cv& cam, unsigned int& nCaptures, int& ctr, cv::Mat& image, cv::Mat& output, std::vector<cv::Vec3f>& circles) {
  int index;
  bool found_ball = false;
  std::string imageHeader;

  imageHeader = "image";

#ifdef TEST
  cam.grab();
  cam.retrieve(image);
  cv::cvtColor(image, image, cv::COLOR_RGB2BGR));
  detectBall(nCaptures, image, output, circles);

  cv::imwrite(imageHeader + std::to_string(ctr) + "_" + std::to_string(index) + ".jpg", image);

  ctr++;
  nCaptures++;
#else
  //    for (index = 0; index < 5; index++) {

  cam.grab();
  cam.retrieve(image);
  cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
  found_ball = detectBall(nCaptures, image, output, circles);

  cv::imwrite(imageHeader + std::to_string(ctr) + "_" + std::to_string(index) + ".jpg", image);

  nCaptures++;
  //    }
#endif
  
  return found_ball;
}

void frameLoop(unsigned int& nCaptures, processInterface *Cv, configContainer *configs) {
  bool nextFrame;
  char grabMsg[BUF_LEN];
  int ctr;

  raspicam::RaspiCam_Cv cam;
  cv::Mat image, output;
  std::vector<cv::Vec3f> circles;

  // Setup interrupt handlers so all interfaces get closed
  Cv_quit = Cv;
  configs_quit = configs;
  cam_quit = &cam;
  signal(SIGINT, quit_handler);

  // Setup camera interface
  setupCamera(cam, configs);

  nextFrame = true;
  ctr = 1;

  while (nextFrame) {

    read(configs->fd_PNav_to_CV, grabMsg, BUF_LEN);
    fprintf(stderr, "Read message from PNav: %s\n", grabMsg);
    sleep(1);

    if (!strcmp(grabMsg, START_STR)) {

      if (grabFrame(cam, nCaptures, ctr, image, output, circles)) {
        Cv->writePipe(configs->fd_CV_to_PNav, FOUND_STR);
      }

      Cv->writePipe(configs->fd_CV_to_PNav, DONE_STR);
      ctr++;
    } else if (!strcmp(grabMsg, EXIT_STR)) {

      cam.release();
      Cv->cleanup(configs);
      exit(EXIT_SUCCESS);
    }

    nextFrame = nCaptures > MAX_IMGS ? false : true;
  }
}

// Run CV process in test mode to continually take up to 2000 images

void testLoop(unsigned int& nCaptures, processInterface *Cv, configContainer *configs) {
  int ctr;

  raspicam::RaspiCam_Cv cam;
  cv::Mat image, output;
  std::vector<cv::Vec3f> circles;

  float cap_freq = configs->cap_Freq;

  // Setup interrupt handlers so all interfaces get closed
  Cv_quit = Cv;
  configs_quit = configs;
  cam_quit = &cam;
  signal(SIGINT, quit_handler);

  // set up camera interface
  setupCamera(cam, configs);

  ctr = 1;

  // start capturing upto MAX_IMGS images
  while (nCaptures < MAX_IMGS) {
    std::cerr << "In Loop, image: " << nCaptures << " @freq:" << 1 / cap_freq << "/s" << std::endl;

    grabFrame(cam, nCaptures, ctr, image, output, circles);

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
