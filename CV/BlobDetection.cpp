// C++ standard libraries
#include <iostream>
#include <string>
#include <vector>
// OpenCV libraries
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
// Header files
#include "BlobDetection.h"

void openFile(cv::Mat& image, std::string& filename) {
	image = cv::imread(filename, cv::IMREAD_COLOR);
}

void BlobDetector::detect(cv::Mat& image, cv::Mat& output, std::vector<cv::KeyPoint>& keypoints) {
	cv::SimpleBlobDetector::Params params;
	cv::Mat blur[3];

	cv::split(image, blur);

	// Color parameters
	params.filterByColor = FILTER_BY_COLOR;
	params.blobColor = DEFAULT_COLOR;

	// Area paramters
	params.filterByArea = FILTER_BY_AREA;
	params.minArea = MIN_AREA;
	params.maxArea = MAX_AREA;

	// Circularity parameters
	params.filterByCircularity = FILTER_BY_CIRCULARITY;
	params.minConvexity = MIN_CONVEXITY;
	params.maxConvexity = MAX_CONVEXITY;

	// Inertia paramters
	params.filterByInertia = FILTER_BY_INTERTIA;
	params.minInertiaRatio = MIN_INERTIA_RATIO;
	params.maxInertiaRatio = MAX_INERTIA_RATIO;

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

	cv::medianBlur(blur[INDEX], blur[INDEX], BLUR_INDEX);

	detector->detect(blur[INDEX], keypoints);

	cv::drawKeypoints(blur[INDEX], keypoints, output, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}

int main() {
	std::string filename;
	cv::Mat image, output;

	filename = FILE_NAME;
	openFile(image, filename);

	if (image.empty()) {
		std::cout << "Image was not loaded properly. Exiting" << std::endl;
		exit(EXIT_FAILURE);
	}

	BlobDetector blob;
	std::vector<cv::KeyPoint> keypoints;

	blob.detect(image, output, keypoints);
	cv::imwrite(OUTPUT_FILE, output);

	exit (EXIT_SUCCESS);
}
