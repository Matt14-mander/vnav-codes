/*
 * @file self_flow.cpp
 * Feel free to edit any part of this file!
 */
#include <memory>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

/** imageCallback This function is called when a new image is published. */
void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    // Convert ROS msg type to OpenCV image type.
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  DELIVERABLE 8 | Optical Flow
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // LK tracker estimates the optical flow for sparse points in the image.
    // Alternatively, dense approaches try to estimate the optical flow for the whole image.
    // Try to calculate your own optical flow using Farneback’s algorithm (see OpenCV documentation).
    //
    // ~~~~ begin solution
    if (image.empty()) {
      ROS_ERROR("Empty image");
      return;
    }

    cv::Mat curr_gray;
    cv::cvtColor(image, curr_gray, cv::COLOR_BGR2GRAY);
    cv::Mat prev_gray;

    if(!prev_gray.empty()) {
      cv::Mat flow;
      cv::calcOpticalFlowFarneback(prev_gray, curr_gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

      for (int y = 0; y < flow.rows; y += 10) {
        for (int x = 0; x < flow.cols; x += 10) {
          cv::Point2f flow_at_point = flow.at<cv::Point2f>(y, x);
          cv::Point start_point(x, y);
          cv::Point end_point(cvRound(x + flow_at_point.x), cvRound(y + flow_at_point.y));
          cv::arrowLine(image, start_point, end_point, cv::Scalar(0, 255, 0), 1, 8, 0, 0.4);

          // cv::line(image, cv::Point(x, y), cv::Point(x + flow_at_point.x, y + flow_at_point.y), cv::Scalar(0, 255, 0));
          // cv::circle(image, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1);

      // cv::Mat hsv(flow.size(), CV_8UC3);
      // for(int y = 0; y < flow.rows; y++) {
      //   for(int x = 0; x < flow.cols; x++) {
      //     float fx = flow.at<cv::Point2f>(y, x).x;
      //     float fy = flow.at<cv::Point2f>(y, x).y;
      //     float angle = atan2(fy, fx) * 180 / CV_PI;
      //     float magnitude = sqrt(fx * fx + fy * fy);
      //     hsv.at<cv::Vec3b>(y, x) = cv::Vec3b(
      //       static_cast<unsigned char>(angle*180/CV_PI/2), 
      //       255, 
      //       static_cast<unsigned char>(std::min(magnitude*10, 255.0f)));
        }
      }

      // cv::Mat bgr;
      // cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

      if (!bgr.empty()){
        cv::imshow("view", bgr);
      }
    }

    prev_gray = curr_gray.clone();
    
    cv::waitKey(1);

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

/**
 * @function main
 * @brief Main function
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "optical_flow");
  ros::NodeHandle local_nh("~");

  cv::namedWindow("view", cv::WINDOW_NORMAL);
  image_transport::ImageTransport it(local_nh);
  image_transport::Subscriber sub = it.subscribe("/tesse/left_cam/rgb/image_raw", 100, imageCallback);

  ros::spin();
  cv::destroyWindow("view");
  return EXIT_SUCCESS;
}
