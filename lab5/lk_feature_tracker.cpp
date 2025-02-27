#include "lk_feature_tracker.h"

#include <numeric>
#include <vector>

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>

using namespace cv;
using namespace cv::xfeatures2d;

/**
   LK feature tracker Constructor.
*/
LKFeatureTracker::LKFeatureTracker() {
  // Feel free to modify if you want!
  cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
}

LKFeatureTracker::~LKFeatureTracker() {
  // Feel free to modify if you want!
  cv::destroyWindow(window_name_);
}

/** TODO This is the main tracking function, given two images, it detects,
 * describes and matches features.
 * We will be modifying this function incrementally to plot different figures
 * and compute different statistics.
 @param[in] frame Current image frame
*/
void LKFeatureTracker::trackFeatures(const cv::Mat& frame) {

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 7 | Feature Tracking: Lucas-Kanade Tracker
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For this part, you will need to:
  //
  //   1. Using OpenCV’s documentation and the C++ API for the LK tracker, track
  //   features for the video sequences we provided you by using the Harris
  //   corner detector (like here). Show the feature tracks at a given frame
  //   extracted when using the Harris corners (consider using the 'show'
  //   function below)
  //
  //   Hint 1: take a look at cv::goodFeaturesToTrack and cv::calcOpticalFlowPyrLK
  //
  //   2. Add an extra entry in the table you made previously for the Harris +
  //   LK tracker
  //
  //   Note: LKFeatureTracker does not inherit from the base tracker like other
  //   feature trackers, so you need to also implement the statistics gathering
  //   code right here.
  //
  // ~~~~ begin solution
  cv::Mat prev_frame;
  if (frame.channels() == 3){
      cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
  } else {
      gray_frame = frame.clone();
  }

  if (prev_frame.empty() || prev_corners_.empty()) {
    ROS_WARN("Previous frame is empty or previous corners are empty, reinitializing!");
    cv::goodFeaturesToTrack(pray_frame, prev_corners_, 100, 0.01, 10, cv::Mat(), 3, false, 0.04);
    prev_frame_ = gray_frame.clone();
    return;
  }

  std::vector<cv::Point2f> curr_corners;
  std::vector<uchar> status;
  std::vector<float> err;

  try{
    cv::calcOpticalFlowPyrLK(prev_frame_, gray_frame, prev_corners_, curr_corners, status, err, 
                              cv::Size(21, 21), 3, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), 0, 0.001);
  }catch (const cv::Exception& e){
    ROS_ERROR("Optical flow calculation failure: %s", e.what());
    return;
  }
  

  std::vector<cv::Point2f> good_prev, good_curr;
  for (int i = 0; i < status.size(); ++i) {
    if (status[i]) {
      good_prev.push_back(prev_corners_[i]);
      good_curr.push_back(curr_corners[i]);
    }
  }

  if (good_curr.size() < 10) {
    ROS_WARN("Not enough good points to track, reinitializing!");
    cv::goodFeaturesToTrack(gray_frame, prev_corners_, 100, 0.01, 10, cv::Mat(), 3, false, 0.04);
    prev_frame_ = gray_frame.clone();
    return;
  }

  // Update statistics
  float const new_num_samples = static_cast<float>(num_samples_) + 1.0f;
  float const old_num_samples = static_cast<float>(num_samples_);
  avg_num_keypoints_img1_ = (avg_num_keypoints_img1_ * old_num_samples + static_cast<float>(good_corners_.size())) / new_num_samples;
  avg_num_keypoints_img2_ = (avg_num_keypoints_img2_ * old_num_samples + static_cast<float>(good_prev.size())) / new_num_samples;
  avg_num_matches_ = (avg_num_matches_ * old_num_samples + static_cast<float>(good_prev.size())) / new_num_samples;

  // Compute inliers using RANSAC
  std::vector<uchar> inlier_mask;
  inlierMaskComputation(good_prev, good_curr, &inlier_mask);
  int num_inliers = std::count(inlier_mask.begin(), inlier_mask.end(), 1);
  avg_num_inliers_ = (avg_num_inliers_ * old_num_samples + static_cast<float>(num_inliers)) / new_num_samples;

  // Update the number of samples
  ++num_samples_;

  // Calculate inlier ratio
  if (good_prev.size() > 0) {
    avg_inlier_ratio_ = (avg_inlier_ratio_ * old_num_samples + static_cast<float>(num_inliers) / static_cast<float>(good_prev.size())) / new_num_samples;
  }

  show(frame, prev_valid, curr_valid);

  prev_corners = good_curr;
  prev_frame = gray_frame.clone();
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 7
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}

/** TODO Display image with tracked features from prev to curr on the image
 * corresponding to 'frame'
 * @param[in] frame The current image frame, to draw the feature track on
 * @param[in] prev The previous set of keypoints
 * @param[in] curr The set of keypoints for the current frame
 */
void LKFeatureTracker::show(const cv::Mat& frame, std::vector<cv::Point2f>& prev,
                            std::vector<cv::Point2f>& curr) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // ~~~~ begin solution
  // Clone the frame to draw on it
  cv::Mat display_frame = frame.clone();

  // Draw lines between the matched keypoints
  for(size_t i = 0; i < prev.size(); ++i) {
    cv::circle(display_frame, curr[i], 1, cv::Scalar(0, 255, 0), 1.5, cv::LINE_8, 0);
    cv::line(display_frame, prev[i], curr[i], cv::Scalar(0, 255, 0), 1, cv::LINE_8, 0);
  }

  cv::imshow(window_name_, display_frame);
  cv::waitKey(10);
  //     Hint: look at cv::line and cv::cirle functions.
  //     Hint 2: use imshow to display the image
  //
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

/** Compute Inlier Mask out of the given matched keypoints.
 @param[in] pts1 List of keypoints detected on the first image.
 @param[in] pts2 List of keypoints detected on the second image.
 @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
*/
void LKFeatureTracker::inlierMaskComputation(const std::vector<cv::Point2f>& pts1,
                                             const std::vector<cv::Point2f>& pts2,
                                             std::vector<uchar>* inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, cv::FM_RANSAC,
                       max_dist_from_epi_line_in_px, confidence_prob,
                       *inlier_mask);
  } catch(...) {
    ROS_WARN("Inlier Mask could not be computed, this can happen if there"
             "are not enough features tracked.");
  }
}

void LKFeatureTracker::lk_printStats() const {
  std::cout << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_ << std::endl;
  std::cout << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_ << std::endl;
  std::cout << "Avg. Number of matches: " << avg_num_matches_ << std::endl;
  std::cout << "Avg. Number of good matches: NA" << std::endl;
  std::cout << "Avg. Number of Inliers: " << avg_num_inliers_ << std::endl;
  std::cout << "Avg. Inliers ratio: " << avg_inlier_ratio_ << std::endl;
  std::cout << "Num. of samples: " << num_samples_ << std::endl;
}
