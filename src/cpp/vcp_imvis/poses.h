#ifndef __VCP_IMVIS_POSES_H__
#define __VCP_IMVIS_POSES_H__

#include <vector>
#include <string>
#include <vcp_imutils/opencv_compatibility.h>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/imgproc.hpp>
#endif

namespace vcp
{
/** @brief Visualization utilities. */
namespace imvis
{
/** @brief Drawing poses. */
namespace poses
{
/** Supported pose estimation models. **/
enum class PoseType : unsigned char
{
  // Don't forget to update the python enum (vcp_imvis)
  NONE=0,
  COCO17, /**< Standard COCO keypoints. */
  COCO18, /**< COCO17 + neck (as used by OpenPose). */
  BODY25, /**< OpenPose 25 keypoints. */
  MPII15   /**< MPII body model. */
};

std::string PoseTypeToString(const PoseType &p);
PoseType PoseTypeFromString(const std::string &s);

/** Pairs of COCO model keypoints that make up the human skeleton. */
extern const std::vector<std::pair<size_t, size_t>> kCOCO17Skeleton;

/** Colors for COCO17 pose results. */
extern const std::vector<cv::Scalar> kCOCO17Colors;

/** Pairs of COCO (17+1) model keypoints that make up the human skeleton. */
extern const std::vector<std::pair<size_t, size_t>> kCOCO18Skeleton;

/** Colors for COCO (17+1) pose results. */
extern const std::vector<cv::Scalar> kCOCO18Colors;

/** Pairs of BODY_25 model keypoints that make up the human skeleton. */
extern const std::vector<std::pair<size_t, size_t>> kBODY25Skeleton;

/** Colors for BODY_25 pose results. */
extern const std::vector<cv::Scalar> kBODY25Colors;

/** Pairs of MPII_15 model keypoints that make up the human skeleton. */
extern const std::vector<std::pair<size_t, size_t>> kMPII15Skeleton;

/** Colors for MPII_15 pose results. */
extern const std::vector<cv::Scalar> kMPII15Colors;


struct PoseModel
{
  PoseType type;
  std::vector<cv::Vec2f> keypoints;
  std::vector<float> scores;

  /** @brief Convert the model's keypoints to pixel coordinates (truncates!). */
  std::vector<cv::Point> KeypointPixelCoords() const;
};

std::ostream& operator<<(std::ostream & os, const PoseModel &pm);

void DrawPose(const PoseModel &model, cv::Mat &image, const float score_threshold=0.1, const int keypoint_radius=5, const int keypoint_thickness=-1, const int skeleton_thickness=7, const bool draw_ellipses=true);
void DrawPoses(const std::vector<PoseModel> &models, cv::Mat &image, const float opacity=1.f, const float score_threshold=0.1, const int keypoint_radius=5, const int keypoint_thickness=-1, const int skeleton_thickness=7, const bool draw_ellipses=true);

} // namespace poses
} // namespace imvis
} // namespace vcp
#endif // __VCP_IMVIS_POSES_H__
