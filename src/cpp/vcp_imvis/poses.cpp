#include "poses.h"
#include "drawing.h"
//#include <vcp_imutils/imutils.h>
//#include <vcp_imutils/matutils.h>
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/string_utils.h>
//#include <vcp_math/geometry3d.h>
//#include <vcp_math/geometry2d.h>
#include <vcp_math/conversions.h>
#include <vcp_math/geometry2d.h>
#include <vcp_math/common.h>

//#include <limits>
#include <opencv2/highgui.hpp>
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/imgproc.hpp>
#endif

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::imvis::poses"
namespace vcp
{
namespace imvis
{
namespace poses
{
//const std::vector<std::string> kCOCO17Keypoints = {
//  "nose",
//  "left_eye",
//  "right_eye",
//  "left_ear",
//  "right_ear",
//  "left_shoulder",
//  "right_shoulder",
//  "left_elbow",
//  "right_elbow",
//  "left_wrist",
//  "right_wrist",
//  "left_hip",
//  "right_hip",
//  "left_knee",
//  "right_knee",
//  "left_ankle",
//  "right_ankle"
//};

std::string PoseTypeToString(const PoseType &p)
{
  switch(p)
  {
    case PoseType::NONE:
      return "unknown";

    case PoseType::COCO17:
      return "COCO17";

    case PoseType::COCO18:
      return "COCO18";

    case PoseType::BODY25:
      return "BODY25";

    case PoseType::MPII15:
      return "MPII15";
  }

  VCP_ERROR("Unmapped PoseType in PoseTypeToString()");
}

PoseType PoseTypeFromString(const std::string &s)
{
  const std::string lower = vcp::utils::string::Lower(s);

  if (lower.compare("unknown") == 0)
    return PoseType::NONE;
  if (lower.compare("coco17") == 0)
    return PoseType::COCO17;
  if (lower.compare("coco18") == 0)
    return PoseType::COCO18;
  if (lower.compare("body25") == 0)
    return PoseType::BODY25;
  if (lower.compare("mpii15") == 0)
    return PoseType::MPII15;

  VCP_ERROR("PoseType '" << s << "' not yet mapped.");
}


const std::vector<std::pair<size_t, size_t>> kCOCO17Skeleton = {
//HRNET//  {15, 13}, {13, 11}, {11, 5}, {12, 14}, {14, 16}, {12, 6},
//  {3, 1},{1, 2},{1, 0},{0, 2},{2,4}, {9, 7}, {7,5}, {5, 6}, {6, 8}, {8, 10}
//COCOAPI  {15, 13}, {13, 11}, {16, 14}, {14, 12}, {11, 12}, {5, 11},
  //{6, 12}, {5, 6}, {5, 7}, {6, 8}, {7, 9}, {8, 10}, {1, 2},
  //{0, 1}, {0, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}
// COCOAPI manually sorted
  {13, 15}, {11, 13}, {14, 16}, {12, 14}, {11, 12}, {5, 11},
    {6, 12}, {5, 6}, {5, 7}, {6, 8}, {7, 9}, {8, 10}, {1, 2},
    {0, 1}, {0, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}
};

const std::vector<std::pair<size_t, size_t>> kCOCO18Skeleton = {
  {1, 2}, {1, 5}, {2, 3}, {3, 4}, {5, 6},
  {6, 7}, {1, 8}, {8, 9}, {9, 10}, {1, 11},
  {11, 12}, {12, 13}, {1, 0}, {0, 14},
  {14, 16}, {0, 15}, {15, 17}
};

const std::vector<std::pair<size_t, size_t>> kBODY25Skeleton = {
  {1,8}, {1,2}, {1,5}, {2,3}, {3,4}, {5,6}, {6,7}, {8,9},
  {9,10}, {10,11}, {8,12}, {12,13}, {13,14}, {1,0}, {0,15},
  {15,17}, {0,16}, {16,18}, {14,19}, {19,20}, {14,21}, {11,22},
  {22,23}, {11,24}
};

const std::vector<std::pair<size_t, size_t>> kMPII15Skeleton = {
  {0,1}, {1,2}, {2,3}, {3,4}, {1,5}, {5,6}, {6,7}, {1,14},
  {14,8}, {8,9}, {9,10}, {14,11}, {11,12}, {12,13}
};

const std::vector<cv::Scalar> kCOCO17Colors = {
  cv::Scalar(255, 0, 85), cv::Scalar(255, 0, 0), cv::Scalar(255, 85, 0),
  cv::Scalar(255, 170, 0), cv::Scalar(255, 255, 0), cv::Scalar(170, 255, 0),
  cv::Scalar(85, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 85),
  cv::Scalar(0, 255, 170), cv::Scalar(0, 255, 255), cv::Scalar(0, 170, 255),
  cv::Scalar(0, 85, 255), cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 170),
  cv::Scalar(170.f, 0.f,   255.f), cv::Scalar(255.f, 0.f,   255.f), cv::Scalar( 85.f, 0.f,   255.f)
//  cv::Scalar(252,176,243), cv::Scalar(252,176,243), cv::Scalar(252,176,243),
//  cv::Scalar(0,176,240), cv::Scalar(0,176,240), cv::Scalar(0,176,240),
//  cv::Scalar(240,2,127),cv::Scalar(240,2,127), cv::Scalar(240,2,127),
//  cv::Scalar(240,2,127), cv::Scalar(240,2,127), cv::Scalar(255,255,0),
//  cv::Scalar(255,255,0), cv::Scalar(169, 209, 142), cv::Scalar(169, 209, 142),
//  cv::Scalar(169, 209, 142),
//  cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0)
};

const std::vector<cv::Scalar> kCOCO18Colors = {
  cv::Scalar(255,     0,    85), cv::Scalar(255,     0,     0), cv::Scalar(255,    85,     0),
  cv::Scalar(255,   170,     0), cv::Scalar(255,   255,     0), cv::Scalar(170,   255,     0),
  cv::Scalar( 85,   255,     0), cv::Scalar(  0,   255,     0), cv::Scalar(  0,   255,    85),
  cv::Scalar(  0,   255,   170), cv::Scalar(  0,   255,   255), cv::Scalar(  0,   170,   255),
  cv::Scalar(  0,    85,   255), cv::Scalar(  0,     0,   255), cv::Scalar(255,     0,   170),
  cv::Scalar(170,     0,   255), cv::Scalar(255,     0,   255), cv::Scalar( 85,     0,   255)
};

const std::vector<cv::Scalar> kBODY25Colors = {
  cv::Scalar(255,     0,    85), cv::Scalar(255,     0,     0), cv::Scalar(255,    85,     0),
  cv::Scalar(255,   170,     0), cv::Scalar(255,   255,     0), cv::Scalar(170,   255,     0),
  cv::Scalar( 85,   255,     0), cv::Scalar(  0,   255,     0), cv::Scalar(255,     0,     0),
  cv::Scalar(  0,   255,    85), cv::Scalar(  0,   255,   170), cv::Scalar(  0,   255,   255),
  cv::Scalar(  0,   170,   255), cv::Scalar(  0,    85,   255), cv::Scalar(  0,     0,   255),
  cv::Scalar(255,     0,   170), cv::Scalar(170,     0,   255), cv::Scalar(255,     0,   255),
  cv::Scalar( 85,     0,   255), cv::Scalar(  0,     0,   255), cv::Scalar(  0,     0,   255),
  cv::Scalar(  0,     0,   255), cv::Scalar(  0,   255,   255), cv::Scalar(  0,   255,   255),
  cv::Scalar(  0,   255,   255)
};

const std::vector<cv::Scalar> kMPII15Colors = {
  cv::Scalar(255,     0,    85), cv::Scalar(255,     0,     0), cv::Scalar(255,    85,     0),
  cv::Scalar(255,   170,     0), cv::Scalar(255,   255,     0), cv::Scalar(170,   255,     0),
  cv::Scalar( 85,   255,     0), cv::Scalar( 43,   255,     0), cv::Scalar(  0,   255,     0),
  cv::Scalar(  0,   255,    85), cv::Scalar(  0,   255,   170), cv::Scalar(  0,   255,   255),
  cv::Scalar(  0,   170,   255), cv::Scalar(  0,    85,   255), cv::Scalar(  0,     0,   255)
};

inline const std::vector<std::pair<size_t, size_t>> &GetSkeleton(PoseType pt)
{
  switch (pt)
  {
    case PoseType::COCO17:
      return kCOCO17Skeleton;

    case PoseType::COCO18:
      return kCOCO18Skeleton;

    case PoseType::BODY25:
      return kBODY25Skeleton;

    case PoseType::MPII15:
      return kMPII15Skeleton;

    default:
      VCP_ERROR("Pose type not yet supported.");
  }
}


inline const std::vector<cv::Scalar> &GetModelColors(PoseType pt)
{
  switch (pt)
  {
    case PoseType::COCO17:
      return kCOCO17Colors;

    case PoseType::COCO18:
      return kCOCO18Colors;

    case PoseType::BODY25:
      return kBODY25Colors;

    case PoseType::MPII15:
      return kMPII15Colors;

    default:
      VCP_ERROR("Pose type not yet supported.");
  }
}



std::vector<cv::Point> PoseModel::KeypointPixelCoords() const
{
  std::vector<cv::Point> px;
  for (const auto &kpt : keypoints)
    px.push_back(vcp::convert::ToPoint(kpt));
  return px;
}

std::ostream& operator<<(std::ostream & os, const PoseModel &pm)
{
  os << "PoseModel(" << PoseTypeToString(pm.type) << ", " << pm.keypoints.size() << " keypoints, " << pm.scores.size() << " scores)";
  return os;
}

void DrawPose(const PoseModel &model, cv::Mat &image, const float score_threshold, const int keypoint_radius, const int keypoint_thickness, const int skeleton_thickness, const bool draw_ellipses)
{
  const auto colors = GetModelColors(model.type);

  // Draw skeleton.
  const auto skeleton = GetSkeleton(model.type);
  for (size_t idx = 0; idx < skeleton.size(); ++idx)
  {
    const auto kpt_pair = skeleton[idx];
    // Check for invalid estimates:
    if (model.scores.size() > 0
        && (model.scores[kpt_pair.first] < score_threshold
            || model.scores[kpt_pair.second] < score_threshold))
    {
      continue;
    }

    if (std::max(kpt_pair.first, kpt_pair.second) >= model.keypoints.size())
    {
      VCP_LOG_FAILURE("Skipping visualization of " << kpt_pair << ", because the keypoint indices are out of bounds for " << model);
      continue;
    }

    if (draw_ellipses)
    {
      const auto p1 = model.keypoints[kpt_pair.first];
      const auto p2 = model.keypoints[kpt_pair.second];

      const auto center = (p1 + p2)/2.f;
      const auto bone_length = vcp::math::geo2d::Distance2f(p1, p2);
      const float dx = p1[0] - p2[0];
      const float dy = p1[1] - p2[1];
      const float angle = vcp::math::Rad2Deg(std::atan2(dy, dx));
      const cv::RotatedRect rrect(vcp::convert::ToPoint2f(center), cv::Size2f(bone_length, skeleton_thickness), angle);
      cv::ellipse(image, rrect, colors[kpt_pair.first], -1);
    }
    else
    {
      cv::Point p1 = vcp::convert::ToPoint(model.keypoints[kpt_pair.first]);
      cv::Point p2 = vcp::convert::ToPoint(model.keypoints[kpt_pair.second]);
      cv::line(image, p1, p2, colors[kpt_pair.first], skeleton_thickness);
    }
  }

  // Draw keypoints/joints.
  const auto keypoints = model.KeypointPixelCoords();
  for (size_t idx = 0; idx < keypoints.size(); ++idx)
  {
    // Skip unreliable estimates
    if (model.scores.size() > 0 && (model.scores[idx] < score_threshold))
        continue;
    cv::circle(image, keypoints[idx], keypoint_radius, colors[idx], keypoint_thickness);
  }
}

void DrawPoses(const std::vector<PoseModel> &models, cv::Mat &image, const float opacity, const float score_threshold, const int keypoint_radius, const int keypoint_thickness, const int skeleton_thickness, const bool draw_ellipses)
{
  if (opacity < 1.f)
  {
    cv::Mat cp = image.clone();
    for (const auto &model : models)
      DrawPose(model, cp, score_threshold, keypoint_radius, keypoint_thickness, skeleton_thickness, draw_ellipses);
    cv::addWeighted(cp, opacity, image, 1.f-opacity, 0.0, image);
  }
  else
  {
    for (const auto &model : models)
      DrawPose(model, image, score_threshold, keypoint_radius, keypoint_thickness, skeleton_thickness, draw_ellipses);
  }
}

} // namespace poses
} // namespace imvis
} // namespace vcp
