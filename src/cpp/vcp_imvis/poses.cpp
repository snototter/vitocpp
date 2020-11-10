#include "poses.h"
#include "drawing.h"
//#include <vcp_imutils/imutils.h>
//#include <vcp_imutils/matutils.h>
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
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
const std::vector<std::string> kCOCO17Keypoints = {
  "nose",
  "left_eye",
  "right_eye",
  "left_ear",
  "right_ear",
  "left_shoulder",
  "right_shoulder",
  "left_elbow",
  "right_elbow",
  "left_wrist",
  "right_wrist",
  "left_hip",
  "right_hip",
  "left_knee",
  "right_knee",
  "left_ankle",
  "right_ankle"
};


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


inline const std::vector<std::pair<size_t, size_t>> &GetSkeleton(PoseType pt)
{
  switch (pt)
  {
    case PoseType::COCO17:
      return kCOCO17Skeleton;

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


void DrawPose()//cv::Mat &image)
{
  int skeleton_thickness = 7;
  int keypoint_radius = 5;
  int keypoint_thickness = -1;
  bool draw_ellipses = true;
  float score_threshold = 0.1;

  cv::Mat image = cv::Mat(800, 1000, CV_8UC3, cv::Scalar::all(255));

  PoseModel model;
  model.type = PoseType::COCO17;
  model.keypoints = {cv::Vec2f(633.82,   326.42493),
                     cv::Vec2f(636.6838,  314.96985),
                     cv::Vec2f(633.82,    317.83362),
                     cv::Vec2f(645.2751,  320.6974 ),
                     cv::Vec2f(688.2316,  320.6974 ),
                     cv::Vec2f(636.6838,  386.56412),
                     cv::Vec2f(719.73315, 383.7003),
                     cv::Vec2f(613.7736 , 483.93237),
                     cv::Vec2f(722.5969,  466.74973),
                     cv::Vec2f(582.2721 , 541.2078 ),
                     cv::Vec2f(708.2781 , 524.02515),
                     cv::Vec2f(645.2751 , 529.7527 ),
                     cv::Vec2f(691.0954 , 524.02515),
                     cv::Vec2f(605.18225, 624.2572 ),
                     cv::Vec2f(665.3215,  638.57605),
                     cv::Vec2f(642.4113,  713.0341 ),
                     cv::Vec2f(659.59393 ,710.17035)};

//  model.keypoints = {cv::Vec2f(874.0971 , 371.16785),
//                     cv::Vec2f(879.33966, 358.0613 ),
//                     cv::Vec2f(860.99054, 358.0613 ),
//                     cv::Vec2f(892.4462 , 344.9548 ),
//                     cv::Vec2f(813.80707, 355.44   ),
//                     cv::Vec2f(871.47577 ,405.2448 ),
//                     cv::Vec2f(784.97266 ,431.4579 ),
//                     cv::Vec2f(758.75964 ,546.79535),
//                     cv::Vec2f(761.3809,  544.174  ),
//                     cv::Vec2f(821.67096, 630.6771 ),
//                     cv::Vec2f(821.67096, 633.2984 ),
//                     cv::Vec2f(876.7184,  625.4345 ),
//                     cv::Vec2f(837.3988 , 635.91974),
//                     cv::Vec2f(792.8366 , 698.83105),
//                     cv::Vec2f(798.0792 , 706.695  ),
//                     cv::Vec2f(716.8187 , 350.19742),
//                     cv::Vec2f(653.9074 , 468.15616)};
  model.keypoints = {cv::Vec2f(538.6297,285.09528),
                     cv::Vec2f(534.5014 ,285.09528),cv::Vec2f(542.758,283.03113),
                     cv::Vec2f(507.66745 ,280.96698),cv::Vec2f(538.6297 ,283.03113),
                     cv::Vec2f(489.0901,318.12167),cv::Vec2f(551.0146,324.31412),
                     cv::Vec2f(470.51273,369.72543),cv::Vec2f(559.2712,377.98203),
                     cv::Vec2f(468.44858,394.49524),cv::Vec2f(563.3995,415.13675),
                     cv::Vec2f(497.34668,411.00845),cv::Vec2f(536.56555,413.0726 ),
                     cv::Vec2f(499.41083,481.18958),cv::Vec2f(532.43726 ,477.06125),
                     cv::Vec2f(503.53912,557.5631 ),cv::Vec2f(528.30896,538.9858 )};

  model.scores = {0.73372257, 0.78613377, 0.5815941 , 0.7665603 , 0.19802724, 0.41984355, 0.41051248, 0.752903  , 0.6720735 , 0.84569514, 0.33345255, 0.22742593, 0.3147768 , 0.32875675, 0.19814341, 0.22989272, 0.18563741};
VCP_LOG_FAILURE(model.keypoints.size() << " keypoints vs " << model.scores.size() << " SCORES");
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
      VCP_LOG_WARNING("TODO remove output - skipping invalid bone");
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
    cv::circle(image, keypoints[idx], keypoint_radius, colors[idx], keypoint_thickness);
  }

  cv::imshow("Fooooo", image);

}

} // namespace poses
} // namespace imvis
} // namespace vcp
