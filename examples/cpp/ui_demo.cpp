#include <iostream>
#include <string>
#include <vector>

#include <opencv2/highgui/highgui.hpp>

#include <vcp_ui/point_selection.h>
#include <vcp_ui/rect_selection.h>

#define VCP_LOG_LEVEL_DEBUG
#undef VCP_LOG_LOCATION
#include <vcp_utils/vcp_logging.h>

int main(int argc, char **argv)
{
  VCP_UNUSED_VAR(argc);
  VCP_UNUSED_VAR(argv);
  const cv::Mat image = cv::imread("flamingo.jpg");

  VCP_LOG_INFO(vcp::ui::PointSelectionUsage());
  cv::Point point;
  if (vcp::ui::SelectPoint(point, image))
    VCP_LOG_INFO("Selected point: " << point);
  else
    VCP_LOG_INFO("No point selected");



  VCP_LOG_INFO(vcp::ui::MultiplePointsSelectionUsage());
  const auto points = vcp::ui::SelectPoints(image);
  if (points.empty())
  {
    VCP_LOG_INFO("No point selected");
  }
  else
  {
    for (const auto &pt : points)
      VCP_LOG_INFO("Selected: " << pt);
  }



  VCP_LOG_INFO(vcp::ui::RectSelectionUsage());
  cv::Rect rect;
  if (vcp::ui::SelectRectangle(rect, image))
    VCP_LOG_INFO("Selected rect: " << rect);
  else
    VCP_LOG_INFO("No rect selected");

  return 0;
}
