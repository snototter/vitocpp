#include "point_selection.h"

#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vcp_utils/vcp_logging.h>

namespace vcp
{
namespace ui
{
namespace
{
typedef struct {
  cv::Point point; /**< User selected point. */
  bool selection_done; /**< Indicates, whether the user wants to quit the selection process. */
  bool selection_valid; /**< Indicates, whether the user selected a valid point. */
  bool changed; /**< Indicates that the user did 'something', i.e. we need to redraw the visualization. */
  bool add_point; /**< Needed for multi-point selection. */
  cv::Point discard_at; /**< Indicates that the user wants to remove the point closest to this positoin (if x/y >= 0). */
} CallbackParameter;

void PointMouseCallback(int event, int x, int y, int /*flags*/, void* user_data)
{
  CallbackParameter *params = static_cast<CallbackParameter*>(user_data);
  switch(event)
  {
  case CV_EVENT_LBUTTONUP:
    params->point = cv::Point(x, y);
    params->selection_valid = true;
    params->changed = true;
    break;

  case CV_EVENT_RBUTTONUP:
    params->selection_done = true;
    params->changed = true;
    break;

  case CV_EVENT_MBUTTONUP:
    params->selection_valid = false;
    params->selection_done = true;
    break;
  }
}

void MultiPointsMouseCallback(int event, int x, int y, int /*flags*/, void* user_data)
{
  CallbackParameter *params = static_cast<CallbackParameter*>(user_data);
  switch(event)
  {
  case CV_EVENT_LBUTTONUP:
    params->point = cv::Point(x, y);
    params->selection_done = false;
    params->selection_valid = true;
    params->add_point = true;
    params->discard_at = cv::Point(-1, -1);
    params->changed = true;
    break;

  case CV_EVENT_MBUTTONUP:
    params->selection_done = false;
    params->add_point = false;
    params->discard_at = cv::Point(x, y);
    params->changed = true;
    break;

  case CV_EVENT_RBUTTONUP:
    params->selection_valid = true;
    params->selection_done = true;
    params->add_point = false;
    params->discard_at = cv::Point(-1, -1);
    params->changed = false;
    break;
  }
}
} // namespace

bool SelectPoint(cv::Point &point, const cv::Mat &image, const cv::Scalar &point_color,
                 const std::string &window_name, const PointMarker &marker, int marker_thickness)
{
  CallbackParameter params;
  params.point = cv::Point(-1,-1);
  params.selection_done = false;
  params.selection_valid = false;
  params.changed = false;

  cv::namedWindow(window_name);
  cv::imshow(window_name, image);

  cv::Mat vis = image.clone();
  while (!params.selection_done)
  {
    // q.a.d. - as long as the user may close the window without us noticing,
    // we update the mouse callback handler
    cv::setMouseCallback(window_name, PointMouseCallback, (void*)&params);

    int key = cv::waitKey(300);
    switch((char)key)
    {
    case '\x1b': // ESC
      params.selection_valid = false;
      params.selection_done = true;
      break;

    case 10: // LF
    case 13: // CR
    case 'q':
    case 'Q':
    case 'c':
    case 'C':
      //params.selection_valid = params.point.x >= 0 && params.point.y >= 0;
      params.selection_done = true;
      break;

    case 'h':
    case 'H':
      VCP_LOG_INFO_DEFAULT(PointSelectionUsage());
      break;
    }
    // q.a.d. - as long as there's no reliable way to detect a closing window (once the user clicked the title bar 'X'), we
    // reopen the window and request the user to close the window using ESC
    if (params.changed)
    {
      vis = image.clone();
      params.changed = false;
      if (params.selection_valid)
      {
        if (marker == PointMarker::DOT)
          cv::circle(vis, params.point, marker_thickness, point_color, cv::FILLED);
        else
        {
          const int left = params.point.x - 10;
          const int right = params.point.x + 10;
          const int top = params.point.y - 10;
          const int bottom = params.point.y + 10;
          cv::line(vis, cv::Point(params.point.x, top), cv::Point(params.point.x, bottom), point_color, marker_thickness);
          cv::line(vis, cv::Point(left, params.point.y), cv::Point(right, params.point.y), point_color, marker_thickness);
        }
      }
    }
    cv::imshow(window_name, vis);
  }
  point = params.point;
  const bool valid = params.selection_valid;
  cv::destroyWindow(window_name);
  return valid;
}


std::vector<cv::Point> SelectPoints(
    const cv::Mat &image, const cv::Scalar &point_color,
    const std::string &window_name, const PointMarker &marker, const int marker_thickness)
{
  std::vector<cv::Point> points;
  CallbackParameter params;
  params.point = cv::Point(-1, -1);
  params.selection_done = false;
  params.selection_valid = false;
  params.changed = false;
  params.discard_at = cv::Point(-1, -1);
  params.add_point = false;

  cv::namedWindow(window_name);

  cv::Mat vis = image.clone();
  cv::imshow(window_name, vis);

  while (!params.selection_done)
  {
    // q.a.d. - as long as the user may close the window without us noticing,
    // we update the mouse callback handler
    cv::setMouseCallback(window_name, MultiPointsMouseCallback, (void*)&params);

    int key = cv::waitKey(300);
    switch((char)key)
    {
    case '\x1b': // ESC
      params.selection_valid = false;
      params.selection_done = true;
      break;

    case 'r':
      // Remove the last point
      if (!points.empty())
      {
        params.selection_done = false;
        params.add_point = false;
        params.discard_at = points[points.size()-1];
        params.changed = true;
      }
      break;

    case 10: // LF
    case 13: // CR
    case 'q':
    case 'Q':
    case 'c':
    case 'C':
      params.selection_valid = true;
      params.selection_done = true;
      break;

    case 'h':
    case 'H':
      VCP_LOG_INFO_DEFAULT(MultiplePointsSelectionUsage());
      break;
    }

    // Update visualization if something changed
    if (params.changed)
    {
      params.changed = false;

      // Delete point closest to given position
      if (params.discard_at.x >= 0 && params.discard_at.y >= 0)
      {
        if (!points.empty())
        {
          //points.pop_back();
          cv::Point diff = points[0] - params.discard_at;
          int min_sqdist = diff.x * diff.x + diff.y * diff.y;
          size_t min_idx = 0;
          for (size_t idx = 1; idx < points.size(); ++idx)
          {
            diff = points[idx] - params.discard_at;
            const int sqdist = diff.x * diff.x + diff.y * diff.y;
            if (sqdist < min_sqdist)
            {
              min_idx = idx;
              min_sqdist = sqdist;
            }
          }
          points.erase(points.begin() + min_idx);
        }
        params.discard_at = cv::Point(-1, -1);
      }
      else if (params.add_point)
      {
        params.add_point = false;
        points.push_back(params.point);
      }

      vis = image.clone();
      if (marker == PointMarker::DOT)
      {
        for (const auto &pt : points)
          cv::circle(vis, pt, marker_thickness, point_color, cv::FILLED);
      }
      else
      {
        for (const auto &pt : points)
        {
          const int left = pt.x - 10;
          const int right = pt.x + 10;
          const int top = pt.y - 10;
          const int bottom = pt.y + 10;
          cv::line(vis, cv::Point(pt.x, top), cv::Point(pt.x, bottom), point_color, marker_thickness);
          cv::line(vis, cv::Point(left, pt.y), cv::Point(right, pt.y), point_color, marker_thickness);
        }
      }
    }

    // q.a.d. - as long as there's no reliable way to detect a closing window (once the user clicked the title bar 'X'), we
    // reopen the window and request the user to close the window using ESC or RMB
    cv::imshow(window_name, vis);
  }
  cv::destroyWindow(window_name);

  if (params.selection_valid)
    return points;

  return std::vector<cv::Point>();
}

std::string PointSelectionUsage()
{
  std::stringstream usage;
  usage << "PointSelection Usage:" << std::endl
        << "-----------------------------------------------------------" << std::endl
        << "* LMB click selects the cursor position" << std::endl
        << "* Confirm the point selection by:" << std::endl
        << "  o Clicking with the RMB" << std::endl
        << "  o Keyboard: 'c', 'q' or return" << std::endl
        << "* Abort selection by:" << std::endl
        << "  o Clicking with the MMB" << std::endl
        << "  o Keyboard: ESC" << std::endl
        << "* 'h' prints this usage help" << std::endl
        << std::endl;
  return usage.str();
}

std::string MultiplePointsSelectionUsage()
{
  std::stringstream usage;
  usage << "MultiplePointsSelection Usage:" << std::endl
        << "-----------------------------------------------------------" << std::endl
        << "* LMB click adds the current cursor position" << std::endl
        << "* MMB click removes the position closest to cursor" << std::endl
        << "* Keyboard 'r' removes the last added point" << std::endl
        << "* Confirm the point selection by:" << std::endl
        << "  o Clicking RMB" << std::endl
        << "  o Keyboard: 'c', 'q' or return" << std::endl
        << "* Abort selection by hitting ESC" << std::endl
        << "* 'h' prints this usage help" << std::endl
        << std::endl;
  return usage.str();
}
} // end namespace ui
} // end namespace vcp
