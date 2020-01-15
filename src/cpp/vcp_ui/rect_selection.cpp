#include "rect_selection.h"

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
/** @brief Internal structure to pass parameters to the callback function. */
typedef struct {
  cv::Point start; /**< User selected start point of the rectangle. */
  cv::Rect roi; /**< User selected rectangle. */
  cv::Rect prev_roi;
  cv::Mat image; /**< Input image. */
  cv::Scalar color; /**< Color of the rectangle. */
  std::string window_name; /**< Window title. */
  bool selection_in_progress; /**< Indicates, whether the user is currently selecting the region of interest. */
  bool selection_done; /**< Indicates, whether the user wants to quit the selection process. */
} CallbackParameter;

/** @brief Display function to refresh the visualization. */
void UpdateImage(const cv::Mat &image, const cv::Rect &roi, const std::string &window_name, const cv::Scalar &color)
{
  cv::Mat img = image.clone();
  cv::rectangle(img, roi, color);
  cv::imshow(window_name, img);
}

/** @brief Callback handler. */
void RectMouseCallback(int event, int x, int y, int /*flags*/, void* user_data)
{
  CallbackParameter *params = static_cast<CallbackParameter*>(user_data);
  switch(event)
  {
  case CV_EVENT_LBUTTONDOWN:
    params->start = cv::Point(x, y);
    params->roi = cv::Rect(x, y, 0, 0);
    params->selection_in_progress = true;
    UpdateImage(params->image, params->roi, params->window_name, params->color);
    break;

  case CV_EVENT_LBUTTONUP:
    if (params->selection_in_progress)
    {
      int left, right, top, bottom;
      if (x < params->start.x)
      {
        left = x;
        right = params->start.x;
      }
      else
      {
        left = params->start.x;
        right = x;
      }

      if (y < params->start.y)
      {
        top = y;
        bottom = params->start.y;
      }
      else
      {
        top = params->start.y;
        bottom = y;
      }
      left = left < 0 ? 0 : left;
      right = right < params->image.cols ? right : params->image.cols-1;
      top = top < 0 ? 0 : top;
      bottom = bottom < params->image.rows ? bottom : params->image.rows-1;
      params->roi = cv::Rect(cv::Point(left, top), cv::Point(right, bottom));

      UpdateImage(params->image, params->roi, params->window_name, params->color);
      params->selection_in_progress = false;
    }
    break;

  case CV_EVENT_RBUTTONUP:
    params->selection_done = true;
    break;

  case CV_EVENT_MOUSEMOVE:
    if (params->selection_in_progress)
    {
      int left, right, top, bottom;
      if (x < params->start.x)
      {
        left = x;
        right = params->start.x;
      }
      else
      {
        left = params->start.x;
        right = x;
      }

      if (y < params->start.y)
      {
        top = y;
        bottom = params->start.y;
      }
      else
      {
        top = params->start.y;
        bottom = y;
      }
      left = left < 0 ? 0 : left;
      right = right < params->image.cols ? right : params->image.cols-1;
      top = top < 0 ? 0 : top;
      bottom = bottom < params->image.rows ? bottom : params->image.rows-1;
      params->roi = cv::Rect(cv::Point(left, top), cv::Point(right, bottom));
      UpdateImage(params->image, params->roi, params->window_name, params->color);
    }
    break;
  }
}
} // namespace

bool SelectRectangle(cv::Rect &rectangle, const cv::Mat &image, const cv::Scalar color, const std::string &window_name)
{
  CallbackParameter params;
  params.color = color;
  params.image = image.clone();
  params.roi = cv::Rect(0, 0, 0, 0);
  params.window_name = window_name;
  params.selection_done = false;
  params.selection_in_progress = false;

  cv::namedWindow(params.window_name);

  while (!params.selection_done)
  {
    // q.a.d. - as long as the user may close the window without us noticing,
    // we update the mouse callback handler
    cv::setMouseCallback(params.window_name, RectMouseCallback, (void*)&params);

    int key = cv::waitKey(300);
    switch((char)key)
    {
    case '\x1b': // ESC
      params.roi = cv::Rect(0, 0, 0, 0);
      params.selection_done = true;
      break;

//    case '\x0a': // LF
    case 10: // LF
    case 13: // CR
    case 'q':
    case 'Q':
    case 'c':
    case 'C':
      params.selection_done = true;
      break;

    case 'r':
    case 'R':
      // Reset rectangle
      params.roi = cv::Rect(0, 0, 0, 0);
      params.selection_in_progress = false;
      break;

    case 'h':
    case 'H':
      VCP_LOG_INFO_DEFAULT(RectSelectionUsage());
      break;
    }
    // q.a.d. - as long as there's no reliable way to detect a closing window (once the user clicked the title bar 'X'), we
    // reopen the window and request the user to close the window using ESC, LF, or RMB
    UpdateImage(params.image, params.roi, params.window_name, params.color);
  }
  rectangle = params.roi;
  cv::destroyWindow(params.window_name);
  return rectangle.width > 0 && rectangle.height > 0;
}

std::string RectSelectionUsage()
{
  std::stringstream usage;
  usage << "RectSelection Usage:" << std::endl
        << "-----------------------------------------------------------" << std::endl
        << "* Press (and hold) LMB to start selection" << std::endl
        << "* Move mouse to draw up rectangle, then release LMB" << std::endl
        << "* Confirm the selected rectangle by:" << std::endl
        << "  o Clicking with the RMB" << std::endl
        << "  o Keyboard: 'c', 'q' or return" << std::endl
        << "* Abort selection by hitting ESC" << std::endl
        << "* Reset the rectangle by pressing 'r'" << std::endl
        << std::endl;
  return usage.str();
}
} // end namespace ui
} // end namespace vcp
