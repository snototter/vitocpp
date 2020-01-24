#include "liveview.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/file_utils.h>
#include <vcp_utils/string_utils.h>
#include <vcp_imutils/matutils.h>
#include <vcp_imvis/collage.h>

namespace vcp
{
namespace best
{
namespace liveview
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::liveview"

class LiveViewImplementation : public LiveView
{
public:
  LiveViewImplementation(const LiveViewParams &params, std::unique_ptr<SinkBuffer> stream_buffer) :
    LiveView(params),
    user_input_(-1), continue_viewing_(false), image_queue_(std::move(stream_buffer)),
    timepoint_previous_request_(std::chrono::steady_clock::now()), average_time_between_requests_(-1.0),
    average_time_between_display_(-1.0)
  {
    VCP_LOG_DEBUG("LiveViewImplementation::LiveViewImplementation()");
  }

  virtual ~LiveViewImplementation()
  {
    VCP_LOG_DEBUG("LiveViewImplementation::~LiveViewImplementation()");
    Stop();
  }

  void PushCollageRequest(const std::vector<cv::Mat> &images) override
  {
    cv::Mat collage;
    int images_per_row = 0, num_rows;
    do
    {
      images_per_row += 2;
      num_rows = static_cast<int>(std::ceil(images.size() / static_cast<double>(images_per_row)));
    } while(num_rows > images_per_row);

    const int fixed_width = params_.max_size.width / images_per_row;
    const int fixed_height = params_.max_size.height / num_rows;
    int max_row_width = 0, max_total_height = 0;
    for (int r = 0; r < num_rows; ++r)
    {
      int row_width = 0, max_row_height = 0;
      for (int c = 0; c < images_per_row; ++c)
      {
        if (r*images_per_row + c < static_cast<int>(images.size()))
        {
          row_width += images[r*images_per_row + c].cols;
          max_row_height = std::max(max_row_height, images[r*images_per_row + c].rows);
        }
      }
      max_row_width = std::max(max_row_width, row_width);
      max_total_height += max_row_height; // Yes, this is only the upper bound, not the exact total height.
    }
    const cv::Size fixed_size =
        (max_row_width > fixed_width || max_total_height > fixed_height)
        ? cv::Size(fixed_width, fixed_height) : cv::Size(-1, -1);
    vcp::imvis::collage::Collage(images, collage, static_cast<int>(images_per_row), 0, fixed_size);
    PushImageRequest(collage);
  }

  void PushImageRequest(const cv::Mat &image) override
  {
    // Remember the duration between different requests
    const auto now = std::chrono::steady_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(now - timepoint_previous_request_);
    timepoint_previous_request_ = now;

    // Emplace the request
    image_queue_mutex_.lock();
    image_queue_->PushBack(image.clone());
    image_queue_mutex_.unlock();

    // Update the moving average
    if (average_time_between_requests_ < 0.0)
    {
      average_time_between_requests_ = duration.count();
    }
    else
    {
      // Exponential moving average
      const double alpha = 0.1;
      average_time_between_requests_ = alpha * duration.count() + (1.0 - alpha) * average_time_between_requests_;
    }
  }

  void SetWaitMs(int wait_ms) override
  {
    wait_ms_ = wait_ms;
  }

  int GetLastUserInput() const override
  {
    return user_input_;
  }

  double GetRequestFrameRate() const override
  {
    return 1000.0 / average_time_between_requests_;
  }

  double GetDisplayFrameRate() const override
  {
    return 1000.0 / average_time_between_display_;
  }


  bool Start() override
  {
    VCP_LOG_DEBUG("LiveViewImplementation::Start()");
    if (continue_viewing_)
    {
      VCP_LOG_FAILURE("LiveView has already been started.");
      return false;
    }

    wait_ms_ = params_.wait_ms;
    continue_viewing_ = true;
    view_thread_ = std::thread(&LiveViewImplementation::View, this);
    return true;
  }

  bool Stop() override
  {
    if (continue_viewing_)
    {
      VCP_LOG_DEBUG("LiveViewImplementation::Stop()");
      continue_viewing_ = false;
      view_thread_.join();
    }
    return true;
  }

private:
  std::atomic<int> wait_ms_;
  std::atomic<int> user_input_;
  std::atomic<bool> continue_viewing_;
  std::unique_ptr<SinkBuffer> image_queue_;
  std::thread view_thread_;
  std::mutex image_queue_mutex_;
  std::chrono::steady_clock::time_point timepoint_previous_request_;
  std::atomic<double> average_time_between_requests_;
  std::atomic<double> average_time_between_display_;

  void View()
  {
    auto timepoint_previous_display = std::chrono::steady_clock::now();

    while (continue_viewing_)
    {
      cv::Mat image;
      image_queue_mutex_.lock();
      if (!image_queue_->Empty())
      {
        image_queue_->Front().copyTo(image);
        image_queue_->PopFront();
      }
      image_queue_mutex_.unlock();

      if (!image.empty())
      {
        // Remember the duration between different requests
        const auto now = std::chrono::steady_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(now - timepoint_previous_display);
        timepoint_previous_display = now;

        // Update the moving average
        if (average_time_between_display_ < 0.0)
        {
          average_time_between_display_= duration.count();
        }
        else
        {
          // Exponential moving average
          const double alpha = 0.1;
          average_time_between_display_ = alpha * duration.count() + (1.0 - alpha) * average_time_between_display_;
        }

        cv::imshow(params_.window_title, image);
        const int k = cv::waitKey(wait_ms_);
        user_input_ = k;
      }
    }
  }
};


std::unique_ptr<LiveView> CreateBufferedLiveView(const LiveViewParams &params, std::unique_ptr<SinkBuffer> stream_buffer)
{
  return std::unique_ptr<LiveViewImplementation>(new LiveViewImplementation(params, std::move(stream_buffer)));
}

} // namespace liveview
} // namespace best
} // namespace vcp
