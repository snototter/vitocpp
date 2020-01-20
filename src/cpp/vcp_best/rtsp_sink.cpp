#include "rtsp_sink.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <deque>
#ifdef ICC_DEBUG_FRAMERATE //FIXME!
  #include <chrono>
  #include <iomanip>
  #include <pvt_utils/string_utils.h>
#endif // ICC_DEBUG_FRAMERATE

#include "rtsp_media_sink.h"
#include "rtsp_client.h"

#include <vcp_utils/vcp_error.h>

namespace vcp
{
namespace best
{
namespace ipcam
{
namespace rtsp
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::ipcam::rtsp"
/**
 * @brief RTSP stream sink.
 */
class RtspStreamSink : public StreamSink
{
public:
  RtspStreamSink(const RtspStreamParams &params, std::unique_ptr<SinkBuffer> sink_buffer) :
    StreamSink(), params_(params), image_queue_(std::move(sink_buffer)), available_(0)
  {
    rtsp_client_env_ = CreateRtspClientEnvironment();
  }

  static void ReceiveFrameCallback(const cv::Mat &frame, void *user_data)
  {
    RtspStreamSink *sink = static_cast<RtspStreamSink*>(user_data);
    sink->available_ = 1;
    sink->EnqueueNextFrame(frame);
  }

  virtual ~RtspStreamSink()
  {
    Terminate();
  }

  void StartStream() override
  {
    rtsp_client_env_->OpenUrl(params_, &RtspStreamSink::ReceiveFrameCallback, this);

    stream_thread_ = std::thread(&RtspStreamSink::Receive, this);
  }

  void Terminate() override
  {
    if (rtsp_client_env_)
    {
      rtsp_client_env_->TerminateEventLoop();
      stream_thread_.join();
      rtsp_client_env_.reset();
    }
  }

  void GetNextFrame(cv::Mat &frame) override
  {
    image_queue_mutex_.lock();
    if (image_queue_->Empty())
    {
      frame = cv::Mat();
      PVT_LOG_WARNING("Image queue is empty!");
    }
    else
    {
      // Retrieve oldest image in queue.
      frame = image_queue_->Front().clone();
      image_queue_->PopFront();
    }
    image_queue_mutex_.unlock();
  }

  int IsAvailable() const override
  {
    return available_;
  }

  int IsFrameAvailable() const override
  {
    image_queue_mutex_.lock();
    const bool empty = image_queue_->Empty();
    image_queue_mutex_.unlock();
    if (empty)
      return 0;
    return 1;
  }

protected:
  RtspStreamParams params_;
  std::unique_ptr<SinkBuffer> image_queue_;
  std::atomic<int> available_;
  std::unique_ptr<RtspClientEnvironment> rtsp_client_env_;
  std::thread stream_thread_;
  mutable std::mutex image_queue_mutex_;

  void Receive()
  {
    // Make the blocking call.
    rtsp_client_env_->DoEventLoop();
  }

  void EnqueueNextFrame(const cv::Mat &frame)
  {
    image_queue_mutex_.lock();
    image_queue_->PushBack(frame.clone());
    image_queue_mutex_.unlock();
  }
};


// TODO ICC_DEBUG_FRAMERATE for all stream sinks, not just RTSP...
class MultiRtspStreamSink : public StreamSink
{
public:
  struct MultiCallbackData
  {
    MultiRtspStreamSink *ptr;
    size_t sink_idx;
  };

  MultiRtspStreamSink(const std::vector<RtspStreamParams> &params, std::vector<std::unique_ptr<SinkBuffer>> sink_buffers) :
    StreamSink(), image_queues_(std::move(sink_buffers))
  {
    PVT_CHECK(!params.empty());
    params_.insert(params_.end(), params.begin(), params.end());

    // One mutex per stream
    image_queue_mutex_.resize(params_.size());

    concat_width_ = 0;
    concat_height_ = params_[0].frame_height;
    // Prepare callback user data (to locate the proper sink/queue by its index).
    for (size_t i = 0; i < params_.size(); ++i)
    {
      MultiCallbackData cbd;
      cbd.ptr = this;
      cbd.sink_idx = i;
      callback_data_.push_back(cbd);

      concat_width_ += params_[i].frame_width;
      concat_height_ = std::max(concat_height_, params_[i].frame_height);

#ifdef ICC_DEBUG_FRAMERATE
      previous_enqueue_time_points_.push_back(std::chrono::high_resolution_clock::now());
      ms_between_frames_.push_back(-1.0);
#endif // ICC_DEBUG_FRAMERATE
    }
    rtsp_client_env_ = CreateRtspClientEnvironment();
  }

  static void ReceiveFrameCallback(const cv::Mat &frame, void *user_data)
  {
    MultiCallbackData *data = static_cast<MultiCallbackData*>(user_data);
    data->ptr->EnqueueNextFrame(frame, data->sink_idx);
  }

  virtual ~MultiRtspStreamSink()
  {
    Terminate();
  }

  void StartStream() override
  {
    for (size_t i = 0; i < params_.size(); ++i)
    {
      rtsp_client_env_->OpenUrl(params_[i], &MultiRtspStreamSink::ReceiveFrameCallback, &callback_data_[i]);
    }

    stream_thread_ = std::thread(&MultiRtspStreamSink::Receive, this);
  }

  void Terminate() override
  {
    if (rtsp_client_env_)
    {
      rtsp_client_env_->TerminateEventLoop();
      stream_thread_.join();
      rtsp_client_env_.reset();
    }
  }

  void GetNextFrame(cv::Mat &frame) override
  {
    const size_t num_sinks = params_.size();
    // Lock all
    for (size_t i = 0; i < num_sinks; ++i)
      image_queue_mutex_[i].lock();

    // Check if we have frames for each sink available
    bool available = true;

    for (size_t i = 0; i < num_sinks; ++i)
      available = available && !image_queues_[i]->Empty();

    if (available)
    {
      // Concat all frames.
      frame = cv::Mat(concat_height_, concat_width_, CV_8UC3);
      int roi_from = 0;
      for (size_t i = 0; i < num_sinks; ++i)
      {
        int roi_to = roi_from + params_[i].frame_width;
        cv::Mat frame_roi = frame(cv::Range(0, params_[i].frame_height), cv::Range(roi_from, roi_to));
        image_queues_[i]->Front().copyTo(frame_roi);
        image_queues_[i]->PopFront();
        roi_from = roi_to;
      }
    }
    else
    {
      frame = cv::Mat();
    }

    // Unlock all
    for (size_t i = 0; i < num_sinks; ++i)
      image_queue_mutex_[i].unlock();
  }

//  void GetMostRecentFrame(cv::Mat &frame) override
//  {
//    // Same as GetNextFrame, but popping the front instead of the back.
//    const size_t num_sinks = params_.size();
//    // Lock all
//    for (size_t i = 0; i < num_sinks; ++i)
//      image_queue_mutex_[i].lock();

//    // Check if we have frames for each sink available
//    bool available = true;

//    for (size_t i = 0; i < num_sinks; ++i)
//      available = available && !image_queues_[i]->Empty();

//    if (available)
//    {
//      // Concat all frames.
//      frame = cv::Mat(concat_height_, concat_width_, CV_8UC3);
//      int roi_from = 0;
//      for (size_t i = 0; i < num_sinks; ++i)
//      {
//        int roi_to = roi_from + params_[i].frame_width;
//        cv::Mat frame_roi = frame(cv::Range::all(), cv::Range(roi_from, roi_to));
//        image_queues_[i]->Back().copyTo(frame_roi);
//        image_queues_[i]->PopBack();
//        roi_from = roi_to;
//      }
//    }
//    else
//    {
//      frame = cv::Mat();
//    }

//    // Unlock all
//    for (size_t i = 0; i < num_sinks; ++i)
//      image_queue_mutex_[i].unlock();
//  }

  int IsAvailable() const override
  {
    if (rtsp_client_env_ != nullptr)
      return 1;
    return 0;
  }

  int IsFrameAvailable() const override
  {
    bool empty = false;
    for (size_t i = 0; i < image_queue_mutex_.size(); ++i)
    {
      image_queue_mutex_[i].lock();
      if (image_queues_[i]->Empty())
        empty = true;
      image_queue_mutex_[i].unlock();
    }

    if (empty)
      return 0;
    return 1;
  }

protected:
  int concat_width_;
  int concat_height_;
  std::vector<std::unique_ptr<SinkBuffer>> image_queues_;
  std::vector<RtspStreamParams> params_;
  std::vector<MultiCallbackData> callback_data_;
  std::unique_ptr<RtspClientEnvironment> rtsp_client_env_;
  mutable std::deque<std::mutex> image_queue_mutex_;
  std::thread stream_thread_;
#ifdef ICC_DEBUG_FRAMERATE
  std::vector<std::chrono::high_resolution_clock::time_point> previous_enqueue_time_points_;
  std::vector<double> ms_between_frames_;
#endif // ICC_DEBUG_FRAMERATE

  void Receive()
  {
    // Make the blocking call.
    rtsp_client_env_->DoEventLoop();
  }

  void EnqueueNextFrame(const cv::Mat &frame, size_t sink_idx)
  {
    image_queue_mutex_[sink_idx].lock();
    image_queues_[sink_idx]->PushBack(frame.clone());
    image_queue_mutex_[sink_idx].unlock();

#ifdef ICC_DEBUG_FRAMERATE
    const std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(now - previous_enqueue_time_points_[sink_idx]);
    previous_enqueue_time_points_[sink_idx] = now;
    const double ms_ema_alpha = 0.1;

    if (ms_between_frames_[sink_idx] < 0.0)
      ms_between_frames_[sink_idx] = duration.count();
    else
      ms_between_frames_[sink_idx] = ms_ema_alpha * duration.count() + (1.0 - ms_ema_alpha) * ms_between_frames_[sink_idx];

    PVT_LOG_INFO_NOFILE("MultiRtspStreamSink received frame after " << std::fixed << std::setw(5) << duration.count() << " ms, "
                        << std::setw(5) << (1000.0 / ms_between_frames_[sink_idx]) << " fps, stream '"
                        << pvt::utils::string::ClipUrl(params_[sink_idx].stream_url) << "'");
#endif // ICC_DEBUG_FRAMERATE
  }
};



std::unique_ptr<StreamSink> CreateBufferedRtspStreamSink(const RtspStreamParams &params, std::unique_ptr<SinkBuffer> sink_buffer)
{
  return std::unique_ptr<RtspStreamSink>(new RtspStreamSink(params, std::move(sink_buffer)));
}

std::unique_ptr<StreamSink> CreateBufferedMultiRtspStreamSink(const std::vector<RtspStreamParams> &params, std::vector<std::unique_ptr<SinkBuffer> > sink_buffers)
{
  return std::unique_ptr<MultiRtspStreamSink>(new MultiRtspStreamSink(params, std::move(sink_buffers)));
}

} // namespace rtsp
} // namespace ipcam
} // namespace best
} // namespace vcp
