#include "rtsp_sink.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <deque>
#include <set>
#include <string>
#include <vcp_utils/file_utils.h>
#ifdef VCP_BEST_DEBUG_FRAMERATE
  #include <chrono>
  #include <iomanip>
  #include <vcp_utils/string_utils.h>
#endif // VCP_BEST_DEBUG_FRAMERATE

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

class MultiRtspStreamSink : public StreamSink
{
public:
  struct MultiCallbackData
  {
    MultiRtspStreamSink *ptr;
    size_t sink_idx;
  };

  MultiRtspStreamSink(const std::vector<IpCameraSinkParams> &params, std::vector<std::unique_ptr<SinkBuffer>> sink_buffers) :
    StreamSink(), image_queues_(std::move(sink_buffers)), rtsp_client_env_(nullptr), verbose_(false)
  {
    VCP_LOG_DEBUG("MultiRtspStreamSink::MultiRtspStreamSink()");
    if (params.empty())
      VCP_ERROR("Cannot create a MultiRtspStreamSink without any configured streams.");

    // Store the configurations and allocate one mutex per stream
    params_.insert(params_.end(), params.begin(), params.end());
    image_queue_mutex_.resize(params_.size());

    // Prepare callback user data (to locate the proper sink/queue by its index).
    verbose_ = false;
    for (size_t i = 0; i < params_.size(); ++i)
    {
      MultiCallbackData cbd;
      cbd.ptr = this;
      cbd.sink_idx = i;
      callback_data_.push_back(cbd);

      verbose_ |= params_[i].verbose;

      // Load calibration if available
      if (params_[i].rectify || vcp::utils::file::Exists(params_[i].calibration_file))
      {
        if (params_[i].calibration_file.empty() || !vcp::utils::file::Exists(params_[i].calibration_file))
        {
          VCP_ERROR("To undistort & rectify the rtsp stream [" << params_[i].sink_label
                    << "], the calibration file '" << params_[i].calibration_file
                    << "' must exist!");
        }

        const auto intrinsics = calibration::LoadIntrinsicsFromFile(params_[i].calibration_file);
        if (intrinsics.size() != 1)
        {
          VCP_ERROR("Loaded invalid number of " << intrinsics.size()
                     << " intrinsic calibrations from " << params_[i].calibration_file << ".");
        }

        intrinsics_.push_back(intrinsics[0]);

        if (verbose_)
          VCP_LOG_INFO_DEFAULT("Loaded intrinsic calibration for rtsp stream ["
                         << params_[i].sink_label << "].");
      }
      else
      {
        intrinsics_.push_back(calibration::StreamIntrinsics());
      }

#ifdef VCP_BEST_DEBUG_FRAMERATE
      previous_enqueue_time_points_.push_back(std::chrono::high_resolution_clock::now());
      ms_between_frames_.push_back(-1.0);
#endif // VCP_BEST_DEBUG_FRAMERATE
    }
  }

  virtual ~MultiRtspStreamSink()
  {
    VCP_LOG_DEBUG("MultiRtspStreamSink::~MultiRtspStreamSink()");
    CloseDevice();
  }

  bool OpenDevice() override
  {
    VCP_LOG_DEBUG("MultiRtspStreamSink::OpenDevice()");
    return true;
  }


  bool CloseDevice() override
  {
    VCP_LOG_DEBUG("MultiRtspStreamSink::CloseDevice()");
    return StopStreaming();
  }


  bool StartStreaming() override
  {
    VCP_LOG_DEBUG("MultiRtspStreamSink::StartStreaming()");
    if (rtsp_client_env_)
    {
      VCP_LOG_FAILURE("RTSP client environment is already initialized.");
      return false;
    }

    rtsp_client_env_ = CreateRtspClientEnvironment();

    if (!rtsp_client_env_)
    {
      VCP_LOG_FAILURE("RTSP client environment cannot be created.");
      return false;
    }

    if (verbose_)
    {
      if (params_.size() == 1)
        VCP_LOG_INFO_DEFAULT("Starting the single RTSP stream.");
      else
        VCP_LOG_INFO_DEFAULT("Starting all " << params_.size() << " RTSP streams.");
    }

    bool success = false;
    for (size_t i = 0; i < params_.size(); ++i)
      success |= rtsp_client_env_->OpenUrl(params_[i], &MultiRtspStreamSink::ReceiveFrameCallback, &callback_data_[i]);

    stream_thread_ = std::thread(&MultiRtspStreamSink::RunEventLoop, this);
    return success;
  }

  bool StopStreaming() override
  {
    VCP_LOG_DEBUG("MultiRtspStreamSink::StopStreaming()");
    if (rtsp_client_env_)
    {
      if (verbose_)
        VCP_LOG_INFO_DEFAULT("Closing the RTSP client environment.");
      rtsp_client_env_->TerminateEventLoop();
      stream_thread_.join();
      rtsp_client_env_.reset();
      if (verbose_)
        VCP_LOG_INFO_DEFAULT("RTSP client environment has terminated.");
    }
    return true;
  }

  std::vector<cv::Mat> Next() override
  {
    std::vector<cv::Mat> frames;
    // Lock, query, unlock all queues:
    for (size_t i = 0; i < params_.size(); ++i)
    {
      image_queue_mutex_[i].lock();
      if (image_queues_[i]->Empty())
        frames.push_back(cv::Mat());
      else
      {
        frames.push_back(image_queues_[i]->Front().clone());
        image_queues_[i]->PopFront();
      }
      image_queue_mutex_[i].unlock();
    }
    return frames;
  }


  int IsDeviceAvailable() const override
  {
    if (rtsp_client_env_ != nullptr)
      return 1;
    return 0;
  }


  virtual size_t NumAvailableFrames() const override
  {
    size_t num = 0;
    for (size_t i = 0; i < image_queue_mutex_.size(); ++i)
    {
      image_queue_mutex_[i].lock();
      if (!image_queues_[i]->Empty())
        ++num;
      image_queue_mutex_[i].unlock();
    }
    return num;
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


  size_t NumStreams() const override
  {
    return params_.size();
  }


  size_t NumDevices() const override
  {
    std::set<std::string> unique_hosts;
    for (const auto &p : params_)
      unique_hosts.insert(p.host);
    return unique_hosts.size();
  }


  FrameType FrameTypeAt(size_t stream_index) const override
  {
    return params_[stream_index].frame_type;
  }


  std::string StreamLabel(size_t stream_index) const override
  {
    return params_[stream_index].sink_label;
  }


  SinkParams SinkParamsAt(size_t stream_index) const override
  {
    return params_[stream_index];
  }


  static void ReceiveFrameCallback(const cv::Mat &frame, void *user_data)
  {
    MultiCallbackData *data = static_cast<MultiCallbackData*>(user_data);
    data->ptr->EnqueueNextFrame(frame, data->sink_idx);
  }

  void SetVerbose(bool verbose) override
  {
    for (size_t i = 0; i < params_.size(); ++i)
      params_[i].verbose = verbose;
  }

  SinkType GetSinkType() const override
  {
    VCP_LOG_FAILURE("GetSinkType() should not be called for an MultiRtspStreamSink, use SinkParamsAt(stream_index).sink_type instead!");
    return SinkType::IPCAM_GENERIC;
  }


  vcp::best::calibration::StreamIntrinsics IntrinsicsAt(size_t stream_index) const override
  {
    return intrinsics_[stream_index];
  }

  bool SetExtrinsicsAt(size_t stream_index, const cv::Mat &R, const cv::Mat &t) override
  {
    return extrinsics_[stream_index].SetExtrinsics(R, t, intrinsics_[stream_index]);
  }

  void ExtrinsicsAt(size_t stream_index, cv::Mat &R, cv::Mat &t) const override
  {
    R = extrinsics_[stream_index].R().clone();
    t = extrinsics_[stream_index].t().clone();
  }

protected:
  std::vector<std::unique_ptr<SinkBuffer>> image_queues_;
  std::unique_ptr<RtspClientEnvironment> rtsp_client_env_;
  bool verbose_;
  std::vector<IpCameraSinkParams> params_;
  std::vector<calibration::StreamIntrinsics> intrinsics_;
  std::vector<calibration::StreamExtrinsics> extrinsics_;
  std::vector<MultiCallbackData> callback_data_;
  mutable std::deque<std::mutex> image_queue_mutex_;
  std::thread stream_thread_;
#ifdef VCP_BEST_DEBUG_FRAMERATE
  std::vector<std::chrono::high_resolution_clock::time_point> previous_enqueue_time_points_;
  std::vector<double> ms_between_frames_;
#endif // VCP_BEST_DEBUG_FRAMERATE

  void RunEventLoop()
  {
    // Make the blocking call.
    rtsp_client_env_->DoEventLoop();
  }

  void EnqueueNextFrame(const cv::Mat &frame, size_t sink_idx)
  {
    // Rectify if needed
    cv::Mat img;
    if (params_[sink_idx].rectify)
    {
      cv::Mat tmp = intrinsics_[sink_idx].UndistortRectify(frame);
      img = imutils::ApplyImageTransformations(tmp, params_[sink_idx].transforms);
    }
    else
    {
      img = imutils::ApplyImageTransformations(frame, params_[sink_idx].transforms);
    }
    image_queue_mutex_[sink_idx].lock();
    image_queues_[sink_idx]->PushBack(img.clone());
    image_queue_mutex_[sink_idx].unlock();

#ifdef VCP_BEST_DEBUG_FRAMERATE
    const std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(now - previous_enqueue_time_points_[sink_idx]);
    previous_enqueue_time_points_[sink_idx] = now;
    const double ms_ema_alpha = 0.1;

    if (ms_between_frames_[sink_idx] < 0.0)
      ms_between_frames_[sink_idx] = duration.count();
    else
      ms_between_frames_[sink_idx] = ms_ema_alpha * duration.count() + (1.0 - ms_ema_alpha) * ms_between_frames_[sink_idx];

    VCP_LOG_DEBUG_DEFAULT("MultiRtspStreamSink received frame after " << std::fixed << std::setw(5) << duration.count() << " ms, "
                        << std::setw(5) << (1000.0 / ms_between_frames_[sink_idx]) << " fps, '"
                        << vcp::utils::string::ClipUrl(params_[sink_idx].stream_url) << "'");
#endif // VCP_BEST_DEBUG_FRAMERATE
  }
};


std::unique_ptr<StreamSink> CreateBufferedMultiRtspStreamSink(const std::vector<IpCameraSinkParams> &params, std::vector<std::unique_ptr<SinkBuffer> > sink_buffers)
{
  return std::unique_ptr<MultiRtspStreamSink>(new MultiRtspStreamSink(params, std::move(sink_buffers)));
}

} // namespace rtsp
} // namespace ipcam
} // namespace best
} // namespace vcp
