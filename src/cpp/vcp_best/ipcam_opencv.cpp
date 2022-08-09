#include "ipcam_opencv.h"

#include <thread>
#include <mutex>
#include <atomic>

#if CV_VERSION_MAJOR < 3
    #include <opencv2/highgui/highgui.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/highgui.hpp>
    #include <opencv2/imgproc.hpp>
#endif

#include "curl_file_handling.h"
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/file_utils.h>
#include <vcp_imutils/opencv_compatibility.h>

namespace vcp
{
namespace best
{
namespace ipcam
{
namespace videocap
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::ipcam::videocap"
/**
 * @brief The MJPEG over HTTP stream sink
 */
class OpenCVIpCamSink: public StreamSink
{
public:
  OpenCVIpCamSink(const IpCameraSinkParams &params, std::unique_ptr<SinkBuffer> sink_buffer)
    : StreamSink(),
      params_(params), capture_(nullptr), continue_stream_(false),
      is_stream_available_(false), image_queue_(std::move(sink_buffer))
  {
    VCP_LOG_DEBUG("OpenCVIpCamSink::OpenCVIpCamSink()");
#ifdef VCP_BEST_DEBUG_FRAMERATE
    previous_frame_timepoint_ = std::chrono::high_resolution_clock::now();
    ms_between_frames_ = -1.0;
#endif // VCP_BEST_DEBUG_FRAMERATE
  }

  virtual ~OpenCVIpCamSink()
  {
    VCP_LOG_DEBUG("OpenCVIpCamSink::~OpenCVIpCamSink()");
    CloseDevice();
  }


  std::vector<cv::Mat> Next() override
  {
    std::vector<cv::Mat> frames;
    image_queue_mutex_.lock();
    if (image_queue_->Empty())
    {
      frames.push_back(cv::Mat());
    }
    else
    {
      // Retrieve oldest image in queue.
      frames.push_back(image_queue_->Front().clone());
      image_queue_->PopFront();
    }
    image_queue_mutex_.unlock();
    return frames;
  }


  size_t NumAvailableFrames() const override
  {
    return static_cast<size_t>(IsFrameAvailable());
  }


  int IsDeviceAvailable() const override
  {
    if (capture_ && capture_->isOpened())
      return 1;
    return 0;
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

  size_t NumStreams() const override
  {
    return 1;
  }

  FrameType FrameTypeAt(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return params_.frame_type;
  }

  std::string StreamLabel(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return params_.sink_label;
  }

  bool OpenDevice() override
  {
    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Opening IP VideoCapture for " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));

    if (capture_ && capture_->isOpened())
    {
      VCP_LOG_FAILURE("VideoCapture for " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url) << " already opened - ignoring OpenDevice() call");
      return false;
    }

    if (capture_)
      capture_->open(params_.stream_url);
    else
      capture_ = std::unique_ptr<cv::VideoCapture>(new cv::VideoCapture(params_.stream_url));

    if (!capture_->isOpened())
    {
      VCP_LOG_FAILURE("Cannot open VideoCapture for " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));
      return false;
    }

    //TODO check WebcamSink if we need to adjust the resolution/frame rate/etc.

    // Load calibration if available
    if (params_.rectify || vcp::utils::file::Exists(params_.calibration_file))
    {
      if (params_.calibration_file.empty() || !vcp::utils::file::Exists(params_.calibration_file))
      {
        VCP_LOG_FAILURE(
              "To undistort & rectify the VideoCapture for "
              << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url)
              << ", the calibration file '" << params_.calibration_file << "' must exist!");
        return false;
      }

      const auto intrinsics = calibration::LoadIntrinsicsFromFile(params_.calibration_file);
      if (intrinsics.size() != 1)
      {
        VCP_LOG_FAILURE("Loaded invalid number of " << intrinsics.size() << " intrinsic calibrations from " << params_.calibration_file << ".");
        return false;
      }

      intrinsics_ = intrinsics[0];

      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Loaded intrinsic calibration for VideoCapture for " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));
    }
    return true;
  }

  bool CloseDevice() override
  {
    StopStreaming();

    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Closing IP VideoCapture \"" << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url) << "\".");
    capture_.reset();
    return true;
  }

  bool StartStreaming() override
  {
    VCP_LOG_DEBUG("OpenCVIpCamSink::StartStreaming()");
    if (!capture_ || !capture_->isOpened())
      OpenDevice();

    if (continue_stream_)
    {
      VCP_LOG_FAILURE("Streaming thread already running, ignoring StartStreaming() call.");
      return false;
    }

    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Starting streaming thread for VideoCapture for " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));

    continue_stream_ = true;
    stream_thread_ = std::thread(&OpenCVIpCamSink::Receive, this);
    return true;
  }

  bool StopStreaming() override
  {
    if (continue_stream_)
    {
      VCP_LOG_DEBUG("OpenCVIpCamSink::StopStreaming()");
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Waiting for streaming thread to disconnect from IP VideoCapture " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));
      // Stop receiver thread.
      continue_stream_ = false;
      stream_thread_.join();
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Successfully stopped IP camera stream: " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));
    }
    return true;
  }

  SinkParams SinkParamsAt(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return params_;
  }

  size_t NumDevices() const override
  {
    return 1;
  }

  void SetVerbose(bool verbose) override
  {
    params_.verbose = verbose;
  }

  SinkType GetSinkType() const override
  {
    VCP_LOG_FAILURE("GetSinkType() should not be called for an OpenCVIpCamSink, use SinkParamsAt(stream_index).sink_type instead!");
    return SinkType::IPCAM_GENERIC;
  }

  vcp::best::calibration::StreamIntrinsics IntrinsicsAt(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return intrinsics_;
  }

  bool SetExtrinsicsAt(size_t stream_index, const cv::Mat &R, const cv::Mat &t) override
  {
    VCP_UNUSED_VAR(stream_index);
    return extrinsics_.SetExtrinsics(R, t, intrinsics_);
  }

  void ExtrinsicsAt(size_t stream_index, cv::Mat &R, cv::Mat &t) const override
  {
    VCP_UNUSED_VAR(stream_index);
    R = extrinsics_.R().clone();
    t = extrinsics_.t().clone();
  }

private:
  IpCameraSinkParams params_;
  std::unique_ptr<cv::VideoCapture> capture_;
  bool continue_stream_;

  std::thread stream_thread_;
  std::atomic<bool> is_stream_available_;
  mutable std::mutex image_queue_mutex_;
  std::unique_ptr<SinkBuffer> image_queue_;

  calibration::StreamIntrinsics intrinsics_;
  calibration::StreamExtrinsics extrinsics_;

#ifdef VCP_BEST_DEBUG_FRAMERATE
  std::chrono::high_resolution_clock::time_point previous_frame_timepoint_;
  double ms_between_frames_;
#endif // VCP_BEST_DEBUG_FRAMERATE


  void Receive()
  {
    while (continue_stream_)
    {
      // Get next frame
      cv::Mat retrieved, converted;
      if (capture_->grab())
      {
        capture_->retrieve(retrieved);

        if (params_.color_as_bgr || retrieved.empty())
          converted = retrieved;
        else
          converted = FlipChannels(retrieved);

        // Push into queue after applying configured transformations.
        const cv::Mat img = imutils::ApplyImageTransformations(converted, params_.transforms);
        image_queue_mutex_.lock();
        image_queue_->PushBack(img.clone());
        image_queue_mutex_.unlock();

#ifdef VCP_BEST_DEBUG_FRAMERATE
        const std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(now - previous_frame_timepoint_);
        previous_frame_timepoint_ = now;
        const double ms_ema_alpha = 0.1;

        if (ms_between_frames_ < 0.0)
          ms_between_frames_ = duration.count();
        else
          ms_between_frames_ = ms_ema_alpha * duration.count() + (1.0 - ms_ema_alpha) * ms_between_frames_;

        VCP_LOG_DEBUG_DEFAULT(
              "OpenCVIpCamSink \"" << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url) << "\" received frame after " << std::fixed << std::setw(5) << duration.count() << " ms, "
              << std::setw(5) << (1000.0 / ms_between_frames_) << " fps");
#endif // VCP_BEST_DEBUG_FRAMERATE
      }
    }
  }
};


std::unique_ptr<StreamSink> CreateBufferedOpenCVIpCamSink(
    const IpCameraSinkParams &params, std::unique_ptr<SinkBuffer> sink_buffer)
{
  return std::unique_ptr<OpenCVIpCamSink>(
        new OpenCVIpCamSink(params, std::move(sink_buffer)));
}

} // namespace videocap
} // namespace ipcam
} // namespace best
} // namespace vcp
