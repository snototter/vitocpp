#include "webcam_sink.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <map>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/highgui/highgui.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/highgui.hpp>
    #include <opencv2/imgproc.hpp>
#endif

#ifdef VCP_BEST_DEBUG_FRAMERATE
    #include <chrono>
    #include <iomanip>
#endif // VCP_BEST_DEBUG_FRAMERATE

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/file_utils.h>
#include <vcp_imutils/opencv_compatibility.h>


/**
 * Notes:
 * I tried to implement it without OpenCV once (using FFMPEG to be more
 * flexible/closer to the device). However, there we observed
 * rolling shutter-like effects when streaming from HD webcams.
 * OpenCV didn't have these issues.
 */


namespace vcp
{
namespace best
{
namespace webcam
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::webcam"


std::ostream &operator<< (std::ostream &out, const WebcamSinkParams &p)
{
  out << p.sink_label << ", device " << p.device_number;
  return out;
}

class WebcamSink : public StreamSink
{
public:
  WebcamSink(std::unique_ptr<SinkBuffer> sink_buffer,
             const WebcamSinkParams &params) : StreamSink(),
    continue_capture_(false), image_queue_(std::move(sink_buffer)),
    capture_(nullptr), params_(params)
  {
#ifdef VCP_BEST_DEBUG_FRAMERATE
    previous_frame_timepoint_ = std::chrono::high_resolution_clock::now();
    ms_between_frames_ = -1.0;
#endif // VCP_BEST_DEBUG_FRAMERATE
  }

  virtual ~WebcamSink()
  {
    CloseDevice();
  }

  bool OpenDevice() override
  {
    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Opening webcam: " << params_);

    if (capture_ && capture_->isOpened())
    {
      VCP_LOG_FAILURE("Webcam #" << params_.device_number << " already opened - ignoring OpenDevice() call");
      return false;
    }

    if (params_.device_number < 0)
    {
#if defined(__linux__) || defined(__unix__)
      const auto devices = ListWebcams(false, false);
      if (devices.empty())
      {
        VCP_LOG_FAILURE("No webcam is connected to your PC!");
        return false;
      }
      params_.device_number = devices[0].device_nr;

      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Trying to open webcam /dev/" << devices[0].name);
#else
      VCP_LOG_FAILURE("Searching for a webcam is only supported on unix-based operating systems!");
      return false;
#endif
    }

    if (capture_)
      capture_->open(params_.device_number);
    else
      capture_ = std::unique_ptr<cv::VideoCapture>(new cv::VideoCapture(params_.device_number));

    if (!capture_->isOpened())
    {
      VCP_LOG_FAILURE("Cannot open webcam: " << params_.device_number);
      return false;
    }

    // Set the requested image size:
    if (params_.resolution.width > 0 && params_.resolution.height > 0)
    {
#if CV_MAJOR_VERSION < 3
      capture_->set(CV_CAP_PROP_FRAME_WIDTH, params_.resolution.width);
      capture_->set(CV_CAP_PROP_FRAME_HEIGHT, params_.resolution.height);
#else
      capture_->set(cv::CAP_PROP_FRAME_WIDTH, params_.resolution.width);
      capture_->set(cv::CAP_PROP_FRAME_HEIGHT, params_.resolution.height);
#endif
    }

    // Set frame rate if requested:
    if (params_.fps > 0.0)
    {
#if CV_MAJOR_VERSION < 3
      capture_->set(CV_CAP_PROP_FPS, params_.fps);
#else
      capture_->set(cv::CAP_PROP_FPS, params_.fps);
#endif
    }
    return true;
  }

  bool CloseDevice() override
  {
    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Closing webcam #" << params_.device_number);
    capture_.reset();
    return true;
  }

  bool StartStreaming() override
  {
    if (!capture_ || !capture_->isOpened())
      OpenDevice();

    if (continue_capture_)
    {
      VCP_LOG_FAILURE("Streaming thread already running, ignoring StartStreaming() call.");
      return false;
    }

    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Starting streaming thread for webcam #" << params_.device_number);
    continue_capture_ = true;
    stream_thread_ = std::thread(&WebcamSink::Receive, this);
    return true;
  }


  bool StopStreaming() override
  {
    if (continue_capture_)
    {
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Waiting for streaming thread to disconnect from webcam #" << params_.device_number);
      continue_capture_ = false;
      stream_thread_.join();
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Disconnected from webcam #" << params_.device_number);
    }
    return true;
  }


  std::vector<cv::Mat> Next() override
  {
    std::vector<cv::Mat> frames;
    cv::Mat frame;
    image_queue_mutex_.lock();
    if (image_queue_->Empty())
    {
      frame = cv::Mat();
    }
    else
    {
      // Retrieve oldest image in queue.
      frame = image_queue_->Front().clone();
      image_queue_->PopFront();
    }
    image_queue_mutex_.unlock();
    frames.push_back(frame);
    return frames;
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

  size_t NumAvailableFrames() const override
  {
    return static_cast<size_t>(IsFrameAvailable());
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
    return SinkType::WEBCAM;
  }

private:
  std::atomic<bool> continue_capture_;
  std::unique_ptr<SinkBuffer> image_queue_;
  std::unique_ptr<cv::VideoCapture> capture_;
  WebcamSinkParams params_;

#ifdef VCP_BEST_DEBUG_FRAMERATE
  std::chrono::high_resolution_clock::time_point previous_frame_timepoint_;
  double ms_between_frames_;
#endif // VCP_BEST_DEBUG_FRAMERATE

  std::thread stream_thread_;
  mutable std::mutex image_queue_mutex_;

  void Receive()
  {
    while (continue_capture_)
    {
      // Get next frame
      cv::Mat retrieved, converted;
      if (capture_->grab())
      {
        capture_->retrieve(retrieved);

        if (params_.color_as_bgr || retrieved.empty())
          converted = retrieved;
        else
          cv::cvtColor(retrieved, converted, CV_BGR2RGB);

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

        VCP_LOG_DEBUG_DEFAULT("WebcamSink #" << params_.device_number << " received frame after " << std::fixed << std::setw(5) << duration.count() << " ms, "
                            << std::setw(5) << (1000.0 / ms_between_frames_) << " fps");
#endif // VCP_BEST_DEBUG_FRAMERATE
      }
    }
    capture_.reset();
  }
};


bool IsWebcamSink(const std::string &type_param)
{
  const std::string type = vcp::utils::string::Lower(type_param);
  if (type.compare("webcam") == 0
      || type.compare("web_cam") == 0
      || type.compare("web-cam") == 0)
  {
    return true;
  }
  return false;
}


WebcamSinkParams WebcamSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param)
{
  std::vector<std::string> configured_keys = config.ListConfigGroupParameters(cam_param);
  const SinkParams sink_params = ParseBaseSinkParamsFromConfig(config, cam_param, configured_keys);

  int device;
  if (config.SettingExists(cam_param + ".device"))
  {
    device = config.GetInteger(cam_param + ".device");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "device"), configured_keys.end());
  }
  else if (config.SettingExists(cam_param + ".device_number"))
  {
    device = config.GetInteger(cam_param + ".device_number");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "device_number"), configured_keys.end());
  }
  else if (config.SettingExists(cam_param + ".number"))
  {
    device = config.GetInteger(cam_param + ".number");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "number"), configured_keys.end());
  }
  else
  {
    VCP_ERROR("Cannot find device parameter for '" << cam_param << "'. Use either 'device', 'device_number' or 'number' to specify it.");
  }

  const cv::Size resolution = ParseResolutionFromConfig(config, cam_param, std::string(), configured_keys);

  const double frame_rate = GetOptionalDoubleFromConfig(config, cam_param, "fps", -1.0);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "fps"), configured_keys.end());

  WarnOfUnusedParameters(cam_param, configured_keys);

  return WebcamSinkParams(sink_params, device, resolution, frame_rate);
}


std::vector<WebcamDeviceInfo> ListWebcams(bool warn_if_no_devices, bool include_incompatible_devices)
{
  // https://unix.stackexchange.com/questions/344784/how-to-map-sys-bus-usb-devices-to-dev-video
  std::vector<WebcamDeviceInfo> infos;
#if defined(__linux__) || defined(__unix__)
  bool (*entry_filter)(const std::string &) = include_incompatible_devices ?
      [](const std::string &f) -> bool { return vcp::utils::string::StartsWith(f, "video"); }
  :
      [](const std::string &f) -> bool {
          if (vcp::utils::string::StartsWith(f, "video"))
          {
            // We need to skip RealSense devices, as we cannot open them with default OpenCV settings.
            // Similarly, we skip Azure Kinect devices, as we had some troubles with frequently failing
            // color streams.
            const std::string name = vcp::utils::file::SlurpAsciiFile(
                  vcp::utils::file::FullFile("/sys/class/video4linux/", vcp::utils::file::FullFile(f, "name")));
            // Return false for the directory list if the hardware info contains "RealSense" or "Azure Kinect"
            return name.find("RealSense") == std::string::npos && name.find("Azure Kinect") == std::string::npos;
          }
          return false;
        };
  const std::vector<std::string> devices = vcp::utils::file::ListDirContents("/dev",
                                                                             entry_filter, true, true, true, &vcp::utils::file::filename_filter::CompareFileLengthsAndNames);

  std::map<std::string, WebcamDeviceInfo> collected;
  for (const auto &dev : devices)
  {
    WebcamDeviceInfo inf;
    inf.dev_name = "/dev/" + dev;
    inf.device_nr = std::atoi(dev.substr(5).c_str()); // Strip the beginning 'video'
    inf.name = vcp::utils::file::SlurpAsciiFile(
          vcp::utils::file::FullFile("/sys/class/video4linux/", vcp::utils::file::FullFile(dev, "name")));
    vcp::utils::string::Trim(inf.name);

    // We need to combine the *** symlinked devices (each physical device can have up to 64 /dev/video* nodes)
    const std::string resolved_path = vcp::utils::file::RealPath("/sys/class/video4linux/" + dev);
    const size_t pos = resolved_path.find("video4linux");
    if (pos != std::string::npos)
    {
      const std::string realpath = resolved_path.substr(0, pos);
      auto ires = collected.insert(std::pair<std::string, WebcamDeviceInfo>(realpath, inf));
      if (ires.second == false)
      {
        if (ires.first->second > inf)
          ires.first->second = inf;
      }
    }
    else
    {
      VCP_LOG_FAILURE("Resolved video device path '" << resolved_path << "' doesn't contain 'video4linux'."
                      << std::endl << "Adding it to the list of available devices.");
      infos.push_back(inf);
    }
  }

  for (auto it = collected.begin(); it != collected.end(); it++)
    infos.push_back(it->second);

#else
  // Non-unix
  VCP_LOG_FAILURE("Listing available webcams is only supported for unix-based operating systems!");
#endif

  if (devices.empty() && warn_if_no_devices)
  {
    VCP_LOG_WARNING("No webcam connected!");
  }
  return infos;
}


std::unique_ptr<StreamSink> CreateBufferedWebcamSink(
    const WebcamSinkParams &params,
    std::unique_ptr<SinkBuffer> sink_buffer)
{
  return std::unique_ptr<WebcamSink>(new WebcamSink(std::move(sink_buffer), params));
}

} // namespace webcam
} // namespace best
} // namespace vcp
