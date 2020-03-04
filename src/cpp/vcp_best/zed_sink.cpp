#include "zed_sink.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include <sstream>

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

namespace vcp
{
namespace best
{
namespace zed
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::zed"


ZedStreams ZedStreamsFromTokens(const std::vector<std::string> &tokens)
{
  ZedStreams streams = ZedStreams::NONE;
  for (const auto &token : tokens)
  {
    const std::string lower = vcp::utils::string::Lower(token);
    if (lower.compare("left") == 0 || lower.compare("l") == 0)
      streams |= ZedStreams::LEFT;
    else if (lower.compare("right") == 0 || lower.compare("r") == 0)
      streams |= ZedStreams::RIGHT;
    else if (lower.compare("depth") == 0 || lower.compare("d") == 0)
      streams |= ZedStreams::DEPTH;
    else
      VCP_ERROR("Cannot convert token '" << token << "' to ZedStreams enum.");
  }
  return streams;
}


ZedStreams ZedStreamsFromString(const std::string &s)
{
  return ZedStreamsFromTokens(
        vcp::utils::string::Split(
          vcp::utils::string::Replace(
            vcp::utils::string::Replace(s, ";", "-"),
            ",", "-"),
          '-'));
}

std::string ZedStreamsToString(const ZedStreams &s)
{
  std::vector<std::string> tokens;
  if ((s & ZedStreams::LEFT) == ZedStreams::LEFT)
    tokens.push_back("left");
  if ((s & ZedStreams::RIGHT) == ZedStreams::RIGHT)
    tokens.push_back("right");
  if ((s & ZedStreams::DEPTH) == ZedStreams::DEPTH)
    tokens.push_back("depth");
  std::stringstream str;
  for (size_t i = 0; i < tokens.size(); ++i)
  {
    if (i > 0)
      str << "-";
    str << tokens[i];
  }
  return str.str();
}

std::string ZedResolutionToString(const sl::RESOLUTION& r)
{
  switch(r)
  {
    case sl::RESOLUTION::HD2K:
      return "HD2K";
    case sl::RESOLUTION::HD1080:
      return "HD1080";
    case sl::RESOLUTION::HD720:
      return "HD720";
    case sl::RESOLUTION::VGA:
      return "VGA";
    default:
      VCP_ERROR("ZED sl::RESOLUTION '" << static_cast<int>(r) << "' is not yet supported.");
  }
}

sl::RESOLUTION ZedResolutionFromString(const std::string &s)
{
  const std::string lower = vcp::utils::string::Lower(s);
  if (lower.compare("2k") == 0
      || lower.compare("hd2k") == 0)
    return sl::RESOLUTION::HD2K;

  if (lower.compare("1080") == 0
      || lower.compare("hd1080") == 0
      || lower.compare("1080p") == 0)
    return sl::RESOLUTION::HD1080;

  if (lower.compare("720") == 0
      || lower.compare("hd720") == 0
      || lower.compare("720p") == 0)
    return sl::RESOLUTION::HD720;

  if (lower.compare("vga") == 0)
    return sl::RESOLUTION::VGA;

  VCP_ERROR("Cannot convert '" << s << "' to sl::RESOLUTION.");
}


std::string ZedDepthModeToString(const sl::DEPTH_MODE &d)
{
  switch(d)
  {
    case sl::DEPTH_MODE::NONE:
      return "none";
    case sl::DEPTH_MODE::PERFORMANCE:
      return "performance";
    case sl::DEPTH_MODE::QUALITY:
      return "quality";
    case sl::DEPTH_MODE::ULTRA:
      return "ultra";
    default:
      VCP_ERROR("ZED sl::DEPTH_MODE '" << static_cast<int>(d) << "' is not yet supported.");
  }
}

sl::DEPTH_MODE ZedDepthModeFromString(const std::string &s)
{
  const std::string lower = vcp::utils::string::Lower(s);
  if (lower.empty() || lower.compare("none") == 0)
    return sl::DEPTH_MODE::NONE;

  if (lower.compare("perf") == 0
      || lower.compare("performance") == 0)
    return sl::DEPTH_MODE::PERFORMANCE;

  if (lower.compare("qual") == 0
      || lower.compare("quality") == 0)
    return sl::DEPTH_MODE::QUALITY;

  if (lower.compare("ultra") == 0)
    return sl::DEPTH_MODE::ULTRA;

  VCP_ERROR("Cannot convert '" << s << "' to sl::DEPTH_MODE.");
}


std::string ZedSensingModeToString(const sl::SENSING_MODE &d)
{
  switch(d)
  {
    case sl::SENSING_MODE::STANDARD:
      return "standard";
    case sl::SENSING_MODE::FILL:
      return "fill";
    default:
      VCP_ERROR("ZED sl::SENSING_MODE '" << static_cast<int>(d) << "' is not yet supported.");
  }
}

sl::SENSING_MODE ZedSensingModeFromString(const std::string &s)
{
  const std::string lower = vcp::utils::string::Lower(s);
  if (lower.compare("standard") == 0
      || lower.compare("default") == 0)
    return sl::SENSING_MODE::STANDARD;

  if (lower.compare("fill") == 0)
    return sl::SENSING_MODE::FILL;

  VCP_ERROR("Cannot convert '" << s << "' to sl::SENSING_MODE.");
}

std::string ZedModelToString(const sl::MODEL &cm)
{
  switch (cm)
  {
    case sl::MODEL::ZED:
      return "ZED v1";
    case sl::MODEL::ZED2:
      return "ZED v2";
    case sl::MODEL::ZED_M:
      return "ZED Mini";
    default:
      VCP_ERROR("Unsupported ZED model '" << static_cast<int>(cm) << "'");
  }
  return std::string();
}

std::ostream &operator<< (std::ostream &out, const ZedSinkParams &p)
{
  out << p.sink_label << ", type: ";
  if (!p.model_name.empty())
    out << p.model_name;
  else
    out << "UNKNOWN";

  if (p.serial_number != std::numeric_limits<unsigned int>::max())
    out << ", #" << p.serial_number;
  return out;
}


bool ZedSinkParams::IsLeftStreamEnabled() const
{
  return (enabled_streams & ZedStreams::LEFT) == ZedStreams::LEFT;
}

bool ZedSinkParams::IsRightStreamEnabled() const
{
  return (enabled_streams & ZedStreams::RIGHT) == ZedStreams::RIGHT;
}

bool ZedSinkParams::IsDepthStreamEnabled() const
{
  return (enabled_streams & ZedStreams::DEPTH) == ZedStreams::DEPTH;
}

cv::Mat GetIntrinsics(const sl::CameraParameters &cp)
{
  cv::Mat K = (cv::Mat_<double>(3, 3)
               << cp.fx, 0.0, cp.cx,
               0.0, cp.fy, cp.cy,
               0.0, 0.0, 1.0);
  return K;
}

cv::Mat GetDistortion(const sl::CameraParameters &cp)
{
  cv::Mat D = (cv::Mat_<double>(5, 1)
               << cp.disto[0], cp.disto[1],
               cp.disto[2], cp.disto[3], cp.disto[4]);
  return D;
}

cv::Mat GetRotation(const sl::float3 &r)
{
  sl::Rotation rot;
  rot.setRotationVector(r);
  cv::Mat R = (cv::Mat_<double>(3, 3)
               << rot.r00, rot.r01, rot.r02,
               rot.r10, rot.r11, rot.r12,
               rot.r20, rot.r21, rot.r22);
  return R;
}

cv::Mat GetTranslation(const sl::float3 &t)
{
  cv::Mat T = (cv::Mat_<double>(3, 1)
               << t.x, t.y, t.z);
  return T;
}

// Cannot pass sl::CameraInformation as const reference, because the internal getters aren't declared const!
bool DumpCalibration(const ZedSinkParams &params, sl::CameraInformation &ci)
{
  // TODO Compare dumped calibration to factory pre-calibration: http://calib.stereolabs.com/?SN=12345
  if (params.calibration_file.empty())
  {
    VCP_LOG_FAILURE("If you want to dump the ZED calibration, you must specify the filename as calibration_file parameter!");
    return false;
  }

  cv::FileStorage fs(params.calibration_file, cv::FileStorage::WRITE);
  if (!fs.isOpened())
    VCP_ERROR("Cannot open '" << params.calibration_file << "' to store calibration!");

  // We store the rectified/undistorted calibration parameters, because by default we
  // yield only these frames. If you ever need unrectified/distorted images, change
  // the zed_->retrieveImage() calls inside Receive() to return sl::VIEW::..._UNRECTIFIED.
  sl::CalibrationParameters &calib = ci.calibration_parameters;

  if (params.IsLeftStreamEnabled() || params.IsDepthStreamEnabled())
  {
    const cv::Mat Kl = GetIntrinsics(calib.left_cam);
    const cv::Mat Dl = GetDistortion(calib.left_cam);
    const int wl = calib.left_cam.image_size.width;
    const int hl = calib.left_cam.image_size.height;

    if (params.IsLeftStreamEnabled())
    {
      fs << "K_left" << Kl;
      fs << "D_left" << Dl;
      fs << "width_left" << wl;
      fs << "height_left" << hl;
    }
    if (params.IsDepthStreamEnabled())
    {
      fs << "K_depth" << Kl;
      fs << "D_depth" << Dl;
      fs << "width_depth" << wl;
      fs << "height_depth" << hl;
    }
  }

  if (params.IsRightStreamEnabled())
  {
    const cv::Mat Kr = GetIntrinsics(calib.right_cam);
    const cv::Mat Dr = GetDistortion(calib.right_cam);
    const int wr = calib.right_cam.image_size.width;
    const int hr = calib.right_cam.image_size.height;

    fs << "K_right" << Kr;
    fs << "D_right" << Dr;
    fs << "width_right" << wr;
    fs << "height_right" << hr;
  }

  // Store stereo transformation if needed
  if (params.IsDepthStreamEnabled() || params.IsRightStreamEnabled())
  {
    const cv::Mat R = GetRotation(calib.R);
    const cv::Mat t = GetTranslation(calib.T);

    fs << "R_right2left" << R;
    fs << "t_right2left" << t;
    fs << "baseline" << static_cast<double>(calib.getCameraBaseline());
  }

  fs << "sink_type" << SinkTypeToString(params.sink_type);
  fs << "label" << params.sink_label;

  fs << "type" << "stereo";
  fs.release();

  if (params.verbose)
    VCP_LOG_INFO("ZED calibration has been saved to '" << params.calibration_file << "'."
                 << std::endl << "          Change the camera's 'calibration_file' parameter if you want to prevent overwriting it upon the next start.");

  return true;
}

cv::Mat ToCvMat(sl::Mat &input)
{
  // Mapping between MAT_TYPE and CV_TYPE
  int cv_type = -1;
  switch (input.getDataType()) {
    case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
    case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
    case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
    case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
    case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
    case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
    case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
    case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
    default:
      VCP_ERROR("Unsupported sl::MAT_TYPE = " << static_cast<int>(input.getDataType()));
  }

  // cv::Mat and sl::Mat will share a single memory structure
  return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}


class ZedSink : public StreamSink
{
public:
  ZedSink(std::unique_ptr<SinkBuffer> sink_buffer_left,
          std::unique_ptr<SinkBuffer> sink_buffer_right,
          std::unique_ptr<SinkBuffer> sink_buffer_depth,
          const ZedSinkParams &params) : StreamSink(),
    continue_capture_(false),
    image_queue_left_(std::move(sink_buffer_left)),
    image_queue_right_(std::move(sink_buffer_right)),
    depth_queue_(std::move(sink_buffer_depth)),
    zed_(nullptr), params_(params),
    is_left_enabled_(params.IsLeftStreamEnabled()),
    is_right_enabled_(params.IsRightStreamEnabled()),
    is_depth_enabled_(params.IsDepthStreamEnabled())
  {
#ifdef VCP_BEST_DEBUG_FRAMERATE
    previous_frame_timepoint_ = std::chrono::high_resolution_clock::now();
    ms_between_frames_ = -1.0;
#endif // VCP_BEST_DEBUG_FRAMERATE
  }

  virtual ~ZedSink()
  {
    CloseDevice();
  }

  bool OpenDevice() override
  {
    if (zed_ && zed_->isOpened())
    {
      VCP_LOG_FAILURE("Device [" << params_ << "] already opened - ignoring OpenDevice() call");
      return false;
    }

    if (!zed_)
      zed_.reset(new sl::Camera());

    sl::InitParameters init_params;
    init_params.camera_resolution = params_.resolution;
    init_params.camera_fps = params_.fps;
    init_params.camera_image_flip = params_.flip_image ? 1 : 0;
    init_params.camera_disable_self_calib = params_.disable_self_calibration;
    init_params.depth_mode = params_.depth_mode;
    init_params.coordinate_units = params_.depth_in_meters ? sl::UNIT::METER : sl::UNIT::MILLIMETER;
    init_params.depth_stabilization = params_.depth_stabilization ? 1 : 0;
    init_params.enable_image_enhancement = params_.enable_image_enhancement;
    init_params.sdk_gpu_id = params_.gpu_id;
    // We only need the default depth measures (i.e. aligned to the left view)
    // If you ever change this, make sure to adjust the calibration, too (see DumpCalibration)
    init_params.enable_right_side_measure = false;

    if (params_.device_id >= 0)
      init_params.input.setFromCameraID(static_cast<unsigned int>(params_.device_id));
    if (params_.serial_number != std::numeric_limits<unsigned int>::max())
      init_params.input.setFromSerialNumber(params_.serial_number);

    const sl::ERROR_CODE err = zed_->open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS)
    {
      VCP_LOG_FAILURE("Cannot open ZED device, sl::ERROR_CODE = " << err);
      return false;
    }

    sl::CameraInformation ci = zed_->getCameraInformation();
    params_.model_name = ZedModelToString(ci.camera_model);
    params_.serial_number = ci.serial_number;

    if (params_.verbose)
    {
      VCP_LOG_INFO_DEFAULT("Opened ZED camera (" << params_.model_name << "), SN: " << params_.serial_number
                           << ", Firmware: " << ci.camera_firmware_version << ", "
                           << ci.camera_resolution.width << "x" << ci.camera_resolution.height
                           << " @" << ci.camera_fps << " fps");
    }

    if (params_.write_calibration)
      return DumpCalibration(params_, ci);

    return true;
  }

  bool CloseDevice() override
  {
    if (zed_ && zed_->isOpened())
    {
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Closing " << params_);
      zed_->close();
      zed_.reset();
    }
    return true;
  }

  bool StartStreaming() override
  {
    if (!zed_|| !zed_->isOpened())
      OpenDevice();

    if (continue_capture_)
    {
      VCP_LOG_FAILURE("Streaming thread already running, ignoring StartStreaming() call.");
      return false;
    }

    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Starting streaming thread for " << params_);
    continue_capture_ = true;
    stream_thread_ = std::thread(&ZedSink::Receive, this);
    return true;
  }


  bool StopStreaming() override
  {
    if (continue_capture_)
    {
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Waiting for streaming thread to disconnect from " << params_);
      continue_capture_ = false;
      stream_thread_.join();
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Disconnected from " << params_);
    }
    return true;
  }


  std::vector<cv::Mat> Next() override
  {
    cv::Mat left, right, depth;
    std::vector<cv::Mat> res;
    image_queue_mutex_.lock();
    if (is_left_enabled_)
    {
      if (image_queue_left_->Empty())
        left = cv::Mat();
      else
      {
        // Retrieve oldest image in queue
        left = image_queue_left_->Front().clone();
        image_queue_left_->PopFront();
      }
      res.push_back(left);
    }

    if (is_right_enabled_)
    {
      if (image_queue_right_->Empty())
        right = cv::Mat();
      else
      {
        // Retrieve oldest image in queue
        right = image_queue_right_->Front().clone();
        image_queue_right_->PopFront();
      }
      res.push_back(right);
    }

    if (is_depth_enabled_)
    {
      if (depth_queue_->Empty())
        depth = cv::Mat();
      else
      {
        // Retrieve oldest image in queue
        depth = depth_queue_->Front().clone();
        depth_queue_->PopFront();
      }
      res.push_back(depth);
    }

    image_queue_mutex_.unlock();
    return res;
  }


  int IsDeviceAvailable() const override
  {
    if (zed_ && zed_->isOpened())
      return 1;
    return 0;
  }

  int IsFrameAvailable() const override
  {
    image_queue_mutex_.lock();
    const bool empty = (is_left_enabled_ && image_queue_left_->Empty())
        || (is_right_enabled_ && image_queue_right_->Empty())
        || (is_depth_enabled_ && depth_queue_->Empty());
    image_queue_mutex_.unlock();
    if (empty)
      return 0;
    return 1;
  }

  size_t NumAvailableFrames() const override
  {
    size_t num = 0;
    image_queue_mutex_.lock();
    if (is_left_enabled_ && !image_queue_left_->Empty())
        ++num;
    if (is_right_enabled_ && !image_queue_right_->Empty())
        ++num;
    if (is_depth_enabled_ && !depth_queue_->Empty())
        ++num;
    image_queue_mutex_.unlock();
    return num;
  }

  size_t NumStreams() const override
  {
    size_t ns = 0;
    if (is_left_enabled_)
      ++ns;
    if (is_right_enabled_)
      ++ns;
    if (is_depth_enabled_)
      ++ns;
    return ns;
  }

  FrameType FrameTypeAt(size_t stream_index) const override
  {
    std::vector<FrameType> types;
    if (is_left_enabled_)
      types.push_back(FrameType::MONOCULAR);
    if (is_right_enabled_)
      types.push_back(FrameType::MONOCULAR);
    if (is_depth_enabled_)
      types.push_back(FrameType::DEPTH);
    if (stream_index >= types.size())
      VCP_ERROR("stream_index " << stream_index << " is out-of-bounds");
    return types[stream_index];
  }

  std::string StreamLabel(size_t stream_index) const override
  {
    std::vector<std::string> labels;
    if (is_left_enabled_)
      labels.push_back(params_.sink_label + "-left");
    if (is_right_enabled_)
      labels.push_back(params_.sink_label + "-right");
    if (is_depth_enabled_)
      labels.push_back(params_.sink_label + "-depth");
    if (stream_index >= labels.size())
      VCP_ERROR("stream_index " << stream_index << " is out-of-bounds");
    return labels[stream_index];
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

private:
  std::atomic<bool> continue_capture_;
  std::unique_ptr<SinkBuffer> image_queue_left_;
  std::unique_ptr<SinkBuffer> image_queue_right_;
  std::unique_ptr<SinkBuffer> depth_queue_;
  std::unique_ptr<sl::Camera> zed_;
  ZedSinkParams params_;
  bool is_left_enabled_;
  bool is_right_enabled_;
  bool is_depth_enabled_;

#ifdef VCP_BEST_DEBUG_FRAMERATE
  std::chrono::high_resolution_clock::time_point previous_frame_timepoint_;
  double ms_between_frames_;
#endif // VCP_BEST_DEBUG_FRAMERATE

  std::thread stream_thread_;
  mutable std::mutex image_queue_mutex_;

  sl::Mat slimg_left, slimg_right, sl_depth;
  // OpenCV matrix headers which share image data with sl::Mat
  cv::Mat cvl, cvr, cvd;
  // Color-converted:
  cv::Mat cvtl, cvtr;
  // Transformed matrices (basic image transformations)
  cv::Mat tcvl, tcvr, tcvd;

  void Receive()
  {
    sl::RuntimeParameters rt_params;
    rt_params.sensing_mode = params_.sensing_mode;
    rt_params.enable_depth = params_.IsDepthStreamEnabled();
    rt_params.confidence_threshold = params_.confidence_threshold;
    rt_params.textureness_confidence_threshold = params_.textureness_threshold;

    while (continue_capture_)
    {
      const sl::ERROR_CODE err = zed_->grab(rt_params);
      if (err == sl::ERROR_CODE::SUCCESS)
      {
        if (is_left_enabled_)
        {
          zed_->retrieveImage(slimg_left, sl::VIEW::LEFT, sl::MEM::CPU);
          cvl = ToCvMat(slimg_left);
          if (params_.color_as_bgr)
            cvtl = cvl;
          else
            cv::cvtColor(cvl, cvtl, cvl.channels() == 3 ? cv::COLOR_BGR2RGB : cv::COLOR_BGRA2RGBA);
          tcvl = imutils::ApplyImageTransformations(cvtl, params_.transforms);
        }
        else
          tcvl = cv::Mat();

        if (is_right_enabled_)
        {
          zed_->retrieveImage(slimg_right, sl::VIEW::RIGHT, sl::MEM::CPU);
          cvr = ToCvMat(slimg_right);
          if (params_.color_as_bgr)
            cvtr = cvr;
          else
            cv::cvtColor(cvr, cvtr, cvr.channels() == 3 ? cv::COLOR_BGR2RGB : cv::COLOR_BGRA2RGBA);
          tcvr = imutils::ApplyImageTransformations(cvtr, params_.transforms);
        }
        else
          tcvr = cv::Mat();

        if (is_depth_enabled_)
        {
          zed_->retrieveMeasure(sl_depth, sl::MEASURE::DEPTH, sl::MEM::CPU);
          cvd = ToCvMat(sl_depth);
          tcvd = imutils::ApplyImageTransformations(cvd, params_.transforms);
        }
        else
          tcvd = cv::Mat();

        image_queue_mutex_.lock();
        if (is_left_enabled_)
          image_queue_left_->PushBack(tcvl.clone());
        if (is_right_enabled_)
          image_queue_right_->PushBack(tcvr.clone());
        if (is_depth_enabled_)
          depth_queue_->PushBack(tcvd.clone());
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

        VCP_LOG_DEBUG_DEFAULT("ZedSink [" << params_ << "] received frame after " << std::fixed << std::setw(5) << duration.count() << " ms, "
                            << std::setw(5) << (1000.0 / ms_between_frames_) << " fps");
#endif // VCP_BEST_DEBUG_FRAMERATE
      }
      else
      {
        VCP_LOG_WARNING("ZED frame request timed out, sl::ERROR_CODE = " << err);
      }
    }
    zed_->close();
    zed_.reset();
  }
};


bool IsZedSink(const std::string &type_param)
{
  const std::string type = vcp::utils::string::Lower(type_param);
  if (type.compare("zed") == 0
      || type.compare("zed2") == 0
      || type.compare("zed-cam") == 0)
  {
    return true;
  }
  return false;
}


ZedSinkParams ZedSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param)
{
  std::vector<std::string> configured_keys = config.ListConfigGroupParameters(cam_param);
  const SinkParams sink_params = ParseBaseSinkParamsFromConfig(config, cam_param, configured_keys);

  ZedSinkParams params(sink_params);

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "streams"), configured_keys.end());
  params.enabled_streams = ZedStreamsFromString(GetOptionalStringFromConfig(config, cam_param,
                                  "streams", ZedStreamsToString(params.enabled_streams)));

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "resolution"), configured_keys.end());
  params.resolution = ZedResolutionFromString(GetOptionalStringFromConfig(config, cam_param,
                                  "resolution", ZedResolutionToString(sl::RESOLUTION::HD720)));

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "depth_mode"), configured_keys.end());
  params.depth_mode = ZedDepthModeFromString(GetOptionalStringFromConfig(config, cam_param,
                                  "depth_mode", ZedDepthModeToString(sl::DEPTH_MODE::ULTRA)));

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "depth_in_meters"), configured_keys.end());
  params.depth_in_meters = GetOptionalBoolFromConfig(config, cam_param, "depth_in_meters", false);

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "depth_stabilization"), configured_keys.end());
  params.depth_stabilization = GetOptionalBoolFromConfig(config, cam_param, "depth_stabilization", true);

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "fps"), configured_keys.end());
  params.fps = GetOptionalIntFromConfig(config, cam_param, "fps", params.fps);

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "flip_image"), configured_keys.end());
  params.flip_image = GetOptionalBoolFromConfig(config, cam_param, "flip_image", false);

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "disable_self_calibration"), configured_keys.end());
  params.disable_self_calibration = GetOptionalBoolFromConfig(config, cam_param, "disable_self_calibration", false);

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "sensing_mode"), configured_keys.end());
  params.sensing_mode = ZedSensingModeFromString(
        GetOptionalStringFromConfig(config, cam_param, "sensing_mode",
                                    ZedSensingModeToString(params.sensing_mode)));

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "confidence_threshold"), configured_keys.end());
  params.confidence_threshold = GetOptionalIntFromConfig(config, cam_param, "confidence_threshold", params.confidence_threshold);

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "textureness_threshold"), configured_keys.end());
  params.textureness_threshold = GetOptionalIntFromConfig(config, cam_param, "textureness_threshold", params.textureness_threshold);

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "gpu_id"), configured_keys.end());
  params.gpu_id = GetOptionalIntFromConfig(config, cam_param, "gpu_id", params.gpu_id);

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "enable_image_enhancement"), configured_keys.end());
  params.enable_image_enhancement = GetOptionalBoolFromConfig(config, cam_param, "enable_image_enhancement", true);

  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "write_calibration"), configured_keys.end());
  params.write_calibration = GetOptionalBoolFromConfig(config, cam_param, "write_calibration", false);

  const std::string ksn = cam_param + ".serial_number";
  if (config.SettingExists(ksn))
  {
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "serial_number"), configured_keys.end());
    params.serial_number = config.GetUnsignedInteger(ksn);
  }

  const std::string kdev = cam_param + ".device";
  if (config.SettingExists(kdev))
  {
    if (config.SettingExists(ksn))
      VCP_ERROR("You cannot specify both the device ID and the serial number of the ZED cam '" << cam_param << "'.");
    params.device_id = config.GetInteger(kdev);
  }

  WarnOfUnusedParameters(cam_param, configured_keys);
  return params;
}


std::vector<ZedDeviceInfo> ListZedDevices(bool warn_if_no_devices, bool list_unavailable_devices)
{
  std::vector<ZedDeviceInfo> infos;
  const std::vector<sl::DeviceProperties> devs = sl::Camera::getDeviceList();
  if (devs.empty())
  {
    if (warn_if_no_devices)
      VCP_LOG_WARNING("No ZED cameras connected!");
  }
  else
  {
    for (const auto &dp : devs)
    {
      ZedDeviceInfo info;
      info.available = dp.camera_state == sl::CAMERA_STATE::AVAILABLE;
      info.serial_number = dp.serial_number;
      info.device_path = std::string(dp.path.c_str());
      info.model_name = ZedModelToString(dp.camera_model);
      if (info.available || (!info.available && list_unavailable_devices))
        infos.push_back(info);
    }
    if (infos.empty() && warn_if_no_devices)
      VCP_LOG_WARNING("All connected ZED cameras are already in use!");
  }

  return infos;
}

std::unique_ptr<StreamSink> CreateBufferedZedSink(const ZedSinkParams &params,
    std::unique_ptr<SinkBuffer> sink_buffer_left, std::unique_ptr<SinkBuffer> sink_buffer_right,
                                                  std::unique_ptr<SinkBuffer> sink_buffer_depth)
{
  return std::unique_ptr<ZedSink>(new ZedSink(std::move(sink_buffer_left),
                                              std::move(sink_buffer_right),
                                              std::move(sink_buffer_depth), params));
}

} // namespace webcam
} // namespace best
} // namespace vcp
