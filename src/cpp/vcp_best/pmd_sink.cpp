#include "pmd_sink.h"

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

#include <royale.hpp>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/file_utils.h>
#include <vcp_imutils/opencv_compatibility.h>
#include <vcp_imutils/matutils.h>

//#define VCP_VERBOSE_TIMING
//#include <vcp_utils/timing_utils.h>

namespace vcp
{
namespace best
{
namespace pmd
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::pmd"

std::ostream &operator<< (std::ostream &out, const PmdSinkParams &p)
{
  out << "PMD['" << p.sink_label << "', s/n " << p.serial_number;
  if (p.rectify)
    out << ", rectified";
  else
    out << ", distorted";
  out << "]";
  return out;
}


class PmdSink : public StreamSink, public royale::IDepthDataListener
{
public:
  PmdSink(std::unique_ptr<SinkBuffer> sink_buffer_gray,
          std::unique_ptr<SinkBuffer> sink_buffer_depth,
          std::unique_ptr<SinkBuffer> sink_buffer_xyz,
          const PmdSinkParams &params) : StreamSink(),
    continue_capture_(false),
    image_queue_gray_(std::move(sink_buffer_gray)),
    depth_queue_(std::move(sink_buffer_depth)),
    xyz_queue_(std::move(sink_buffer_xyz)),
    pmd_camera_(nullptr), params_(params)
  {
#ifdef VCP_BEST_DEBUG_FRAMERATE
    previous_frame_timepoint_ = std::chrono::high_resolution_clock::now();
    ms_between_frames_ = -1.0;
#endif // VCP_BEST_DEBUG_FRAMERATE
  }

  virtual ~PmdSink()
  {
    CloseDevice();
  }

  bool OpenDevice() override
  {
    if (IsConnected())
    {
      VCP_LOG_FAILURE("Device [" << params_ << "] already connected - ignoring OpenDevice() call");
      return false;
    }

    royale::CameraManager manager;
    if (params_.serial_number.empty())
    {
      royale::Vector<royale::String> dev_list (manager.getConnectedCameraList());
      if (dev_list.empty())
      {
        VCP_LOG_FAILURE("No available PMD device!");
        return false;
      }
      params_.serial_number = std::string(dev_list[0].data());
      dev_list.clear();
    }
    pmd_camera_ = manager.createCamera(params_.serial_number);

    auto status = pmd_camera_->initialize();
    if (status != royale::CameraStatus::SUCCESS)
    {
      VCP_LOG_FAILURE("Cannot initialize PMD sensor, error: " << royale::getErrorString(status) << "!");
      pmd_camera_.reset();
      return false;
    }

    royale::LensParameters intrinsics;
    status = pmd_camera_->getLensParameters(intrinsics);
    if (status != royale::CameraStatus::SUCCESS)
    {
      VCP_LOG_FAILURE("Cannot retrieve intrinsics for PMD sensor, error: " << royale::getErrorString(status) << "!");
      pmd_camera_.reset();
      return false;
    }
    SetIntrinsics(intrinsics);

    status = pmd_camera_->registerDataListener(this);
    if (status != royale::CameraStatus::SUCCESS)
    {
      VCP_LOG_FAILURE("Cannot register data listener for PMD sensor, error: " << royale::getErrorString(status) << "!");
      pmd_camera_.reset();
      return false;
    }

    if (params_.verbose)
    {
      VCP_LOG_INFO_DEFAULT(
            "Opened PMD sensor (" << params_.serial_number << ")");
    }

    return true;
  }

  bool CloseDevice() override
  {
    if (pmd_camera_)
    {
      pmd_camera_.reset();
    }
    return true;
  }

  bool StartStreaming() override
  {
    if (!pmd_camera_)
      OpenDevice();

    auto status = pmd_camera_->startCapture();
    if (status != royale::CameraStatus::SUCCESS)
    {
      VCP_LOG_FAILURE("Cannot start streaming from PMD sensor, error: " << royale::getErrorString(status) << "!");
      return false;
    }

    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Started streaming for " << params_);
    return true;
  }


  bool StopStreaming() override
  {
    if (pmd_camera_)
    {
      auto status = pmd_camera_->stopCapture();
      if (status != royale::CameraStatus::SUCCESS)
      {
        VCP_LOG_FAILURE("Cannot stop streaming from PMD sensor, error: " << royale::getErrorString(status) << "!");
        return false;
      }
      else
      {
        return true;
      }
    }
    else
    {
      return false;
    }
  }


  std::vector<cv::Mat> Next() override
  {
    cv::Mat gray, depth, xyz;
    std::vector<cv::Mat> res;
    image_queue_mutex_.lock();

    if (depth_queue_->Empty())
    {
      depth = cv::Mat();
    }
    else
    {
      // Retrieve oldest image in queue
      depth = depth_queue_->Front().clone();
      depth_queue_->PopFront();
    }
    res.push_back(depth);

    if (params_.enable_gray)
    {
      if (image_queue_gray_->Empty())
      {
        gray = cv::Mat();
      }
      else
      {
        // Retrieve oldest image in queue
        gray = image_queue_gray_->Front().clone();
        image_queue_gray_->PopFront();
      }
      res.push_back(gray);
    }

    if (params_.enable_pointcloud)
    {
      if (xyz_queue_->Empty())
      {
        xyz = cv::Mat();
      }
      else
      {
        xyz = xyz_queue_->Front().clone();
        xyz_queue_->PopFront();
      }
      res.push_back(xyz);
    }

    image_queue_mutex_.unlock();
    return res;
  }


  int IsDeviceAvailable() const override
  {
    if (IsConnected())
      return 1;
    return 0;
  }

  int IsFrameAvailable() const override
  {
    image_queue_mutex_.lock();
    bool empty = depth_queue_->Empty();
    if (params_.enable_gray)
    {
      empty = empty || image_queue_gray_->Empty();
    }
    if (params_.enable_pointcloud)
    {
      empty = empty || xyz_queue_->Empty();
    }
    image_queue_mutex_.unlock();
    if (empty)
      return 0;
    return 1;
  }

  size_t NumAvailableFrames() const override
  {
    size_t num = 0;
    image_queue_mutex_.lock();
    if (!depth_queue_->Empty())
        ++num;
    if (params_.enable_gray && !image_queue_gray_->Empty())
        ++num;
    if (params_.enable_pointcloud && !xyz_queue_->Empty())
        ++num;
    image_queue_mutex_.unlock();
    return num;
  }

  size_t NumStreams() const override
  {
    size_t cnt = 1;
    if (params_.enable_gray)
      ++cnt;
    if (params_.enable_pointcloud)
      ++cnt;
    return cnt;
  }

  FrameType FrameTypeAt(size_t stream_index) const override
  {
    std::vector<FrameType> types { FrameType::DEPTH };
    if (params_.enable_gray)
      types.push_back(FrameType::MONOCULAR);
    if (params_.enable_pointcloud)
      types.push_back(FrameType::POINTCLOUD);

    if (stream_index >= types.size())
      VCP_ERROR("stream_index " << stream_index << " is out-of-bounds");
    return types[stream_index];
  }

  std::string StreamLabel(size_t stream_index) const override
  {
    std::vector<std::string> labels = { params_.sink_label + "-depth"};
    if (params_.enable_gray)
      labels.push_back(params_.sink_label + "-gray");
    if (params_.enable_pointcloud)
      labels.push_back(params_.sink_label + "-xyz");

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

  vcp::best::calibration::StreamIntrinsics IntrinsicsAt(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return intrinsics_;
  }

  bool SetExtrinsicsAt(size_t stream_index, const cv::Mat &R, const cv::Mat &t) override
  {
    VCP_UNUSED_VAR(stream_index);
    return extrinsics_.SetExtrinsics(R, t);
  }

  void ExtrinsicsAt(size_t stream_index, cv::Mat &R, cv::Mat &t) const override
  {
    VCP_UNUSED_VAR(stream_index);
    R = extrinsics_.R().clone();
    t = extrinsics_.t().clone();
  }

  void SetVerbose(bool verbose) override
  {
    params_.verbose = verbose;
  }

  SinkType GetSinkType() const override
  {
    return SinkType::PMD;
  }

  void onNewData (const royale::DepthData *data) override
  {
    // Callback from the royale SDK once a new data frame is available.
    // Decoding 1 frame and pushing the gray, depth and xyz frames takes
    // around 1.2ms.
    std::lock_guard<std::mutex> lock(receive_mutex_);

    cv::Mat depth16(data->height, data->width, CV_16UC1, cv::Scalar::all(0.0));
    cv::Mat gray8(data->height, data->width, CV_8UC1, cv::Scalar::all(0.0));
    cv::Mat xyz(data->height, data->width, CV_32FC3, cv::Scalar::all(0.0));
    //TODO set default to quiet NaN?

    std::size_t data_idx = 0;
    for (int row = 0; row < data->height; ++row)
    {
      uint16_t *depth_ptr = depth16.ptr<uint16_t>(row);
      unsigned char *gray_ptr = gray8.ptr<unsigned char>(row);
      cv::Vec3f *xyz_ptr = xyz.ptr<cv::Vec3f>(row);
      for (int col = 0; col < data->width; ++col, ++data_idx)
      {
        const auto &point = data->points.at(data_idx);
        if ((point.depthConfidence > params_.confidence_threshold)
            && (point.noise < params_.noise_std_threshold))
        {
          // Meters --> millimeters
          depth_ptr[col] = static_cast<uint16_t>(point.z * 1000.0f);
          if (params_.enable_gray)
          {
            gray_ptr[col] = static_cast<unsigned char>(
                  std::min(255.0f, static_cast<float>(point.grayValue) / params_.gray_divisor * 255.0f));
          }
          if (params_.enable_pointcloud)
          {
            xyz_ptr[col].val[0] = point.x * 1000.0f;
            xyz_ptr[col].val[1] = point.y * 1000.0f;
            xyz_ptr[col].val[2] = point.z * 1000.0f;
          }
        }
      }
    }

    // Upon receiving the first frame, we have to precompute the
    // rectification mapping:
    if (params_.rectify && intrinsics_.Empty())
    {
      intrinsics_ = calibration::StreamIntrinsics::FromMonocular(
            camera_matrix_, distortion_coefficients_, params_.sink_label,
            std::string(), gray8.size());
    }

    cv::Mat frame_gray, frame_depth;
    if (params_.rectify)
    {
      frame_depth = intrinsics_.UndistortRectify(depth16);
      if (params_.enable_gray)
      {
        frame_gray = intrinsics_.UndistortRectify(gray8);
      }
    }
    else
    {
      frame_depth = depth16;
      frame_gray = gray8;
    }

//    const cv::Mat img = imutils::ApplyImageTransformations(converted, params_.transforms);
    image_queue_mutex_.lock();
    depth_queue_->PushBack(frame_depth.clone());
    if (params_.enable_gray)
    {
      image_queue_gray_->PushBack(frame_gray.clone());
    }
    if (params_.enable_pointcloud)
    {
      xyz_queue_->PushBack(xyz.clone());
    }
    image_queue_mutex_.unlock();
  }


private:
  std::atomic<bool> continue_capture_;
  std::unique_ptr<SinkBuffer> image_queue_gray_;
  std::unique_ptr<SinkBuffer> depth_queue_;
  std::unique_ptr<SinkBuffer> xyz_queue_;
  std::unique_ptr<royale::ICameraDevice> pmd_camera_;
  PmdSinkParams params_;
  calibration::StreamIntrinsics intrinsics_;
  calibration::StreamExtrinsics extrinsics_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;

#ifdef VCP_BEST_DEBUG_FRAMERATE
  std::chrono::high_resolution_clock::time_point previous_frame_timepoint_;
  double ms_between_frames_;
#endif // VCP_BEST_DEBUG_FRAMERATE

  std::thread stream_thread_;
  mutable std::mutex image_queue_mutex_;
  std::mutex receive_mutex_;

  bool IsConnected() const
  {
    bool connected = false;
    if (pmd_camera_)
      pmd_camera_->isConnected(connected);
    return connected;
  }

  void SetIntrinsics(const royale::LensParameters &lens)
  {
    camera_matrix_ = (cv::Mat1d (3, 3) << lens.focalLength.first, 0, lens.principalPoint.first,
                 0, lens.focalLength.second, lens.principalPoint.second,
                 0, 0, 1);

    distortion_coefficients_ = (cv::Mat1d (5, 1) << lens.distortionRadial[0],
        lens.distortionRadial[1], lens.distortionTangential.first,
        lens.distortionTangential.second, lens.distortionRadial[2]);
  }
};


bool IsPmdSink(const std::string &type_param)
{
  const std::string type = vcp::utils::string::Lower(type_param);
  if (type.compare("pmd") == 0)
  {
    return true;
  }
  return false;
}


PmdSinkParams PmdSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param)
{
  std::vector<std::string> configured_keys = config.ListConfigGroupParameters(cam_param);
  const SinkParams sink_params = ParseBaseSinkParamsFromConfig(config, cam_param, configured_keys);

  PmdSinkParams params(sink_params);

  params.serial_number = GetOptionalStringFromConfig(
        config, cam_param, "serial_number", std::string());
  configured_keys.erase(
        std::remove(configured_keys.begin(), configured_keys.end(),
                    "serial_number"), configured_keys.end());

  params.confidence_threshold = static_cast<uint8_t>(
        GetOptionalIntFromConfig(
          config, cam_param, "confidence_threshold", params.confidence_threshold));
  configured_keys.erase(
        std::remove(configured_keys.begin(), configured_keys.end(),
                    "confidence_threshold"), configured_keys.end());

  params.noise_std_threshold = static_cast<float>(GetOptionalDoubleFromConfig(
        config, cam_param, "noise_std_threshold", params.noise_std_threshold));
  configured_keys.erase(
        std::remove(configured_keys.begin(), configured_keys.end(),
                    "noise_std_threshold"), configured_keys.end());

  params.gray_divisor = static_cast<float>(GetOptionalDoubleFromConfig(
        config, cam_param, "gray_divisor", params.gray_divisor));
  configured_keys.erase(
        std::remove(configured_keys.begin(), configured_keys.end(),
                    "gray_divisor"), configured_keys.end());

  params.enable_gray = GetOptionalBoolFromConfig(
        config, cam_param, "enable_gray", params.enable_gray);
  configured_keys.erase(
        std::remove(configured_keys.begin(), configured_keys.end(),
                    "enable_gray"), configured_keys.end());

  params.enable_pointcloud = GetOptionalBoolFromConfig(
        config, cam_param, "enable_pointcloud", params.enable_pointcloud);
  configured_keys.erase(
        std::remove(configured_keys.begin(), configured_keys.end(),
                    "enable_pointcloud"), configured_keys.end());

  WarnOfUnusedParameters(cam_param, configured_keys);
  return params;
}


std::vector<PmdDeviceInfo> ListPmdDevices(bool warn_if_no_devices)
{
  std::vector<PmdDeviceInfo> infos;

  royale::CameraManager manager;
  royale::Vector<royale::String> devs (manager.getConnectedCameraList());

  if (devs.empty())
  {
    if (warn_if_no_devices)
      VCP_LOG_WARNING("No PMD sensors connected!");
  }
  else
  {
    for (const auto &dp : devs)
    {
      PmdDeviceInfo info;
      info.serial_number = std::string(dp.data());
      infos.push_back(info);
    }
  }

  return infos;
}


std::unique_ptr<StreamSink> CreateBufferedPmdSink(
    const PmdSinkParams &params, std::unique_ptr<SinkBuffer> sink_buffer_gray,
    std::unique_ptr<SinkBuffer> sink_buffer_depth,
    std::unique_ptr<SinkBuffer> sink_buffer_xyz)
{
  return std::unique_ptr<PmdSink>(
        new PmdSink(std::move(sink_buffer_gray),
                    std::move(sink_buffer_depth),
                    std::move(sink_buffer_xyz), params));
}

} // namespace webcam
} // namespace best
} // namespace vcp
