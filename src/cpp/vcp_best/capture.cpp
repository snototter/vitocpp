#include "capture.h"
#include "sink.h"

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/string_utils.h>
#include <vcp_math/geometry3d.h>
#include <vcp_utils/file_utils.h>
#include <vcp_utils/sort_utils.h>

#include <sstream>
#include <exception>
#include <thread>
#include <iomanip>

#include "file_sink.h"
#include "webcam_sink.h"

//FIXME #include "rectifier.h"
#ifdef VCP_BEST_WITH_IPCAM
  #include "ipcam_sink.h"
#endif
#ifdef VCP_BEST_WITH_K4A
  #include "k4a_sink.h"
#endif
//#ifdef WITH_MATRIXVISION
//  #include "capture_matrixvision.h"
//#endif
#ifdef VCP_BEST_WITH_REALSENSE2
  #include "realsense2_sink.h"
#endif
#include <chrono>

//FIXME add WaitForNext(timeout)

namespace vcp
{
namespace best
{

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best"

class MultiDeviceCapture : public Capture
{
public:
  MultiDeviceCapture(const vcp::config::ConfigParams &config) : Capture()
  {
    VCP_LOG_DEBUG("MultiDeviceCapture::MultiDeviceCapture()");
    num_devices_ = 0;
    LoadConfig(config);
    SanityCheck();
  }

  void LoadConfig(const vcp::config::ConfigParams &config)
  {
    VCP_LOG_DEBUG("MultiDeviceCapture::LoadConfig()");

    std::vector<file::ImageDirectorySinkParams> imgdir_params;
#ifdef VCP_BEST_WITH_IPCAM
    std::vector<ipcam::IpCameraSinkParams> ipcam_params;
#endif
//#ifdef WITH_MATRIXVISION
//    std::vector<MvBlueFox3SinkParams> mvbluefox_params;
//#endif
#ifdef VCP_BEST_WITH_K4A
    std::vector<k4a::K4ASinkParams> k4a_params;
#endif
#ifdef VCP_BEST_WITH_REALSENSE2
    std::vector<realsense2::RealSense2SinkParams> realsense2_params;
#endif
    std::vector<file::VideoFileSinkParams> video_params;
    std::vector<webcam::WebcamSinkParams> webcam_params;


    // Filter all "camera[0-9,a-z]*" and "sink[0-9,a-z]*" parameters out of the configuration:
    const std::vector<std::string> cam_config_names = GetCameraConfigParameterNames(config);
    // Parse those into the correct device/sink type configuration:
    for (const auto &cam_config_name : cam_config_names)
    {
      const SinkType sink_type = GetSinkTypeFromConfig(config, cam_config_name);
      switch(sink_type)
      {
        case SinkType::IMAGE_DIR:
          imgdir_params.push_back(file::ImageDirectorySinkParamsFromConfig(config, cam_config_name));
          break;

        case SinkType::VIDEO_FILE:
          video_params.push_back(file::VideoFileSinkParamsFromConfig(config, cam_config_name));
          break;

        case SinkType::WEBCAM:
          webcam_params.push_back(webcam::WebcamSinkParamsFromConfig(config, cam_config_name));
          break;

#ifdef VCP_BEST_WITH_IPCAM
        case SinkType::IPCAM_MONOCULAR:
          ipcam_params.push_back(ipcam::MonocularIpCameraSinkParamsFromConfig(config, cam_config_name));
          break;

        case SinkType::IPCAM_STEREO:
          {
            const auto &stereo_params = ipcam::StereoIpCameraSinkParamsFromConfig(config, cam_config_name);
            ipcam_params.push_back(stereo_params.first);
            ipcam_params.push_back(stereo_params.second);
          }
          break;
#endif

#ifdef VCP_BEST_WITH_K4A
        case SinkType::K4A:
          k4a_params.push_back(k4a::K4ASinkParamsFromConfig(config, cam_config_name));
          break;
#endif // VCP_BEST_WITH_K4A

#ifdef VCP_BEST_WITH_REALSENSE2
        case SinkType::REALSENSE:
          realsense2_params.push_back(realsense2::RealSense2SinkParamsFromConfig(config, cam_config_name));
          break;
#endif // VCP_BEST_WITH_REALSENSE2

        default:
          VCP_LOG_FAILURE("Sink type '" << sink_type << "' is not yet supported!");
          break;
      }
    }
//#ifdef WITH_MATRIXVISION
//      else if (IsMatrixVision(cam_type))
//      {
//        mvbluefox_params.push_back(MvBlueFox3SinkParamsFromConfig(config, id.str()));
//        cck_mvbluefox.push_back(id.str());
//      }
//#endif // WITH_MATRIXVISION

    // Initialize the individual captures
    for (const auto &p : imgdir_params)
      AddSink(file::CreateImageDirectorySink(p));

    for (const auto &p : video_params)
      AddSink(file::CreateVideoFileSink(p));

    for (const auto &p : webcam_params)
      AddSink(webcam::CreateWebcamSink<VCP_BEST_STREAM_BUFFER_CAPACITY>(p));

#ifdef VCP_BEST_WITH_IPCAM
    // Empty vector is gracefully handled by AddSink(nullptr)
    AddSink(ipcam::CreateIpCameraSink(ipcam_params));
#endif
#ifdef WITH_IPCAMERA
    if (!ip_mono_params.empty())
    {
      if (!ip_stereo_params.empty())
        PVT_LOG_FAILURE("Connecting to a mixture of monocular and stereo IP camera setups will not work (if you're streaming via RTSP). Expect empty/invalid frames!");

      sinks_.push_back(pvt::icc::CreateMonoIpCamSink(ip_mono_params));
      for (const auto &p : ip_mono_params)
      {
        AddLabel(p.label);
        AddCalibrationFile(p.calibration_file);
        AddStreamType(StreamType::MONO);
        sink_types_.push_back(SinkType::IPCAM_MONOCULAR);
      }
      AddCorrespondingConfigKeys(cck_ip_mono);
    }

    if (!ip_stereo_params.empty())
    {
      sinks_.push_back(pvt::icc::CreateStereoIpCamSink(ip_stereo_params));
      for (const auto &p : ip_stereo_params)
      {
        AddLabel(p.label);
        AddCalibrationFile(p.calibration_file);
        AddStreamType(StreamType::STEREO);
        sink_types_.push_back(SinkType::IPCAM_STEREO);
      }
      AddCorrespondingConfigKeys(cck_ip_stereo);
    }
#endif // WITH_IPCAMERA

#ifdef VCP_BEST_WITH_K4A
    AddK4ASinks(k4a_params);
#endif // VCP_BEST_WITH_K4A

#ifdef WITH_MATRIXVISION
    if (!mvbluefox_params.empty())
    {
      for (const auto &p : mvbluefox_params)
      {
        sinks_.push_back(pvt::icc::CreateSink(p));
        AddLabel(p.label);
        AddCalibrationFile(p.calibration_file);
        AddStreamType(p.stream_type);
        sink_types_.push_back(SinkType::MVBLUEFOX3);
      }
      AddCorrespondingConfigKeys(cck_mvbluefox);
    }
#endif // WITH_MATRIXVISION

#ifdef VCP_BEST_WITH_REALSENSE2
    for (const auto &p : realsense2_params)
      AddSink(realsense2::CreateRealSense2Sink<VCP_BEST_STREAM_BUFFER_CAPACITY>(p));
    multiple_realsenses_ = realsense2_params.size() > 1;
#endif

    num_devices_ = 0;
    for (const auto &s : sinks_)
      num_devices_ += s->NumDevices();
  }

  void AddSink(std::unique_ptr<StreamSink> sink)
  {
    if (sink == nullptr)
      return;

    for (size_t i = 0; i < sink->NumStreams(); ++i)
    {
      sink_params_.push_back(sink->SinkParamsAt(i));
      frame_types_.push_back(sink->FrameTypeAt(i));
      frame_labels_.push_back(sink->StreamLabel(i));
    }
    sinks_.push_back(std::move(sink));
  }

#ifdef VCP_BEST_WITH_K4A
    void AddK4ASinks(const std::vector<k4a::K4ASinkParams> &k4a_params)
    {
      // Check if there are multiple sync'ed K4A devices or not.
      if (k4a_params.empty())
        return;
      bool k4a_requires_sync = false;
      if (k4a_params.size() > 1)
      {
        for (const auto &p : k4a_params)
          k4a_requires_sync |= p.RequiresWiredSync();
      }

      if (k4a_requires_sync)
      {
        //TODO: we need a separate multi-k4a-capture/sink class :-/
        // See best practices in green screen demo:
        // https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/green_screen/MultiDeviceCapturer.h
        VCP_ERROR("Synchronization of multiple K4As is not yet supported!");
      }
      else
      {
        for (const auto &p : k4a_params)
          AddSink(k4a::CreateK4ASink<VCP_BEST_STREAM_BUFFER_CAPACITY>(p));
      }
    }
#endif // VCP_BEST_WITH_K4A

  virtual ~MultiDeviceCapture()
  {
    VCP_LOG_DEBUG("MultiDeviceCapture::~MultiDeviceCapture()");
    CloseDevices();
    for (size_t i = 0; i < sinks_.size(); ++i)
      sinks_[i].reset();
  }

  size_t NumStreams() const override
  {
    return sink_params_.size();
  }

  size_t NumDevices() const override
  {
    return num_devices_;
  }

  std::vector<std::string> ConfigurationKeys() const override
  {
    std::vector<std::string> keys;
    for (const auto &p : sink_params_)
      keys.push_back(p.configuration_key);
    return keys;
  }

  std::vector<std::string> FrameLabels() const override
  {
    return frame_labels_;
  }

  std::vector<FrameType> FrameTypes() const override
  {
    return frame_types_;
  }

  bool AreAllDevicesAvailable() const override
  {
    VCP_LOG_DEBUG("AreDevicesAvailable()");
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      if (!sinks_[i]->IsDeviceAvailable())
        return false;
    }
    return true;
  }


  bool AreAllFramesAvailable() const override
  {
    VCP_LOG_DEBUG("AreAllFramesAvailable()");
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      if (!sinks_[i]->IsFrameAvailable())
        return false;
    }
    return true;
  }


  size_t NumAvailableFrames() const override
  {
    VCP_LOG_DEBUG("NumAvailableFrames()");
    size_t num = 0;
    for (const auto &sink : sinks_)
      num += sink->NumAvailableFrames();
    return num;
  }


  bool OpenDevices() override
  {
    VCP_LOG_DEBUG("MultiDeviceCapture::OpenDevices()");
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success = success && sinks_[i]->OpenDevice();
    return success;
  }


  bool CloseDevices() override
  {
    StopStreams();
    VCP_LOG_DEBUG("MultiDeviceCapture::CloseDevices()");
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success = success && sinks_[i]->CloseDevice();
    return success;
  }


  bool StartStreams() override
  {
    VCP_LOG_DEBUG("MultiDeviceCapture::StopStreams()");
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      success = success && sinks_[i]->StartStreaming();
#ifdef VCP_BEST_WITH_REALSENSE2
      // According to the RealSense white paper on multiple sensors, they
      // "found it useful to start the cameras sequentially in time with some delay".
      if (multiple_realsenses_ && (i+1) < sinks_.size()
          && sink_params_[i].sink_type == sink_params_[i+1].sink_type
          && sink_params_[i].sink_type == SinkType::REALSENSE)
      {
        VCP_LOG_INFO("Pausing capture initialization temporarily before starting the next RealSense stream!");
        std::this_thread::sleep_for(std::chrono::milliseconds(1500)); // 1.5 sec worked okay-ish so far...
      }
#endif // VCP_WITH_REALSENSE2
    }
#ifdef VCP_BEST_DEBUG_FRAMERATE
  prev_frame_timestamp_ = std::chrono::high_resolution_clock::now();
  ms_between_frames_ = -1.0;
#endif // VCP_BEST_DEBUG_FRAMERATE
    return success;
  }

  bool WaitForInitialFrames(double timeout_ms) const override
  {
    VCP_LOG_DEBUG("MultiDeviceCapture::WaitForInitialFrames()");
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    double elapsed_ms = 0;
    while (elapsed_ms < timeout_ms)
    {
      bool available = true;
      for (size_t i = 0; i < sinks_.size(); ++i)
        available = available && sinks_[i]->IsFrameAvailable();
      if (available)
        return true;
      VCP_LOG_WARNING_NSEC("Not all sinks are ready. Continue waiting for "
                        << std::fixed << std::setprecision(2) << (std::max(timeout_ms - elapsed_ms, 0.0) / 1000) << " sec.", 0.5);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                std::chrono::high_resolution_clock::now() - start);
      elapsed_ms = duration.count();
    }
    VCP_LOG_FAILURE("Not all sinks are ready. Aborting after " << std::fixed << std::setprecision(2) << elapsed_ms/1000 << " sec.");
    return false;
  }


  bool StopStreams() override
  {
    VCP_LOG_DEBUG("MultiDeviceCapture::StopStreams()");
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success = success && sinks_[i]->StopStreaming();
    return success;
  }


  std::vector<cv::Mat> Next() override
  {
    VCP_LOG_DEBUG("Next()");
#ifdef VCP_BEST_DEBUG_FRAMERATE
    const std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(now - prev_frame_timestamp_);
    const double ms_ema_alpha = 0.1;
    prev_frame_timestamp_ = now;
    if (ms_between_frames_ < 0.0)
      ms_between_frames_ = duration.count();
    else
      ms_between_frames_ = ms_ema_alpha * duration.count() + (1.0 - ms_ema_alpha) * ms_between_frames_;
    VCP_LOG_DEBUG_DEFAULT("Next() called after " << std::fixed << std::setw(5) << duration.count() << " ms, "
                         << std::setw(5) << (1000.0 / ms_between_frames_) << "fps");
#endif // VCP_BEST_DEBUG_FRAMERATE

    // Collect the frames of all captures and store them in a single vector
    std::vector<cv::Mat> frames;
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      std::vector<cv::Mat> captured = sinks_[i]->Next();
      frames.insert(frames.end(), captured.begin(), captured.end());
    }
    return frames;
  }


  std::vector<cv::Mat> Previous() override
  {
    VCP_LOG_DEBUG("Previous()");
    // Collect frames of all sinks - may throw an exception if not implemented.
    std::vector<cv::Mat> frames;
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      std::vector<cv::Mat> captured = sinks_[i]->Previous();
      frames.insert(frames.end(), captured.begin(), captured.end());
    }
    return frames;
  }


  std::vector<cv::Mat> FastForward(size_t num_frames) override
  {
    VCP_LOG_DEBUG("FastForward(" << num_frames << ")");
    // Collect frames of all sinks - may throw an exception if not implemented.
    std::vector<cv::Mat> frames;
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      std::vector<cv::Mat> captured = sinks_[i]->FastForward(num_frames);
      frames.insert(frames.end(), captured.begin(), captured.end());
    }
    return frames;
  }

private:
  std::vector<std::unique_ptr<StreamSink>> sinks_; // Potentially less than streams/frames
  std::vector<FrameType> frame_types_; // Note: they will be per stream/frame, not per device
  std::vector<SinkParams> sink_params_; // Note: they will be per stream/frame, not per device
  std::vector<std::string> frame_labels_;
  size_t num_devices_;

#ifdef VCP_BEST_DEBUG_FRAMERATE
  std::chrono::high_resolution_clock::time_point prev_frame_timestamp_;
  double ms_between_frames_;
#endif // VCP_BEST_DEBUG_FRAMERATE

#ifdef VCP_BEST_WITH_REALSENSE2
  bool multiple_realsenses_;
#endif // VCP_BEST_WITH_REALSENSE2

  void SanityCheck()
  {
    if (!vcp::utils::HasUniqueItems(frame_labels_))
      VCP_ERROR("Frame labels aren't unique - aborting before you'll run into runtime issues!");
  }
};


std::ostream& operator<<(std::ostream & os, const Capture &cap)
{
  // Query all interesting fields.
  const bool av_devs = cap.AreAllDevicesAvailable();
  const bool av_frames = cap.AreAllFramesAvailable();
  const size_t nd = cap.NumDevices();
  const size_t ns = cap.NumStreams();
  const auto ftypes = cap.FrameTypes();
  std::vector<std::string> ftypes_str;
  const auto cfg_keys = cap.ConfigurationKeys();
  const auto flbls = cap.FrameLabels();

  // Prepare for formatting:
  size_t longest_ftype_len = 0;
  size_t longest_lbl_len = 0;
  size_t longest_cfg_len = 0;

  for (size_t i = 0; i < ns; ++i)
  {
    const std::string fts = FrameTypeToString(ftypes[i]);
    if (fts.length() > longest_ftype_len)
      longest_ftype_len = fts.length();
    ftypes_str.push_back(fts);

    if (flbls[i].length() > longest_lbl_len)
      longest_lbl_len = flbls[i].length();

    if (cfg_keys[i].length() > longest_cfg_len)
      longest_cfg_len = cfg_keys[i].length();
  }

/*
Print a header:

Capture:
    Frame#  Label     Type        Config. Param
  -----------------------------------------------
         0  camera1   monocular   camera1
         1  Mountain  unknown     camera42
         2  camera99  rgbd-image  camera99
         3  Bamboo    unknown     camera8642

*/
  longest_lbl_len = std::max(longest_lbl_len, std::string("Type").length());
  longest_ftype_len = std::max(longest_ftype_len, static_cast<size_t>(2));
  const size_t longest_fnr = std::max(vcp::utils::string::ToStr(ns).length(), std::string("Stream").length());
  // Add a header:
  os << "Capture:" << std::endl
     << "    " << std::setw(longest_fnr) << "Stream" << "  "
     << std::setw(longest_lbl_len) << std::left << "Label"
     << "  " << std::setw(longest_ftype_len) << std::left << "Type"
     << "  Config. Param" << std::endl << "  ";
  const size_t divider_len = std::max(static_cast<size_t>(13), longest_cfg_len) \
      + longest_lbl_len + longest_ftype_len + longest_fnr + 10;
  for (size_t i = 0; i < divider_len; ++i)
    os << "-";
  os << std::endl;

  for (size_t i = 0; i < ns; ++i)
     os << "    " << std::setw(longest_fnr) << std::right << i << "  " << std::left
        << std::setw(longest_lbl_len) << flbls[i] << "  "
        << std::setw(longest_ftype_len) << ftypes_str[i] << "  "
        << std::setw(longest_cfg_len) << cfg_keys[i] << std::endl;

  os << "  ";
  for (size_t i = 0; i < divider_len; ++i)
    os << "-";
  os << std::endl;

  os << "    " << ns << " " << (ns == 1 ? "stream" : "streams") << " from " << nd << " " << (nd == 1 ? "device" : "devices") << std::endl;
  if (nd == 1)
    os << "    " << (av_devs ? '*' : '!') << " Device " << (av_devs ? "is" : "is not") << " available" << std::endl;
  else
    os << "    " << (av_devs ? '*' : '!') << " Devices " << (av_devs ? "are" : "are not") << " available" << std::endl;

  if (ns == 1)
    os << "    " << (av_frames ? '*' : '!') << " Frame " << (av_frames ? "is" : "is not") << " enqueued" << std::endl;
  else
    os << "    " << (av_frames ? '*' : '!') << " Frames " << (av_frames ? "are" : "are not") << " enqueued" << std::endl;

  return os;
}


std::unique_ptr<Capture> CreateCapture(const vcp::config::ConfigParams &config)
{
  return std::unique_ptr<MultiDeviceCapture>(new MultiDeviceCapture(config));
}


////--------------------------------------------------------------------------------------------
//// Rectified Capture

//class RectifiedCaptureImpl : public RectifiedCapture
//{
//public:
//  RectifiedCaptureImpl(const pvt::config::ConfigParams &config, bool force_loading_extrinsics) : RectifiedCapture()
//  {
//    plain_capture_ = pvt::icc::CreateRawCapture(config);
//    if (config.SettingExists("multicam_calibration"))
//    {
//      LoadCombinedCalibration(config, force_loading_extrinsics);
//    }
//    else
//    {
//      LoadSeparateIntrinsics();
//      LoadSeparateExtrinsics(config, force_loading_extrinsics);
//    }
//  }

//  virtual ~RectifiedCaptureImpl() {}

//  bool IsAvailable() const override { return plain_capture_->IsAvailable(); }

//  bool IsFrameAvailable() const override { return plain_capture_->IsFrameAvailable(); }

//  bool Start() override { return plain_capture_->Start(); }

//  std::vector<cv::Mat> NextFrame() override
//  {
//    const std::vector<cv::Mat> frames = plain_capture_->NextFrame();
//    std::vector<cv::Mat> rectified;
//    rectified.reserve(frames.size());

//    for (size_t i = 0; i < frames.size(); ++i)
//      rectified.push_back(rectifier_[i]->Rectify(frames[i]));

//    return rectified;
//  }


//  std::vector<cv::Mat> PreviousFrame() override
//  {
//    const std::vector<cv::Mat> frames = plain_capture_->PreviousFrame();
//    std::vector<cv::Mat> rectified;
//    rectified.reserve(frames.size());

//    for (size_t i = 0; i < frames.size(); ++i)
//      rectified.push_back(rectifier_[i]->Rectify(frames[i]));

//    return rectified;
//  }


//  std::vector<cv::Mat> FastForward(size_t num_frames) override
//  {
//    const std::vector<cv::Mat> frames = plain_capture_->FastForward(num_frames);
//    std::vector<cv::Mat> rectified;
//    rectified.reserve(frames.size());

//    for (size_t i = 0; i < frames.size(); ++i)
//      rectified.push_back(rectifier_[i]->Rectify(frames[i]));

//    return rectified;
//  }


//  bool Terminate() override { return plain_capture_->Terminate(); }

//  cv::Mat K(size_t sink_index) const override { return rectifier_[sink_index]->K(); }

//  cv::Mat P2(size_t sink_index) const override { return rectifier_[sink_index]->P2(); }

//  cv::Mat R(size_t sink_index) const override
//  {
//    if (extrinsics_.empty())
//      return cv::Mat();
//    return extrinsics_[sink_index].first;
//  }

//  cv::Mat t(size_t sink_index) const override
//  {
//    if (extrinsics_.empty())
//      return cv::Mat();
//    return extrinsics_[sink_index].second;
//  }

//  cv::Mat C(size_t sink_index) const override
//  {
//    if (extrinsics_.empty())
//      return cv::Mat();
//    const cv::Mat R = extrinsics_[sink_index].first;
//    const cv::Mat t = extrinsics_[sink_index].second;
//    if (R.empty() || t.empty())
//      return cv::Mat();
//    return -R.t() * t;
//  }

//  cv::Vec4d ImagePlane(size_t sink_index) const override
//  {
//    if (extrinsics_.empty())
//      return cv::Vec4d();
//    return pvt::math::geo3d::ImagePlaneInWorldCoordinateSystem(extrinsics_[sink_index].first, extrinsics_[sink_index].second);
//  }

//  void StereoRt(size_t sink_index, cv::Mat &R, cv::Mat &t) const override
//  {
//    if (rectifier_.empty())
//    {
//      R = cv::Mat();
//      t = cv::Mat();
//    }
//    else
//    {
//      rectifier_[sink_index]->StereoExtrinsics(R, t);
//    }
//  }

//  size_t NumStreams() const override { return plain_capture_->NumStreams(); }
//  std::string GetSinkLabel(size_t sink_index) const override { return plain_capture_->GetSinkLabel(sink_index); }
//  std::string GetCalibrationFile(size_t sink_index) const override { return plain_capture_->GetCalibrationFile(sink_index); }
//  std::string GetCorrespondingConfigKey(size_t sink_index) const override { return plain_capture_->GetCorrespondingConfigKey(sink_index); }
//  StreamType GetStreamType(size_t stream_index) const override { return plain_capture_->GetStreamType(stream_index); }
//  std::vector<StreamType> GetStreamTypes() const override { return plain_capture_->GetStreamTypes(); }

//  void SaveCalibration(const std::string &filename) const override
//  {
//    if (!IsAvailable())
//    {
//      throw std::runtime_error("Capture must be started/available while saving the calibration - we need to query the stream resolutions!");
//    }

//    const size_t num_streams = NumStreams();

//    // Query frame sizes
//    std::vector<cv::Size> frame_sizes;
//    frame_sizes.reserve(num_streams);
//    while (true)
//    {
//      if (plain_capture_->IsFrameAvailable())
//      {
//        std::vector<cv::Mat> frames = plain_capture_->NextFrame();
//        bool all_frames_valid = true;
//        for (const auto &f : frames)
//          all_frames_valid = all_frames_valid && !f.empty();

//        if (all_frames_valid)
//        {
//          for (const auto &f : frames)
//            frame_sizes.push_back(cv::Size(f.cols, f.rows));
//          break;
//        }
//      }
//      else
//      {
//        std::this_thread::sleep_for(std::chrono::milliseconds(50));
//      }
//    }

//    // Store calibration for all configured streams:
//    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
//    for (size_t i = 0; i < num_streams; ++i)
//    {
//      std::stringstream ss;
//      if (GetStreamType(i) != StreamType::STEREO)
//      {
//        ss << "K" << (i+1);
//        fs << ss.str() << K(i);

//        ss.str("");
//        ss.clear();
//        ss << "D" << (i+1);
//        fs << ss.str() << cv::Mat::zeros(5,1, CV_64FC1);
//      }
//      else
//      {
//        ss << "K" << (i+1) << "-1";
//        fs << ss.str() << K(i);

//        ss.str(""); ss.clear();
//        ss << "D" << (i+1) << "-1";
//        fs << ss.str() << cv::Mat::zeros(5,1, CV_64FC1);

//        const cv::Mat Pstereo2 = P2(i);
//        ss.str(""); ss.clear();
//        ss << "K" << (i+1) << "-2";
//        fs << ss.str() << Pstereo2.colRange(0, 3);

//        ss.str(""); ss.clear();
//        ss << "D" << (i+1) << "-2";
//        fs << ss.str() << cv::Mat::zeros(5,1, CV_64FC1);

//        // Store relative stereo extrinsics
//        cv::Mat Rstereo, tstereo;
//        StereoRt(i, Rstereo, tstereo);

//        ss.str(""); ss.clear();
//        ss << "R" << (i+1) << "-stereo";
//        fs << ss.str() << Rstereo;

//        ss.str(""); ss.clear();
//        ss << "t" << (i+1) << "-stereo";
//        fs << ss.str() << tstereo;
//      }

//      ss.str("");
//      ss.clear();
//      ss << "R" << (i+1);
//      fs << ss.str() << R(i);

//      ss.str("");
//      ss.clear();
//      ss << "t" << (i+1);
//      fs << ss.str() << t(i);

//      ss.str("");
//      ss.clear();
//      ss << "img_width" << (i+1);
//      const int width = GetStreamType(i) == StreamType::STEREO ? (frame_sizes[i].width / 2) : frame_sizes[i].width;
//      fs << ss.str() << width;

//      ss.str("");
//      ss.clear();
//      ss << "img_height" << (i+1);
//      fs << ss.str() << frame_sizes[i].height;

//      ss.str("");
//      ss.clear();
//      ss << "label" << (i+1);
//      fs << ss.str() << GetSinkLabel(i);
//    }
//    fs.release();
//  }


//private:
//  std::unique_ptr<pvt::icc::Capture> plain_capture_;
//  std::vector<std::unique_ptr<Rectifier>> rectifier_;
//  std::vector<std::pair<cv::Mat, cv::Mat>> extrinsics_;

//  void LoadCombinedCalibration(const pvt::config::ConfigParams &config, bool force_loading_extrinsics)
//  {
//    const bool has_extrinsics = config.SettingExists("multicam_calibration");
//    if (!has_extrinsics)
//    {
//      if (force_loading_extrinsics)
//        throw std::runtime_error("Configuration doesn't provide a multicam_calibration file (loading extrinsics was forced)");
//      return;
//    }

//    const std::string filename = config.GetString("multicam_calibration");
//    if (!pvt::utils::file::Exists(filename))
//    {
//      const std::string msg = "Extrinsic calibration file specified, but does not exist! Check location of '" + filename + "'";
//      if (force_loading_extrinsics)
//        throw std::runtime_error(msg);

//      PVT_LOG_FAILURE(msg);
//      return;
//    }

//    cv::FileStorage fs(filename, cv::FileStorage::READ);
//    const size_t num_sinks = NumStreams();
//    for (size_t i = 0; i < num_sinks; ++i)
//    {
//      // Check if the calibration matches the sink label
//      std::stringstream ss;
//      ss << "label" << (i+1);
//      std::string label;
//      fs[ss.str()] >> label;
//      const std::string expected_label = GetSinkLabel(i);
//      if (expected_label.compare(label) != 0)
//      {
//        ss.str("");
//        ss.clear();
//        ss << "Expected calibration of camera" << (i+1) << " to belong to '" << expected_label << "' but got '" << label << "'.";
//        if (force_loading_extrinsics)
//        {
//          throw std::runtime_error(ss.str());
//        }
//        else
//        {
//          ss << " Ignoring extrinsics for this stream.";
//          PVT_LOG_FAILURE_NOFILE(ss.str());
//          extrinsics_.push_back(std::pair<cv::Mat, cv::Mat>(cv::Mat(), cv::Mat()));
//          continue;
//        }
//      }

//      // Load rectifier (usually, if you replay a calibrated multicam capture, this should be all None (i.e. no distortion coefficients):
//      rectifier_.push_back(GetRectifier(fs, GetStreamType(i), (i+1)));

//      // Load extrinsics:
//      ss.str("");
//      ss.clear();
//      ss << "R" << (i+1);
//      cv::Mat R;
//      fs[ss.str()] >> R;

//      ss.str("");
//      ss.clear();
//      ss << "t" << (i+1);
//      cv::Mat t;
//      fs[ss.str()] >> t;

//      extrinsics_.push_back(std::pair<cv::Mat, cv::Mat>(R, t));
//    }
//  }

//  void LoadSeparateIntrinsics()
//  {
//    const size_t num_sinks = NumStreams();
//    for (size_t i = 0; i < num_sinks; ++i)
//    {
//      auto rectifier = GetRectifier(GetCalibrationFile(i), GetStreamType(i), GetSinkLabel(i));
//      rectifier_.push_back(std::move(rectifier));
//    }
//  }

//  void LoadSeparateExtrinsics(const pvt::config::ConfigParams &config, bool force_loading_extrinsics)
//  {
//    const bool has_extrinsics = config.SettingExists("extrinsic_calibration_file");
//    if (!has_extrinsics)
//    {
//      if (force_loading_extrinsics)
//        throw std::runtime_error("Configuration doesn't provide an extrinsic calibration file (loading extrinsics was forced)");
//      return;
//    }

//    const std::string filename = config.GetString("extrinsic_calibration_file");
//    if (!pvt::utils::file::Exists(filename))
//    {
//      const std::string msg = "Extrinsic calibration file specified, but does not exist! Check location of '" + filename + "'";
//      if (force_loading_extrinsics)
//        throw std::runtime_error(msg);

//      PVT_LOG_FAILURE(msg);
//      return;
//    }

//    cv::FileStorage fs(filename, cv::FileStorage::READ);
//    // Note there's an OpenCV (python wrapper) bug, we cannot write integers (only doubles): https://github.com/opencv/opencv/issues/10506
////    int num_cameras;
////    fs["num_cameras"] >> num_cameras;
//    const size_t num_cameras = plain_capture_->NumStreams();
//    for (size_t i = 0; i < num_cameras; ++i)
//    {
//      std::stringstream ss;
//      ss << "R" << (i+1);
//      cv::Mat R;
//      fs[ss.str()] >> R;

//      ss.str("");
//      ss.clear();
//      ss << "t" << (i+1);
//      cv::Mat t;
//      fs[ss.str()] >> t;

//      ss.str("");
//      ss.clear();
//      ss << "label" << (i+1);
//      std::string label;
//      fs[ss.str()] >> label;

//      const std::string expected_label = GetSinkLabel(i);
//      if (expected_label.compare(label) != 0)
//      {
//        ss.str("");
//        ss.clear();
//        ss << "Expected R" << (i+1) << ", t" << (i+1) << " to belong to '" << expected_label << "' but got '" << label << "'";
//        throw std::runtime_error(ss.str());
//      }
//      extrinsics_.push_back(std::pair<cv::Mat, cv::Mat>(R, t));
//    }
//    fs.release();
//  }
//};


//std::unique_ptr<RectifiedCapture> CreateRectifiedCapture(const pvt::config::ConfigParams &config, bool force_loading_extrinsics)
//{
//  return std::unique_ptr<RectifiedCaptureImpl>(new RectifiedCaptureImpl(config, force_loading_extrinsics));
//}

} // namespace best
} // namespace vcp
