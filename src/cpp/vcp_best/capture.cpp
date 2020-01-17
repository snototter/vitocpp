#include "capture.h"
#include "sink.h"

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/string_utils.h>
#include <vcp_math/geometry3d.h>
#include <vcp_utils/file_utils.h>


#include <sstream>
#include <exception>
#include <thread>

#include "file_sink.h"
#include "webcam_sink.h"

//#include "rectifier.h"
//#include "capture_file.h"
//#include "capture_webcam.h"
//#ifdef WITH_IPCAMERA
//  #include "capture_ipcam.h"
//#endif
//#ifdef WITH_K4A
//  #include "capture_k4a.h"
//#endif
//#ifdef WITH_MATRIXVISION
//  #include "capture_matrixvision.h"
//#endif
//#ifdef WITH_REALSENSE2
//  #include "capture_realsense2.h"
//  #include <chrono>
//#endif

namespace vcp
{
namespace best
{

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::capture"
class MultiDeviceCapture : public Capture
{
public:
  MultiDeviceCapture(const vcp::config::ConfigParams &config) : Capture()
  {
    VCP_LOG_DEBUG("MultiDeviceCapture()");
    LoadConfig(config);
  }

  void LoadConfig(const vcp::config::ConfigParams &config)
  {
    VCP_LOG_DEBUG("LoadConfig()");
    //size_t num_cameras = GetNumCamerasFromConfig(config);

//#ifdef WITH_IPCAMERA
//    std::vector<MonoIpCameraSinkParams> ip_mono_params;
//    std::vector<StereoIpCameraSinkParams> ip_stereo_params;
//    std::vector<std::string> cck_ip_mono; // Corresponding configuration file keys (camera1, etc)
//    std::vector<std::string> cck_ip_stereo; // Corresponding configuration file keys (camera1, etc)
//#endif
    std::vector<ImageDirectorySinkParams> imgdir_params;
//#ifdef WITH_MATRIXVISION
//    std::vector<MvBlueFox3SinkParams> mvbluefox_params;
//    std::vector<std::string> cck_mvbluefox; // Corresponding configuration file keys (camera1, etc)
//#endif
//#ifdef WITH_K4A
//    std::vector<K4ASinkParams> k4a_params;
//    std::vector<std::string> cck_k4a; // Corresponding configuration file keys (camera1, etc)
//#endif
//#ifdef WITH_REALSENSE2
//    std::vector<RealSense2SinkParams> realsense_params;
//    std::vector<std::string> cck_realsense; // Corresponding configuration file keys (camera1, etc)
//#endif
    std::vector<VideoFileSinkParams> video_params;
    std::vector<WebcamSinkParams> webcam_params;

    // Filter all "camera[0-9,a-z]*" parameters out of the configuration:
    const std::vector<std::string> cam_config_names = GetCameraConfigParameterNames(config);

    for (const auto &cam_config_name : cam_config_names)
    {
      const SinkType sink_type = GetSinkTypeFromConfig(config, cam_config_name);

      switch(sink_type)
      {
        case SinkType::IMAGE_DIR:
          imgdir_params.push_back(ImageDirectorySinkParamsFromConfig(config, cam_config_name));
          break;

        case SinkType::VIDEO_FILE:
          video_params.push_back(VideoFileSinkParamsFromConfig(config, cam_config_name));
          break;

        case SinkType::WEBCAM:
          webcam_params.push_back(WebcamSinkParamsFromConfig(config, cam_config_name));
          break;

        default:
          VCP_LOG_FAILURE("Sink type '" << sink_type << "' is not yet supported!");
          break;
      }

//#ifdef WITH_IPCAMERA
//      else if (IsMonocularIpCamera(cam_type))
//      {
//        ip_mono_params.push_back(MonoIpCameraSinkParamsFromConfig(config, id.str()));
//        cck_ip_mono.push_back(id.str());
//      }
//      else if (IsStereoIpCamera(cam_type))
//      {
//        ip_stereo_params.push_back(StereoIpCameraSinkParamsFromConfig(config, id.str()));
//        cck_ip_stereo.push_back(id.str());
//      }
//#endif // WITH_IPCAMERA
//#ifdef WITH_K4A
//      else if (IsK4A(cam_type))
//      {
//        k4a_params.push_back(K4ASinkParamsFromConfig(config, id.str()));
//        cck_k4a.push_back(id.str());
//      }
//#endif // WITH_K4A
//#ifdef WITH_MATRIXVISION
//      else if (IsMatrixVision(cam_type))
//      {
//        mvbluefox_params.push_back(MvBlueFox3SinkParamsFromConfig(config, id.str()));
//        cck_mvbluefox.push_back(id.str());
//      }
//#endif // WITH_MATRIXVISION
//#ifdef WITH_REALSENSE2
//      else if (IsRealSense2(cam_type))
//      {
//        realsense_params.push_back(RealSense2SinkParamsFromConfig(config, id.str()));
//        cck_realsense.push_back(id.str());
//      }
//#endif // WITH_REALSENSE2
    }

    // TODO if you extend the sinks, be sure to set label, calibration file, StreamType(s) and corresponding config key,
    // since 'camera1' may be loaded after 'camera7' if you mix types, due to the following ordering!
    // Additionally, be sure to add the correct SinkType!

    // Initialize the individual captures
    for (const auto &p : imgdir_params)
      AddSink(CreateImageDirectorySink(p), p);

    for (const auto &p : video_params)
      AddSink(CreateVideoFileSink(p), p);

    for (const auto &p : webcam_params)
      AddSink(CreateWebcamSink<VCP_BEST_STREAM_BUFFER_CAPACITY>(p), p);

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

#ifdef WITH_K4A
    if (!k4a_params.empty())
    {
      bool k4a_requires_sync = false;
      if (k4a_params.size() > 1)
      {
        for (const auto &p : k4a_params)
        {
          if (p.device_params.wired_sync_mode != K4A_WIRED_SYNC_MODE_STANDALONE)
            k4a_requires_sync = true;
        }
      }

      if (k4a_requires_sync)
      {
        //TODO: we need a separate multi-k4a-capture class :-/
        // See best practices in green screen demo:
        // https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/green_screen/MultiDeviceCapturer.h
        PVT_ABORT("Synchronization of multiple K4As is not yet supported!");
      }
      else
      {
        // Note: labels and calibration files will be modified when creating the k4a sinks (as one sink adds 2 frames)!
        std::vector<std::string> labels;
        std::vector<std::string> calibs;
        std::vector<StreamType> types;
        for (const auto &p : k4a_params)
        {
          sinks_.push_back(pvt::icc::CreateSink(p, labels, calibs, types));
          sink_types_.push_back(SinkType::K4A);
        }

        AddLabels(labels);
        AddCalibrationFiles(calibs);
        AddStreamTypes(types);
        // We need to add them twice (!) because a realsense has two streams, thus two consecutive sink indices
        AddCorrespondingConfigKeys(cck_k4a);
        AddCorrespondingConfigKeys(cck_k4a);
      }
    }
    // TODO check if k4a initialization requires a wait/sleep (similar to realsense devices, see use of multiple_realsenses_)
#endif // WITH_K4A

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

#ifdef WITH_REALSENSE2
    if (!realsense_params.empty())
    {
      // Note: labels and calibration files will be modified when creating the realsense sinks (as one sink adds 2 frames)!
      std::vector<std::string> labels;
      std::vector<std::string> calibs;
      std::vector<StreamType> types;
      for (const auto &p : realsense_params)
      {
        sinks_.push_back(pvt::icc::CreateSink(p, labels, calibs, types));
        sink_types_.push_back(SinkType::REALSENSE);
      }

      AddLabels(labels);
      AddCalibrationFiles(calibs);
      AddStreamTypes(types);
      // We need to add them twice (!) because a realsense has two streams, thus two consecutive sink indices
      AddCorrespondingConfigKeys(cck_realsense);
      AddCorrespondingConfigKeys(cck_realsense);
    }
    multiple_realsenses_ = realsense_params.size() > 1;
#endif // WITH_REALSENSE2
  }

  void AddSink(std::unique_ptr<StreamSink> sink, const SinkParams params)
  {
    for (size_t i = 0; i < sink->NumStreams(); ++i)
    {
      //sink_types_.push_back(sink->Type(i));
      frame_types_.push_back(sink->FrameTypeAt(i));
      sink_params_.push_back(params);
      frame_labels_.push_back(sink->StreamLabel(i));
    }
    sinks_.push_back(std::move(sink));
  }

  virtual ~MultiDeviceCapture()
  {
    VCP_LOG_DEBUG("~MultiDeviceCapture()");
    StopStreams();
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
    return sinks_.size();
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

  bool AreDevicesAvailable() const override
  {
    VCP_LOG_DEBUG("AreDevicesAvailable()");
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      if (!sinks_[i]->IsDeviceAvailable())
        return false;
    }
    return true;
  }


  bool AreFramesAvailable() const override
  {
    VCP_LOG_DEBUG("AreFramesAvailable()");
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      if (!sinks_[i]->IsFrameAvailable())
        return false;
    }
    return true;
  }


  bool OpenDevices() override
  {
    VCP_LOG_DEBUG("OpenDevices()");
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success = success && sinks_[i]->OpenDevice();
    return success;
  }


  bool CloseDevices() override
  {
    VCP_LOG_DEBUG("CloseDevices()");
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success = success && sinks_[i]->CloseDevice();
    return success;
  }


  bool StartStreams() override
  {
    VCP_LOG_DEBUG("StartStreams()");
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      success = success && sinks_[i]->StartStreaming();
#ifdef VCP_WITH_REALSENSE2
      // According to the RealSense white paper on multiple sensors, they
      // "found it useful to start the cameras sequentially in time with some delay".
      if (multiple_realsenses_ && (i+1) < sinks_.size() &&
          sink_types_[i] == sink_types_[i+1] && sink_types_[i] == SinkType::REALSENSE)
      {
        PVT_LOG_INFO("Pausing thread temporarily before starting the next RealSense stream!");
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      }
#endif // VCP_WITH_REALSENSE2
    }
    return success;
  }

  bool WaitForInitialFrames(double timeout_ms) const override
  {
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    double elapsed_ms = 0;
    while (elapsed_ms < timeout_ms)
    {
      bool available = true;
      for (size_t i = 0; i < sinks_.size(); ++i)
        available = available && sinks_[i]->IsFrameAvailable();
      if (available)
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                std::chrono::high_resolution_clock::now() - start);
      elapsed_ms = duration.count();
    }
    VCP_LOG_FAILURE("Not all sinks are ready. Waited for " << elapsed_ms/1000 << " sec.");
    return false;
  }


  bool StopStreams() override
  {
    VCP_LOG_DEBUG("StopStreams()");
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success = success && sinks_[i]->StopStreaming();
    return success;
  }


  std::vector<cv::Mat> Next() override
  {
    VCP_LOG_DEBUG("Next()");
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

#ifdef WITH_REALSENSE2
  bool multiple_realsenses_;
#endif // WITH_REALSENSE2
};


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

} // namespace icc
} // namespace pvt
