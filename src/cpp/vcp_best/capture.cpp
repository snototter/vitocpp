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
#include <utility>

#include "file_sink.h"
#include "webcam_sink.h"

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
#ifdef VCP_BEST_WITH_ZED
  #include "zed_sink.h"
#endif
#include <chrono>

namespace vcp
{
namespace best
{

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best"


std::ostream& operator<<(std::ostream & os, const StreamStorageParams &ssp)
{
  switch (ssp.type)
  {
    case StreamStorageParams::Type::NONE:
      os << "Stream will not be saved";
      break;

    case StreamStorageParams::Type::IMAGE_DIR:
      os << "Image sequence [" << ssp.path << "]";
      break;

    case StreamStorageParams::Type::VIDEO:
      os << "Video [" << ssp.path << "]";
      break;

    default:
      VCP_ERROR("StreamStorageParams " << static_cast<short>(ssp.type) << " is not yet supported.");
  }
  return os;
}

class MultiDeviceCapture : public Capture
{
public:
  MultiDeviceCapture(const vcp::config::ConfigParams &config) : Capture()
  {
    VCP_LOG_DEBUG("MultiDeviceCapture::MultiDeviceCapture()");
    num_devices_ = 0;
    LoadSinkConfigs(config);
    LoadGlobalConfig(config);
    SanityCheck();
  }

  void LoadGlobalConfig(const vcp::config::ConfigParams &config)
  {
    if (config.SettingExists("verbose"))
    {
      const bool verbose = config.GetBoolean("verbose");
      VCP_LOG_INFO("Overriding sink verbosity (set to " << (verbose ? "true" : "false") << ").");
      for (auto &sink : sinks_)
        sink->SetVerbose(verbose);
    }

    const std::string kext = config.SettingExists("extrinsic_calibration")
        ? "extrinsic_calibration"
        : (config.SettingExists("extrinsic_calibration_file")
           ? "extrinsic_calibration_file"
           : "");
    if (!kext.empty())
    {
      const std::string ext_calib_file = config.GetString(kext);
      if (vcp::utils::file::Exists(ext_calib_file))
      {
        const auto extrinsics = calibration::LoadExtrinsicsFromFile(ext_calib_file);
        for (const auto &ext : extrinsics)
          SetExtrinsicsAt(ext.first, ext.second.R(), ext.second.t());
      }
      else
      {
        VCP_LOG_FAILURE("Parameter '" << kext << "' has been set, but file '"
                        << ext_calib_file << "' does not exist!");
      }
    }
  }

  void LoadSinkConfigs(const vcp::config::ConfigParams &config)
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
#ifdef VCP_BEST_WITH_ZED
    std::vector<zed::ZedSinkParams> zed_params;
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

#ifdef VCP_BEST_WITH_ZED
        case SinkType::ZED:
          zed_params.push_back(zed::ZedSinkParamsFromConfig(config, cam_config_name));
          break;
#endif // VCP_BEST_WITH_ZED

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

#ifdef VCP_BEST_WITH_K4A
    std::vector<k4a::K4ASinkParams> k4a_single;
    std::vector<std::vector<k4a::K4ASinkParams>> k4a_multi;
    k4a::GroupK4ASinkParams(k4a_params, k4a_single, k4a_multi);
    for (const auto &p : k4a_single)
      AddSink(k4a::CreateK4ASink<VCP_BEST_STREAM_BUFFER_CAPACITY>(p));
    for (const auto &ps : k4a_multi)
      AddSink(k4a::CreateK4ASyncedSink<VCP_BEST_STREAM_BUFFER_CAPACITY>(ps));
#endif // VCP_BEST_WITH_K4A

#ifdef WITH_MATRIXVISION
    if (!mvbluefox_params.empty())
    {
//      for (const auto &p : mvbluefox_params)
//      {
//        sinks_.push_back(pvt::icc::CreateSink(p));
//        AddLabel(p.label);
//        AddCalibrationFile(p.calibration_file);
//        AddStreamType(p.stream_type);
//        sink_types_.push_back(SinkType::MVBLUEFOX3);
//      }
//      AddCorrespondingConfigKeys(cck_mvbluefox);
    }
#endif // WITH_MATRIXVISION

#ifdef VCP_BEST_WITH_REALSENSE2
    for (const auto &p : realsense2_params)
      AddSink(realsense2::CreateRealSense2Sink<VCP_BEST_STREAM_BUFFER_CAPACITY>(p));
    multiple_realsenses_ = realsense2_params.size() > 1;
#endif

#ifdef VCP_BEST_WITH_ZED
    for (const auto &p : zed_params)
      AddSink(zed::CreateZedSink<VCP_BEST_STREAM_BUFFER_CAPACITY>(p));
#endif

    num_devices_ = 0;
    for (const auto &s : sinks_)
      num_devices_ += s->NumDevices();
  }

  void AddSink(std::unique_ptr<StreamSink> sink)
  {
    if (sink == nullptr)
      return;

    const size_t sink_idx = sinks_.size();
    for (size_t i = 0; i < sink->NumStreams(); ++i)
    {
      //sink_params_.push_back(sink->SinkParamsAt(i));
      frame_types_.push_back(sink->FrameTypeAt(i));
      frame_labels_.push_back(sink->StreamLabel(i));
      frame2sink_.push_back(std::make_pair(sink_idx, i));
    }
    sinks_.push_back(std::move(sink));
  }

  virtual ~MultiDeviceCapture()
  {
    VCP_LOG_DEBUG("MultiDeviceCapture::~MultiDeviceCapture()");
    CloseDevices();
    for (size_t i = 0; i < sinks_.size(); ++i)
      sinks_[i].reset();
  }

  size_t NumStreams() const override
  {
    return frame_types_.size();
  }

  size_t NumDevices() const override
  {
    return num_devices_;
  }

  std::vector<std::string> ConfigurationKeys() const override
  {
    std::vector<std::string> keys;
    for (size_t i = 0; i < NumStreams(); ++i)
      keys.push_back(ConfigurationKeyAt(i));
//    for (const auto &p : sink_params_)
//      keys.push_back(p.configuration_key);
    return keys;
  }

  std::string ConfigurationKeyAt(size_t stream_index) const override
  {
    return SinkParamsAt(stream_index).configuration_key;
    //return sink_params_[stream_index].configuration_key;
  }

  std::vector<std::string> FrameLabels() const override
  {
    return frame_labels_;
  }

  std::string FrameLabelAt(size_t stream_index) const override
  {
    return frame_labels_[stream_index];
  }

  std::string CanonicFrameLabelAt(size_t stream_index) const override
  {
    return vcp::utils::string::Canonic(frame_labels_[stream_index]);
  }

  std::vector<FrameType> FrameTypes() const override
  {
    return frame_types_;
  }

  FrameType FrameTypeAt(size_t stream_index) const override
  {
    return frame_types_[stream_index];
  }

  bool IsStreamRectified(size_t stream_index) const override
  {
    return SinkParamsAt(stream_index).rectify;
  }

  bool IsStepAbleImageDirectory(size_t stream_index) const override
  {
    const auto &lookup = frame2sink_[stream_index];
    return file::IsImageDirectorySink(sinks_[lookup.first]);
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
          && sinks_[i]->GetSinkType() == sinks_[i+1]->GetSinkType()
          && sinks_[i]->GetSinkType() == SinkType::REALSENSE)
      {
        VCP_LOG_INFO("Pausing capture initialization temporarily before starting the next RealSense stream!");
        std::this_thread::sleep_for(std::chrono::milliseconds(1500)); // 1.5 sec worked okay-ish so far...
      }
#endif // VCP_WITH_REALSENSE2
#ifdef VCP_BEST_WITH_K4A
      // Similar to RealSense, it is safer to wait between starting multiple Kinects.
      // Otherwise, we experienced problems within the k4a_open_device calls, where one sensor
      // wasn't properly initialized.
      if ((i+1) < sinks_.size()
          && sinks_[i]->GetSinkType() == sinks_[i+1]->GetSinkType()
          && sinks_[i]->GetSinkType() == SinkType::K4A)
      {
        VCP_LOG_INFO("Pausing capture initialization temporarily before starting the next K4A stream!");
        std::this_thread::sleep_for(std::chrono::milliseconds(1500)); // 1.5 sec worked okay-ish so far...
      }
#endif // VCP_BEST_WITH_K4A
    }
#ifdef VCP_BEST_DEBUG_FRAMERATE
  prev_frame_timestamp_ = std::chrono::high_resolution_clock::now();
  ms_between_frames_ = -1.0;
#endif // VCP_BEST_DEBUG_FRAMERATE
    return success;
  }

  bool WaitForFrames(double timeout_ms, bool verbose) const override
  {
    VCP_LOG_DEBUG("MultiDeviceCapture::WaitForFrames()");

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    double elapsed_ms = 0;
    while (elapsed_ms < timeout_ms)
    {
      if (AreAllFramesAvailable())
        return true;

      if (verbose)
      {
        VCP_LOG_WARNING_NSEC("Not all sinks are ready. Continue waiting for "
                        << std::fixed << std::setprecision(2) << (std::max(timeout_ms - elapsed_ms, 0.0) / 1000) << " sec.", 0.5);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                std::chrono::high_resolution_clock::now() - start);
      elapsed_ms = duration.count();
    }
    if (verbose)
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


  bool SaveReplayConfiguration(const std::string &folder, const std::map<std::string, StreamStorageParams> &storage_params, bool save_extrinsics) const override
  {
    // Sanity checks.
    if (storage_params.size() != frame_labels_.size())
    {
      VCP_LOG_FAILURE("Number of storage_params (" << storage_params.size() << ") and configured streams (" << frame_labels_.size() << ") differs.");
      return false;
    }
    // Ensure that the output folder exists.
    if (!vcp::utils::file::Exists(folder))
    {
      if (!vcp::utils::file::CreatePath(folder))
      {
        VCP_LOG_FAILURE("Cannot create path '" << folder << "' to store the 'replay' configuration.");
        return false;
      }
    }
    if (!vcp::utils::file::IsDir(folder))
    {
      VCP_LOG_FAILURE("Path '" << folder << "' is not a directory.");
      return false;
    }

    std::unique_ptr<vcp::config::ConfigParams> config = vcp::config::CreateEmptyConfigParamsCpp();
    for (auto it = storage_params.begin(); it != storage_params.end(); ++it)
    {
      size_t fidx;
      if (!FrameIndexByLabel(it->first, fidx))
      {
        VCP_LOG_FAILURE("Invalid frame label [" << it->first << "].");
        return false;
      }

      const std::string cam_group = "camera-" + vcp::utils::string::ToStr(fidx);

      switch(it->second.type)
      {
        case StreamStorageParams::Type::NONE:
          VCP_LOG_INFO("Skipping configuration of stream '" << it->first << "' as it won't be saved.");
          continue;

        case StreamStorageParams::Type::IMAGE_DIR:
          config->SetString(cam_group + ".sink_type", "imagedir");
          config->SetString(cam_group + ".directory", it->second.path);
          break;

        case StreamStorageParams::Type::VIDEO:
          config->SetString(cam_group + ".sink_type", "video");
          config->SetString(cam_group + ".video", it->second.path);
          break;

        default:
          VCP_ERROR("StreamStorageParams::Type " << static_cast<uchar>(it->second.type) << " is not yet supported.");
      }

      config->SetString(cam_group + ".label", it->first);
      config->SetString(cam_group + ".label_canonic", vcp::utils::string::Canonic(it->first));
      config->SetString(cam_group + ".original_config_key", ConfigurationKeyAt(fidx));
      config->SetString(cam_group + ".frame_type", FrameTypeToString(frame_types_[fidx]));
      config->SetBoolean(cam_group + ".color_as_bgr", false);

      const auto &intrinsics = sinks_[frame2sink_[fidx].first]->IntrinsicsAt(frame2sink_[fidx].second);
      if (!intrinsics.Empty())
      {
        // Calibration file will be stored relative to the configuration file.
        // Thus, ensure that the "calibration" subfolder exists.
        const std::string csf = vcp::utils::file::FullFile(folder, "calibration");
        if (!vcp::utils::file::Exists(csf))
        {
          if (!vcp::utils::file::CreatePath(csf))
          {
            VCP_LOG_FAILURE("Cannot create calibration output folder '" << csf << "' for the intrinsic calibration files.");
            return false;
          }
        }
        const std::string rel_calib_file = vcp::utils::file::FullFile("calibration",
                                          std::string("calib-") + vcp::utils::string::Canonic(it->first) + ".xml");
        config->SetString(cam_group + ".calibration_file", rel_calib_file);
        const std::string calib_file = vcp::utils::file::FullFile(folder, rel_calib_file);
        if (!intrinsics.Save(calib_file, SinkParamsAt(fidx).rectify, frame_types_[fidx]))
        {
          VCP_LOG_FAILURE("Couldn't save calibration file '" << calib_file << "' for stream '" << frame_labels_[fidx] << "'.");
          return false;
        }

        config->SetBoolean(cam_group + ".rectify", SinkParamsAt(fidx).rectify);
      }
    }

    if (save_extrinsics)
    {
      const std::string rel_ext_file = vcp::utils::file::FullFile("calibration", "extrinsics.xml");
      const std::string ext_file = vcp::utils::file::FullFile(folder, rel_ext_file);
      if (SaveExtrinsicCalibration(ext_file))
        config->SetString("extrinsic_calibration_file", rel_ext_file);
      else
        VCP_LOG_FAILURE("Couldn't save extrinsics to '" << ext_file << "'.");
    }

    const std::string cfg_file = vcp::utils::file::FullFile(folder, "replay.cfg");
    const bool success = config->SaveConfiguration(cfg_file);
    if (success)
      VCP_LOG_INFO("Replay-able configuration has been saved to '" << cfg_file << "'");
    // Error message will be displayed within config->SaveConfiguration()

    return success;
  }

  cv::Mat CameraMatrixAt(size_t stream_index) const override
  {
    const auto &lookup = frame2sink_[stream_index];
    const calibration::StreamIntrinsics intrinsics = sinks_[lookup.first]->IntrinsicsAt(lookup.second);
    if (intrinsics.Empty())
      return cv::Mat();
    if (SinkParamsAt(stream_index).rectify)
      return intrinsics.IntrinsicsRectified();
    return intrinsics.IntrinsicsOriginal();
  }

  void StereoTransformation(size_t stream_index, cv::Mat &R, cv::Mat &t) const  override
  {
    const auto &lookup = frame2sink_[stream_index];
    const calibration::StreamIntrinsics intrinsics = sinks_[lookup.first]->IntrinsicsAt(lookup.second);
    if (intrinsics.Empty() || !intrinsics.HasTransformationToReference())
    {
      R = cv::Mat();
      t = cv::Mat();
    }
    else
    {
      intrinsics.TransformationToReference(R, t);
    }
  }

  void ExtrinsicsAt(size_t stream_index, cv::Mat &R, cv::Mat &t) const override
  {
    const auto &lookup = frame2sink_[stream_index];
    sinks_[lookup.first]->ExtrinsicsAt(lookup.second, R, t);
  }

  std::vector<size_t> StreamsFromSameSink(size_t stream_index, bool include_self) const override
  {
    const auto &sink_params = SinkParamsAt(stream_index);
    const bool use_current_config_key = sink_params.original_configuration_key.empty();
    const std::string needle = use_current_config_key
        ? sink_params.configuration_key
        : sink_params.original_configuration_key;

    std::vector<size_t> indices;
    for (size_t i = 0; i < NumStreams(); ++i)
    {
      if (!include_self && i == stream_index)
        continue;
      if ((use_current_config_key && needle.compare(SinkParamsAt(i).configuration_key) == 0)
          || (!use_current_config_key && needle.compare(SinkParamsAt(i).original_configuration_key) == 0))
          indices.push_back(i);
    }
    return indices;
  }

  SinkParams SinkParamsAt(size_t stream_index) const override
  {
    const auto &lookup = frame2sink_[stream_index];
    return sinks_[lookup.first]->SinkParamsAt(lookup.second);
  }

  bool SetExtrinsicsAt(size_t stream_index, const cv::Mat &R, const cv::Mat &t) override
  {
    const auto &lookup = frame2sink_[stream_index];
    return sinks_[lookup.first]->SetExtrinsicsAt(lookup.second, R, t);
  }

  bool SetExtrinsicsAt(const std::string &stream_label, const cv::Mat &R, const cv::Mat &t) override
  {
    for (size_t i = 0; i < frame_labels_.size(); ++i)
    {
      if (stream_label.compare(frame_labels_[i]) == 0)
        return SetExtrinsicsAt(i, R, t);
    }
    VCP_LOG_FAILURE("Cannot set extrinsics because stream '" << stream_label << "' is unknown."
                    << "Known labels: " << frame_labels_);
    return false;
  }

  bool SaveExtrinsicCalibration(const std::string &filename) const override
  {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
      VCP_LOG_FAILURE("Cannot open FileStorage to store extrinsics at '" << filename << "'.");
      return false;
    }

    fs << "num_cameras" << static_cast<int>(frame_labels_.size());
    for (size_t i = 0; i < frame_labels_.size(); ++i)
    {
      std::stringstream ss;
      ss << "label" << i;
      fs << ss.str() << frame_labels_[i];

      cv::Mat R, t;
      ExtrinsicsAt(i, R, t);

      if (!R.empty() && !t.empty())
      {
        ss.str("");
        ss.clear();
        ss << "R" << i;
        fs << ss.str() << R;

        ss.str("");
        ss.clear();
        ss << "t" << i;
        fs << ss.str() << t;
      }
    }
    fs.release();
    return true;
  }

private:
  std::vector<std::unique_ptr<StreamSink>> sinks_; // Potentially less than streams/frames
  std::vector<std::pair<size_t, size_t>> frame2sink_; /**< Stores the index into sinks_ (along with the corresponding stream index) for each frame, e.g. if we iterate frame_types_, we can easily lookup the corresponding sink. */
  std::vector<FrameType> frame_types_;  // Note: they will be per stream/frame (i.e. possibly duplicated), not per device
  std::vector<std::string> frame_labels_; /**< Unique label per stream (the user can provide per-sink labels, which will be post-fixed by the corresponding sensor sink). */
  size_t num_devices_;

#ifdef VCP_BEST_DEBUG_FRAMERATE
  std::chrono::high_resolution_clock::time_point prev_frame_timestamp_;
  double ms_between_frames_;
#endif // VCP_BEST_DEBUG_FRAMERATE

#ifdef VCP_BEST_WITH_REALSENSE2
  bool multiple_realsenses_;
#endif // VCP_BEST_WITH_REALSENSE2

  bool FrameIndexByLabel(const std::string &frame_label, size_t &idx) const
  {
    for (size_t i = 0; i < frame_labels_.size(); ++i)
    {
      if (frame_labels_[i].compare(frame_label) == 0)
      {
        idx = i;
        return true;
      }
    }
    return false;
  }

  void SanityCheck()
  {
    if (!vcp::utils::HasUniqueItems(frame_labels_))
      VCP_ERROR("Frame labels aren't unique - adjust the configuration!");

    for (size_t i = 0; i < frame_types_.size(); ++i)
    {
      if (frame_types_[i] == FrameType::UNKNOWN)
      {
        VCP_LOG_WARNING("'Unknown' frame type for stream '"
                        << frame_labels_[i] << "', configured via parameter '"
                        << ConfigurationKeyAt(i) << "'");
      }
    }
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


} // namespace best
} // namespace vcp
