//-----------------------------------------------------------------------------
// PVT capture wrapper code

#include "conversion/np_cv_conversion.h"
#include "conversion/cv_core_conversion.h"

#include <exception>
#include <string>
#include <sstream>
#include <vector>
#include <memory>
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/imgproc.hpp>
#endif
#include <vcp_config/config_params.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/vcp_error.h>

#include "config.h"

#include <vcp_utils/timing_utils.h>
#include <vcp_best/capture.h>
#include <vcp_best/liveview.h>
#include <vcp_best/rgbd.h>
#include <vcp_imutils/opencv_compatibility.h>

#ifdef VCP_BEST_WITH_IPCAM
  #include <vcp_best/ipcam_sink.h>
#endif
#ifdef VCP_BEST_WITH_K4A
  #include <vcp_best/k4a_sink.h>
#endif
//#ifdef VCP_BEST_WITH_MATRIXVISION
//  #include <vcp_best/capture_matrixvision.h>
//#endif
#ifdef VCP_BEST_WITH_REALSENSE2
  #include <vcp_best/realsense2_sink.h>
#endif
#ifdef VCP_BEST_WITH_ZED
  #include <vcp_best/zed_sink.h>
#endif
#include <vcp_best/webcam_sink.h>

namespace vcp
{
namespace python
{
namespace best
{
class CaptureWrapper
{
public:
  CaptureWrapper() : capture_(nullptr) {}

  // Disable copying
  CaptureWrapper(const CaptureWrapper&) = delete;
  CaptureWrapper &operator=(const CaptureWrapper&) = delete;

  virtual ~CaptureWrapper()
  {
    if (capture_)
      capture_.reset();
  }

  void LoadConfigCpp(const std::string &config_file, const std::string &rel_path_base_dir)
  {
    auto config = vcp::config::LoadConfigParamsCpp(config_file);
    AdjustFilePaths(*config, rel_path_base_dir);
    capture_ = vcp::best::CreateCapture(*config);
  }

  void LoadConfigJsonString(const std::string &json_str, const std::string &rel_path_base_dir)
  {
    LoadConfigJson(json_str, true, rel_path_base_dir);
  }

  void LoadConfigJsonFile(const std::string &json_file, const std::string &rel_path_base_dir)
  {
    LoadConfigJson(json_file, false, rel_path_base_dir);
  }

  void LoadConfigParams(const vcp::python::config::ConfigWrapper &config)
  {
    const vcp::config::ConfigParams &params = config.AsConfigParams();
    capture_ = vcp::best::CreateCapture(params);
  }

  /** @brief Returns true if the capturing device/s is/are available. */
  bool AreAllDevicesAvailable() const
  {
    return capture_->AreAllDevicesAvailable();
  }

  /** @brief Returns true if frames (from all devices) can be retrieved (i.e. have been enqueued). */
  bool AreAllFramesAvailable() const
  {
    return capture_->AreAllFramesAvailable();
  }

  /** @brief Returns the number of available frames.
   *
   * Might be useful if one of your sinks stops working/streaming and you still want to continue
   * processing, etc. In such a case, @see AreAllFramesAvailable() would return false, whereas here
   * you get the actual number of frames and can decide yourself.
   */
  size_t NumAvailableFrames() const
  {
    return capture_->NumAvailableFrames();
  }

  /** @brief Initialize devices. */
  bool OpenDevices()
  {
    return capture_->OpenDevices();
  }

  /** @brief Close devices/tear down connections. */
  bool CloseDevices()
  {
    return capture_->CloseDevices();
  }

  /** @brief Starts the image streams. */
  bool StartStreams()
  {
    return capture_->StartStreams();
  }

  /** @brief Use this blocking call to wait for the next set of images to become available.
   * Specify a timeout in milliseconds, after that the result will indicate whether
   * frames from all devices/sinks are available or not.
   */
  bool WaitForFrames(double timeout_ms, bool verbose)
  {
    return capture_->WaitForFrames(timeout_ms, verbose);
  }

  /** @brief Stops the image streams. */
  bool StopStreams()
  {
    return capture_->StopStreams();
  }



  /** @brief Returns the currently available frames.
   *
   * If no data is available for a specific sink, it
   * will yield an empty cv::Mat instead.
   *
   * As the sinks push images into the frame buffers
   * which are protected with a mutex each, busy
   * polling Next() can lead to unwanted side effects.
   * For example, you may observe "stuttering" streams - which
   * are almost ever caused by the frame queue (since your
   * main polling thread constantly locks the queue's mutex).
   * So make sure, you do some processing, or let your main thread
   * sleep; as a rule of thumb: query roughly at the expected frame rate.
   */
  py::list NextFrame(bool flip_channels)
  {
    const std::vector<cv::Mat> frames = capture_->Next();
    return FramesToList(frames, flip_channels);
  }

  /** @brief Some sinks support seeking backwards (i.e. image directory and video sinks). Others will throw a std::runtime_error. */
  py::list PreviousFrame(bool flip_channels)
  {
    const std::vector<cv::Mat> frames = capture_->Previous();
    return FramesToList(frames, flip_channels);
  }

  /** @brief Some sinks (video and imagedir) support fast forwarding (skipping frames). Others will throw a std::runtime_error. */
  py::list FastForward(size_t num_frames, bool flip_channels)
  {
    const std::vector<cv::Mat> frames = capture_->FastForward(num_frames);
    return FramesToList(frames, flip_channels);
  }

  /** @brief Returns the current frame number. */
  size_t FrameNumber() const
  {
    return capture_->CurrentFrameNumber();
  }


  /** @brief returns the number of configured streams (NOT the number of sinks). */
  size_t NumStreams() const
  {
    return capture_->NumStreams();
  }

  /** @brief returns the number of devices/sinks (NOT the number of streams/frames).
   *
   * While this number corresponds to the number of "software sinks" used in VCP,
   * it may not be exactly the number of physical devices: For example, you could
   * configure 2 IP streams from the same device, one configured by its IP and the
   * other by its hostname. Handling such cases would be too complex.
   */
  size_t NumDevices() const
  {
    return capture_->NumDevices();
  }


  /** @brief Return the (user-defined) frame labels.
   * Note that:
   * - The order may differ from the one you used in the configuration file.
   *   This is because some devices must be handled specifically (e.g. multiple
   *   simultaneous RTSP streams, or synchronized/triggered RGBD sensors).
   * - The labels may also differ because you specify a per-device label. Sinks
   *   with multiple streams (e.g. or RGBD) will prefix the label accordingly.
   */
  std::vector<std::string> FrameLabels() const
  {
    return capture_->FrameLabels();
  }

  std::string FrameLabel(size_t stream_index) const
  {
    return capture_->FrameLabelAt(stream_index);
  }

  std::string CanonicFrameLabel(size_t stream_index) const
  {
    return capture_->CanonicFrameLabelAt(stream_index);
  }


  /** @brief Returns the configuration parameter name, i.e. "cameraX", for
   * each stream/frame.
   * See comments on @see FrameLabels() why this might be of interest
   * to you. However, it's usually better to query the @see FrameLabels()
   * and/or @see FrameTypes().
   */
  std::vector<std::string> ConfigurationKeys() const
  {
    return capture_->ConfigurationKeys();
  }

  std::string ConfigurationKey(size_t stream_index) const
  {
    return capture_->ConfigurationKeyAt(stream_index);
  }


  /** @brief Returns the FrameType for each stream/frame.
   * See comments on @see FrameLabels() why you cannot assume
   * that the frames will be in the same order you specified
   * in your configuration file.
   * Thus, use @see ConfigurationKeys() in combination with
   * these FrameType information to access a specific stream.
   *
   * This has only be taken into account if you mix the data
   * sources (e.g. stream webcams + RGBD + RTSP).
   */
  std::vector<std::string> FrameTypes() const
  {
    const auto ft = capture_->FrameTypes();
    std::vector<std::string> fts;
    for (const auto &t : ft)
      fts.push_back(vcp::best::FrameTypeToString(t));
    return fts;
  }

  std::string FrameTypeAt(size_t stream_index) const
  {
    return vcp::best::FrameTypeToString(capture_->FrameTypeAt(stream_index));
  }

  std::string SinkTypeAt(size_t stream_index) const
  {
    return vcp::best::SinkTypeToString(capture_->SinkParamsAt(stream_index).sink_type);
  }

  bool IsFrameTypeUnknown(size_t stream_index) const
  {
    return capture_->FrameTypeAt(stream_index) == vcp::best::FrameType::UNKNOWN;
  }

  bool IsFrameMonocular(size_t stream_index) const
  {
    return capture_->FrameTypeAt(stream_index) == vcp::best::FrameType::MONOCULAR;
  }

  bool IsFrameStereo(size_t stream_index) const
  {
    return capture_->FrameTypeAt(stream_index) == vcp::best::FrameType::STEREO;
  }

  bool IsFrameImage(size_t stream_index) const
  {
    return IsFrameMonocular(stream_index) || IsFrameStereo(stream_index);
  }

  bool IsFrameDepth(size_t stream_index) const
  {
    return capture_->FrameTypeAt(stream_index) == vcp::best::FrameType::DEPTH;
  }

  bool IsFrameInfrared(size_t stream_index) const
  {
    return capture_->FrameTypeAt(stream_index) == vcp::best::FrameType::INFRARED;
  }

  bool IsFrameRectified(size_t stream_index) const
  {
    return capture_->IsStreamRectified(stream_index);
  }

  bool IsFrameRGB(size_t stream_index) const
  {
    return IsFrameImage(stream_index) && capture_->SinkParamsAt(stream_index).color_as_bgr;
  }

  bool IsStepAbleImageSequence(size_t stream_index) const
  {
    return capture_->IsStepAbleImageDirectory(stream_index);
  }

  std::vector<size_t> StreamsFromSameSink(size_t stream_index, bool include_self) const
  {
    return capture_->StreamsFromSameSink(stream_index, include_self);
  }

  py::object CameraMatrix(size_t stream_index) const
  {
    const cv::Mat M = capture_->CameraMatrixAt(stream_index);
    if (M.empty())
      return py::none();
    return vcp::python::conversion::MatToNDArray(M);
  }

  py::object Distortion(size_t stream_index) const
  {
    const cv::Mat D = capture_->DistortionAt(stream_index);
    if (D.empty())
      return py::none();
    return vcp::python::conversion::MatToNDArray(D);
  }

  py::tuple StereoTransformation(size_t stream_index) const
  {
    cv::Mat R, t;
    capture_->StereoTransformation(stream_index, R, t);
    return py::make_tuple(vcp::python::conversion::MatToNDArray(R),
                          vcp::python::conversion::MatToNDArray(t));
  }

  py::tuple Extrinsics(size_t stream_index) const
  {
    cv::Mat R, t;
    capture_->ExtrinsicsAt(stream_index, R, t);
    return py::make_tuple(vcp::python::conversion::MatToNDArray(R),
                          vcp::python::conversion::MatToNDArray(t));
  }

  void SetExtrinsics(size_t stream_index, const cv::Mat &R, const cv::Mat &t)
  {
    capture_->SetExtrinsicsAt(stream_index, R, t);
  }

  void SetExtrinsics(const std::string &stream_label, const cv::Mat &R, const cv::Mat &t)
  {
    capture_->SetExtrinsicsAt(stream_label, R, t);
  }

  bool SaveReplayConfig(const std::string &folder, const std::map<std::string, vcp::best::StreamStorageParams> &storage_params, bool save_extrinsics)
  {
    return capture_->SaveReplayConfiguration(folder, storage_params, save_extrinsics);
  }

  bool SaveExtrinsics(const std::string &filename) const
  {
    return capture_->SaveExtrinsicCalibration(filename);
  }

private:
  std::unique_ptr<vcp::best::Capture> capture_;

  py::list FramesToList(const std::vector<cv::Mat> frames, bool flip_channels)
  {
    py::list numpy_frames;
    for (const auto &mat : frames)
    {
      if (mat.empty())
      {
        numpy_frames.append(py::none());
      }
      else
      {
        if (flip_channels && (mat.channels() == 3 || mat.channels() == 4))
        {
          cv::Mat converted;
          cv::cvtColor(mat, converted, mat.channels() == 3 ? CVTCOLOR_BGR2RGB : CVTCOLOR_BGRA2RGBA);
          numpy_frames.append(vcp::python::conversion::MatToNDArray(converted));
        }
        else
        {
          numpy_frames.append(vcp::python::conversion::MatToNDArray(mat));
        }
      }
    }
    return numpy_frames;
  }


  void LoadConfigJson(const std::string &json_str, bool is_json_string, const std::string &rel_path_base_dir)
  {
    auto config = vcp::config::LoadConfigParamsJson(json_str, is_json_string);
    AdjustFilePaths(*config, rel_path_base_dir);
    capture_ = vcp::best::CreateCapture(*config);
  }

  void AdjustFilePaths(vcp::config::ConfigParams &params, const std::string &rel_path_base_dir)
  {
    if (rel_path_base_dir.empty())
      return;
    // These parameters are usually used in my camera configuration files to
    // link to files/paths - thus, they should be adjusted if the current
    // working directory differs from the one you had in mind when creating
    // the configuration...
    const std::vector<std::string> param_names = {
      "extrinsic_calibration_file",               // Multi-cam calibration
      "extrinsic_calibration",                    // Backwards compatibility (with PVT)
      "calibration_file",                         // Intrinsic calibration
      "video_file", "video", "file", "filename",  // VideoFileSink
      "directory"                                 // ImageDirectorySink
    };
    params.EnsureAbsolutePaths(param_names, rel_path_base_dir, true);
  }
};



class RgbdAlignmentWrapper
{
public:
  RgbdAlignmentWrapper() : alignment_(nullptr) {}

  RgbdAlignmentWrapper(const cv::Mat &K_c, const cv::Mat &K_d,
                const cv::Mat &R_d2c, const cv::Mat &t_d2c,
                const cv::Size &size_c, const cv::Size &size_d,
                const cv::Mat &D_c, const cv::Mat &D_d)
  {
    alignment_ = vcp::best::rgbd::CreateRgbdAlignment(K_c, K_d, R_d2c, t_d2c, size_c, size_d, D_c, D_d);
  }

  // Disable copying
  RgbdAlignmentWrapper(const RgbdAlignmentWrapper&) = delete;
  RgbdAlignmentWrapper &operator=(const RgbdAlignmentWrapper&) = delete;

  virtual ~RgbdAlignmentWrapper()
  {
    if (alignment_)
      alignment_.reset();
  }

  py::object AlignDepthToColor(const cv::Mat &depth)
  {
    if (!alignment_)
    {
      VCP_LOG_FAILURE("RgbdAlignment was not properly initialized!");
      return py::none();
    }

    cv::Mat warped = alignment_->AlignDepth2Color(depth);
    if (warped.empty())
      return py::none();
    return vcp::python::conversion::MatToNDArray(warped);
  }

  py::tuple AlignDepthIrToColor(const cv::Mat &depth, const cv::Mat &infrared)
  {
    if (!alignment_)
    {
      VCP_LOG_FAILURE("RgbdAlignment was not properly initialized!");
      return py::make_tuple(py::none(), py::none());
    }

    cv::Mat warped_depth, warped_ir;
    alignment_->AlignDepthIR2Color(depth, infrared, warped_depth, warped_ir);

    return py::make_tuple(warped_depth.empty() ? py::none() : vcp::python::conversion::MatToNDArray(warped_depth),
                          warped_ir.empty() ? py::none() : vcp::python::conversion::MatToNDArray(warped_ir));
  }

private:
  std::unique_ptr<vcp::best::rgbd::RgbdAlignment> alignment_;
};


py::list ListMvBlueFox3Devices(bool warn_if_no_devices)
{
  py::list list;
#ifdef VCP_BEST_WITH_MATRIXVISION
  const std::vector<vcp::best::MvBlueFox3DeviceInfo> devices = vcp::best::ListMatrixVisionBlueFox3Devices(warn_if_no_devices);

  for (const auto &dev : devices)
    list.append(py::make_tuple(dev.serial_number, dev.name, dev.in_use));
#else
  (void)warn_if_no_devices;
  VCP_ERROR("You didn't enable VCP_BEST_WITH_MATRIXVISION.");
#endif
  return list;
}


py::list ListRealSense2Devices(bool warn_if_no_devices)
{
  py::list list;
#ifdef VCP_BEST_WITH_REALSENSE2
  const std::vector<vcp::best::realsense2::RealSense2DeviceInfo> devices = vcp::best::realsense2::ListRealSense2Devices(warn_if_no_devices);

  for (const auto &dev : devices)
    list.append(py::make_tuple(dev.serial_number, dev.name));

#else
  (void)warn_if_no_devices;
  VCP_ERROR("You didn't enable VCP_BEST_WITH_REALSENSE2.");
#endif
  return list;
}


py::list ListZedDevices(bool warn_if_no_devices, bool include_unavailable)
{
  py::list list;
#ifdef VCP_BEST_WITH_ZED
  const std::vector<vcp::best::zed::ZedDeviceInfo> devices = vcp::best::zed::ListZedDevices(warn_if_no_devices, include_unavailable);

  for (const auto &dev : devices)
    list.append(py::make_tuple(dev.serial_number, dev.model_name, dev.device_path, dev.available));

#else
  (void)warn_if_no_devices;
  (void)include_unavailable;
  VCP_ERROR("You didn't enable VCP_BEST_WITH_ZED.");
#endif
  return list;
}


py::list ListWebcams(bool warn_if_no_devices, bool include_incompatible_devices)
{
  py::list list;
  const auto devices = vcp::best::webcam::ListWebcams(warn_if_no_devices, include_incompatible_devices);
  for (const auto &dev : devices)
    list.append(py::make_tuple(dev.device_nr, dev.dev_name, dev.name));
  return list;
}


py::list ListK4ADevices(bool warn_if_no_devices)
{
  py::list list;
#ifdef VCP_BEST_WITH_K4A
  const std::vector<vcp::best::k4a::K4ADeviceInfo> devices = vcp::best::k4a::ListK4ADevices(warn_if_no_devices);

  for (const auto &dev : devices)
    list.append(py::make_tuple(dev.serial_number, dev.name));

#else
  (void)warn_if_no_devices;
  throw std::runtime_error("You didn't compile the pypvt wrappers with Kinect Azure support (CMake should have added -DWITH_K4A automatically if PVT_ROOT_DIR/include/pvt_icc/k4a_sink.h exists?!)");
#endif
  return list;
}


//bool StoreExtrinsics(const std::string &filename, const std::vector<std::string> &labels, const py::list &extrinsics)
//{
//  // First, set extrinsics
//  for (size_t idx = 0, idx < labels.size(); ++i)
//  {
//  }
//  const int num_cameras = static_cast<int>(labels.size());
//  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
//  if (!fs.isOpened())
//  {
//    VCP_LOG_FAILURE("Cannot open FileStorage '" << filename << "'");
//    return false;
//  }
//  fs << "num_cameras" << num_cameras;
//  for (int i = 0; i < num_cameras; ++i)
//  {
//    const std::string label = labels[i];
//    const py::tuple &rt = extrinsics[i].cast<py::tuple>();
//    const cv::Mat R = vcp::python::conversion::NDArrayToMat(rt[0].cast<py::array>());
//    const cv::Mat t = vcp::python::conversion::NDArrayToMat(rt[1].cast<py::array>());
//    std::stringstream ss;
//    ss << "label" << i;
//    fs << ss.str() << label;
//    ss.str("");
//    ss.clear();
//    ss << "R" << i;
//    fs << ss.str() << R;
//    ss.str("");
//    ss.clear();
//    ss << "t" << i;
//    fs << ss.str() << t;
//  }
//  fs.release();
//  return true;
//}

class LiveViewWrapper
{
public:
  LiveViewWrapper(const std::string &window_title, const cv::Size &max_win_size, const int wait_ms)
  {
    vcp::best::liveview::LiveViewParams params(window_title, max_win_size, wait_ms);
    liveview_ = vcp::best::liveview::CreateLiveView<2>(params);
  }

  // Disable copying
  LiveViewWrapper(const LiveViewWrapper&) = delete;
  LiveViewWrapper &operator=(const LiveViewWrapper&) = delete;

  virtual ~LiveViewWrapper()
  {
    if (liveview_)
    {
      liveview_->Stop();
      liveview_.reset();
    }
  }


  /** @brief Start displaying. */
  bool Start()
  {
    return liveview_->Start();
  }

  /** @brief Stop displaying. */
  bool Stop()
  {
    return liveview_->Stop();
  }

  /** @brief Enqueue a single image to be displayed.
   * If the image is larger than the configured max_size,
   * it will be resized.
   */
  void PushImageRequest(const cv::Mat &image, const bool flip_channels)
  {
    if (liveview_)
    {
      if (flip_channels && (image.channels() == 3 || image.channels() == 4))
      {
        cv::Mat converted;
        cv::cvtColor(image, converted, image.channels() == 3 ? CVTCOLOR_RGB2BGR : CVTCOLOR_RGBA2BGRA);
        liveview_->PushImageRequest(converted);
      }
      else
      {
        liveview_->PushImageRequest(image);
      }
    }
    else
    {
      VCP_ERROR("Liveview is not available");
    }
  }

  /** @brief The given images will be converted to a collage. */
  void PushCollageRequest(const std::vector<cv::Mat> &images, const bool flip_channels)
  {
    if (liveview_)
    {
      if (flip_channels)
      {
        std::vector<cv::Mat> flipped;
        for (const auto &image : images)
        {
          cv::Mat converted;
          if (image.channels() == 3 || image.channels() == 4)
            cv::cvtColor(image, converted, image.channels() == 3 ? CVTCOLOR_RGB2BGR : CVTCOLOR_RGBA2BGRA);
          else
            converted = image;
          flipped.push_back(converted);
        }
        liveview_->PushCollageRequest(flipped);
      }
      else
      {
        liveview_->PushCollageRequest(images);
      }
    }
    else
    {
      VCP_ERROR("Liveview is not available");
    }
  }

  /** @brief Change the time (in ms) to wait after displaying the image(s) (to let the display thread finish rendering). */
  void SetWaitMs(int wait_ms)
  {
    liveview_->SetWaitMs(wait_ms);
  }

  /** @brief Returns the result of the last cv::waitKey(). */
  int GetLastUserInput() const
  {
    return liveview_->GetLastUserInput();
  }

  /** @brief Returns the frame rate of the incoming view requests, NOT the display frame rate! */
  double GetRequestFrameRate() const
  {
    return liveview_->GetRequestFrameRate();
  }

  /** @brief Returns the current display frame rate. */
  double GetDisplayFrameRate() const
  {
    return liveview_->GetDisplayFrameRate();
  }

private:
  std::unique_ptr<vcp::best::liveview::LiveView> liveview_;
};
} // namespace best
} // namespace python
} // namespace vcp


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Python module declarations
PYBIND11_MODULE(best_cpp, m)
{
  namespace pybest = vcp::python::best;
  m.doc() = "vcp BESt capturing module - allows streaming from multiple cameras.";

  py::class_<pybest::CaptureWrapper>(m, "Capture")
// Initialization
      .def(py::init<>())
      .def("load_libconfig", &pybest::CaptureWrapper::LoadConfigCpp,
           "Init the capture from a libconfig configuration file.\n"
           "If you set rel_path_base_dir, all relative file paths will\n"
           "be prefixed accordingly.",
           py::arg("config_file"), py::arg("rel_path_base_dir")=std::string())
      .def("load_json_string", &pybest::CaptureWrapper::LoadConfigJsonString,
           "Init the capture from a JSON string.\n"
           "If you set rel_path_base_dir, all relative file paths will\n"
           "be prefixed accordingly.",
           py::arg("json_str"), py::arg("rel_path_base_dir")=std::string())
      .def("load_json_file", &pybest::CaptureWrapper::LoadConfigJsonFile,
           "Init the capture from a JSON file.\n"
           "If you set rel_path_base_dir, all relative file paths will\n"
           "be prefixed accordingly.",
           py::arg("json_file"), py::arg("rel_path_base_dir")=std::string())
      .def("load_vcp_config", &pybest::CaptureWrapper::LoadConfigParams,
           "Init the capture from a vcp.config.Config() object.\n"
           "If you want to ensure absolute file paths, call config.ensure_absolute_file_paths() before!",
           py::arg("config"))
// Workflow:
      .def("open", &pybest::CaptureWrapper::OpenDevices,
           "Initialize devices, returns bool.")
      .def("close", &pybest::CaptureWrapper::CloseDevices,
           "Close devices/tear down connections, returns bool.")
      .def("start", &pybest::CaptureWrapper::StartStreams,
           "Starts the image streams, returns bool.")
      .def("stop", &pybest::CaptureWrapper::StopStreams,
           "Stops the image streams, returns bool.")
// Query frames:
      .def("wait_for_frames", &pybest::CaptureWrapper::WaitForFrames,
           "Use this blocking call to wait for the next set of images to\n"
           "become available.\n"
           "Specify a timeout in milliseconds (float), after which the\n"
           "boolean result will indicate whether frames from all devices/sinks\n"
           "are available or not.",
           py::arg("timeout_ms")=5000.0, py::arg("verbose")=true)
      .def("next", &pybest::CaptureWrapper::NextFrame,
           "Returns the currently available frames as a list of images,\n"
           "i.e. numpy ndarray or None. The output list will have exactly\n"
           "one item per configured stream (NOT sink)."
           "Empty frames (i.e. None) may occur if you query faster than the stream's\n"
           "frame rate.\n"
           "Use the flip_channel flag to flip the layer order of color.\n",
           py::arg("flip_channels")=false)
      .def("previous", &pybest::CaptureWrapper::PreviousFrame,
           "Some sinks support retrieving the previous frame, i.e. video and\n"
           "image directory. Others will raise an exception. @see next()\n"
           "for description of the return value and parameter.",
           py::arg("flip_channels")=false)
      .def("fast_forward", &pybest::CaptureWrapper::FastForward,
           "Some sinks support skipping frames (if num_frames > 1), i.e. video and\n"
           "image directory. Others will raise an exception. @see next()\n"
           "for description of the return value and 'flip_channel'.",
           py::arg("num_frames"), py::arg("flip_channel")=false)
// Info & status queries
      .def("frame_number", &pybest::CaptureWrapper::FrameNumber,
           "Returns the current frame number (keeps track of the next(),\n"
           "previous() and fast_forward() calls).\n"
           "The counter always starts at 0 upon initialization of the capture!\n"
           "Note that after retrieving the first frameset (e.g. via next()),\n"
           "the frame_number() will be 1!")
      .def("all_devices_available", &pybest::CaptureWrapper::AreAllDevicesAvailable,
           "Returns True if the capturing device/s is/are available.")
      .def("all_frames_available", &pybest::CaptureWrapper::AreAllFramesAvailable,
           "Returns true if frames (from all devices) can be retrieved (i.e. have been enqueued).")
      .def("num_available_frames", &pybest::CaptureWrapper::NumAvailableFrames,
           "Returns the number of available frames.\n\n"
           "Might be useful if one of your sinks stops working/streaming and you still want to continue\n"
           "processing, etc. In such a case, @see all_frames_available() would return false, whereas here\n"
           "you get the actual number of frames and can decide what to do for yourself.")
      .def("num_streams", &pybest::CaptureWrapper::NumStreams,
           "Returns the number of configured streams (NOT the number of sinks).")
      .def("num_devices", &pybest::CaptureWrapper::NumDevices,
           "Returns the number of devices/sinks (NOT the number of streams/frames).\n"
           "While this number corresponds to the number of 'software sinks' used in VCP,\n"
           "it may not be exactly the number of physical devices: For example, you could\n"
           "configure 2 IP streams from the same device, one configured by its IP and the\n"
           "other by its hostname. Handling such cases would be too complex.")
      .def("frame_labels", &pybest::CaptureWrapper::FrameLabels,
           "Returns the (user-defined) frame labels.\n"
           "Note that:\n"
           " - The order may differ from the one you used in the configuration file.\n"
           "   This is because some devices must be handled specifically (e.g. multiple\n"
           "   simultaneous RTSP streams, or synchronized/triggered RGBD sensors).\n"
           " - The labels may also differ because you specify a per-device label. Sinks\n"
           "   with multiple streams (e.g. or RGBD) will prefix the label accordingly.")
      .def("frame_label", &pybest::CaptureWrapper::FrameLabel,
           "Returns the (user-defined) frame label for frame/stream at the given stream_index.",
           py::arg("stream_index"))
      .def("canonic_frame_label", &pybest::CaptureWrapper::CanonicFrameLabel,
           "Returns a canonic version of the (user-defined) frame label, i.e. special characters\n"
           "are stripped or replaced, so you'll get a label which can be used, e.g., as a file name.",
           py::arg("stream_index"))
      .def("configuration_keys", &pybest::CaptureWrapper::ConfigurationKeys,
           "Returns the configuration parameter name, e.g. 'cameraX', for\n"
           "each stream/frame.\n"
           "See comments on @see frame_labels() why this might be of interest\n"
           "to you. However, it's usually better to query the @see frame_labels()\n"
           "and/or @see frame_types() instead.")
      .def("configuration_key", &pybest::CaptureWrapper::ConfigurationKey,
           "Returns the configuration parameter name for frame/stream at the given stream_index.",
           py::arg("stream_index"))
      .def("frame_types", &pybest::CaptureWrapper::FrameTypes,
           "Returns the FrameType (string) for each stream/frame.\n"
           "See comments on @see frame_labels() why you cannot assume\n"
           "that the frames will be in the same order you specified\n"
           "in your configuration file.\n"
           "Thus, use @see configuration_keys() in combination with\n"
           "this FrameType information to access a specific stream.\n\n"
           "This has only be taken into account if you mix the data\n"
           "sources (e.g. stream webcams + RGBD + RTSP).")
      .def("frame_type", &pybest::CaptureWrapper::FrameTypeAt,
           "Returns the FrameType of the frame/stream at the given stream_index.",
           py::arg("stream_index"))
      .def("sink_type", &pybest::CaptureWrapper::SinkTypeAt,
           "Returns the SinkType (not frame type!) of the frame/stream at the given stream_index.")
      .def("is_unknown_type", &pybest::CaptureWrapper::IsFrameTypeUnknown,
           "Returns true if the FrameType of the frame at the given stream_index\n"
           "is 'unknown' (i.e. it hasn't been set or cannot be deduced from the\n"
           "imaging sensor).",
           py::arg("stream_index"))
      .def("is_monocular", &pybest::CaptureWrapper::IsFrameMonocular,
           "Check if the frame at the given index is of type 'monocular',\n"
           "i.e. grayscale or color. Note that for some streams, we cannot\n"
           "automatically derive the frame type, e.g. USB stereo cameras vs\n"
           "monocular cameras - there, it is the user's responsibility to\n"
           "properly set the 'frame_type' parameter wihtin the configuration.",
           py::arg("stream_index"))
      .def("is_stereo", &pybest::CaptureWrapper::IsFrameStereo,
           "Check if the frame at the given index is of type 'stereo',\n"
           "Note that for some streams, we cannot automatically derive the\n"
           "frame type, e.g. USB stereo cameras vs monocular cameras - there,\n"
           "it is the user's responsibility to properly set the 'frame_type'\n"
           "parameter wihtin the configuration.",
           py::arg("stream_index"))
      .def("is_image", &pybest::CaptureWrapper::IsFrameImage,
           "Check if the frame at the given index is a grayscale or color\n"
           "image (i.e. 'monocular' or 'stereo').",
           py::arg("stream_index"))
      .def("is_depth", &pybest::CaptureWrapper::IsFrameDepth,
           "Check if the frame at the given index is of type 'depth',\n"
           "i.e. depth measurements (16bit Kinect, RealSense), (float32 ZED).",
           py::arg("stream_index"))
      .def("is_infrared", &pybest::CaptureWrapper::IsFrameInfrared,
           "Check if the frame at the given index is of type 'infrared',\n"
           "i.e. intensity measurements (16bit Kinect, 8bit RealSense).",
           py::arg("stream_index"))
      .def("is_rectified", &pybest::CaptureWrapper::IsFrameRectified,
           "Returns True if the frame at the given index is rectified.",
           py::arg("stream_index"))
      .def("is_rgb", &pybest::CaptureWrapper::IsFrameRGB,
           "Returns True if the frame at the given index is RGB.",
           py::arg("stream_index"))
      .def("is_step_able_image_sequence", &pybest::CaptureWrapper::IsStepAbleImageSequence,
           "Returns True if the frame at the given index comes from\n"
           "an image sequence where each call to @see next() would\n"
           "load the next frameset from disk. Thus, you could use this\n"
           "information to make your application 'step through' the stream\n"
           "instead of assuming it is a live stream.", py::arg("stream_index"))
      .def("same_sink", &pybest::CaptureWrapper::StreamsFromSameSink,
           "Returns a list of frame indices which originate from the\n"
           "same sink as the given frame index.\n"
           ":param stream_index: Index (int >= 0) of the stream/frame of interest.\n"
           ":param include_self: True, if stream_index should be included.",
           py::arg("stream_index"), py::arg("include_self")=false)
// Calibration-related
      .def("camera_matrix", &pybest::CaptureWrapper::CameraMatrix,
           "Returns the 3x3 camera matrix holding the stream's intrinsics.",
           py::arg("stream_index"))
      .def("intrinsics", &pybest::CaptureWrapper::CameraMatrix,
           "Alias for @see camera_matrix().")
      .def("distortion_coefficients", &pybest::CaptureWrapper::Distortion,
           "Returns the Nx1 distortion coefficients of this stream (if calibrated).",
           py::arg("stream_index"))
      .def("stereo_transformation", &pybest::CaptureWrapper::StereoTransformation,
           "Returns the transformation from this stream/view to the reference\n"
           "view as tuple (R,t) if exists. Only useful for calibrated stereo\n"
           "setups.",
           py::arg("stream_index"))
      .def("extrinsics", &pybest::CaptureWrapper::Extrinsics,
           "Returns the extrinsic camera pose for the given stream\n"
           "as tuple (R,t), where R is 3x3 and t is 3x1.",
           py::arg("stream_index"))
      .def("set_extrinsics", (void (pybest::CaptureWrapper::*)(size_t, const cv::Mat &, const cv::Mat &)) &pybest::CaptureWrapper::SetExtrinsics,
           "Sets the extrinsics (R, t) for the given stream. If they\n"
           "are invalid (None), the corresponding sensor/sink will try\n"
           "to derive the transformation from the corresponding reference\n"
           "view (e.g. a stereo camera's left view or an RGBD sensor's color\n"
           "stream).\n\n"
           ":param stream_index: Which stream to set the extrinsics for.\n"
           ":param R: Rotation matrix as 3x3 numpy ndarray.\n"
           ":param t: Translation vector as 3x1 numpy ndarray.",
           py::arg("stream_index"), py::arg("R"), py::arg("t"))
      .def("set_extrinsics", (void (pybest::CaptureWrapper::*)(const std::string &, const cv::Mat &, const cv::Mat &)) &pybest::CaptureWrapper::SetExtrinsics,
           "Sets the extrinsics (R, t) for the given stream. If they\n"
           "are invalid (None), the corresponding sensor/sink will try\n"
           "to derive the transformation from the corresponding reference\n"
           "view (e.g. a stereo camera's left view or an RGBD sensor's color\n"
           "stream).\n\n"
           ":param stream_label: Which stream to set the extrinsics for.\n"
           ":param R: Rotation matrix as 3x3 numpy ndarray.\n"
           ":param t: Translation vector as 3x1 numpy ndarray.",
           py::arg("stream_index"), py::arg("R"), py::arg("t"))
// Saving
      .def("save_replay_config", &pybest::CaptureWrapper::SaveReplayConfig,
           "Store a configuration file plus required intrinsic calibrations\n"
           "to the given output folder.\nIf (1) the extrinsic calibrations\n"
           "have been loaded upon startup or (2) have been injected by you\n"
           "via @see set_extrinsics) and you set save_extrinsics to True,\n"
           "they'll be stored, too.\n\n"
           "Note that you have to record/store the streams (image data) yourself!\n\n"
           ":param folder: Output folder where to store the configuration\n"
           "               and calibration to. Will be created if it does\n"
           "               not exist. Existing files WILL BE OVERWRITTEN!\n"
           "               The config will be stored to <folder>/replay.cfg,\n"
           "               calibration files to <folder>/<calibration>/calib-<stream_label>.xml\n\n"
           ":param storage_params: A dict of frame_label: vcp.best.StreamStorageParams,\n"
           "               one for each stream - specifying whether (and where\n"
           "               the corresponding stream will be saved to).\n"
           "               If a StreamStorageParams.path is relative, the 'folder' will be prepended.\n"
           ":param save_extrinsics: Boolean flag.\n"
           ":return: Boolean (True upon success).",
           py::arg("folder"), py::arg("storage_params"), py::arg("save_extrinsics"))
      .def("save_extrinsics", &pybest::CaptureWrapper::SaveExtrinsics,
           "Store the capture's extrinsics (which have to be set PIOR to this call\n"
           "by (1) loading from disk, i.e. using the config file or (2) by setting\n"
           "them via @see set_extrinsics) to the given (XML) file.\n"
           ":param filename: String, where to store the extrinsics.\n"
           ":returns: True if extrinsics have been written.",
           py::arg("filename"));


  //      .def("get_camera_matrix", &pybest::CaptureWrapper::ReturnNone,
//           "Returns None - method is only useful for a RectifiedCapture object",
//           py::arg("sink_index"))
//      .def("get_camera_matrix_right", &pybest::CaptureWrapper::ReturnNone,
//           "Returns None - method is only useful for a RectifiedCapture object",
//           py::arg("sink_index"))
//      .def("get_stereo_Rt", &pybest::CaptureWrapper::ReturnNoneTuple,
//           "Returns (None,None) - method is only useful for a RectifiedCapture object",
//           py::arg("sink_index"))
//      .def("is_stereo_stream", &pybest::CaptureWrapper::IsStereoStream,
//           "Returns true if stream at sink_index is a stereo stream.", py::arg("sink_index"))
//      .def("is_depth_stream", &pybest::CaptureWrapper::IsDepthStream,
//           "Returns true if stream at sink_index is a depth stream.", py::arg("sink_index"));


//      .def("get_camera_matrix", &pybest::RectifiedCaptureWrapper::GetCameraMatrix,
//           "Returns the 3x3 camera matrix (holding the intrinsics).\n"
//           "In a stereo setup, this will be the left (reference) camera's\n"
//           "matrix, see also get_camera_matrix_right()", py::arg("sink_index"))
//      .def("get_camera_matrix_right", &pybest::RectifiedCaptureWrapper::GetRightCameraMatrix,
//           "In a stereo setup, this returns the 3x4 camera matrix holding the\n"
//           "intrinsics and translation of the right lens. See also get_camera_matrix()",
//           py::arg("sink_index"))
//      .def("get_stereo_Rt", &pybest::RectifiedCaptureWrapper::GetStereoRt,
//           "Returns the tuple(R,t), see get_rotation() get_translation().",
//           py::arg("sink_index"))
//      .def("get_rotation", &pybest::RectifiedCaptureWrapper::GetRotation,
//           "Returns the 3x3 rotation matrix (or None).", py::arg("sink_index"))
//      .def("get_translation", &pybest::RectifiedCaptureWrapper::GetTranslation,
//           "Returns the 3x1 translation vector (or None).", py::arg("sink_index"))
//      .def("get_optical_center", &pybest::RectifiedCaptureWrapper::GetCameraCenter,
//           "Returns the 3x1 optical center C in world coordinates (or None).",
//           py::arg("sink_index"))
//      .def("get_image_plane", &pybest::RectifiedCaptureWrapper::GetImagePlane,
//           "Returns the image plane in Hessian normal form (4-element vector).",
//           py::arg("sink_index"))
//      .def("save_calibration", &pybest::RectifiedCaptureWrapper::SaveCalibration,
//           "Saves calibration to disk using OpenCV's FileStorage.",py::arg("filename"))
//      .def("is_stereo_stream", &pybest::RectifiedCaptureWrapper::IsStereoStream,
//           "Returns true if stream at sink_index is a stereo stream.", py::arg("sink_index"))
//      .def("is_depth_stream", &pybest::RectifiedCaptureWrapper::IsDepthStream,
//           "Returns true if stream at sink_index is a depth stream.", py::arg("sink_index"));

  py::class_<pybest::LiveViewWrapper>(m, "LiveView")
      .def(py::init<std::string, cv::Size, int>(), "Provide the window name, maximum window size (w,h) and wait_ms",
           py::arg("window_name"), py::arg("max_win_size")=cv::Size(1920, 1200), py::arg("wait_ms")=10)
      .def("push_image_request", &pybest::LiveViewWrapper::PushImageRequest,
           "Push an incoming image into the liveview queue.",
           py::arg("image"), py::arg("flip_channels")=false)
      .def("push_collage_request", &pybest::LiveViewWrapper::PushCollageRequest,
           "Push a list of incoming images into the liveview queue.",
           py::arg("images"), py::arg("flip_channels")=false)
      .def("set_wait_ms", &pybest::LiveViewWrapper::SetWaitMs,
           "Set wait time parameter passed to cv::waitKey().", py::arg("wait_ms"))
      .def("get_last_user_input", &pybest::LiveViewWrapper::GetLastUserInput,
           "Returns the result of the last cv::waitKey() call.")
      .def("get_request_fps", &pybest::LiveViewWrapper::GetRequestFrameRate,
           "Returns the frame rate of the incoming view requests, NOT the display frame rate!")
      .def("get_display_fps", &pybest::LiveViewWrapper::GetDisplayFrameRate,
           "Returns the current display frame rate.");

  py::class_<vcp::best::StreamStorageParams> storage_params(m, "StreamStorageParams");
  storage_params.def(py::init<const vcp::best::StreamStorageParams::Type &, const std::string &>())
      .def(py::init<>())
      .def_readwrite("type", &vcp::best::StreamStorageParams::type)
      .def_readwrite("path", &vcp::best::StreamStorageParams::path);
  py::enum_<vcp::best::StreamStorageParams::Type>(storage_params, "Type")
      .value("Skip", vcp::best::StreamStorageParams::Type::NONE)
      .value("ImageSequence", vcp::best::StreamStorageParams::Type::IMAGE_DIR)
      .value("Video", vcp::best::StreamStorageParams::Type::VIDEO)
      .export_values();


  py::class_<pybest::RgbdAlignmentWrapper>(m, "RgbdAlignment")
// Initialization
//      .def(py::init<>(), "Default constructor")
      /*const cv::Mat &K_c, const cv::Mat &K_d,
                    const cv::Mat &R_d2c, const cv::Mat &t_d2c,
                    const cv::Size &size_c, const cv::Size &size_d,
                    const cv::Mat &D_c, const cv::Mat &D_d*/
      .def(py::init<const cv::Mat &, const cv::Mat &, const cv::Mat &, const cv::Mat &,
                    const cv::Size &, const cv::Size &, const cv::Mat &, const cv::Mat &>(),
           "Prepares the alignment from the given stereo calibration data.\n\n"
           ":param K_color: 3x3 intrinsic calibration of color stream (single or double precision).\n"
           ":param K_depth: 3x3 intrinsics of depth stream (single or double precision).\n"
           ":param R_d2c:   3x3 rotation matrix from depth to color (single or double precision).\n"
           ":param t_d2c:   3x1 translation vector from depth to color (single or double precision).\n"
           ":param size_color: color stream resolution as tuple(width, height) of type int.\n"
           ":param size_depth: depth stream resolution as tuple(width, height) of type int.\n"
           ":param D_color:    Dx1 distortion coefficients (Brown Conrady model) for\n"
           "                   color stream (single or double precision).\n"
           "                   Should be [k1, k2, p1, p2] additional radial\n"
           "                   coefficients can be provided starting from \n"
           "                   the 5th element, up to [k1, k2, p1, p2, k3, k4, k5, k6]."
           ":param D_depth:    Dx1 distortion coefficients (Brown Conrady model) for\n"
           "                   depth stream (single or double precision).\n",
           py::arg("K_color"), py::arg("K_depth"),
           py::arg("R_d2c"), py::arg("t_d2c"),
           py::arg("size_color"), py::arg("size_depth"),
           py::arg("D_color")=cv::Mat(), py::arg("D_depth")=cv::Mat())
      .def("align_d2c", &pybest::RgbdAlignmentWrapper::AlignDepthToColor,
           "Align the given depth image to the color stream.\n\n"
           ":param depth: single-channel 16bit (!) depth image\n"
           ":return: single-channel 16bit depth image aligned to the color stream",
           py::arg("depth"))
      .def("align_di2c", &pybest::RgbdAlignmentWrapper::AlignDepthIrToColor,
           "Align the given depth & infrared image to the color stream.\n\n"
           ":param depth: single-channel 16bit depth image\n"
           ":param ir:    single-channel 16bit intensity/infrared image\n"
           ":return: tuple(aligned depth, aligned infrared)",
           py::arg("depth"), py::arg("ir"));


  m.def("list_k4a_devices", &pybest::ListK4ADevices,
        "Returns a list of connected Kinect Azure devices.", py::arg("warn_if_no_devices")=true);

  m.def("list_mvbluefox3_devices", &pybest::ListMvBlueFox3Devices,
        "Returns a list of connected mvBlueFox3 devices.", py::arg("warn_if_no_devices")=true);

  m.def("list_realsense2_devices", &pybest::ListRealSense2Devices,
        "Returns a list of connected RealSense2 devices.", py::arg("warn_if_no_devices")=true);

  m.def("list_zed_devices", &pybest::ListZedDevices,
        "Returns a list of connected ZED devices.",
        py::arg("warn_if_no_devices")=true,
        py::arg("include_unavailable_devices")=false);

  m.def("list_webcams", &pybest::ListWebcams,
        "Returns a list of connected webcams.",
        py::arg("warn_if_no_devices")=true,
        py::arg("include_incompatible_devices")=false);

//  m.def("store_extrinsics", &pybest::StoreExtrinsics,
//        "Saves the extrinsic calibration to the given file.", //TODO doc
//        py::arg("filename"), py::arg("labels"), py::arg("extrinsics"));

  //FUCK WITH IPCAM
#ifdef VCP_BEST_WITH_IPCAM
  py::enum_<vcp::best::ipcam::IpApplicationProtocol>(m, "IpApplicationProtocol")
      .value("HTTP", vcp::best::ipcam::IpApplicationProtocol::HTTP)
      .value("RTSP", vcp::best::ipcam::IpApplicationProtocol::RTSP)
      .export_values();

  py::enum_<vcp::best::ipcam::IpStreamEncoding>(m, "IpStreamEncoding")
      .value("MJPEG", vcp::best::ipcam::IpStreamEncoding::MJPEG)
      .value("H264", vcp::best::ipcam::IpStreamEncoding::H264)
      .export_values();

  py::enum_<vcp::best::ipcam::IpTransportProtocol>(m, "IpTransportProtocol")
      .value("TCP", vcp::best::ipcam::IpTransportProtocol::TCP)
      .value("UDP", vcp::best::ipcam::IpTransportProtocol::UDP)
      .export_values();
#endif // VCP_BEST_WITH_IPCAM
}
