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
    capture_ = vcp::best::CreateCapture(*config, rel_path_base_dir);
  }

  void LoadConfigJsonString(const std::string &json_str, const std::string &rel_path_base_dir)
  {
    LoadConfigJson(json_str, true, rel_path_base_dir);
  }

  void LoadConfigJsonFile(const std::string &json_file, const std::string &rel_path_base_dir)
  {
    LoadConfigJson(json_file, false, rel_path_base_dir);
  }

  void LoadConfigParams(const vcp::python::config::ConfigWrapper &config, const std::string &rel_path_base_dir)
  {
    const vcp::config::ConfigParams &params = config.AsConfigParams();
    capture_ = vcp::best::CreateCapture(params, rel_path_base_dir);
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

  /** @brief After starting, you can wait for the first set of images to become available.
   * Specify a timeout in milliseconds, after that the result will indicate whether
   * frames from all devices/sinks are available or not.
   */
  bool WaitForInitialFrames(double timeout_ms)
  {
    return capture_->WaitForInitialFrames(timeout_ms);
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

  std::string FrameLabel(size_t sink_index) const
  {
    const auto lbls = capture_->FrameLabels();
    return lbls[sink_index];
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
    const auto keys = capture_->ConfigurationKeys();
    return keys[stream_index];
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


//  py::object ReturnNone(size_t /*sink_index*/) const
//  {
//    return py::none();
//  }


//  py::tuple ReturnNoneTuple(size_t /*sink_index*/) const
//  {
//    return py::make_tuple(py::none(), py::none());
//  }


private:
  std::unique_ptr<vcp::best::Capture> capture_;

  //FIXME rename param to flip_channels
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
          cv::cvtColor(mat, converted, mat.channels() == 3 ? CV_BGR2RGB : CV_BGRA2RGBA);
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
    const auto config = vcp::config::LoadConfigParamsJson(json_str, is_json_string);
    capture_ = vcp::best::CreateCapture(*config, rel_path_base_dir);
  }

};



//class RectifiedCaptureWrapper
//{
//public:
//  RectifiedCaptureWrapper() : capture_(nullptr) {}

//  // Disable copying
//  RectifiedCaptureWrapper(const RectifiedCaptureWrapper&) = delete;
//  RectifiedCaptureWrapper &operator=(const RectifiedCaptureWrapper&) = delete;

//  void LoadConfigCpp(const std::string &config_file, bool ensure_extrinsics_are_loaded)
//  {
//    const auto config = pvt::config::LoadConfigParamsCpp(config_file);
//    capture_ = pvt::icc::CreateRectifiedCapture(*config, ensure_extrinsics_are_loaded);
//  }

//  void LoadConfigJson(const std::string &json_str, bool is_json_string, bool ensure_extrinsics_are_loaded)
//  {
//    const auto config = pvt::config::LoadConfigParamsJson(json_str, is_json_string);
//    capture_ = pvt::icc::CreateRectifiedCapture(*config, ensure_extrinsics_are_loaded);
//  }

//  void LoadConfigParams(const pvt::python::config::ConfigWrapper &config, bool ensure_extrinsics_are_loaded)
//  {
//    const pvt::config::ConfigParams &params = config.AsConfigParams();
//    capture_ = pvt::icc::CreateRectifiedCapture(params, ensure_extrinsics_are_loaded);
//  }

//  virtual ~RectifiedCaptureWrapper()
//  {
//    if (capture_)
//      capture_.reset();
//  }

//  bool Start()
//  {
//    if (capture_)
//      return capture_->Start();
//    return false;
//  }

//  bool IsAvailable()
//  {
//    if (capture_)
//      return capture_->IsAvailable();
//    return false;
//  }

//  bool IsFrameAvailable()
//  {
//    if (capture_)
//      return capture_->IsFrameAvailable();
//    return false;
//  }

//  /** @brief Collect all frames from the capture and optionally flipping the channels so you can work with RGB data (OpenCV uses BGR by default). */
//  py::list NextFrame(bool return_rgb)
//  {
//    py::list numpy_frames;
//    std::vector<cv::Mat> frames;
//    if (capture_)
//      frames = capture_->NextFrame();
//    for (const auto &mat : frames)
//    {
//      if (mat.empty())
//      {
//        numpy_frames.append(py::none());
//      }
//      else
//      {
//        if (return_rgb && (mat.channels() == 3 || mat.channels() == 4))
//        {
//          cv::Mat converted;
//          cv::cvtColor(mat, converted, mat.channels() == 3 ? CV_BGR2RGB : CV_BGRA2RGBA);
//          numpy_frames.append(pvt::python::conversion::MatToNDArray(converted));
//        }
//        else
//        {
//          numpy_frames.append(pvt::python::conversion::MatToNDArray(mat));
//        }
//      }
//    }
//    return numpy_frames;
//  }


//  /** @brief Collect all frames from the capture and optionally flipping the channels so you can work with RGB data (OpenCV uses BGR by default). */
//  py::list PreviousFrame(bool return_rgb)
//  {
//    py::list numpy_frames;
//    std::vector<cv::Mat> frames;
//    if (capture_)
//      frames = capture_->PreviousFrame();
//    for (const auto &mat : frames)
//    {
//      if (mat.empty())
//      {
//        numpy_frames.append(py::none());
//      }
//      else
//      {
//        if (return_rgb && (mat.channels() == 3 || mat.channels() == 4))
//        {
//          cv::Mat converted;
//          cv::cvtColor(mat, converted, mat.channels() == 3 ? CV_BGR2RGB : CV_BGRA2RGBA);
//          numpy_frames.append(pvt::python::conversion::MatToNDArray(converted));
//        }
//        else
//        {
//          numpy_frames.append(pvt::python::conversion::MatToNDArray(mat));
//        }
//      }
//    }
//    return numpy_frames;
//  }


//  /** @brief Collect all frames from the capture and optionally flipping the channels so you can work with RGB data (OpenCV uses BGR by default). */
//  py::list FastForward(size_t num_frames, bool return_rgb)
//  {
//    py::list numpy_frames;
//    std::vector<cv::Mat> frames;
//    if (capture_)
//      frames = capture_->FastForward(num_frames);
//    for (const auto &mat : frames)
//    {
//      if (mat.empty())
//      {
//        numpy_frames.append(py::none());
//      }
//      else
//      {
//        if (return_rgb && (mat.channels() == 3 || mat.channels() == 4))
//        {
//          cv::Mat converted;
//          cv::cvtColor(mat, converted, mat.channels() == 3 ? CV_BGR2RGB : CV_BGRA2RGBA);
//          numpy_frames.append(pvt::python::conversion::MatToNDArray(converted));
//        }
//        else
//        {
//          numpy_frames.append(pvt::python::conversion::MatToNDArray(mat));
//        }
//      }
//    }
//    return numpy_frames;
//  }


//  bool Stop()
//  {
//    if (capture_)
//      return capture_->Terminate();
//    return false;
//  }

//  std::string GetLabel(size_t sink_index) const
//  {
//    return capture_->GetSinkLabel(sink_index);
//  }

//  std::vector<std::string> GetLabels() const
//  {
//    std::vector<std::string> labels;
//    for (size_t i = 0; i < capture_->NumStreams(); ++i)
//      labels.push_back(capture_->GetSinkLabel(i));
//    return labels;
//  }

//  std::string GetCorrespondingConfigKey(size_t sink_index) const
//  {
//    return capture_->GetCorrespondingConfigKey(sink_index);
//  }

//  std::vector<std::string> GetCorrespondingConfigKeys() const
//  {
//    std::vector<std::string> keys;
//    for (size_t i = 0; i < capture_->NumStreams(); ++i)
//      keys.push_back(capture_->GetCorrespondingConfigKey(i));
//    return keys;
//  }

//  std::string GetStreamType(size_t stream_index) const
//  {
//    std::string rep = pvt::icc::StreamTypeToString(capture_->GetStreamType(stream_index));
//    pvt::utils::string::ToLower(rep);
//    return rep;
//  }

//  std::vector<std::string> GetStreamTypes() const
//  {
//    std::vector<std::string> types;
//    for (size_t i = 0; i < capture_->NumStreams(); ++i)
//      types.push_back(GetStreamType(i));
//    return types;
//  }

//  py::object GetCameraMatrix(size_t sink_index)
//  {
//    const cv::Mat K = capture_->K(sink_index);
//    if (K.empty())
//      return py::none();
//    py::array npK = pvt::python::conversion::MatToNDArray(K);
//    npK.resize({3, 3}); //TODO needs to be checked!
//    return npK;
//  }

//  // In a stereo set up, you might be interested in the right camera's matrix too (3x4, [K T] after rectification)
//  py::object GetRightCameraMatrix(size_t sink_index)
//  {
//    const cv::Mat P2 = capture_->P2(sink_index);
//    if (P2.empty())
//      return py::none();
//    py::array npp2 = pvt::python::conversion::MatToNDArray(P2);
//    npp2.resize({3,4});
//    return npp2;
//  }

//  py::object GetRotation(size_t sink_index)
//  {
//    const cv::Mat R = capture_->R(sink_index);
//    if (R.empty())
//      return py::none();
//    py::array npR = pvt::python::conversion::MatToNDArray(R);
//    npR.resize({3,3});
//    return npR;
//  }

//  py::object GetTranslation(size_t sink_index)
//  {
//    const cv::Mat t = capture_->t(sink_index);
//    if (t.empty())
//      return py::none();
//    py::array npt = pvt::python::conversion::MatToNDArray(t);
//    npt.resize({3,1});
//    return npt;
//  }

//  py::object GetCameraCenter(size_t sink_index)
//  {
//    const cv::Mat C = capture_->C(sink_index);
//    if (C.empty())
//      return py::none();
//    py::array npc = pvt::python::conversion::MatToNDArray(C);
//    npc.resize({3,1});
//    return npc;
//  }

//  cv::Vec4d GetImagePlane(size_t sink_index)
//  {
//    return capture_->ImagePlane(sink_index);
//  }

//  py::tuple GetStereoRt(size_t sink_index)
//  {
//    py::object R = GetRotation(sink_index);
//    py::object t = GetTranslation(sink_index);
//    return py::make_tuple(R, t);
//  }

//  void SaveCalibration(const std::string &filename)
//  {
//    capture_->SaveCalibration(filename);
//  }

//  bool IsStreamType(size_t sink_index, const pvt::icc::StreamType &type) { return capture_->GetStreamType(sink_index) == type; }

//  bool IsStereoStream(size_t sink_index) { return IsStreamType(sink_index, pvt::icc::StreamType::STEREO); }

//  bool IsDepthStream(size_t sink_index) { return IsStreamType(sink_index, pvt::icc::StreamType::RGBD_DEPTH); }


//private:
//  std::unique_ptr<pvt::icc::RectifiedCapture> capture_;
//};



py::list ListMvBlueFox3Devices(bool warn_if_no_devices)
{
  py::list list;
#ifdef VCP_BEST_WITH_MATRIXVISION
  const std::vector<vcp::best::MvBlueFox3DeviceInfo> devices = vcp::best::ListMatrixVisionBlueFox3Devices(warn_if_no_devices);

  for (const auto &dev : devices)
    list.append(py::make_tuple(dev.serial_number, dev.name, dev.in_use));
#else
  (void)warn_if_no_devices;
  VCP_ERROR("Your vcp library build is missing MatrixVision support!");
#endif
  return list;
}


py::list ListRealSense2Devices(bool warn_if_no_devices)
{
  py::list list;
#ifdef VCP_BEST_WITH_REALSENSE2
  const std::vector<vcp::best::rs2::RealSense2DeviceInfo> devices = pvt::icc::ListRealSense2Devices(warn_if_no_devices);

  for (const auto &dev : devices)
    list.append(py::make_tuple(dev.serial_number, dev.name));

#else
  (void)warn_if_no_devices;
  throw std::runtime_error("You didn't compile the pypvt wrappers with RealSense2 support (CMake should have added -DWITH_REALSENSE2 automatically if PVT_ROOT_DIR/include/pvt_icc/realsense2_sink.h exists?!)");
#endif
  return list;
}


py::list ListK4ADevices(bool warn_if_no_devices)
{
  py::list list;
#ifdef WITH_K4A //FIXME
  const std::vector<pvt::icc::K4ADeviceInfo> devices = pvt::icc::ListK4ADevices(warn_if_no_devices);

  for (const auto &dev : devices)
    list.append(py::make_tuple(dev.serial_number, dev.name));

#else
  (void)warn_if_no_devices;
  throw std::runtime_error("You didn't compile the pypvt wrappers with Kinect Azure support (CMake should have added -DWITH_K4A automatically if PVT_ROOT_DIR/include/pvt_icc/k4a_sink.h exists?!)");
#endif
  return list;
}


bool StoreExtrinsics(const std::string &filename, const std::vector<std::string> &labels, const py::list &extrinsics)
{
//  const int num_cameras = static_cast<int>(labels.size());
//  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
//  if (!fs.isOpened())
//  {
//    PVT_LOG_FAILURE("Cannot open FileStorage '" << filename << "'");
//    return false;
//  }

//  fs << "num_cameras" << num_cameras;
//  for (int i = 0; i < num_cameras; ++i)
//  {
//    const std::string label = labels[i];
//    const py::tuple &rt = extrinsics[i].cast<py::tuple>();
//    const cv::Mat R = pvt::python::conversion::NDArrayToMat(rt[0].cast<py::array>());
//    const cv::Mat t = pvt::python::conversion::NDArrayToMat(rt[1].cast<py::array>());
//    std::stringstream ss;
//    ss << "label" << (i+1);
//    fs << ss.str() << label;

//    ss.str("");
//    ss.clear();
//    ss << "R" << (i+1);
//    fs << ss.str() << R;

//    ss.str("");
//    ss.clear();
//    ss << "t" << (i+1);
//    fs << ss.str() << t;
//  }

//  fs.release();
//  return true;
  //FIXME
  return false;
}

// FIXME ensureabsolutepaths, etc.

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
        cv::cvtColor(image, converted, image.channels() == 3 ? CV_RGB2BGR : CV_RGBA2BGRA);
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
            cv::cvtColor(image, converted, image.channels() == 3 ? CV_RGB2BGR : CV_RGBA2BGRA);
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
           "If you set rel_path_base_dir, all relative file paths will\n"
           "be prefixed accordingly.",
           py::arg("config"), py::arg("rel_path_base_dir")=std::string())
      .def("all_devices_available", &pybest::CaptureWrapper::AreAllDevicesAvailable,
           "Returns True if the capturing device/s is/are available.")
      .def("all_frames_available", &pybest::CaptureWrapper::AreAllFramesAvailable,
           "Returns true if frames (from all devices) can be retrieved (i.e. have been enqueued).")
      .def("num_available_frames", &pybest::CaptureWrapper::NumAvailableFrames,
           "Returns the number of available frames.\n\n"
           "Might be useful if one of your sinks stops working/streaming and you still want to continue\n"
           "processing, etc. In such a case, @see all_frames_available() would return false, whereas here\n"
           "you get the actual number of frames and can decide what to do for yourself.")
      .def("open", &pybest::CaptureWrapper::OpenDevices,
           "Initialize devices, returns bool.")
      .def("close", &pybest::CaptureWrapper::CloseDevices,
           "Close devices/tear down connections, returns bool.")
      .def("start", &pybest::CaptureWrapper::StartStreams,
           "Starts the image streams, returns bool.")
      .def("stop", &pybest::CaptureWrapper::StopStreams,
           "Stops the image streams, returns bool.")
      .def("wait_for_initial_frames", &pybest::CaptureWrapper::WaitForInitialFrames,
           "After opening & starting, you can wait for the first set of images to become available.\n"
           "Specify a timeout in milliseconds (float)), after which the result (bool) will indicate whether\n"
           "frames from all devices/sinks are available or not.",
           py::arg("timeout_ms")=5000.0);

//      .def("next_frame", &pybest::CaptureWrapper::NextFrame,
//           "Returns the currently available frames as a list of numpy ndarrays\n"
//           "(or None). Exactly one list element per stream in this capture.\n"
//           "Empty (None) frames may occur if you query faster than the stream's\n"
//           "frame rate.\n"
//           "Use the return_rgb flag to receive (color) streams as RGB (true) or\n"
//           "BGR (false).", py::arg("return_rgb")=true)
//      .def("previous_frame", &pybest::CaptureWrapper::PreviousFrame,
//           "Some sinks support retrieving the previous frame (i.e. video and\n"
//           "image directory). Otherwise operates in the same way as next_frame.",
//           py::arg("return_rgb")=true)
//      .def("fast_forward", &pybest::CaptureWrapper::FastForward,
//           "Some sinks support skipping frames (if num_frames > 1), i.e. video and\n"
//           "image directory. Otherwise operates in the same way as next_frame.",
//           py::arg("num_frames"), py::arg("return_rgb")=true)
//      .def("get_labels", &pybest::CaptureWrapper::GetLabels,
//           "Returns a list of all registered labels - the label\n"
//           "indices follow the same ordering as the frames returned \n"
//           "from next_frame().")
//      .def("get_label", &pybest::CaptureWrapper::GetLabel,
//           "Returns the label of the sink at the given index - the label\n"
//           "indices follow the same ordering as the frames returned\n"
//           "from next_frame().", py::arg("sink_index"))
//      .def("get_corresponding_config_keys", &pybest::CaptureWrapper::GetCorrespondingConfigKeys,
//           "Returns a list of corresponding config file keys - as\n"
//           "camera1, camera2 may not be the same ordering as cam_idx=0, cam_idx=1\n"
//           "in a multi-cam setup (when you mix different camera types)!")
//      .def("get_corresponding_config_key", &pybest::CaptureWrapper::GetCorrespondingConfigKey,
//           "Return the corresponding config file key (i.e. 'cameraX') for the stream at sink_index",
//           py::arg("sink_index"))
//      .def("get_stream_type", &pybest::CaptureWrapper::GetStreamType,
//           "Returns the type (mono, stereo, rgbd_color, rgbd_depth, unknown, ...) of the\n"
//           "stream at index i, i.e. frames[i] is of get_stream_type(i),\n"
//           "where frames = next_frame()",
//           py::arg("stream_index"))
//      .def("get_stream_types", &pybest::CaptureWrapper::GetStreamTypes,
//           "Returns the type (mono, stereo, rgbd_color, rgbd_depth, unknown, ...) for\n"
//           "each stream (so you know frames[i], with frames = next_frame(), will be of type[i]).")
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

  m.def("list_k4a_devices", &pybest::ListK4ADevices,
        "Returns a list of connected Kinect Azure devices.", py::arg("warn_if_no_devices")=true);

  m.def("list_mvbluefox3_devices", &pybest::ListMvBlueFox3Devices,
        "Returns a list of connected mvBlueFox3 devices.", py::arg("warn_if_no_devices")=true);

  m.def("list_realsense2_devices", &pybest::ListRealSense2Devices,
        "Returns a list of connected RealSense2 devices.", py::arg("warn_if_no_devices")=true);

  m.def("store_extrinsics", &pybest::StoreExtrinsics,
        "Saves the extrinsic calibration to the given file.", //TODO doc
        py::arg("filename"), py::arg("labels"), py::arg("extrinsics"));

}
