//-----------------------------------------------------------------------------
// PVT capture wrapper code

#include "conversion/np_cv_conversion.h"
#include "conversion/cv_core_conversion.h"

#include <exception>
#include <string>
#include <sstream>
#include <vector>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pvt_config/config_params.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/pvt_error.h>

#include "config.h"

#include <vcp_utils/timing_utils.h>
#include <vcp_icc/capture.h>
#include <vcp_best/liveview.h>

#ifdef VCP_BEST_WITH_IPCAMERA
  #include <pvt_icc/capture_ipcam.h>
#endif
#ifdef WITH_K4A
  #include <pvt_icc/k4a_sink.h>
#endif
#ifdef WITH_MATRIXVISION
  #include <pvt_icc/matrixvision_sink.h>
#endif
#ifdef WITH_REALSENSE2
  #include <pvt_icc/realsense2_sink.h>
#endif

namespace pvt
{
namespace python
{
namespace icc
{
class CaptureWrapper
{
public:
  CaptureWrapper() : capture_(nullptr) {}

  // Disable copying
  CaptureWrapper(const CaptureWrapper&) = delete;
  CaptureWrapper &operator=(const CaptureWrapper&) = delete;

  void LoadConfigCpp(const std::string &config_file)
  {
    const auto config = pvt::config::LoadConfigParamsCpp(config_file);
    capture_ = pvt::icc::CreateRawCapture(*config);
  }

  void LoadConfigJson(const std::string &json_str, bool is_json_string)
  {
    const auto config = pvt::config::LoadConfigParamsJson(json_str, is_json_string);
    capture_ = pvt::icc::CreateRawCapture(*config);
  }

  void LoadConfigParams(const pvt::python::config::ConfigWrapper &config)
  {
    const pvt::config::ConfigParams &params = config.AsConfigParams();
    capture_ = pvt::icc::CreateRawCapture(params);
  }

  virtual ~CaptureWrapper()
  {
    if (capture_)
      capture_.reset();
  }

  bool Start()
  {
    if (capture_)
      return capture_->Start();
    return false;
  }

  bool IsAvailable()
  {
    if (capture_)
      return capture_->IsAvailable();
    return false;
  }

  bool IsFrameAvailable()
  {
    if (capture_)
      return capture_->IsFrameAvailable();
    return false;
  }


  /** @brief Collect all frames from the capture and optionally flip the channels so you can work with RGB data (OpenCV uses BGR by default). */
  py::list NextFrame(bool return_rgb)
  {
    py::list numpy_frames;
    std::vector<cv::Mat> frames;
    if (capture_)
      frames = capture_->NextFrame();
    for (const auto &mat : frames)
    {
      if (mat.empty())
      {
        numpy_frames.append(py::none());
      }
      else
      {
        if (return_rgb && (mat.channels() == 3 || mat.channels() == 4))
        {
          cv::Mat converted;
          cv::cvtColor(mat, converted, mat.channels() == 3 ? CV_BGR2RGB : CV_BGRA2RGBA);
          numpy_frames.append(pvt::python::conversion::MatToNDArray(converted));
        }
        else
        {
          numpy_frames.append(pvt::python::conversion::MatToNDArray(mat));
        }
      }
    }
    return numpy_frames;
  }


  /** @brief Collect all frames from the capture and optionally flip the channels so you can work with RGB data (OpenCV uses BGR by default). */
  py::list PreviousFrame(bool return_rgb)
  {
    py::list numpy_frames;
    std::vector<cv::Mat> frames;
    if (capture_)
      frames = capture_->PreviousFrame();
    for (const auto &mat : frames)
    {
      if (mat.empty())
      {
        numpy_frames.append(py::none());
      }
      else
      {
        if (return_rgb && (mat.channels() == 3 || mat.channels() == 4))
        {
          cv::Mat converted;
          cv::cvtColor(mat, converted, mat.channels() == 3 ? CV_BGR2RGB : CV_BGRA2RGBA);
          numpy_frames.append(pvt::python::conversion::MatToNDArray(converted));
        }
        else
        {
          numpy_frames.append(pvt::python::conversion::MatToNDArray(mat));
        }
      }
    }
    return numpy_frames;
  }


  /** @brief Collect all frames from the capture and optionally flip the channels so you can work with RGB data (OpenCV uses BGR by default). */
  py::list FastForward(size_t num_frames, bool return_rgb)
  {
    py::list numpy_frames;
    std::vector<cv::Mat> frames;
    if (capture_)
      frames = capture_->FastForward(num_frames);
    for (const auto &mat : frames)
    {
      if (mat.empty())
      {
        numpy_frames.append(py::none());
      }
      else
      {
        if (return_rgb && (mat.channels() == 3 || mat.channels() == 4))
        {
          cv::Mat converted;
          cv::cvtColor(mat, converted, mat.channels() == 3 ? CV_BGR2RGB : CV_BGRA2RGBA);
          numpy_frames.append(pvt::python::conversion::MatToNDArray(converted));
        }
        else
        {
          numpy_frames.append(pvt::python::conversion::MatToNDArray(mat));
        }
      }
    }
    return numpy_frames;
  }


  bool Stop()
  {
    if (capture_)
      return capture_->Terminate();
    return false;
  }

  std::string GetLabel(size_t sink_index) const
  {
    return capture_->GetSinkLabel(sink_index);
  }

  std::vector<std::string> GetLabels() const
  {
    std::vector<std::string> labels;
    for (size_t i = 0; i < capture_->NumStreams(); ++i)
      labels.push_back(capture_->GetSinkLabel(i));
    return labels;
  }

  std::string GetCorrespondingConfigKey(size_t sink_index) const
  {
    return capture_->GetCorrespondingConfigKey(sink_index);
  }

  std::vector<std::string> GetCorrespondingConfigKeys() const
  {
    std::vector<std::string> keys;
    for (size_t i = 0; i < capture_->NumStreams(); ++i)
      keys.push_back(capture_->GetCorrespondingConfigKey(i));
    return keys;
  }


  std::string GetStreamType(size_t stream_index) const
  {
    std::string rep = pvt::icc::StreamTypeToString(capture_->GetStreamType(stream_index));
    pvt::utils::string::ToLower(rep);
    return rep;
  }

  std::vector<std::string> GetStreamTypes() const
  {
    std::vector<std::string> types;
    for (size_t i = 0; i < capture_->NumStreams(); ++i)
      types.push_back(GetStreamType(i));
    return types;
  }


  bool IsStreamType(size_t sink_index, const pvt::icc::StreamType &type) { return capture_->GetStreamType(sink_index) == type; }

  bool IsStereoStream(size_t sink_index) { return IsStreamType(sink_index, pvt::icc::StreamType::STEREO); }

  bool IsDepthStream(size_t sink_index) { return IsStreamType(sink_index, pvt::icc::StreamType::RGBD_DEPTH); }


  py::object ReturnNone(size_t /*sink_index*/) const
  {
    return py::none();
  }


  py::tuple ReturnNoneTuple(size_t /*sink_index*/) const
  {
    return py::make_tuple(py::none(), py::none());
  }

private:
  std::unique_ptr<pvt::icc::Capture> capture_;
};



class RectifiedCaptureWrapper
{
public:
  RectifiedCaptureWrapper() : capture_(nullptr) {}

  // Disable copying
  RectifiedCaptureWrapper(const RectifiedCaptureWrapper&) = delete;
  RectifiedCaptureWrapper &operator=(const RectifiedCaptureWrapper&) = delete;

  void LoadConfigCpp(const std::string &config_file, bool ensure_extrinsics_are_loaded)
  {
    const auto config = pvt::config::LoadConfigParamsCpp(config_file);
    capture_ = pvt::icc::CreateRectifiedCapture(*config, ensure_extrinsics_are_loaded);
  }

  void LoadConfigJson(const std::string &json_str, bool is_json_string, bool ensure_extrinsics_are_loaded)
  {
    const auto config = pvt::config::LoadConfigParamsJson(json_str, is_json_string);
    capture_ = pvt::icc::CreateRectifiedCapture(*config, ensure_extrinsics_are_loaded);
  }

  void LoadConfigParams(const pvt::python::config::ConfigWrapper &config, bool ensure_extrinsics_are_loaded)
  {
    const pvt::config::ConfigParams &params = config.AsConfigParams();
    capture_ = pvt::icc::CreateRectifiedCapture(params, ensure_extrinsics_are_loaded);
  }

  virtual ~RectifiedCaptureWrapper()
  {
    if (capture_)
      capture_.reset();
  }

  bool Start()
  {
    if (capture_)
      return capture_->Start();
    return false;
  }

  bool IsAvailable()
  {
    if (capture_)
      return capture_->IsAvailable();
    return false;
  }

  bool IsFrameAvailable()
  {
    if (capture_)
      return capture_->IsFrameAvailable();
    return false;
  }

  /** @brief Collect all frames from the capture and optionally flipping the channels so you can work with RGB data (OpenCV uses BGR by default). */
  py::list NextFrame(bool return_rgb)
  {
    py::list numpy_frames;
    std::vector<cv::Mat> frames;
    if (capture_)
      frames = capture_->NextFrame();
    for (const auto &mat : frames)
    {
      if (mat.empty())
      {
        numpy_frames.append(py::none());
      }
      else
      {
        if (return_rgb && (mat.channels() == 3 || mat.channels() == 4))
        {
          cv::Mat converted;
          cv::cvtColor(mat, converted, mat.channels() == 3 ? CV_BGR2RGB : CV_BGRA2RGBA);
          numpy_frames.append(pvt::python::conversion::MatToNDArray(converted));
        }
        else
        {
          numpy_frames.append(pvt::python::conversion::MatToNDArray(mat));
        }
      }
    }
    return numpy_frames;
  }


  /** @brief Collect all frames from the capture and optionally flipping the channels so you can work with RGB data (OpenCV uses BGR by default). */
  py::list PreviousFrame(bool return_rgb)
  {
    py::list numpy_frames;
    std::vector<cv::Mat> frames;
    if (capture_)
      frames = capture_->PreviousFrame();
    for (const auto &mat : frames)
    {
      if (mat.empty())
      {
        numpy_frames.append(py::none());
      }
      else
      {
        if (return_rgb && (mat.channels() == 3 || mat.channels() == 4))
        {
          cv::Mat converted;
          cv::cvtColor(mat, converted, mat.channels() == 3 ? CV_BGR2RGB : CV_BGRA2RGBA);
          numpy_frames.append(pvt::python::conversion::MatToNDArray(converted));
        }
        else
        {
          numpy_frames.append(pvt::python::conversion::MatToNDArray(mat));
        }
      }
    }
    return numpy_frames;
  }


  /** @brief Collect all frames from the capture and optionally flipping the channels so you can work with RGB data (OpenCV uses BGR by default). */
  py::list FastForward(size_t num_frames, bool return_rgb)
  {
    py::list numpy_frames;
    std::vector<cv::Mat> frames;
    if (capture_)
      frames = capture_->FastForward(num_frames);
    for (const auto &mat : frames)
    {
      if (mat.empty())
      {
        numpy_frames.append(py::none());
      }
      else
      {
        if (return_rgb && (mat.channels() == 3 || mat.channels() == 4))
        {
          cv::Mat converted;
          cv::cvtColor(mat, converted, mat.channels() == 3 ? CV_BGR2RGB : CV_BGRA2RGBA);
          numpy_frames.append(pvt::python::conversion::MatToNDArray(converted));
        }
        else
        {
          numpy_frames.append(pvt::python::conversion::MatToNDArray(mat));
        }
      }
    }
    return numpy_frames;
  }


  bool Stop()
  {
    if (capture_)
      return capture_->Terminate();
    return false;
  }

  std::string GetLabel(size_t sink_index) const
  {
    return capture_->GetSinkLabel(sink_index);
  }

  std::vector<std::string> GetLabels() const
  {
    std::vector<std::string> labels;
    for (size_t i = 0; i < capture_->NumStreams(); ++i)
      labels.push_back(capture_->GetSinkLabel(i));
    return labels;
  }

  std::string GetCorrespondingConfigKey(size_t sink_index) const
  {
    return capture_->GetCorrespondingConfigKey(sink_index);
  }

  std::vector<std::string> GetCorrespondingConfigKeys() const
  {
    std::vector<std::string> keys;
    for (size_t i = 0; i < capture_->NumStreams(); ++i)
      keys.push_back(capture_->GetCorrespondingConfigKey(i));
    return keys;
  }

  std::string GetStreamType(size_t stream_index) const
  {
    std::string rep = pvt::icc::StreamTypeToString(capture_->GetStreamType(stream_index));
    pvt::utils::string::ToLower(rep);
    return rep;
  }

  std::vector<std::string> GetStreamTypes() const
  {
    std::vector<std::string> types;
    for (size_t i = 0; i < capture_->NumStreams(); ++i)
      types.push_back(GetStreamType(i));
    return types;
  }

  py::object GetCameraMatrix(size_t sink_index)
  {
    const cv::Mat K = capture_->K(sink_index);
    if (K.empty())
      return py::none();
    py::array npK = pvt::python::conversion::MatToNDArray(K);
    npK.resize({3, 3}); //TODO needs to be checked!
    return npK;
  }

  // In a stereo set up, you might be interested in the right camera's matrix too (3x4, [K T] after rectification)
  py::object GetRightCameraMatrix(size_t sink_index)
  {
    const cv::Mat P2 = capture_->P2(sink_index);
    if (P2.empty())
      return py::none();
    py::array npp2 = pvt::python::conversion::MatToNDArray(P2);
    npp2.resize({3,4});
    return npp2;
  }

  py::object GetRotation(size_t sink_index)
  {
    const cv::Mat R = capture_->R(sink_index);
    if (R.empty())
      return py::none();
    py::array npR = pvt::python::conversion::MatToNDArray(R);
    npR.resize({3,3});
    return npR;
  }

  py::object GetTranslation(size_t sink_index)
  {
    const cv::Mat t = capture_->t(sink_index);
    if (t.empty())
      return py::none();
    py::array npt = pvt::python::conversion::MatToNDArray(t);
    npt.resize({3,1});
    return npt;
  }

  py::object GetCameraCenter(size_t sink_index)
  {
    const cv::Mat C = capture_->C(sink_index);
    if (C.empty())
      return py::none();
    py::array npc = pvt::python::conversion::MatToNDArray(C);
    npc.resize({3,1});
    return npc;
  }

  cv::Vec4d GetImagePlane(size_t sink_index)
  {
    return capture_->ImagePlane(sink_index);
  }

  py::tuple GetStereoRt(size_t sink_index)
  {
    py::object R = GetRotation(sink_index);
    py::object t = GetTranslation(sink_index);
    return py::make_tuple(R, t);
  }

  void SaveCalibration(const std::string &filename)
  {
    capture_->SaveCalibration(filename);
  }

  bool IsStreamType(size_t sink_index, const pvt::icc::StreamType &type) { return capture_->GetStreamType(sink_index) == type; }

  bool IsStereoStream(size_t sink_index) { return IsStreamType(sink_index, pvt::icc::StreamType::STEREO); }

  bool IsDepthStream(size_t sink_index) { return IsStreamType(sink_index, pvt::icc::StreamType::RGBD_DEPTH); }


private:
  std::unique_ptr<pvt::icc::RectifiedCapture> capture_;
};



py::list ListMvBlueFox3Devices(bool warn_if_no_devices)
{
  py::list list;
#ifdef WITH_MATRIXVISION
  const std::vector<pvt::icc::MvBlueFox3DeviceInfo> devices = pvt::icc::ListMatrixVisionBlueFox3Devices(warn_if_no_devices);

  for (const auto &dev : devices)
    list.append(py::make_tuple(dev.serial_number, dev.name, dev.in_use));
#else
  (void)warn_if_no_devices;
  throw std::runtime_error("You didn't compile the pypvt wrappers with matrix vision support (CMake should have added -DWITH_MATRIXVISION if PVT_ROOT_DIR/include/pvt_icc/matrixvision_sink.h exists?!)");
#endif
  return list;
}


py::list ListRealSense2Devices(bool warn_if_no_devices)
{
  py::list list;
#ifdef WITH_REALSENSE2
  const std::vector<pvt::icc::RealSense2DeviceInfo> devices = pvt::icc::ListRealSense2Devices(warn_if_no_devices);

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
#ifdef WITH_K4A
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
  const int num_cameras = static_cast<int>(labels.size());
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  if (!fs.isOpened())
  {
    PVT_LOG_FAILURE("Cannot open FileStorage '" << filename << "'");
    return false;
  }

  fs << "num_cameras" << num_cameras;
  for (int i = 0; i < num_cameras; ++i)
  {
    const std::string label = labels[i];
    const py::tuple &rt = extrinsics[i].cast<py::tuple>();
    const cv::Mat R = pvt::python::conversion::NDArrayToMat(rt[0].cast<py::array>());
    const cv::Mat t = pvt::python::conversion::NDArrayToMat(rt[1].cast<py::array>());
    std::stringstream ss;
    ss << "label" << (i+1);
    fs << ss.str() << label;

    ss.str("");
    ss.clear();
    ss << "R" << (i+1);
    fs << ss.str() << R;

    ss.str("");
    ss.clear();
    ss << "t" << (i+1);
    fs << ss.str() << t;
  }

  fs.release();
  return true;
}


class LiveViewWrapper
{
public:
  LiveViewWrapper(const std::string &window_title)
  {
    liveview_ = pvt::icc::CreateLiveView<3>(window_title);
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

  void PushViewRequest(const cv::Mat &image, bool is_rgb)
  {
    if (liveview_)
    {
      if (is_rgb && (image.channels() == 3 || image.channels() == 4))
      {
        cv::Mat converted;
        cv::cvtColor(image, converted, image.channels() == 3 ? CV_RGB2BGR : CV_RGBA2BGRA);
        liveview_->PushViewRequest(converted);
      }
      else
      {
        liveview_->PushViewRequest(image);
      }
    }
    else
    {
      throw std::runtime_error("Liveview is not available");
    }
  }

  void SetWaitMs(int wait_ms)
  {
    liveview_->SetWaitMs(wait_ms);
  }

  int GetUserInput() const
  {
    return liveview_->GetUserInput();
  }

  double GetRequestFrameRate() const
  {
    return liveview_->GetRequestFrameRate();
  }

  double GetDisplayFrameRate() const
  {
    return liveview_->GetDisplayFrameRate();
  }

private:
  std::unique_ptr<pvt::icc::LiveView> liveview_;
};
} // namespace icc
} // namespace python
} // namespace pvt


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Python module declarations
PYBIND11_MODULE(capture, m)
{
  namespace pyicc = pvt::python::icc;
  m.doc() = "PVT capture module - allows streaming from cameras (see pypvt3/examples/icc*).";

  py::class_<pyicc::CaptureWrapper>(m, "Capture")
      .def(py::init<>())
      .def("load_libconfig", &pyicc::CaptureWrapper::LoadConfigCpp,
           "Init the capture from a libconfig configuration file.", py::arg("config_file"))
      .def("load_json", &pyicc::CaptureWrapper::LoadConfigJson,
           "Init the capture from a JSON string (is_json_string=True) or file.",
           py::arg("json_str"), py::arg("is_json_string")=true)
      .def("load_pvt_config", &pyicc::CaptureWrapper::LoadConfigParams,
           "Init the capture from a pypvt3.config.Config() object.",
           py::arg("config"))
      .def("start", &pyicc::CaptureWrapper::Start,
           "Starts/opens the image stream.")
      .def("is_available", &pyicc::CaptureWrapper::IsAvailable,
           "Checks if the capture can be queried for frames.")
      .def("is_frame_available", &pyicc::CaptureWrapper::IsFrameAvailable,
           "Checks if frames (for each capture) are available (e.g. already enqueued).")
      .def("stop", &pyicc::CaptureWrapper::Stop,
           "Stops the stream and gracefully shuts down the capture.")
      .def("next_frame", &pyicc::CaptureWrapper::NextFrame,
           "Returns the currently available frames as a list of numpy ndarrays\n"
           "(or None). Exactly one list element per stream in this capture.\n"
           "Empty (None) frames may occur if you query faster than the stream's\n"
           "frame rate.\n"
           "Use the return_rgb flag to receive (color) streams as RGB (true) or\n"
           "BGR (false).", py::arg("return_rgb")=true)
      .def("previous_frame", &pyicc::CaptureWrapper::PreviousFrame,
           "Some sinks support retrieving the previous frame (i.e. video and\n"
           "image directory). Otherwise operates in the same way as next_frame.",
           py::arg("return_rgb")=true)
      .def("fast_forward", &pyicc::CaptureWrapper::FastForward,
           "Some sinks support skipping frames (if num_frames > 1), i.e. video and\n"
           "image directory. Otherwise operates in the same way as next_frame.",
           py::arg("num_frames"), py::arg("return_rgb")=true)
      .def("get_labels", &pyicc::CaptureWrapper::GetLabels,
           "Returns a list of all registered labels - the label\n"
           "indices follow the same ordering as the frames returned \n"
           "from next_frame().")
      .def("get_label", &pyicc::CaptureWrapper::GetLabel,
           "Returns the label of the sink at the given index - the label\n"
           "indices follow the same ordering as the frames returned\n"
           "from next_frame().", py::arg("sink_index"))
      .def("get_corresponding_config_keys", &pyicc::CaptureWrapper::GetCorrespondingConfigKeys,
           "Returns a list of corresponding config file keys - as\n"
           "camera1, camera2 may not be the same ordering as cam_idx=0, cam_idx=1\n"
           "in a multi-cam setup (when you mix different camera types)!")
      .def("get_corresponding_config_key", &pyicc::CaptureWrapper::GetCorrespondingConfigKey,
           "Return the corresponding config file key (i.e. 'cameraX') for the stream at sink_index",
           py::arg("sink_index"))
      .def("get_stream_type", &pyicc::CaptureWrapper::GetStreamType,
           "Returns the type (mono, stereo, rgbd_color, rgbd_depth, unknown, ...) of the\n"
           "stream at index i, i.e. frames[i] is of get_stream_type(i),\n"
           "where frames = next_frame()",
           py::arg("stream_index"))
      .def("get_stream_types", &pyicc::CaptureWrapper::GetStreamTypes,
           "Returns the type (mono, stereo, rgbd_color, rgbd_depth, unknown, ...) for\n"
           "each stream (so you know frames[i], with frames = next_frame(), will be of type[i]).")
      .def("get_camera_matrix", &pyicc::CaptureWrapper::ReturnNone,
           "Returns None - method is only useful for a RectifiedCapture object",
           py::arg("sink_index"))
      .def("get_camera_matrix_right", &pyicc::CaptureWrapper::ReturnNone,
           "Returns None - method is only useful for a RectifiedCapture object",
           py::arg("sink_index"))
      .def("get_stereo_Rt", &pyicc::CaptureWrapper::ReturnNoneTuple,
           "Returns (None,None) - method is only useful for a RectifiedCapture object",
           py::arg("sink_index"))
      .def("is_stereo_stream", &pyicc::CaptureWrapper::IsStereoStream,
           "Returns true if stream at sink_index is a stereo stream.", py::arg("sink_index"))
      .def("is_depth_stream", &pyicc::CaptureWrapper::IsDepthStream,
           "Returns true if stream at sink_index is a depth stream.", py::arg("sink_index"));


  py::class_<pyicc::RectifiedCaptureWrapper>(m, "RectifiedCapture")
      .def(py::init<>())
      .def("load_libconfig", &pyicc::RectifiedCaptureWrapper::LoadConfigCpp,
           "Init the capture from a libconfig configuration file.",
           py::arg("config_file"), py::arg("ensure_extrinsics_are_loaded")=false)
      .def("load_json", &pyicc::RectifiedCaptureWrapper::LoadConfigJson,
           "Init the capture from a JSON string (is_json_string=True) or file.",
           py::arg("json_str"), py::arg("is_json_string")=true, py::arg("ensure_extrinsics_are_loaded")=false)
      .def("load_pvt_config", &pyicc::RectifiedCaptureWrapper::LoadConfigParams,
           "Init the capture from a pypvt.config.Config() object.",
           py::arg("config"), py::arg("ensure_extrinsics_are_loaded")=false)
      .def("start", &pyicc::RectifiedCaptureWrapper::Start,
           "Starts/opens the image stream.")
      .def("is_available", &pyicc::RectifiedCaptureWrapper::IsAvailable,
           "Checks if the capture can be queried for frames.")
      .def("is_frame_available", &pyicc::RectifiedCaptureWrapper::IsFrameAvailable,
           "Checks if frames (for each capture) are available (e.g. already enqueued).")
      .def("stop", &pyicc::RectifiedCaptureWrapper::Stop,
           "Stops the stream and gracefully shuts down the capture.")
      .def("next_frame", &pyicc::RectifiedCaptureWrapper::NextFrame,
           "Returns the currently available frames as a list of numpy ndarrays\n"
           "(or None). Exactly one list element per stream in this capture.\n"
           "Empty (None) frames may occur if you query faster than the stream's\n"
           "frame rate.\n"
           "Use the return_rgb flag to receive (color) streams as RGB (true) or\n"
           "BGR (false).", py::arg("return_rgb")=true)
      .def("previous_frame", &pyicc::RectifiedCaptureWrapper::PreviousFrame,
           "Some sinks support retrieving the previous frame (i.e. video and\n"
           "image directory). Otherwise operates in the same way as next_frame.",
           py::arg("return_rgb")=true)
      .def("fast_forward", &pyicc::RectifiedCaptureWrapper::FastForward,
           "Some sinks support skipping frames (if num_frames > 1), i.e. video and\n"
           "image directory. Otherwise operates in the same way as next_frame.",
           py::arg("num_frames"), py::arg("return_rgb")=true)
      .def("get_labels", &pyicc::RectifiedCaptureWrapper::GetLabels,
           "Returns a list of all registered labels - the label\n"
           "indices follow the same ordering as the frames returned\n"
           "from next_frame().")
      .def("get_label", &pyicc::RectifiedCaptureWrapper::GetLabel,
           "Returns the label of the sink at the given index - the label\n"
           "indices follow the same ordering as the frames returned\n"
           "from next_frame().", py::arg("sink_index"))
      .def("get_corresponding_config_keys", &pyicc::RectifiedCaptureWrapper::GetCorrespondingConfigKeys,
           "Returns a list of corresponding config file keys - as\n"
           "camera1, camera2 may not be the same ordering as cam_idx=0, cam_idx=1\n"
           "in a multi-cam setup (when you mix different camera types)!")
      .def("get_corresponding_config_key", &pyicc::RectifiedCaptureWrapper::GetCorrespondingConfigKey,
           "Return the corresponding config file key (i.e. 'cameraX') for the stream at sink_index",
           py::arg("sink_index"))
      .def("get_stream_type", &pyicc::RectifiedCaptureWrapper::GetStreamType,
           "Returns the type (mono, stereo, rgbd_color, rgbd_depth, unknown, ...)\n"
           "of the stream at index i, i.e. frames[i] is of get_stream_type(i),\n"
           "where frames = next_frame()", py::arg("stream_index"))
      .def("get_stream_types", &pyicc::RectifiedCaptureWrapper::GetStreamTypes,
           "Returns the type (mono, stereo, rgbd_color, rgbd_depth, unknown, ...)\n"
           "for each stream (so you know frames[i], with frames = next_frame(), will be of type[i]).")
      .def("get_camera_matrix", &pyicc::RectifiedCaptureWrapper::GetCameraMatrix,
           "Returns the 3x3 camera matrix (holding the intrinsics).\n"
           "In a stereo setup, this will be the left (reference) camera's\n"
           "matrix, see also get_camera_matrix_right()", py::arg("sink_index"))
      .def("get_camera_matrix_right", &pyicc::RectifiedCaptureWrapper::GetRightCameraMatrix,
           "In a stereo setup, this returns the 3x4 camera matrix holding the\n"
           "intrinsics and translation of the right lens. See also get_camera_matrix()",
           py::arg("sink_index"))
      .def("get_stereo_Rt", &pyicc::RectifiedCaptureWrapper::GetStereoRt,
           "Returns the tuple(R,t), see get_rotation() get_translation().",
           py::arg("sink_index"))
      .def("get_rotation", &pyicc::RectifiedCaptureWrapper::GetRotation,
           "Returns the 3x3 rotation matrix (or None).", py::arg("sink_index"))
      .def("get_translation", &pyicc::RectifiedCaptureWrapper::GetTranslation,
           "Returns the 3x1 translation vector (or None).", py::arg("sink_index"))
      .def("get_optical_center", &pyicc::RectifiedCaptureWrapper::GetCameraCenter,
           "Returns the 3x1 optical center C in world coordinates (or None).",
           py::arg("sink_index"))
      .def("get_image_plane", &pyicc::RectifiedCaptureWrapper::GetImagePlane,
           "Returns the image plane in Hessian normal form (4-element vector).",
           py::arg("sink_index"))
      .def("save_calibration", &pyicc::RectifiedCaptureWrapper::SaveCalibration,
           "Saves calibration to disk using OpenCV's FileStorage.",py::arg("filename"))
      .def("is_stereo_stream", &pyicc::RectifiedCaptureWrapper::IsStereoStream,
           "Returns true if stream at sink_index is a stereo stream.", py::arg("sink_index"))
      .def("is_depth_stream", &pyicc::RectifiedCaptureWrapper::IsDepthStream,
           "Returns true if stream at sink_index is a depth stream.", py::arg("sink_index"));

  py::class_<pyicc::LiveViewWrapper>(m, "LiveView")
      .def(py::init<std::string>(), "Provide the window name")
      .def("push_view_request", &pyicc::LiveViewWrapper::PushViewRequest,
           "Push an incoming image into the liveview queue.",
           py::arg("image"), py::arg("is_rgb")=false)
      .def("set_wait_ms", &pyicc::LiveViewWrapper::SetWaitMs,
           "Set wait time parameter passed to cv::waitKey().", py::arg("wait_ms"))
      .def("get_user_input", &pyicc::LiveViewWrapper::GetUserInput,
           "Returns the result of the last cv::waitKey().")
      .def("get_request_fps", &pyicc::LiveViewWrapper::GetRequestFrameRate,
           "Returns the frame rate of the incoming view requests, NOT the display frame rate!")
      .def("get_display_fps", &pyicc::LiveViewWrapper::GetDisplayFrameRate,
           "Returns the current display frame rate.");

  m.def("list_k4a_devices", &pyicc::ListK4ADevices,
        "Returns a list of connected Kinect Azure devices.", py::arg("warn_if_no_devices")=true);

  m.def("list_mvbluefox3_devices", &pyicc::ListMvBlueFox3Devices,
        "Returns a list of connected mvBlueFox3 devices.", py::arg("warn_if_no_devices")=true);

  m.def("list_realsense2_devices", &pyicc::ListRealSense2Devices,
        "Returns a list of connected RealSense2 devices.", py::arg("warn_if_no_devices")=true);

  m.def("store_extrinsics", &pyicc::StoreExtrinsics,
        "Saves the extrinsic calibration to the given file.", //TODO doc
        py::arg("filename"), py::arg("labels"), py::arg("extrinsics"));

}
