#include "calibration.h"
#include "sink.h"
#include <vcp_utils/vcp_error.h>

#include <vcp_utils/string_utils.h>
#include <vcp_utils/file_utils.h>
#include <vcp_math/common.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <vector>
#include <algorithm>

namespace vcp
{
namespace best
{
namespace calibration
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::calibration"

/** @brief Returns all node names (which are children of root) within the given cv::FileStorage. */
std::vector<std::string> ListFileStorageNodes(const cv::FileStorage &fs)
{
  std::vector<std::string> keys;
  cv::FileNode fn = fs.root();
  for (cv::FileNodeIterator it = fn.begin(); it != fn.end(); ++it)
  {
    const cv::FileNode item = *it;
    keys.push_back(item.name());
  }
  return keys;
}


StreamIntrinsics::StreamIntrinsics() : skip_undistort_rectify_(false)
{}

StreamIntrinsics::StreamIntrinsics(const StreamIntrinsics &other)
{
  label_ = other.label_;
  resolution_ = other.resolution_;
  identifier_ = other.identifier_;
  skip_undistort_rectify_ = other.skip_undistort_rectify_;
  intrinsics_rectified_ = other.intrinsics_rectified_.clone();
  intrinsics_original_ = other.intrinsics_original_.clone();
  distortion_ = other.distortion_.clone();
  R_to_ref_ = other.R_to_ref_.clone();
  t_to_ref_ = other.t_to_ref_.clone();
  undistort_rectify_map1_ = other.undistort_rectify_map1_.clone();
  undistort_rectify_map2_ = other.undistort_rectify_map2_.clone();
}

StreamIntrinsics StreamIntrinsics::FromMonocular(const cv::Mat &intrinsics,
                                          const cv::Mat &distortion,
                                          const std::string &label,
                                          const cv::Size &resolution,
                                          const cv::Mat &R, const cv::Mat &t)
{
  StreamIntrinsics si;
  si.intrinsics_original_ = intrinsics.clone();
  si.distortion_ = distortion.clone();
  si.skip_undistort_rectify_ = !HasLensDistortion(distortion);
  si.label_ = label;
  si.resolution_ = resolution;

  if (si.skip_undistort_rectify_)
  {
    si.intrinsics_rectified_ = si.intrinsics_original_.clone();
  }
  else
  {
    si.intrinsics_rectified_ = cv::getOptimalNewCameraMatrix(intrinsics, distortion, resolution, 1.0, resolution, 0);
    cv::initUndistortRectifyMap(intrinsics, distortion, cv::Mat(), si.intrinsics_rectified_, resolution, CV_16SC2, si.undistort_rectify_map1_, si.undistort_rectify_map2_);
  }

  if (!R.empty())
    si.R_to_ref_ = R.clone();
  if (!t.empty())
    si.t_to_ref_ = t.clone();

  return si;
}

StreamIntrinsics::~StreamIntrinsics()
{}

bool StreamIntrinsics::Empty() const
{
  return intrinsics_original_.empty();
}

cv::Mat StreamIntrinsics::UndistortRectify(const cv::Mat &image) const
{
  if (image.empty() || skip_undistort_rectify_)
    return image;
  cv::Mat rectified;
  cv::remap(image, rectified, undistort_rectify_map1_, undistort_rectify_map2_, cv::INTER_LINEAR);
  return rectified;
}

cv::Mat StreamIntrinsics::IntrinsicsOriginal() const
{
  return intrinsics_original_;
}

cv::Mat StreamIntrinsics::IntrinsicsRectified() const
{
  return intrinsics_rectified_;
}

bool StreamIntrinsics::HasDistortion() const
{
  return HasLensDistortion(distortion_);
}

bool StreamIntrinsics::HasResolution() const
{
  return resolution_.width > 0 && resolution_.height > 0;
}

cv::Mat StreamIntrinsics::Distortion() const
{
  return distortion_;
}

bool StreamIntrinsics::HasTransformationToReference() const
{
  return !(R_to_ref_.empty() || t_to_ref_.empty());
}

void StreamIntrinsics::TransformationToReference(cv::Mat &R, cv::Mat &t) const
{
  R = R_to_ref_.clone();
  t = t_to_ref_.clone();
}

std::string StreamIntrinsics::StreamLabel() const
{
  return label_;
}

cv::Size StreamIntrinsics::Resolution() const
{
  return resolution_;
}

void StreamIntrinsics::SetResolution(int width, int height)
{
  resolution_ = cv::Size(width, height);
}

std::string StreamIntrinsics::Identifier() const
{
  return identifier_;
}

void StreamIntrinsics::SetIdentifier(const std::string &id)
{
  identifier_ = id;
}

bool StreamIntrinsics::Save(const std::string &calibration_file, const bool rectified, const FrameType &frame_type) const
{
  if (Empty())
  {
    VCP_LOG_WARNING("Cannot save empty/invalid intrinsics for stream '" << identifier_ << "'.");
    return false;
  }

  if (calibration_file.empty())
  {
    VCP_LOG_FAILURE("StreamIntrinsics::Save() called with empty file name for stream '" << identifier_ << "'.");
    return false;
  }

  cv::FileStorage fs(calibration_file, cv::FileStorage::WRITE);
  if (!fs.isOpened())
  {
    VCP_LOG_FAILURE("Cannot open '" << calibration_file << "' to store calibration for stream '" << identifier_ << "'.");
    return false;
  }

  if (!identifier_.empty())
    fs << "label" << identifier_;

  if (rectified)
  {
    fs << "M" << intrinsics_rectified_;
    fs << "M_distorted" << intrinsics_original_;
    fs << "D_distorted" << distortion_;
  }
  else
  {
    fs << "M" << intrinsics_original_;
    fs << "D" << distortion_;
  }
  fs << "M_is_rectified" << rectified;

  if (resolution_.width > 0)
    fs << "width" << resolution_.width;
  if (resolution_.height > 0)
    fs << "height" << resolution_.height;

  if (HasTransformationToReference())
  {
    fs << "R" << R_to_ref_;
    fs << "t" << t_to_ref_;
  }

  switch (frame_type)
  {
    case FrameType::DEPTH:
    case FrameType::INFRARED:
    case FrameType::MONOCULAR:
      fs << "type" << "monocular";
      break;
    case FrameType::STEREO:
      fs << "type" << "stereo";
      break;
    case FrameType::UNKNOWN:
      VCP_LOG_WARNING("FrameType is 'unknown' for stream '" << identifier_ << "'. Assuming it to be monocular.");
      break;
    default:
      VCP_ERROR("FrameType '" << static_cast<short>(frame_type) << "' is not yet supported.");
  }

  fs.release();
  return true;
}

std::ostream &operator<<(std::ostream &out, const StreamIntrinsics &si)
{
  const cv::Mat K = si.IntrinsicsOriginal();
  const double fx = K.at<double>(0, 0);
  const double fy = K.at<double>(1, 1);
  const double cx = K.at<double>(0, 2);
  const double cy = K.at<double>(1, 2);

  out << "Intrinsics for stream '" << si.StreamLabel() << "', fl=("
      << fx << ", " << fy << "), pp=(" << cx << ", " << cy << ")";
  if (si.HasDistortion())
    out << ", dc=" << si.Distortion();
  if (si.HasResolution())
    out << ", res=" << si.Resolution();
  return out;
}


/** @brief Helper to read a FileStorage node (if it exists). */
bool ReadMat(const cv::FileStorage &fs, const std::string &key, const std::vector<std::string> &keys, cv::Mat &mat)
{
  if (std::find(keys.begin(), keys.end(), key) != keys.end())
  {
    fs[key] >> mat;
    return true;
  }
  else
  {
    mat = cv::Mat();
    return false;
  }
}

/** @brief Helper to read a FileStorage node (if it exists). */
template <typename _T>
bool ReadScalar(const cv::FileStorage &fs, const std::string &key, const std::vector<std::string> &keys, _T &val, const _T &def)
{
  if (std::find(keys.begin(), keys.end(), key) != keys.end())
  {
    fs[key] >> val;
    return true;
  }
  else
  {
    val = def;
    return false;
  }
}


StreamIntrinsics LoadGenericMonocularCalibration(const cv::FileStorage &fs,
                                                 const std::string &filename,
                                                 const std::string &postfix,
                                                 const std::vector<std::string> &calib_keys,
                                                 const std::string &postfix_ref_transform=std::string())
{
  StreamIntrinsics intrinsics;

  cv::Mat K, D, R, t;
  int width, height;
  std::string lbl;

  // Read intrinsics, either as "K" or "M".
  const std::string kk = "K" + postfix;
  const std::string km = "M" + postfix;
  if (std::find(calib_keys.begin(), calib_keys.end(), kk) != calib_keys.end())
    ReadMat(fs, kk, calib_keys, K);
  else if (std::find(calib_keys.begin(), calib_keys.end(), km) != calib_keys.end())
    ReadMat(fs, km, calib_keys, K);
  else
  {
    VCP_LOG_WARNING("Calibration file '" << filename << "' doesn't contain intrinsics (neither K" << postfix << " nor M" << postfix << ").");
    return intrinsics;
  }

  // Read distortion coefficients, either as "D" or "distortion".
  if (!ReadMat(fs, "D" + postfix, calib_keys, D))
    ReadMat(fs, "distortion" + postfix, calib_keys, D);

  ReadScalar(fs, "width" + postfix, calib_keys, width, -1);
  ReadScalar(fs, "height" + postfix, calib_keys, height, -1);

  const std::string def_lbl = postfix.empty() ? "unknown" : (postfix[0] == '_' ? postfix.substr(1) : postfix);
  ReadScalar(fs, "label" + postfix, calib_keys, lbl, def_lbl);

  if (!postfix_ref_transform.empty())
  {
    ReadMat(fs, "R" + postfix_ref_transform, calib_keys, R);
    ReadMat(fs, "t" + postfix_ref_transform, calib_keys, t);
  }
  else
  {
    R = cv::Mat();
    t = cv::Mat();
  }

  return StreamIntrinsics::FromMonocular(K, D, lbl, cv::Size(width, height), R, t);
}



std::vector<StreamIntrinsics> LoadGenericRGBDCalibration(const cv::FileStorage &fs,
                                                         const std::string &filename,
                                                         const std::vector<std::string> &calib_keys)
{
  std::vector<StreamIntrinsics> intrinsics;

  const StreamIntrinsics rgb = LoadGenericMonocularCalibration(fs, filename, "_color", calib_keys);
  if (!rgb.Empty())
    intrinsics.push_back(rgb);

  const StreamIntrinsics depth = LoadGenericMonocularCalibration(fs, filename, "_depth", calib_keys, "_depth2color");
  if (!depth.Empty())
    intrinsics.push_back(depth);

  return intrinsics;
}

std::vector<calibration::StreamIntrinsics> LoadRealSenseCalibration(const cv::FileStorage &fs,
                                                                    const std::string &filename,
                                                                    const std::vector<std::string> &calib_keys)
{
  // Load standard RGB and depth stream calibration, as with every RGBD sensor.
  std::vector<calibration::StreamIntrinsics> intrinsics = LoadGenericRGBDCalibration(fs, filename, calib_keys);

  const StreamIntrinsics ir_left = LoadGenericMonocularCalibration(fs, filename, "_ir_left", calib_keys, "_ir_left2color");
  if (!ir_left.Empty())
    intrinsics.push_back(ir_left);

  const StreamIntrinsics ir_right = LoadGenericMonocularCalibration(fs, filename, "_ir_right", calib_keys, "_ir_right2color");
  if (!ir_right.Empty())
    intrinsics.push_back(ir_right);

  if (std::find(calib_keys.begin(), calib_keys.end(), "serial_number") != calib_keys.end())
  {
    std::string sn;
    fs["serial_number"] >> sn;
    for (auto &calib : intrinsics)
      calib.SetIdentifier(sn);
  }

  return intrinsics;
}


std::vector<calibration::StreamIntrinsics> LoadK4ACalibration(const cv::FileStorage &fs,
                                                              const std::string &filename,
                                                              const std::vector<std::string> &calib_keys)
{
  // Load standard RGB and depth stream calibration, as with every RGBD sensor.
  std::vector<calibration::StreamIntrinsics> intrinsics = LoadGenericRGBDCalibration(fs, filename, calib_keys);

  const StreamIntrinsics ir = LoadGenericMonocularCalibration(fs, filename, "_ir", calib_keys, "_ir2color");
  if (!ir.Empty())
    intrinsics.push_back(ir);

  if (std::find(calib_keys.begin(), calib_keys.end(), "serial_number") != calib_keys.end())
  {
    std::string sn;
    fs["serial_number"] >> sn;
    for (auto &calib : intrinsics)
      calib.SetIdentifier(sn);
  }

  return intrinsics;
}

std::vector<StreamIntrinsics> LoadZedCalibration(const cv::FileStorage &fs,
                                                 const std::string &filename,
                                                 const std::vector<std::string> &calib_keys)
{
  std::vector<StreamIntrinsics> intrinsics;

  const StreamIntrinsics left = LoadGenericMonocularCalibration(fs, filename, "_left", calib_keys);
  if (!left.Empty())
    intrinsics.push_back(left);

  const StreamIntrinsics right = LoadGenericMonocularCalibration(fs, filename, "_right", calib_keys, "_right2left");
  if (!right.Empty())
    intrinsics.push_back(right);

  // R,t is only stored for completeness (left is the reference view for depth computation)
  const StreamIntrinsics depth = LoadGenericMonocularCalibration(fs, filename, "_depth", calib_keys, "_right2left");
  if (!depth.Empty())
    intrinsics.push_back(depth);

  if (std::find(calib_keys.begin(), calib_keys.end(), "serial_number") != calib_keys.end())
  {
    int isn;
    fs["serial_number"] >> isn;
    const std::string sn = vcp::utils::string::ToStr(isn);
    for (auto &calib : intrinsics)
      calib.SetIdentifier(sn);
  }

  return intrinsics;
}


std::vector<StreamIntrinsics> LoadIntrinsicsBySinkType(const cv::FileStorage &calibration_fs,
                                                       const std::string &filename,
                                                       const std::string &sink_type,
                                                       const std::vector<std::string> &calib_keys)
{
  std::vector<StreamIntrinsics> intrinsics;
  const std::string t = vcp::utils::string::Canonic(sink_type, true);
  if (t.empty())
  {
    VCP_LOG_FAILURE("Invalid (empty) 'sink_type' in calibration file '" << filename << "'");
  }
  else if (t.compare("realsense") == 0)
    intrinsics = LoadRealSenseCalibration(calibration_fs, filename, calib_keys);
  else if (t.compare("k4a") == 0)
    intrinsics = LoadK4ACalibration(calibration_fs, filename, calib_keys);
  else if (t.compare("zed") == 0)
    intrinsics = LoadZedCalibration(calibration_fs, filename, calib_keys);
  else
  {
    VCP_LOG_FAILURE("Unsupported calibration sink_type '" << sink_type << "'");
  }
  return intrinsics;
}


std::vector<StreamIntrinsics> LoadIntrinsicsByGenericType(const cv::FileStorage &calibration_fs,
                                                          const std::string &filename,
                                                          const std::string &type,
                                                          const std::vector<std::string> &calib_keys)
{
  std::vector<StreamIntrinsics> intrinsics;
  const std::string t = vcp::utils::string::Canonic(type, true);

  if (t.compare("mono") == 0
      || t.compare("monocular") == 0)
    intrinsics.push_back(LoadGenericMonocularCalibration(calibration_fs, filename, std::string(), calib_keys, std::string()));
//  else if (t.compare("stereo") == 0)
//    return false; //TODO
  else if (t.compare("rgbd") == 0)
    intrinsics = LoadGenericRGBDCalibration(calibration_fs, filename, calib_keys);
  else
    VCP_LOG_FAILURE("Unsupported intrinsic calibration type '" << type << "'");

  VCP_LOG_FIXME("Loading generic calibration: " << intrinsics);

  return intrinsics;
}

std::vector<StreamIntrinsics> LoadIntrinsicsFromFile(const std::string &calibration_file,
                                                     const std::vector<std::string> &label_order)
{
  std::vector<StreamIntrinsics> intrinsics;
  if (!vcp::utils::file::Exists(calibration_file))
  {
    VCP_LOG_FAILURE("Calibration file '" << calibration_file << "' does not exist.");
    return intrinsics;
  }

  cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    VCP_LOG_FAILURE("Cannot open calibration file '" << calibration_file << "'");
    return intrinsics;
  }

  const auto keys = ListFileStorageNodes(fs);
//  VCP_LOG_FIXME("Available calibration nodes in '" << calibration_file << "':"<< std::endl << keys);

  const auto sink_type = std::find(keys.begin(), keys.end(), "sink_type");
  if (sink_type != keys.end())
  {
    std::string sink_type_val;
    fs["sink_type"] >> sink_type_val;
    intrinsics = LoadIntrinsicsBySinkType(fs, calibration_file, sink_type_val, keys);
  }
  else
  {
    // If there's no specialized loader (defined by the "sink_type" node),
    // there must be a "type" node specifying whether we deal with monocular
    // stereo, rgbd, ... streams.
    const auto type = std::find(keys.begin(), keys.end(), "type");
    if (type == keys.end())
    {
      VCP_LOG_FAILURE("Missing 'type' node in calibration file.");
      fs.release();
      return intrinsics;
    }
    std::string type_val;
    fs["type"] >> type_val;
    intrinsics = LoadIntrinsicsByGenericType(fs, calibration_file, type_val, keys);
  }
  fs.release();

  if (label_order.empty())
    return intrinsics;

  VCP_LOG_FIXME("Sort intrinsics: " << std::endl << intrinsics);
  return intrinsics;
}



bool HasLensDistortion(const cv::Mat &distortion)
{
  if (distortion.empty())
    return false;

  if (distortion.cols != 1 || distortion.type() != CV_64FC1)
    VCP_ERROR("Distortion coefficients must be a Nx1 CV_64FC1 vector.");

  for (int r = 0; r < distortion.rows; ++r)
  {
    if (!vcp::math::eps_zero(distortion.at<double>(r)))
      return true;
  }

  // All distortion parameters are (too close to) 0
  return false;
}


//class NoneRectifier : public Rectifier
//{
//public:
//  NoneRectifier() {}

//  virtual ~NoneRectifier() {}

//  cv::Mat Rectify(const cv::Mat &image) const override { return image; }
//  cv::Mat K() const override { return cv::Mat(); }
//};


//class MonoRectifier : public Rectifier
//{
//public:
//  MonoRectifier(const std::string &filename)
//  {
//    cv::FileStorage fs(filename, cv::FileStorage::READ);
//    Init(fs);
//    fs.release();
//  }

//  MonoRectifier(cv::FileStorage &fs) { Init(fs); }

//  MonoRectifier(cv::FileStorage &fs, int cam_number) { Init(fs, cam_number); }

//  virtual ~MonoRectifier() {}

//  cv::Mat Rectify(const cv::Mat &image) const override
//  {
//    if (image.empty() || skip_rectification_)
//      return image;
//    cv::Mat rectified;
//    cv::remap(image, rectified, map1_, map2_, cv::INTER_LINEAR);
//    return rectified;
//  }

//  cv::Mat K() const override { return K_; }

//private:
//  cv::Mat K_;
//  cv::Mat map1_;
//  cv::Mat map2_;
//  bool skip_rectification_; // If there is no distortion, skip the remapping step.

//  void Init(cv::FileStorage &fs)
//  {
//    // Load intrinsics and distortion coefficients.
//    cv::Mat intrinsics, distortion;
//    int img_width, img_height;
//    fs["K"] >> intrinsics;
//    fs["D"] >> distortion;
//    fs["img_width"] >> img_width;
//    fs["img_height"] >> img_height;

//    // Note: we don't need to adjust the principal point offset (the MATLAB toolbox already provides 0-based principal points).

//    const bool has_distortion = HasLensDistortion(distortion);

//    if (has_distortion)
//    {
//      // Precompute undistortion and rectification transformations for speed up.
//      const cv::Size img_size(img_width, img_height);
//      // Store "rectified" camera matrix, see https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#getoptimalnewcameramatrix
//      K_ = cv::getOptimalNewCameraMatrix(intrinsics, distortion, img_size, 1.0, img_size, 0);
//      cv::initUndistortRectifyMap(intrinsics, distortion, cv::Mat(), K_, img_size, CV_16SC2, map1_, map2_);
//    }
//    else
//    {
//      K_ = intrinsics.clone();
//    }
//    skip_rectification_ = !has_distortion;
//  }

//  void Init(cv::FileStorage &fs, int cam_number)
//  {
//    // Load intrinsics and distortion coefficients.
//    cv::Mat intrinsics, distortion;
//    int img_width, img_height;
//    std::stringstream ss;
//    ss << "K" << cam_number;
//    fs[ss.str()] >> intrinsics;

//    ss.str("");
//    ss.clear();
//    ss << "D" << cam_number;
//    fs[ss.str()] >> distortion;

//    ss.str("");
//    ss.clear();
//    ss << "img_width" << cam_number;
//    fs[ss.str()] >> img_width;

//    ss.str("");
//    ss.clear();
//    ss << "img_height" << cam_number;
//    fs[ss.str()] >> img_height;

//    const bool has_distortion = HasLensDistortion(distortion);
//    if (has_distortion)
//    {
//      // Precompute undistortion and rectification transformations for speed up.
//      const cv::Size img_size(img_width, img_height);
//      // Store "rectified" camera matrix, see https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#getoptimalnewcameramatrix
//      K_ = cv::getOptimalNewCameraMatrix(intrinsics, distortion, img_size, 1.0, img_size, 0);
//      cv::initUndistortRectifyMap(intrinsics, distortion, cv::Mat(), K_, img_size, CV_16SC2, map1_, map2_);
//    }
//    else
//    {
//      K_ = intrinsics.clone();
//    }
//    skip_rectification_ = !has_distortion;
//  }
//};


//class RgbdMonoRectifier : public Rectifier
//{
//public:
//  RgbdMonoRectifier(const std::string &filename, const StreamType stream_type)
//  {
//    cv::FileStorage fs(filename, cv::FileStorage::READ);
//    Init(fs, stream_type);
//    fs.release();
//  }

//  RgbdMonoRectifier(cv::FileStorage &fs, const StreamType stream_type) { Init(fs, stream_type); }

//  RgbdMonoRectifier(cv::FileStorage &fs, int cam_number) { Init(fs, cam_number); }

//  virtual ~RgbdMonoRectifier() {}

//  cv::Mat Rectify(const cv::Mat &image) const override
//  {
//    if (image.empty() || skip_rectification_)
//      return image;
//    cv::Mat rectified;
//    cv::remap(image, rectified, map1_, map2_, cv::INTER_LINEAR);
//    return rectified;
//  }

//  cv::Mat K() const override { return K_; }

//  void StereoExtrinsics(cv::Mat &R, cv::Mat &t) const override
//  {
//    R = R_rgbd_;
//    t = t_rgbd_;
//  }

//private:
//  cv::Mat K_;
//  cv::Mat map1_;
//  cv::Mat map2_;
//  cv::Mat R_rgbd_;
//  cv::Mat t_rgbd_;
//  bool skip_rectification_;

//  void Init(cv::FileStorage &fs, const StreamType stream_type)
//  {
//    // Load intrinsics and distortion coefficients.
//    cv::Mat intrinsics, distortion;
//    int img_width, img_height;

//    if (stream_type == StreamType::RGBD_COLOR)
//    {
//      fs["K_rgb"] >> intrinsics;
//      fs["D_rgb"] >> distortion;
//      fs["width_rgb"] >> img_width;
//      fs["height_rgb"] >> img_height;
//      R_rgbd_ = cv::Mat::eye(3,3,CV_64FC1); // We consider the color stream as the reference stream.
//      t_rgbd_ = cv::Mat::zeros(3,1,CV_64FC1);
//    }
//    else if (stream_type == StreamType::RGBD_DEPTH)
//    {
//      fs["K_depth"] >> intrinsics;
//      fs["D_depth"] >> distortion;
//      fs["width_depth"] >> img_width;
//      fs["height_depth"] >> img_height;
//      fs["R_depth2rgb"] >> R_rgbd_;
//      fs["t_depth2rgb"] >> t_rgbd_;
//    }
//    else
//    {
//      VCP_ERROR("Invalid stream type parameter: " << stream_type);
//    }

//    // Note: we don't need to adjust the principal point offset (the MATLAB toolbox already provides 0-based principal points).

//    const bool has_distortion = HasLensDistortion(distortion);
//    if (has_distortion)
//    {
//      // Precompute undistortion and rectification transformations for speed up.
//      const cv::Size img_size(img_width, img_height);
//      // Store "rectified" camera matrix, see https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#getoptimalnewcameramatrix
//      K_ = cv::getOptimalNewCameraMatrix(intrinsics, distortion, img_size, 1.0, img_size, 0);
//      cv::initUndistortRectifyMap(intrinsics, distortion, cv::Mat(), K_, img_size, CV_16SC2, map1_, map2_);
//    }
//    else
//    {
//      K_ = intrinsics.clone();
//    }
//    skip_rectification_ = !has_distortion;
//  }

//  void Init(cv::FileStorage &fs, int cam_number)
//  {
//    // Load intrinsics and distortion coefficients.
//    cv::Mat intrinsics, distortion;
//    int img_width, img_height;

//    std::stringstream ss;
//    ss << "K" << cam_number;
//    fs[ss.str()] >> intrinsics;

//    ss.str("");
//    ss.clear();
//    ss << "D" << cam_number;
//    fs[ss.str()] >> distortion;

//    ss.str("");
//    ss.clear();
//    ss << "img_width" << cam_number;
//    fs[ss.str()] >> img_width;

//    ss.str("");
//    ss.clear();
//    ss << "img_height" << cam_number;
//    fs[ss.str()] >> img_height;

//    // If you force us to load a multicam_calibration (a single calibration file containing all the stuff), we assume there is no detailed RGBD sensor calibration for now.
//    VCP_LOG_WARNING("Assuming that there is no RGBD sensor calibration (relative rotation/translation) available");
//    R_rgbd_ = cv::Mat::eye(3,3,CV_64FC1);
//    t_rgbd_ = cv::Mat::zeros(3,1,CV_64FC1);

//    const bool has_distortion = HasLensDistortion(distortion);
//    if (has_distortion)
//    {
//      // Precompute undistortion and rectification transformations for speed up.
//      const cv::Size img_size(img_width, img_height);
//      // Store "rectified" camera matrix, see https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#getoptimalnewcameramatrix
//      K_ = cv::getOptimalNewCameraMatrix(intrinsics, distortion, img_size, 1.0, img_size, 0);
//      cv::initUndistortRectifyMap(intrinsics, distortion, cv::Mat(), K_, img_size, CV_16SC2, map1_, map2_);
//    }
//    else
//    {
//      K_ = intrinsics.clone();
//    }
//    skip_rectification_ = !has_distortion;
//  }
//};


//class StereoRectifier : public Rectifier
//{
//public:
//  StereoRectifier(const std::string &filename)
//  {
//    cv::FileStorage fs(filename, cv::FileStorage::READ);
//    Init(fs);
//    fs.release();
//  }
//  StereoRectifier(cv::FileStorage &fs) { Init(fs); }
//  StereoRectifier(cv::FileStorage &fs, int cam_number) { Init(fs, cam_number); }

//  virtual ~StereoRectifier() {}

//  cv::Mat Rectify(const cv::Mat &image) const override
//  {
//    if (image.empty())
//      return image;

//    cv::Rect roi_left(0, 0, image.cols/2, image.rows);
//    cv::Rect roi_right(roi_left.width, 0, roi_left.width, roi_left.height);
//    const cv::Mat left_in = image(roi_left);
//    const cv::Mat right_in = image(roi_right);

//    cv::Mat out(image.size(), image.type());
//    cv::Mat left_out = out(roi_left);
//    cv::Mat right_out = out(roi_right);

//    cv::remap(left_in, left_out, map11_, map12_, cv::INTER_LINEAR);
//    cv::remap(right_in, right_out, map21_, map22_, cv::INTER_LINEAR);
//    return out;
//  }

//  cv::Mat K() const override { return K1_; }

//  cv::Mat P2() const override { return P2_; }

//  void StereoExtrinsics(cv::Mat &R, cv::Mat &t) const override
//  {
//    R = R_stereo_;
//    t = t_stereo_;
//  }

//private:
//  cv::Mat K1_;
//  cv::Mat P2_;
//  cv::Mat map11_, map12_;
//  cv::Mat map21_, map22_;
//  cv::Mat R_stereo_;
//  cv::Mat t_stereo_;

//  void Init(cv::FileStorage &fs)
//  {
//    // Load intrinsics and distortion coefficients.
//    int img_width, img_height;
//    cv::Mat M1, M2, D1, D2, R, T;
//    fs["K1"] >> M1;
//    fs["D1"] >> D1;
//    fs["K2"] >> M2;
//    fs["D2"] >> D2;
//    fs["R"] >> R;
//    fs["t"] >> T;
//    fs["img_width"] >> img_width;
//    fs["img_height"] >> img_height;

//    // Note: we don't need to adjust the principal point offset (the MATLAB toolbox already provides 0-based principal points).

//    // Set up mappings.
//    const double alpha = 0.0; // Crop to valid regions
//    const cv::Size input_size(img_width, img_height);
//    cv::Mat R1, R2, P1, Q;
//    cv::stereoRectify(M1, D1, M2, D2, input_size, R, T, R1, R2, P1, P2_, Q, cv::CALIB_ZERO_DISPARITY, alpha);
//    cv::initUndistortRectifyMap(M1, D1, R1, P1, input_size, CV_16SC2, map11_, map12_);
//    cv::initUndistortRectifyMap(M2, D2, R2, P2_, input_size, CV_16SC2, map21_, map22_);
//    // Store the rectified camera matrices, see: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereorectify
//    K1_ = P1(cv::Range(0,3), cv::Range(0,3)).clone();

//    R_stereo_ = cv::Mat::eye(3, 3, CV_64FC1);
//    P2_.col(3).copyTo(t_stereo_);
//  }

//  void Init(cv::FileStorage &fs, int cam_number)
//  {
//    // Load intrinsics and distortion coefficients.
//    int img_width, img_height;
//    cv::Mat M1, M2, D1, D2, R, T;
//    std::stringstream ss;
//    ss << "K" << cam_number << "-1";
//    fs[ss.str()] >> M1;
//    ss.str(""); ss.clear();
//    ss << "D" << cam_number << "-1";
//    fs[ss.str()] >> D1;

//    ss.str(""); ss.clear();
//    ss << "K" << cam_number << "-2";
//    fs[ss.str()] >> M2;
//    ss.str(""); ss.clear();
//    ss << "D" << cam_number << "-2";
//    fs[ss.str()] >> D2;

//    ss.str(""); ss.clear();
//    ss << "R" << cam_number << "-stereo";
//    fs[ss.str()] >> R;
//    ss.str(""); ss.clear();
//    ss << "t" << cam_number << "-stereo";
//    fs[ss.str()] >> T;

//    ss.str(""); ss.clear();
//    ss << "img_width" << cam_number;
//    fs[ss.str()] >> img_width;
//    ss.str(""); ss.clear();
//    ss << "img_height" << cam_number;
//    fs[ss.str()] >> img_height;

//    // Set up mappings.
//    const double alpha = 0.0; // Crop to valid regions
//    const cv::Size input_size(img_width, img_height);
//    cv::Mat R1, R2, P1, Q;
//    cv::stereoRectify(M1, D1, M2, D2, input_size, R, T, R1, R2, P1, P2_, Q, cv::CALIB_ZERO_DISPARITY, alpha);
//    cv::initUndistortRectifyMap(M1, D1, R1, P1, input_size, CV_16SC2, map11_, map12_);
//    cv::initUndistortRectifyMap(M2, D2, R2, P2_, input_size, CV_16SC2, map21_, map22_);
//    // Store the rectified camera matrices, see: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereorectify
//    K1_ = P1(cv::Range(0,3), cv::Range(0,3)).clone();

//    R_stereo_ = cv::Mat::eye(3, 3, CV_64FC1);
//    P2_.col(3).copyTo(t_stereo_);
//  }
//};


//std::unique_ptr<Rectifier> GetRectifier(cv::FileStorage &fs, const StreamType stream_type, int cam_number)
//{
//  switch(stream_type)
//  {
//  case StreamType::MONO:
//    return std::unique_ptr<MonoRectifier>(new MonoRectifier(fs, cam_number));

//  case StreamType::STEREO:
//    return std::unique_ptr<StereoRectifier>(new StereoRectifier(fs, cam_number));

//  case StreamType::RGBD_COLOR:
//  case StreamType::RGBD_DEPTH:
//    return std::unique_ptr<RgbdMonoRectifier>(new RgbdMonoRectifier(fs, cam_number));


} // namespace calibration
} // namespace best
} // namespace vcp
