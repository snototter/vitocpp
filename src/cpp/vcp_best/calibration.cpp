#include "calibration.h"
#include "sink.h"
#include <vcp_utils/vcp_error.h>

#include <vcp_utils/string_utils.h>
#include <vcp_utils/file_utils.h>
#include <vcp_math/common.h>
#include <vcp_imutils/matutils.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <sstream>
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
                                          const std::string &identifier,
                                          const cv::Size &resolution,
                                          const cv::Mat &R, const cv::Mat &t)
{
  StreamIntrinsics si;
  si.intrinsics_original_ = intrinsics.clone();
  si.distortion_ = distortion.clone();
  si.skip_undistort_rectify_ = !HasLensDistortion(distortion);
  si.label_ = label;
  si.identifier_ = identifier;
  si.resolution_ = resolution;

  if (si.skip_undistort_rectify_)
  {
    si.intrinsics_rectified_ = si.intrinsics_original_.clone();
  }
  else
  {
    // Abort if resolution is not set (otherwise, OpenCV initUndistortRectifyMap would crash)
    if (!si.HasResolution())
      VCP_ERROR("Stream '" << label << "/" << identifier << "' has no valid resolution - needed for rectification.");

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

  if (undistort_rectify_map1_.empty() || undistort_rectify_map2_.empty())
  {
    VCP_LOG_FAILURE("Invalid rectification maps for stream '" << StreamLabel() << "' (" << Identifier() << ")");
    return image;
  }

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
    fs << "identifier" << identifier_;

  if (!label_.empty())
    fs << "label" << label_;


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

  out << "Intrinsics for stream '" << si.StreamLabel() << "'";
  if (!si.Identifier().empty())
    out << ", id='" << si.Identifier() << "'";
  out << ", fl=("
      << fx << ", " << fy << "), pp=(" << cx << ", " << cy << ")";
  if (si.HasDistortion())
    out << ", dc=" << (si.Distortion().rows > si.Distortion().cols ? si.Distortion().t() : si.Distortion());
  if (si.HasResolution())
    out << ", res=" << si.Resolution();
  if (si.HasTransformationToReference())
  {
    cv::Mat R, t;
    si.TransformationToReference(R, t);
    out << ", transformation to sensor's reference view (R, t^T):" << std::endl << R << ", " << t.t();
  }
  return out;
}

StreamExtrinsics::StreamExtrinsics()
{
}

StreamExtrinsics::StreamExtrinsics(const StreamExtrinsics &other)
  : R_(other.R_.clone()), t_(other.t_.clone())
{
}

StreamExtrinsics::StreamExtrinsics(const cv::Mat &R, const cv::Mat &t)
  : R_(R.clone()), t_(t.clone())
{
}

StreamExtrinsics::~StreamExtrinsics()
{
}

bool StreamExtrinsics::Empty() const
{
  return R_.empty() || t_.empty();
}

bool StreamExtrinsics::SetExtrinsics(const cv::Mat &R, const cv::Mat &t,
                                     const StreamIntrinsics &intrinsics, const cv::Mat &R_reference, const cv::Mat &t_reference)
{
  if (R.empty() || t.empty())
  {
    // Compute extrinsics from the (hopefully) known transformation to this stream's reference view/stream:
    if (intrinsics.Empty() || !intrinsics.HasTransformationToReference() || R_reference.empty() || t_reference.empty())
      return false;
    // Get known sensor transformation:
    cv::Mat R_view2ref, t_view2ref;
    intrinsics.TransformationToReference(R_view2ref, t_view2ref);

    // Check types
    if (R_reference.type() != t_reference.type() || R_view2ref.type() != R_reference.type() || t_view2ref.type() != t_reference.type())
    {
      VCP_LOG_FAILURE("Matrix type mismatch for extrinsics/known view-to-reference transformation:"
                      << "R_ref=" << imutils::CVMatTypeToString(R_reference)
                      << ", t_ref=" << imutils::CVMatTypeToString(t_reference) << std::endl
                      << "R_view2ref=" << imutils::CVMatTypeToString(R_view2ref)
                      << ", t_view2ref=" << imutils::CVMatTypeToString(t_view2ref));
    }
    // Build a 4x4 extrinsic transformation matrix for the reference view
    cv::Mat Mref = cv::Mat::eye(4, 4, R_reference.type());
    R_reference.copyTo(Mref(cv::Rect(0, 0, 3, 3)));
    t_reference.copyTo(Mref(cv::Rect(3, 0, 1, 3)));
    // The known sensor transformation warps from the current view to the
    // reference view, so we need to invert it:
    cv::Mat M_ref2view = cv::Mat::eye(4, 4, R_reference.type());
    const cv::Mat R_view2ref_inv = R_view2ref.t();
    const cv::Mat t_view2ref_inv = -t_view2ref;
    R_view2ref_inv.copyTo(M_ref2view(cv::Rect(0, 0, 3, 3)));
    t_view2ref_inv.copyTo(M_ref2view(cv::Rect(3, 0, 1, 3)));
    // Transform
    const cv::Mat M = M_ref2view * Mref;
    M(cv::Rect(0, 0, 3, 3)).copyTo(R_);
    M(cv::Rect(3, 0, 1, 3)).copyTo(t_);
  }
  else
  {
    // Take as-is
    R_ = R.clone();
    t_ = t.clone();
  }
  return !Empty();
}

cv::Mat StreamExtrinsics::R() const
{
  return R_;
}

cv::Mat StreamExtrinsics::t() const
{
  return t_;
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
  std::string lbl, identifier;

  // Read intrinsics, either as "K" or "M".
  const std::string kk = "K" + postfix;
  const std::string km = "M" + postfix;
  if (std::find(calib_keys.begin(), calib_keys.end(), kk) != calib_keys.end())
    ReadMat(fs, kk, calib_keys, K);
  else if (std::find(calib_keys.begin(), calib_keys.end(), km) != calib_keys.end())
    ReadMat(fs, km, calib_keys, K);
  else
  {
    VCP_LOG_WARNING("Calibration file '" << filename << "' doesn't contain intrinsics (neither \"K" << postfix << "\" nor \"M" << postfix << "\").");
    return intrinsics;
  }

  // Read distortion coefficients, either as "D" or "distortion".
  if (!ReadMat(fs, "D" + postfix, calib_keys, D))
    ReadMat(fs, "distortion" + postfix, calib_keys, D);

  ReadScalar(fs, "width" + postfix, calib_keys, width, -1);
  ReadScalar(fs, "height" + postfix, calib_keys, height, -1);

  const std::string def_lbl = postfix.empty() ? "unknown" : (postfix[0] == '_' ? postfix.substr(1) : postfix);
  ReadScalar(fs, "label" + postfix, calib_keys, lbl, def_lbl);

  ReadScalar(fs, "identifier" + postfix, calib_keys, identifier, std::string());

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

  return StreamIntrinsics::FromMonocular(K, D, lbl, identifier, cv::Size(width, height), R, t);
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

  VCP_LOG_INFO_DEFAULT("Loaded sensor-specific calibration from '" << filename << "':" << std::endl << intrinsics);
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
//    return false; //TODO implement stereo rectification
  else if (t.compare("rgbd") == 0)
    intrinsics = LoadGenericRGBDCalibration(calibration_fs, filename, calib_keys);
  else
  {
    VCP_LOG_FAILURE("Unsupported intrinsic calibration type '" << type << "'");
  }

  VCP_LOG_INFO_DEFAULT("Loaded generic calibration from '" << filename << "':" << std::endl << intrinsics);

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
    VCP_LOG_FAILURE("Cannot open calibration file '" << calibration_file << "'.");
    return intrinsics;
  }

  const auto keys = ListFileStorageNodes(fs);

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

  //TODO need to sort the intrinsics - or better, return a map!
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


std::map<std::string, StreamExtrinsics> LoadExtrinsicsFromFile(const std::string &calibration_file)
{
  std::map<std::string, StreamExtrinsics> extrinsics;
  if (!vcp::utils::file::Exists(calibration_file))
  {
    VCP_LOG_FAILURE("Calibration file '" << calibration_file << "' does not exist.");
    return extrinsics;
  }

  cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    VCP_LOG_FAILURE("Cannot open calibration file '" << calibration_file << "'.");
    return extrinsics;
  }

  const auto keys = ListFileStorageNodes(fs);

  const auto nc = std::find(keys.begin(), keys.end(), "num_cameras");
  if (nc == keys.end())
  {
    VCP_LOG_FAILURE("Extrinsic calibration file doesn't specify the 'num_cameras' parameter/tag!");
    return extrinsics;
  }

  int num_cameras;
  fs["num_cameras"] >> num_cameras;

  for (int i = 0; i < num_cameras; ++i)
  {
    std::stringstream ss;
    std::string label;
    ss << "label" << i;
    fs[ss.str()] >> label;

    // Check if this stream is actually calibrated:
    ss.str("");
    ss.clear();
    ss << "R" << i;
    const std::string key_r = ss.str();

    ss.str("");
    ss.clear();
    ss << "t" << i;
    const std::string key_t = ss.str();

    const auto pos_r = std::find(keys.begin(), keys.end(), key_r);
    const auto pos_t = std::find(keys.begin(), keys.end(), key_t);
    if (pos_r == keys.end() || pos_t == keys.end())
    {
      VCP_LOG_WARNING("No extrinsics available to load for stream '" << label << "'.");
      extrinsics.insert(std::pair<std::string, StreamExtrinsics>(label, StreamExtrinsics()));
    }
    else
    {
      cv::Mat R, t;
      fs[key_r] >> R;
      fs[key_t] >> t;
      if (R.empty() || t.empty())
      {
        VCP_LOG_FAILURE("Invalid extrinsics for stream '" << label << "'." << std::endl << "R: " << R << ", t: " << t);
      }
      else
      {
        VCP_LOG_INFO("Extrinsics loaded for stream '" << label << "'."); //<< ", " << R << t);
      }
      extrinsics.insert(std::pair<std::string, StreamExtrinsics>(label, StreamExtrinsics(R, t)));
    }
  }

  return extrinsics;
}

} // namespace calibration
} // namespace best
} // namespace vcp
