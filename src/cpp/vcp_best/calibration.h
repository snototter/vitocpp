#ifndef __VCP_BEST_CALIBRATION_H__
#define __VCP_BEST_CALIBRATION_H__

#include <string>
#include <vector>
#include <map>
#include <iostream>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
#else
    #include <opencv2/core.hpp>
#endif

namespace vcp
{
namespace best
{

// Forward declaration (defined in sink.h)
enum class FrameType : short;

/** @brief Functionality to undistort & rectify images, load calibration files, etc. */
namespace calibration
{

/** @brief Encapsulates a stream's intrinsic calibration (needed to undistort & rectify it). */
class StreamIntrinsics
{
public:
  /** @brief C'tor. */
  StreamIntrinsics();

  /** @brief Copy constructor. */
  StreamIntrinsics(const StreamIntrinsics &other);

  /** @brief Class method to instantiate an object from a monocular calibration. */
  static StreamIntrinsics FromMonocular(const cv::Mat &intrinsics,
                                        const cv::Mat &distortion,
                                        const std::string &label, const std::string &identifier,
                                        const cv::Size &resolution,
                                        const cv::Mat &R=cv::Mat(), const cv::Mat &t=cv::Mat());

  /** @brief Destructor. */
  ~StreamIntrinsics();

  /** @brief True if the calibration has not been set. */
  bool Empty() const;

  /** @brief Undistort and rectify an image of the corresponding stream. */
  cv::Mat UndistortRectify(const cv::Mat &image) const;

  /** @brief Returns the initial camera matrix (before undistrotion/rectification).
   *
   * May be the same as @see IntrinsicsRectified() if there's no distortion.
   */
  cv::Mat IntrinsicsOriginal() const;

  /** @brief Returns the camera matrix after undistortion & rectification. */
  cv::Mat IntrinsicsRectified() const;

  /** @brief Returns true if the distortion coefficients are set and not zero. */
  bool HasDistortion() const;

  /** @brief Returns a Nx1 matrix holding the distortion coefficients. */
  cv::Mat Distortion() const;

  /** @brief Returns true if an extrinsic transformation (R, t) from the corresponding view to a "reference view" has been configured. */
  bool HasTransformationToReference() const;

  /** @brief Sets the extrinsic transformation from the corresponding view to a "reference view". See also @see HasTransformationToReference(). */
  void TransformationToReference(cv::Mat &R, cv::Mat &t) const;

  /** @brief Returns the stream label assigned to this calibrated stream. */
  std::string StreamLabel() const;

  /** @brief Returns the stream's image resolution. */
  cv::Size Resolution() const;

  /** @brief Setter. */
  void SetResolution(int width, int height);

  /** @brief Returns true if @see Resolution() yields a valid size object. */
  bool HasResolution() const;

  /** @brief Returns the device identifier (e.g. serial number) if available, or an empty string.
   *
   * You can use this to check whether you actually loaded the correct calibration file for your
   * current device.
   */
  std::string Identifier() const;

  /** @brief Setter. */
  void SetIdentifier(const std::string &id);

  /** @brief Save the calibration to disk. */
  bool Save(const std::string &calibration_file, const bool rectified, const FrameType &frame_type) const;

private:
  bool skip_undistort_rectify_;  /**< If there's no distortion, we can skip undistortion & rectification (except for stereo setups). */
  std::string label_;            /**< Label assigned to this calibration (in case we need to match it to a replay/recorded video sequence). */
  cv::Mat intrinsics_rectified_; /**< 3x3 camera matrix after applying undistortion/rectification. */
  cv::Mat intrinsics_original_;  /**< The initial camera matrix (before undistrotion/rectification). */
  cv::Mat distortion_;           /**< Nx1 distortion coefficients. */
  cv::Mat R_to_ref_;             /**< Rotation matrix (or empty) to transform points from this view to a reference view (e.g. in stereo or RGBD sensors). */
  cv::Mat t_to_ref_;             /**< Translation vector (or empty) to transform this view to a reference view. */
  cv::Mat undistort_rectify_map1_; /**< Pixel map 1/2 for undistortion/rectifaction. */
  cv::Mat undistort_rectify_map2_; /**< Pixel map 2/2 for undistortion/rectifaction. */
  cv::Size resolution_;          /**< Image resolution. */
  std::string identifier_;       /**< Some sinks store device identifiers (e.g. serial number), so you can ensure that you loaded the correct calibration file. */
};

/** @brief Overloaded output operator for StreamIntrinsics. */
std::ostream &operator<<(std::ostream &out, const StreamIntrinsics &si);


/** @brief Encapsulates a stream's extrinsic transformation. */
class StreamExtrinsics
{
public:
  /** @brief C'tor. */
  StreamExtrinsics();

  /** @brief C'tor. */
  StreamExtrinsics(const cv::Mat &R, const cv::Mat &t);

  /** @brief Copy constructor. */
  StreamExtrinsics(const StreamExtrinsics &other);

  /** @brief Destructor. */
  ~StreamExtrinsics();

  /** @brief True if the transformation has not been set or is invalid. */
  bool Empty() const;

  bool SetExtrinsics(const cv::Mat &R, const cv::Mat &t,
                     const StreamIntrinsics &intrinsics=StreamIntrinsics(),
                     const cv::Mat &R_reference=cv::Mat(), const cv::Mat &t_reference=cv::Mat());

  /** @brief Returns the stream label assigned to this calibrated stream. */
//  std::string StreamLabel() const;

  cv::Mat R() const;
  cv::Mat t() const;

//  /** @brief Returns the device identifier (e.g. serial number) if available, or an empty string.
//   *
//   * You can use this to check whether you actually loaded the correct calibration file for your
//   * current device.
//   */
//  std::string Identifier() const;

//  /** @brief Setter. */
//  void SetIdentifier(const std::string &id);

//  /** @brief Save the calibration to disk. */
//  bool Save(const std::string &calibration_file, const bool rectified, const FrameType &frame_type) const;

private:
  cv::Mat R_;             /**< Rotation as 3x3 matrix (or empty). */
  cv::Mat t_;             /**< Translation as 3x1 vector (or empty). */
//  std::string identifier_;       /**< Some sinks store device identifiers (e.g. serial number), so you can ensure that you loaded the correct calibration file. */
};



/** @brief Load an intrinsic calibration file using cv::FileStorage (thus, supporting XML and YAML).
 *
 * If you want to retrieve the intrinsics in a specific order, make sure that the
 * calibration file contains a <label_xy> node for each stream, e.g. <label_left>left-view</label_left>.
 * Otherwise, the output vector would have the same order as the corresponding "DumpCalibration"
 * function. For example, RGBD cameras: "rgb", "depth", "infrared" (or "ir1", then "ir2" for RealSense).
 */
std::vector<StreamIntrinsics> LoadIntrinsicsFromFile(const std::string &calibration_file,
                                           const std::vector<std::string> &label_order=std::vector<std::string>());

/** @brief Returns true, if there are any distortion coefficients != 0 ('distortion' must be Nx1, CV_64FC1). */
bool HasLensDistortion(const cv::Mat &distortion);

/** @brief Load an extrinsic calibration file using cv::FileStorage (thus, supporting XML and YAML).
 *
 * If the extrinsic calibration file covers a single camera, it must contain the following tags:
 * * <label>stream label</label>
 * * <R>3x3 rotation matrix</R>
 * * <t>3x1 translation vector</t>
 *
 * Otherwise - if there are extrinsics for multiple cameras within the calibration file,
 * the following tags are needed:
 * * Mandatory: <num_cameras>N (int)</num_cameras>
 * * For X in [0, N-1]:
 *   * <labelX>stream label (string)</labelX>
 *   * [optional - only present if stream is calibrated] <RX>3x3 rotation matrix</RX>
 *   * [optional - only present if stream is calibrated] <tX>3x1 translation vector</tX>
 */
std::map<std::string, StreamExtrinsics> LoadExtrinsicsFromFile(const std::string &calibration_file);

} // namespace calibration
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_CALIBRATION_H__
