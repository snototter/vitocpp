#ifndef __VCP_BEST_CAPTURE_H__
#define __VCP_BEST_CAPTURE_H__

#include <iostream>
#include <memory>
#include <string>
#include <vector>


#include <opencv2/core/core.hpp>
#include <vcp_config/config_params.h>
#include "sink.h"

namespace vcp
{

/** @brief BESt (Best Effort Streaming) module.
 *
 * Configuration can easily be done via:
 * libconfig++
 * TODO refer to examples/data/best/*.cfg
 * or python bindings/demo
 */
namespace best
{
/** @brief Provides access to image streams of (potentially) multiple,
 * (potentially) different sinks/sensors/devices. Use this, if:
 * - you want to stream from a single device.
 * - you want to stream from multiple devices of the same type.
 * - you want to stream from multiple devices with different types, e.g.
 *   a combined stream of an RGBD sensor and some IP cameras.A wrapper using multiple, different captures (e.g. if you
 *
 * Who in the right mind would want to do that? ;-) We do, frequently!
 *
 * Workflow:
 * 1. Create capture object via vcp::best::CreateCapture().
 * 2. Open() devices, so they can be properly initialized.
 *    Check if initialization succeeded: AreDevicesAvailable().
 * 3. Start() streaming.
 * 4. Next() - empty cv::Mat entries correspond to missing
 *    frames.
 *    You can query if frames are available for all sinks, via AreFramesAvailable().
 *    However, this is NOT guaranteed to work if you use @see FastForward()
 *    or @see Previous(). The issue is that the user may choose to arbitrarily
 *    query the next/previous frames on those sinks.
 * 5. Stop() streaming.
 * 6. Repeat 4-6 if you want to restart streaming.
 * 7. Close() or simply shut down - Capture will gracefully shut down anyways.
 */
class Capture
{
public:
  virtual ~Capture() {}

  /** @brief Returns true if the capturing device/s is/are available. */
  virtual bool AreDevicesAvailable() const = 0;

  /** @brief Returns true if frames (from all devices) can be retrieved (i.e. have been enqueued). */
  virtual bool AreFramesAvailable() const = 0;

  /** @brief Initialize devices. */
  virtual bool OpenDevices() = 0;

  /** @brief Close devices/tear down connections. */
  virtual bool CloseDevices() = 0;

  /** @brief Starts the image streams. */
  virtual bool StartStreams() = 0;

  /** @brief Stops the image streams. */
  virtual bool StopStreams() = 0;

  /** @brief Returns the currently available frames.
   *
   * If no data is available for a specific sink, it
   * will yield an empty cv::Mat instead.
   */
  virtual std::vector<cv::Mat> Next() = 0;


  /** @brief Some sinks support seeking backwards (i.e. image directory and video sinks). Others will throw a std::runtime_error. */
  virtual std::vector<cv::Mat> Previous() = 0;


  /** @brief Some sinks (video and imagedir) support fast forwarding (skipping frames). Others will throw a std::runtime_error. */
  virtual std::vector<cv::Mat> FastForward(size_t num_frames) = 0;


  /** @brief returns the number of configured streams (NOT the number of sinks). */
  virtual size_t NumStreams() const = 0;

  /** @brief returns the number of devices/sinks (NOT the number of streams/frames). */
  virtual size_t NumDevices() const = 0;


  /** @brief Return the (user-defined) frame labels.
   * Note that:
   * - The order may differ from the one you used in the configuration file.
   *   This is because some devices must be handled specifically (e.g. multiple
   *   simultaneous RTSP streams, or synchronized/triggered RGBD sensors).
   * - The labels may also differ because you specify a per-device label. Sinks
   *   with multiple streams (e.g. or RGBD) will prefix the label accordingly.
   */
  virtual std::vector<std::string> FrameLabels() const = 0;


  /** @brief Returns the configuration parameter name, i.e. "cameraX", for
   * each stream/frame.
   * See comments on @see FrameLabels() why this might be of interest
   * to you. Usually you won't ever need it.
   */
  virtual std::vector<std::string> ConfigurationKeys() const = 0;


  /** @brief
  virtual std::vector<SinkType> SinkTypes() const = 0;

//  /** @brief Provides the key (group name, e.g. 'camera3') of the vcp configuration file or an empty string.
//   *
//   * Is needed because if you mix camera types (webcam + video + ipcam,...), the ordering
//   * in the configuration file (camera1, camera2) may not relate to the
//   * ordering of sink[idx]!
//   */
//  virtual std::string GetCorrespondingConfigKey(size_t sink_index) const;
//  virtual FrameType GetStreamType(size_t stream_index) const;
//  virtual std::vector<StreamType> GetStreamTypes() const { return stream_types_; }


protected:
  Capture() {}

//  void AddLabels(const std::vector<std::string> &labels) { sink_labels_.insert(sink_labels_.end(), labels.begin(), labels.end()); }
//  void AddLabel(const std::string &label) { sink_labels_.push_back(label); }

//  void AddCalibrationFiles(const std::vector<std::string> &sink_calibration_files) {
//    sink_calibration_files_.insert(sink_calibration_files_.end(), sink_calibration_files.begin(), sink_calibration_files.end()); }
//  void AddCalibrationFile(const std::string &calibration_file) { sink_calibration_files_.push_back(calibration_file); }

//  void AddStreamTypes(const std::vector<StreamType> &stream_types) {
//    stream_types_.insert(stream_types_.end(), stream_types.begin(), stream_types.end()); }
//  void AddStreamType(const StreamType &stream_type) { stream_types_.push_back(stream_type); }

//  void AddCorrespondingConfigKeys(const std::vector<std::string> &config_keys) {
//    corresponding_config_keys_.insert(corresponding_config_keys_.end(), config_keys.begin(), config_keys.end()); }
//  void AddCorrespondingConfigKey(const std::string &config_key) { corresponding_config_keys_.push_back(config_key); }

//private:
//  std::vector<std::string> sink_labels_;
//  std::vector<std::string> sink_calibration_files_;
//  std::vector<StreamType> stream_types_;
//  std::vector<std::string> corresponding_config_keys_;
};

std::unique_ptr<Capture> CreateCapture(const vcp::config::ConfigParams &config);


////---------------------------------------------------------------------------------------------------------------------------
///** @brief Base class to access rectified image stream(s). Requires valid calibration files. */
////TODO move to capture
//class RectifiedCapture : public Capture
//{
//public:
//  virtual ~RectifiedCapture() {}

//  virtual cv::Mat K(size_t sink_index) const = 0;
//  virtual cv::Mat P2(size_t sink_index) const = 0;
//  virtual cv::Mat R(size_t sink_index) const = 0;
//  virtual cv::Mat t(size_t sink_index) const = 0;
//  virtual cv::Mat C(size_t sink_index) const = 0;
//  virtual cv::Vec4d ImagePlane(size_t sink_index) const = 0;
//  virtual void StereoRt(size_t sink_index, cv::Mat &R, cv::Mat &t) const = 0;
//  virtual void SaveCalibration(const std::string &filename) const = 0;

//protected:
//  RectifiedCapture() : Capture() {}
//};

//std::unique_ptr<RectifiedCapture> CreateRectifiedCapture(const pvt::config::ConfigParams &config, bool force_loading_extrinsics);

} // namespace best
} // namespace vcp

#endif // __VCP_BEST_CAPTURE_H__
