#ifndef __VCP_BEST_CAPTURE_H__
#define __VCP_BEST_CAPTURE_H__

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include <vcp_config/config_params.h>
#include "sink.h"

namespace vcp
{

/** @brief BESt (Best Effort Streaming) module.
 *
 * Please refer to the Python examples which use the
 * corresponding Python bindings: examples/python3/best_demo.py
 * Exemplary libconfig++ configuration files for streaming from
 * different supported devices can be found at examples/data/data-best/<*.cfg>
 */
namespace best
{


/** @brief Information on how the vcp::best::Capture user will store each stream.
 *
 * Within vcp::best, this struct is used to store a "replay configuration", i.e.
 * a configuration file which you can use to replay a streaming "session".
 * For this, we need to know whether you store(d) the streams as a video, image, or not at all.
 */
struct StreamStorageParams
{
  enum class Type : short
  {
    NONE, /**< The corresponding stream won't be stored at all. */
    IMAGE_DIR, /**< The stream will be stored as an image sequence (usually for depth images). */
    VIDEO /**< The stream will be stored as a video file. */
  };

  StreamStorageParams(const Type &type, const std::string &path)
    : type(type), path(path)
  {}

  StreamStorageParams() : type(Type::NONE), path()
  {}

  Type type;
  std::string path;
};

std::ostream& operator<<(std::ostream & os, const StreamStorageParams &ssp);


/** @brief Provides access to image streams of (potentially) multiple,
 * (potentially) different sinks/sensors/devices. Use this, if:
 * - you want to stream from a single device.
 * - you want to stream from multiple devices of the same type.
 * - you want to stream from multiple devices with different types, e.g.
 *   a combined stream of an RGBD sensor and some IP cameras.A wrapper using multiple, different captures (e.g. if you
 *
 * Who in the right mind would want to do that? ;-) We do, frequently!
 *
 * Typical streaming workflow:
 * 1. Write a configuration file and create the capture object
 *    via vcp::best::CreateCapture().
 * 2. OpenDevices(), so they can be properly initialized.
 *    Check if initialization succeeded: @see AreAllDevicesAvailable().
 * 3. StartStreams().
 *    Note that some devices take quite long (5-10 seconds in our tests)
 *    to initialize the streams, so you may want to use a blocking call
 *    to @see WaitForFrames().
 * 4. Next() returns a vector of the currently available (enqueued) frames.
 *    * Empty cv::Mat entries correspond to missing frames.
 *    * You can query if frames are available for all sinks via @see AreAllFramesAvailable().
 *      However, this is NOT guaranteed to work if you use @see FastForward()
 *      or @see Previous(). The issue is that the user may choose to arbitrarily
 *      query the next/previous frames on those sinks.
 *    * You can check how many valid frames are available, via @see NumAvailableFrames().
 * 5. StopStreaming() or simply shut down the capture (see Step 7).
 * 6. Repeat 3-5 if you want to restart streaming.
 *    Note: didn't test restarting exhaustively, so there may some devices
 *    that don't support that.
 * 7. CloseDevices() to gracefully shut down.
 */
class Capture
{
public:
  virtual ~Capture() {}

  /** @brief Returns true if the capturing device/s is/are available. */
  virtual bool AreAllDevicesAvailable() const = 0;

  /** @brief Returns true if frames (from all devices) can be retrieved (i.e. have been enqueued). */
  virtual bool AreAllFramesAvailable() const = 0;

  /** @brief Returns the number of available frames.
   *
   * Might be useful if one of your sinks stops working/streaming and you still want to continue
   * processing, etc. In such a case, @see AreAllFramesAvailable() would return false, whereas here
   * you get the actual number of frames and can decide yourself.
   */
  virtual size_t NumAvailableFrames() const = 0;

  /** @brief Initialize devices. */
  virtual bool OpenDevices() = 0;

  /** @brief Close devices/tear down connections. */
  virtual bool CloseDevices() = 0;

  /** @brief Starts the image streams. */
  virtual bool StartStreams() = 0;

  /** @brief Use this blocking call to wait for the next set of images to become available.
   * Specify a timeout in milliseconds, after that the result will indicate whether
   * frames from all devices/sinks are available or not.
   */
  virtual bool WaitForFrames(double timeout_ms, bool verbose) const = 0;

  /** @brief Stops the image streams. */
  virtual bool StopStreams() = 0;

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
  virtual std::vector<cv::Mat> Next() = 0;

  /** @brief Some sinks support seeking backwards (i.e. image directory and video sinks). Others will throw a std::runtime_error. */
  virtual std::vector<cv::Mat> Previous() = 0;

  /** @brief Some sinks (video and imagedir) support fast forwarding (skipping frames). Others will throw a std::runtime_error. */
  virtual std::vector<cv::Mat> FastForward(size_t num_frames) = 0;

  /** @brief Returns the number of configured streams (NOT the number of sinks). */
  virtual size_t NumStreams() const = 0;

  /** @brief Returns the number of devices/sinks (NOT the number of streams/frames).
   *
   * While this number corresponds to the number of "software sinks" used in VCP,
   * it may not be exactly the number of physical devices: For example, you could
   * configure 2 IP streams from the same device, one configured by its IP and the
   * other by its hostname. Handling such cases would be too complex.
   */
  virtual size_t NumDevices() const = 0;


  /** @brief Returns the current frame number.
   *
   * The internal counter is expected to keep track of the @see Next(), @see Previous()
   * and @see FastForward() calls.
   * The counter always starts at 0 upon initialization of the capture.
   */
  virtual size_t CurrentFrameNumber() const = 0;

  /** @brief Return the (user-defined) frame labels.
   * Note that:
   * - The order may differ from the one you used in the configuration file.
   *   This is because some devices must be handled specifically (e.g. multiple
   *   simultaneous RTSP streams, or synchronized/triggered RGBD sensors).
   * - The labels may also differ because you specify a per-device label. Sinks
   *   with multiple streams (e.g. or RGBD) will prefix the label accordingly.
   */
  virtual std::vector<std::string> FrameLabels() const = 0;

  /** @brief Look up the FrameLabel for a specific stream/frame, also refer to @see FrameLabels(). */
  virtual std::string FrameLabelAt(size_t stream_index) const = 0;

  /** @brief Look up a canonic version (special characters will be stripped/replaced) of the FrameLabel for a specific stream/frame, also refer to @see FrameLabels(). */
  virtual std::string CanonicFrameLabelAt(size_t stream_index) const = 0;

  /** @brief Returns the configuration parameter name, i.e. "cameraX", for
   * each stream/frame.
   * See comments on @see FrameLabels() why this might be of interest
   * to you. However, it's usually better to query the @see FrameLabels()
   * and/or @see FrameTypes().
   */
  virtual std::vector<std::string> ConfigurationKeys() const = 0;

  /** @brief Look up the configuration key/parameter name for a specific stream/frame, also refer to @see ConfigurationKeys(). */
  virtual std::string ConfigurationKeyAt(size_t stream_index) const = 0;

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
  virtual std::vector<FrameType> FrameTypes() const = 0;

  /** @brief Look up the FrameType for a specific stream/frame, also refer to @see FrameTypes(). */
  virtual FrameType FrameTypeAt(size_t stream_index) const = 0;

  /** @brief Check if the given stream is rectified. */
  virtual bool IsStreamRectified(size_t stream_index) const = 0;

  /** @brief Returns true if the stream at the given index originates from a "step-through-able" @see ImageDirectorySink.
   * Note: in the (very distant) future, we might implement a "TimedImageDirectorySink", which acts like a live stream.
   * For this, IsStepAbleImageDirectory is expected to return false.
   * Is used within the Python bindings to check whether a "MulticamStreamer" (using callbacks to receive framesets) or
   * a "MulticamStepper" (loop-based frameset retrieval, one frameset per iteration) should be used.
   */
  virtual bool IsStepAbleImageDirectory(size_t stream_index) const = 0;

  /** @brief Stores a configuration file (along with intrinsic calibrations if available) to "replay" the recorded streams.
   *
   * - Note that you have to record/store the streams yourself!
   * - Provide the output folder as argument, the configuration will be stored as "<folder>/replay.cfg"
   * - Intrinsic calibrations will be stored as "<folder>/calibration/calib-<stream_label>.xml"
   * - If save_extrinsics is true (and available, i.e. either it (1) has been loaded upon creating this capture or
   *   (2) it has been set by you via @see SetExtrinsicsAt), it will be stored as "<folder>/calibration/extrinsics.xml".
   * - "folder" will be created if it does not exist
   * - Existing contents will be overwritten!
   * - There must be exactly one storage_param for each configured stream, i.e. storage_param[i] belongs to stream/frame[i]
   * - "folder" will be prepended to relative storage_params[i].path entries.
   */
  virtual bool SaveReplayConfiguration(const std::string &folder, const std::map<std::string, StreamStorageParams> &storage_params, bool save_extrinsics) const = 0;

  /** @brief Returns the 3x3 intrinsic camera matrix. */
  virtual cv::Mat CameraMatrixAt(size_t stream_index) const = 0;

  /** @brief Returns the Nx1 distortion coefficients. */
  virtual cv::Mat DistortionAt(size_t stream_index) const = 0;

  /** @brief Sets the stereo transformation from stream_index to its reference view (if calibrated). Otherwise, they'll be set to empty matrices. */
  virtual void StereoTransformation(size_t stream_index, cv::Mat &R, cv::Mat &t) const = 0;

  /** @brief Sets the extrinsics (3x3 rotation R, 3x1 translation t) if available. Otherwise, they'll be set to empty matrices. */
  virtual void ExtrinsicsAt(size_t stream_index, cv::Mat &R, cv::Mat &t) const = 0;

  /** @brief Given a stream/frame index, this yields a list of frame indices which originate from the same sink (physical device/sensor).
   * This also works for previously recorded streams, but only iff you used @see SaveReplayConfiguration() to create the
   * configuration file.
   */
  virtual std::vector<size_t> StreamsFromSameSink(size_t stream_index, bool include_self) const = 0;

  /** @brief Returns the SinkParams (parametrization PER SINK) for the given STREAM INDEX. */
  virtual SinkParams SinkParamsAt(size_t stream_index) const = 0;

  /** @brief Use this to "inject" extrinsics into the sinks.
   *
   * Returns true if the extrinsics have been successfully set. Returns false, if you pass empty matrices and
   * the stream (1) doesn't have a pre-calibrated transformation to a reference view, or (2) the reference view
   * has no valid extrinsics yet.
   *
   * Required for example by examples/python3/tools/calibrate-extrinsics.py, a multi-stream sink (e.g. a
   * depth sensor) has to/will take care of adjusting the extrinsics for sensors where R & t are empty (cannot be
   * estimated visually, e.g. the depth stream). For such streams, the underlying sink is expected to have a
   * (probably factory-calibrated) transformation between the stream and a reference view (e.g. the sensor's
   * left and/or color view) which will be used to compute its extrinsics from R & t of the reference view.
   */
  virtual bool SetExtrinsicsAt(size_t stream_index, const cv::Mat &R, const cv::Mat &t) = 0;

  /** @brief "Inject" extrinsics by the stream's label. */
  virtual bool SetExtrinsicsAt(const std::string &stream_label, const cv::Mat &R, const cv::Mat &t) = 0;

  /** @brief Stores the extrinsic calibration (if it has (1) been loaded upon startup, i.e. already
   * existed or (2) has been injected via @see SetExtrinsicsAt) as cv::FileStorage-compatible XML file.
   */
  virtual bool SaveExtrinsicCalibration(const std::string &filename) const = 0;

protected:
  Capture() {}
};


/** Overload the stream operator '<<' to print a Capture. */
std::ostream& operator<<(std::ostream & os, const Capture &cap);


/** @brief Creates a capture based on the provided configuration. */
std::unique_ptr<Capture> CreateCapture(const vcp::config::ConfigParams &config);

} // namespace best
} // namespace vcp

#endif // __VCP_BEST_CAPTURE_H__
