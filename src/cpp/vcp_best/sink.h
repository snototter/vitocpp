#ifndef __VCP_BEST_SINK_H__
#define __VCP_BEST_SINK_H__

#include <string>
#include <memory>
#include <exception>
#include <opencv2/core/core.hpp>
#include "sink_buffer.h"


namespace vcp
{
namespace best
{
/**
 * @brief Abstract base class for all device sinks.
 *
 FIXME adapt doc* Intended use:
 * * Instantiate (via factory, pass configuration parameters).
 * * StartStream() starts a separate std::thread to receive camera stream.
 * * GetNextFrame() retrieves the next unprocessed frame (or empty if no frame
 *   is available). <tt>Note</tt> that the actual sink might use a limited
 *   image queue (i.e. the next frame is only guaranteed to be the oldest
 *   within the sink's buffer).
 * * GetMostRecentFrame() retrieves the most recent frame (or empty if no frame
 *   is available).
 * * Terminate() stops the stream and cleans up.
 */
class StreamSink
{
protected:
  StreamSink() {}

public:
  virtual ~StreamSink() {}

  virtual bool OpenDevice() = 0;
  virtual bool StartStreaming() = 0;
  virtual bool StopStreaming() = 0;
  virtual bool CloseDevice() = 0;

  virtual std::vector<cv::Mat> GetNextFrames() = 0;

  /** @brief Classes which support backwards seeking must overwrite this method. */
  virtual std::vector<cv::Mat> GetPreviousFrames()
  {
    throw std::runtime_error("Sink does not support stream seeking!");
  }


  /** @brief Classes which support skipping frames must overwrite this method. */
  virtual std::vector<cv::Mat> FastForward(size_t /*num_frames*/)
  {
    throw std::runtime_error("Sink does not support stream seeking!");
  }

  //TODO doc
  virtual int IsDeviceAvailable() const = 0; // Must be an int, because std::atomic uses stdbool.h which redefines "bool" and thus, breaks the compiler (pure virtual function)

  /** @brief Informs you whether the image queue is filled or empty. */
  virtual int IsFrameAvailable() const = 0;
};


///** @brief Returns a StreamSink wrapper to access video files. Stream starts from (0-based) first_frame. */
//std::unique_ptr<StreamSink> CreateVideoFileSink(const std::string &video_filename, size_t first_frame = 0, double fps=-1.0, bool color_as_bgr = false);


///** @brief Returns a StreamSink wrapper to access a directory full of images as stream. Stream starts from (0-based) first_frame. */
//std::unique_ptr<StreamSink> CreateImageDirectorySink(const std::string &image_directory, size_t first_frame = 0, bool color_as_bgr = false);

} // namespace best
} // namespace vcp
#endif // __VCP_BEST_SINK_H__
