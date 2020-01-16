#ifndef __VCP_BEST_FILE_SINK_H__
#define __VCP_BEST_FILE_SINK_H__

#include <string>
#include <memory>
#include <exception>
#include <opencv2/core/core.hpp>
#include "sink.h"
#include "common_types.h"

namespace vcp
{
namespace best
{
/** @brief Configuration parameters to "stream" from a video file. */
class VideoFileSinkParams : public SinkParams
{
public:
  virtual ~VideoFileSinkParams() {}

  std::string video_filename;
  size_t first_frame;
  double fps;

  VideoFileSinkParams(
      const SinkType &stype,
      const FrameType &ftype,
      const std::string &camera_label,
      const std::string &filename,
      size_t start_frame=0,
      double frame_rate=-1.0,
      const std::string &calib_file=std::string(),
      bool return_bgr=false)
    : SinkParams(stype, ftype, camera_label, calib_file, return_bgr),
      video_filename(filename),
      first_frame(start_frame),
      fps(frame_rate)
  {}
};


/** @brief Configuration parameters to "stream" an image sequence (i.e. a folder of images). */
class ImageDirectorySinkParams : public SinkParams
{
public:
  virtual ~ImageDirectorySinkParams() {}

  std::string image_directory;
  size_t first_frame;
  double fps;

  VideoFileSinkParams(
      const SinkType &stype,
      const FrameType &ftype,
      const std::string &camera_label,
      const std::string &folder,
      size_t start_frame=0,
      double frame_rate=-1.0,
      const std::string &calib_file=std::string(),
      bool return_bgr=false)
    : SinkParams(stype, ftype, camera_label, calib_file, return_bgr),
      image_directory(folder),
      first_frame(start_frame),
      fps(frame_rate)
  {}
};


/** @brief Returns a StreamSink wrapper to access video files. */
std::unique_ptr<StreamSink> CreateVideoFileSink(const VideoFileSinkParams &params);


/** @brief Returns a StreamSink wrapper to access a directory full of images. */
std::unique_ptr<StreamSink> CreateImageDirectorySink();

} // namespace best
} // namespace vcp
#endif // __VCP_BEST_FILE_SINK_H__
