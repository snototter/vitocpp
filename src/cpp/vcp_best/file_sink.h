#ifndef __VCP_BEST_FILE_SINK_H__
#define __VCP_BEST_FILE_SINK_H__

#include <string>
#include <memory>
#include <exception>
#include "sink.h"

namespace vcp
{
namespace best
{
/** @brief Media playback from disk, i.e. video & image sequence streaming. */
namespace file
{
/** @brief Configuration parameters to "stream" from a video file. */
struct VideoFileSinkParams : public SinkParams
{
  std::string filename;
  size_t first_frame;
  double fps;

  VideoFileSinkParams(
      const SinkParams &sink_params,
      const std::string &video_file,
      size_t start_frame=0,
      double frame_rate=-1.0)
    : SinkParams(sink_params),
      filename(video_file),
      first_frame(start_frame),
      fps(frame_rate)
  {}
};


/** @brief Configuration parameters to "stream" an image sequence (i.e. a folder of images). */
struct ImageDirectorySinkParams : public SinkParams
{
  std::string directory;
  size_t first_frame;
  double fps;

  ImageDirectorySinkParams(
      const SinkParams &sink_params,
      const std::string &folder,
      size_t start_frame=0,
      double frame_rate=-1.0)
    : SinkParams(sink_params),
      directory(folder),
      first_frame(start_frame),
      fps(frame_rate)
  {}
};

/** @brief Given the cameraXX.type (configuration) parameter, checks if the configuration belongs to a VideoFileSink. */
bool IsVideoFileSink(const std::string &type_param);

/** @brief Given the cameraXX.type (configuration) parameter, checks if the configuration belongs to a VideoFileSink. */
bool IsImageDirectorySink(const std::string &type_param);

/** @brief Checks if the given StreamSink is actually an ImageDirectorySink (i.e. can be stepped through). */
bool IsImageDirectorySink(const std::unique_ptr<StreamSink> &ptr);


/** @brief Parses the configuration group "cam_param" into a VideoFileSinkParams configuration. */
VideoFileSinkParams VideoFileSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param);


/** @brief Parses the configuration group "cam_param" into an ImageDirectorySinkParams configuration. */
ImageDirectorySinkParams ImageDirectorySinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param);

//FIXME replace by templated (create buffered sink...) version (and delete sink buffer for seekable videofilesink)
/** @brief Returns a StreamSink wrapper to access video files. */
std::unique_ptr<StreamSink> CreateVideoFileSink(const VideoFileSinkParams &params);


/** @brief Returns a StreamSink wrapper to access a directory full of images. */
std::unique_ptr<StreamSink> CreateImageDirectorySink(const ImageDirectorySinkParams &params);

} // namespace file
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_FILE_SINK_H__
