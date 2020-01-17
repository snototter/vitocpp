#include "file_sink.h"

#include <thread>
#include <mutex>
#include <atomic>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vcp_utils/timing_utils.h>
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/file_utils.h>
#include <vcp_utils/string_utils.h>
#include <vcp_imutils/matutils.h>

#include "capture.h"

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "file_sink"

namespace vcp
{
namespace best
{
namespace
{
////TODO/** @brief Replays a video at the specified frame rate - does NOT support backwards seeking. */
//class TimedVideoFileSink : public StreamSink
//{
//public:
//  TimedVideoFileSink(const std::string &filename, size_t first_frame,
//                     double fps, bool color_as_bgr, std::unique_ptr<SinkBuffer> sink_buffer)
//    : StreamSink(), continue_capture_(false), capture_(nullptr), image_queue_(std::move(sink_buffer)),
//      filename_(filename), first_frame_(first_frame), frame_rate_(fps), color_as_bgr_(color_as_bgr), eof_(true)
//  {
//  }

//  virtual ~TimedVideoFileSink()
//  {
//    Terminate();
//  }

//  void StartStream() override
//  {
//    if (capture_ && capture_->isOpened())
//      PVT_EXIT("Video file already opened");

//    if (!pvt::utils::file::Exists(filename_))
//      PVT_EXIT("Video file '" << filename_ << "' does not exist!");

//    capture_ = std::unique_ptr<cv::VideoCapture>(new cv::VideoCapture(filename_));
//    if (!capture_->isOpened())
//      PVT_LOG_FAILURE("Cannot open video file: " << filename_);

//    if (!capture_->set(CV_CAP_PROP_POS_FRAMES, static_cast<double>(first_frame_)))
//      PVT_LOG_FAILURE("Cannot jump to specified first frame [" << first_frame_ << "]");

//    eof_ = false;
//    continue_capture_ = true;
//    stream_thread_ = std::thread(&TimedVideoFileSink::Receive, this);
//  }

//  void Terminate() override
//  {
//    if (continue_capture_)
//    {
//      continue_capture_ = false;
//      stream_thread_.join();
//    }
//  }

//  void GetNextFrame(cv::Mat &frame) override
//  {
//    image_queue_mutex_.lock();
//    if (image_queue_->Empty())
//    {
//      frame = cv::Mat();
//    }
//    else
//    {
//      // Retrieve oldest image in queue.
//      frame = image_queue_->Front().clone();
//      image_queue_->PopFront();
//    }
//    image_queue_mutex_.unlock();
//  }

//  int IsAvailable() const override
//  {
//    if (capture_ && !eof_)
//      return 1;
//    return 0;
//  }

//  int IsFrameAvailable() const override
//  {
//    image_queue_mutex_.lock();
//    const bool empty = image_queue_->Empty();
//    image_queue_mutex_.unlock();
//    if (empty)
//      return 0;
//    return 1;
//  }

//private:
//  std::atomic<bool> continue_capture_;
//  std::unique_ptr<cv::VideoCapture> capture_;
//  std::unique_ptr<SinkBuffer> image_queue_;

//  std::string filename_;
//  size_t first_frame_;
//  double frame_rate_;
//  bool color_as_bgr_;
//  bool eof_;

//  std::thread stream_thread_;
//  mutable std::mutex image_queue_mutex_;

//  void Receive()
//  {
//    const int64_t mus_per_frame = static_cast<int64_t>(1000000.0 / frame_rate_);

//    std::chrono::steady_clock::time_point tp_start = std::chrono::steady_clock::now();
//    while (continue_capture_)
//    {
//      // Get next frame
//      cv::Mat frame;
//      if (capture_->grab())
//      {
//        cv::Mat loaded;
//        capture_->retrieve(loaded);
//        if (color_as_bgr_)
//          frame = loaded;
//        else
//        {
//          if (loaded.channels() == 3)
//            cv::cvtColor(loaded, frame, CV_BGR2RGB);
//          else if (loaded.channels() == 4)
//            cv::cvtColor(loaded, frame, CV_BGRA2RGBA);
//          else
//            frame = loaded;
//        }
//      }
//      else
//      {
//        frame = cv::Mat();
//        eof_ = true;
//      }

//      // Push into queue
//      image_queue_mutex_.lock();
//      image_queue_->PushBack(frame.clone());
//      image_queue_mutex_.unlock();
//      const std::chrono::steady_clock::time_point tp_stop = std::chrono::steady_clock::now();
//      const int64_t elapsed = std::chrono::duration_cast<std::chrono::microseconds>(tp_stop - tp_start).count();

//      const int64_t to_sleep = mus_per_frame - elapsed;
//      if (to_sleep > 0)
//        std::this_thread::sleep_for(std::chrono::microseconds(to_sleep));
//      else if (to_sleep < 0)
//      {
//        PVT_LOG_WARNING("TimedVideoSink delayed (Time budget: " << (mus_per_frame/1000.0) << "ms, decoding took " << (elapsed/1000.0) << "ms)");
//      }

//      tp_start = std::chrono::steady_clock::now();
//    }
//    capture_.reset();
//  }
//};


/** @brief Replays a video frame-by-frame. Supports retrieving previous frames. */
class VideoFileSink : public StreamSink
{
public:
  VideoFileSink(const VideoFileSinkParams &params) : StreamSink(),
    capture_(nullptr), params_(params), eof_(true)
  {
    VCP_LOG_DEBUG("VideoFileSink()");
  }

  virtual ~VideoFileSink()
  {
    // cv::VideoCapture destructor will be invoked (and gracefully cleans up) automatically.
    VCP_LOG_DEBUG("~VideoFileSink()");
  }

  bool OpenDevice() override
  {
    VCP_LOG_DEBUG("OpenDevice()");
    if (!vcp::utils::file::Exists(params_.filename))
    {
      VCP_LOG_FAILURE("Video file '" << params_.filename << "' does not exist!");
      return false;
    }
    return true;
  }

  bool StartStreaming() override
  {
    VCP_LOG_DEBUG("StartStreaming()");
    if (capture_ && capture_->isOpened())
    {
      VCP_LOG_FAILURE("Video file '" << params_.filename << "' has already been opened!");
      return false;
    }

    if (capture_)
      capture_.reset();

    capture_ = std::unique_ptr<cv::VideoCapture>(new cv::VideoCapture(params_.filename));
    if (!capture_->isOpened())
    {
      VCP_LOG_FAILURE("Cannot open video file '" << params_.filename << "'");
      return false;
    }

    if (!capture_->set(CV_CAP_PROP_POS_FRAMES, static_cast<double>(params_.first_frame)))
      VCP_LOG_FAILURE("Cannot jump to specified first frame ['" << params_.first_frame << "] of file '" << params_.filename << "'");

    eof_ = false;
    return true;
  }

  bool StopStreaming() override
  {
    VCP_LOG_DEBUG("StopStreaming()");
    eof_ = true;
    if (capture_)
      capture_.release();
    capture_.reset();
    return true;
  }

  bool CloseDevice() override
  {
    VCP_LOG_DEBUG("CloseDevice()");
    return StopStreaming();
  }

  std::vector<cv::Mat> Next() override
  {
    VCP_LOG_DEBUG("Next()");
    std::vector<cv::Mat> frames;
    if (capture_)
    {
      if (capture_->grab())
      {
        cv::Mat loaded;
        capture_->retrieve(loaded);
        if (params_.color_as_bgr)
        {
          frames.push_back(loaded);
        }
        else
        {
          cv::Mat converted;
          if (loaded.channels() == 3)
            cv::cvtColor(loaded, converted, CV_BGR2RGB);
          else if (loaded.channels() == 4)
            cv::cvtColor(loaded, converted, CV_BGRA2RGBA);
          else
            converted = loaded;
          frames.push_back(converted);
        }
      }
      else
      {
        frames.push_back(cv::Mat());
        eof_ = true;
      }
    }
    else
    {
      frames.push_back(cv::Mat());
      eof_ = true;
      VCP_LOG_FAILURE("VideoFileSink is not available for '" << params_.filename << "'");
    }
    return frames;
  }


  std::vector<cv::Mat> Previous() override
  {
    VCP_LOG_DEBUG("Previous()");
    std::vector<cv::Mat> frames;
    if (capture_)
    {
      const double current_frame = capture_->get(CV_CAP_PROP_POS_FRAMES);
      if (current_frame > 1.0)
      {
        // Frame index already points to the next frame (which is not yet loaded, so seeking back must decrease by 2)
        if (capture_->set(CV_CAP_PROP_POS_FRAMES, current_frame - 2.0))
        {
          return Next();
        }
        else
        {
          frames.push_back(cv::Mat());
          VCP_LOG_FAILURE("VideoFileSink cannot seek to frame [" << static_cast<long>(current_frame-2.0) << "] for '" << params_.filename << "'");
        }
      }
      else
      {
        frames.push_back(cv::Mat());
        VCP_LOG_FAILURE("VideoFileSink cannot query current frame for '" << params_.filename << "'");
      }
    }
    else
    {
      frames.push_back(cv::Mat());
      VCP_LOG_FAILURE("VideoFileSink is not available for '" << params_.filename << "'");
    }
    return frames;
  }

  std::vector<cv::Mat> FastForward(size_t num_frames) override
  {
    VCP_LOG_DEBUG("FastForward()");
    std::vector<cv::Mat> frames;
    if (capture_)
    {
      const double current_frame = capture_->get(CV_CAP_PROP_POS_FRAMES);
      // Frame index already points to the next frame (which is not yet loaded, so
      // the skip step must be decreased by 1 (ffwd 1 = next frame, ffwd 2 = skip 1 frame, etc.)
      const double next_frame_index = current_frame + static_cast<double>(num_frames - 1);
      if (capture_->set(CV_CAP_PROP_POS_FRAMES, next_frame_index))
      {
        frames = Next();
      }
      else
      {
        frames.push_back(cv::Mat());
        VCP_LOG_FAILURE("VideoFileSink cannot seek to frame [" << static_cast<long>(next_frame_index) << "] for '" << params_.filename << "'");
      }
    }
    else
    {
      frames.push_back(cv::Mat());
      VCP_LOG_FAILURE("VideoFileSink is not available for '" << params_.filename << "'");
    }
    return frames;
  }


  int IsDeviceAvailable() const override
  {
    VCP_LOG_DEBUG("IsDeviceAvailable()");
    if (vcp::utils::file::Exists(params_.filename))
      return 1;
    return 0;
  }


  int IsFrameAvailable() const override
  {
    VCP_LOG_DEBUG("IsFrameAvailable()");
    // By processing a video frame-by-frame, we don't
    // have a queue but only need to check for EOF.
    if (capture_ && !eof_)
      return 1;
    return 0;
  }

  size_t NumStreams() const override
  {
    return 1;
  }

  SinkType Type(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return SinkType::VIDEO_FILE;
  }

  std::string StreamLabel(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return params_.sink_label;
  }

private:
  std::unique_ptr<cv::VideoCapture> capture_;
  VideoFileSinkParams params_;
  bool eof_;
};


class ImageDirectorySink : public StreamSink
{
public:
  ImageDirectorySink(const ImageDirectorySinkParams &params)
    : StreamSink(), params_(params), frame_idx_(0), load_images_(true)
  {
    VCP_LOG_DEBUG("ImageDirectorySink()");
  }

  virtual ~ImageDirectorySink()
  {
    VCP_LOG_DEBUG("~ImageDirectorySink()");
  }

  bool OpenDevice() override
  {
    VCP_LOG_DEBUG("OpenDevice()");
    if (!vcp::utils::file::IsDir(params_.directory))
    {
      VCP_LOG_FAILURE("Cannot access ''" << params_.directory << "'");
      return false;
    }

    namespace vuff = vcp::utils::file::filename_filter;
    filenames_ = vcp::utils::file::ListDirContents(params_.directory, &vuff::HasImageExtension, true, true, true,
                                      &vuff::CompareFileLengthsAndNames);

    if (filenames_.empty())
    {
      VCP_LOG_DEBUG("No image files within '" << params_.directory << "', now looking for binary matrix dumps (cvm/cvz).");

      load_images_ = false;
      // Check if the directory contains binary mat files.
      filenames_ = vcp::utils::file::ListDirContents(params_.directory, [](const std::string &f) -> bool {
        const std::string cvmat_ext(".cvm");
        const std::string cvzip_ext(".cvz");
        const std::string ext = vcp::utils::file::GetExtension(f);
        return (cvmat_ext.compare(ext) == 0) || (cvzip_ext.compare(ext) == 0);
      }, true, true, true, &vuff::CompareFileLengthsAndNames);

      if (filenames_.empty())
      {
        VCP_LOG_FAILURE("ImageDirectorySink cannot stream empty folder ''" << params_.directory << "'");
        return false;
      }
    }
    else
    {
      VCP_LOG_DEBUG("Image directory '" << params_.directory << "' contains " << filenames_.size() << " images.");
      load_images_ = true;
    }
    return true;
  }

  bool CloseDevice() override
  {
    VCP_LOG_DEBUG("CloseDevice()");
    filenames_.clear();
    return true;
  }

  bool StartStreaming() override
  {
    VCP_LOG_DEBUG("StartStreaming()");
    frame_idx_ = params_.first_frame;
    return frame_idx_ < filenames_.size();
  }

  bool StopStreaming() override
  {
    VCP_LOG_DEBUG("StopStreaming()");
    // Set frame pointer to invalid index
    frame_idx_ = filenames_.size();
    return true;
  }


  std::vector<cv::Mat> Next() override
  {
    std::vector<cv::Mat> frames;
    VCP_LOG_DEBUG("Next() '" << params_.directory << "' loading next frame " << (frame_idx_+1) << "/" << filenames_.size());
    if (frame_idx_ < filenames_.size())
    {
      cv::Mat loaded;
      if (load_images_)
      {
        loaded = cv::imread(vcp::utils::file::FullFile(params_.directory, filenames_[frame_idx_]), cv::IMREAD_UNCHANGED);
        if (params_.color_as_bgr)
        {
          // Default OpenCV behavior
          frames.push_back(loaded);
        }
        else
        {
          cv::Mat converted;
          if (loaded.channels() == 3)
            cv::cvtColor(loaded, converted, CV_BGR2RGB);
          else if (loaded.channels() == 4)
            cv::cvtColor(loaded, converted, CV_BGRA2RGBA);
          else
            converted = loaded;
          frames.push_back(converted);
        }
      }
      else
        frames.push_back(vcp::imutils::LoadMat(vcp::utils::file::FullFile(params_.directory, filenames_[frame_idx_])));

      ++frame_idx_;
    }
    else
    {
      frames.push_back(cv::Mat());
    }
    return frames;
  }


  std::vector<cv::Mat> Previous() override
  {
    VCP_LOG_DEBUG("Previous() '" << params_.directory << "' loading previous frame " << (frame_idx_-2) << "/" << filenames_.size());
    if (frame_idx_ > 1)
    {
      // Frame index already points to the next frame, i.e.
      // the one which is not yet loaded, so seeking back
      // must actually decrease by 2.
      frame_idx_ -= 2;
      return Next();
    }
    else
    {
      std::vector<cv::Mat> empty = {cv::Mat()};
      VCP_LOG_WARNING("ImageDirectorySink cannot seek back beyond the first frame for '" << params_.directory << "'");
      return empty;
    }
  }

  std::vector<cv::Mat> FastForward(size_t num_frames) override
  {
    VCP_LOG_DEBUG("FastForward(" << num_frames << ")");
    frame_idx_ += (num_frames-1);
    return Next();
  }


  int IsDeviceAvailable() const override
  {
    VCP_LOG_DEBUG("IsDeviceAvailable()");
    if (filenames_.empty())
      return 0;
    return 1;
  }

  int IsFrameAvailable() const override
  {
    VCP_LOG_DEBUG("IsFrameAvailable()");
    // Processing an image directory frame-by-frame, we don't
    // have an image queue (so only check whether images are
    // left to be loaded)
    if (frame_idx_ < filenames_.size())
      return 1;
    return 0;
  }

  size_t NumStreams() const override
  {
    return 1;
  }

  SinkType Type(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return SinkType::IMAGE_DIR;
  }

  std::string StreamLabel(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return params_.sink_label;
  }

private:
  ImageDirectorySinkParams params_;
  size_t frame_idx_;
  bool load_images_;                    /**< Whether we should load (and decode) images or raw/zipped cv::Mat files. */
  std::vector<std::string> filenames_;  /**< Stores the relative filenames. */
};
} // namespace


/** @brief Given the cameraXX.type (configuration) parameter, checks if the configuration belongs to a VideoFileSink. */
bool IsVideoFileSink(const std::string &type_param)
{
  std::string type(type_param);
  vcp::utils::string::ToLower(type);
  if (type.compare("video") == 0
      || type.compare("video-file") == 0
      || type.compare("file") == 0
      || type.compare("videofile") == 0)
  {
    return true;
  }
  return false;
}

/** @brief Given the cameraXX.type (configuration) parameter, checks if the configuration belongs to a VideoFileSink. */
bool IsImageDirectorySink(const std::string &type_param)
{
  std::string type = vcp::utils::string::Replace(type_param, "-", "_");
  vcp::utils::string::ToLower(type);
  if (type.compare("img_dir") == 0
      || type.compare("image_dir") == 0
      || type.compare("image_directory") == 0
      || type.compare("img_sequence") == 0)
  {
    return true;
  }
  return false;
}


VideoFileSinkParams VideoFileSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param)
{
  std::vector<std::string> configured_keys = config.ListConfigGroupParameters(cam_param);
  const SinkParams sink_params = ParseBaseSinkParamsFromConfig(config, cam_param, configured_keys);

  std::string file;
  if (config.SettingExists(cam_param + ".video_file"))
  {
    file = config.GetString(cam_param + ".video_file");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "video_file"), configured_keys.end());
  }
  else if (config.SettingExists(cam_param + ".file"))
  {
    file = config.GetString(cam_param + ".file");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "file"), configured_keys.end());
  }
  else if (config.SettingExists(cam_param + ".filename"))
  {
    file = config.GetString(cam_param + ".filename");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "filename"), configured_keys.end());
  }
  else if (config.SettingExists(cam_param + ".video"))
  {
    file = config.GetString(cam_param + ".video");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "video"), configured_keys.end());
  }
  else
  {
    VCP_ERROR("Cannot find video parameter for '" << cam_param << "'. Use either 'file', 'filename', 'video' or 'video_file' to specify it.");
  }

  const size_t start_frame = static_cast<size_t>(GetOptionalIntFromConfig(config, cam_param, "first_frame", 0));
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "first_frame"), configured_keys.end());

  const double frame_rate = GetOptionalDoubleFromConfig(config, cam_param, "fps", -1.0);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "fps"), configured_keys.end());

  WarnOfUnusedParameters(cam_param, configured_keys);

  return VideoFileSinkParams(sink_params, file, start_frame, frame_rate);
}

ImageDirectorySinkParams ImageDirectorySinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param)
{
  std::vector<std::string> configured_keys = config.ListConfigGroupParameters(cam_param);
  const SinkParams sink_params = ParseBaseSinkParamsFromConfig(config, cam_param, configured_keys);

  const std::string folder = config.GetString(cam_param + ".directory");
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "directory"), configured_keys.end());

  const size_t start_frame = static_cast<size_t>(GetOptionalIntFromConfig(config, cam_param, "first_frame", 0));
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "first_frame"), configured_keys.end());

  const double frame_rate = GetOptionalDoubleFromConfig(config, cam_param, "fps", -1.0);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "fps"), configured_keys.end());

  WarnOfUnusedParameters(cam_param, configured_keys);

  return ImageDirectorySinkParams(sink_params, folder, start_frame, frame_rate);
}

std::unique_ptr<StreamSink> CreateSink(const VideoFileSinkParams &params)
{
  if (params.fps > 0.0)
  {
    VCP_ERROR("Not yet implemented");
  }
  else
    return std::unique_ptr<VideoFileSink>(new VideoFileSink(params));
}


std::unique_ptr<StreamSink> CreateSink(const ImageDirectorySinkParams &params)
{
  if (params.fps > 0.0)
  {
    VCP_ERROR("Not yet implemented");
  }
  else
    return std::unique_ptr<ImageDirectorySink>(new ImageDirectorySink(params));
}


} // namespace icc
} // namespace pvt
