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
  {}

  virtual ~VideoFileSink()
  {
    // cv::VideoCapture destructor will be invoked (and gracefully cleans up) automatically...
  }

  bool OpenDevice() override
  {
    if (!vcp::utils::file::Exists(params_.video_filename))
    {
      VCP_LOG_FAILURE("Video file '" << params_.video_filename << "' does not exist!");
      return false;
    }
    return true;
  }

  bool StartStreaming() override
  {
    if (capture_ && capture_->isOpened())
    {
      VCP_LOG_FAILURE("Video file '" << params_.video_filename << "' has already been opened!");
      return false;
    }

    if (capture_)
      capture_.reset();

    capture_ = std::unique_ptr<cv::VideoCapture>(new cv::VideoCapture(filename_));
    if (!capture_->isOpened())
    {
      VCP_LOG_FAILURE("Cannot open video file '" << params_.video_filename << "'");
      return false;
    }

    if (!capture_->set(CV_CAP_PROP_POS_FRAMES, static_cast<double>(params_.first_frame)))
      VCP_LOG_FAILURE("Cannot jump to specified first frame ['" << params_.first_frame << "] of file '" << params_.video_filename << "'");

    eof_ = false;
    return true;
  }

  bool StopStreaming() override
  {
    if (capture_)
      capture_.release();
    capture_.reset();
    return true;
  }

  bool CloseDevice() override
  {
    return StopStreaming();
  }

  //FIXME
  void GetNextFrame(cv::Mat &frame) override
  {
    if (capture_ && capture_->grab())
    {
      cv::Mat loaded;
      capture_->retrieve(loaded);
      if (color_as_bgr_)
      {
        frame = loaded;
      }
      else
      {
        if (loaded.channels() == 3)
          cv::cvtColor(loaded, frame, CV_BGR2RGB);
        else if (loaded.channels() == 4)
          cv::cvtColor(loaded, frame, CV_BGRA2RGBA);
        else
          frame = loaded;
      }
    }
    else
    {
      frame = cv::Mat();
      eof_ = true;
    }
  }


  void GetPreviousFrame(cv::Mat &frame) override
  {
    if (capture_)
    {
      const double current_frame = capture_->get(CV_CAP_PROP_POS_FRAMES);
      if (current_frame > 1.0)
      {
        // Frame index already points to the next frame (which is not yet loaded, so seeking back must decrease by 2)
        if (capture_->set(CV_CAP_PROP_POS_FRAMES, current_frame-2.0))
        {
          GetNextFrame(frame);
        }
        else
        {
          frame = cv::Mat();
          PVT_LOG_FAILURE("Cannot seek to frame [" << static_cast<long>(current_frame-2.0) << "] for video " << filename_);
        }
      }
      else
      {
        frame = cv::Mat();
        PVT_LOG_FAILURE("Cannot query current frame for video " << filename_);
      }
    }
    else
    {
      frame = cv::Mat();
      PVT_LOG_FAILURE("Capture is not available for video " << filename_);
    }
  }

  void FastForward(cv::Mat &frame, size_t num_frames) override
  {
    if (capture_)
    {
      const double current_frame = capture_->get(CV_CAP_PROP_POS_FRAMES);
      // Frame index already points to the next frame (which is not yet loaded, so skip step must be decreased by 1 (ffwd 1 = next frame, ffwd 2 = skip 1 frame, etc.)
      const double next_frame_index = current_frame + static_cast<double>(num_frames-1);
      if (capture_->set(CV_CAP_PROP_POS_FRAMES, next_frame_index))
      {
        GetNextFrame(frame);
      }
      else
      {
        frame = cv::Mat();
        PVT_LOG_FAILURE("Cannot seek to frame [" << next_frame_index << "] for video " << filename_);
      }
    }
    else
    {
      frame = cv::Mat();
      PVT_LOG_FAILURE("Capture is not available for video " << filename_);
    }
  }


  int IsAvailable() const override
  {
    if (capture_ && !eof_)
      return 1;
    return 0;
  }

  int IsFrameAvailable() const override
  {
    return IsAvailable(); // Processing a video frame-by-frame, we don't have a queue but only need to check for EOF
  }

private:
  std::unique_ptr<cv::VideoCapture> capture_;
  VideoFileSinkParams params_;
  bool eof_;
};


class ImageDirectorySink : public StreamSink
{
public:
  ImageDirectorySink(const std::string &directory, size_t first_frame, bool color_as_bgr) : StreamSink(),
    directory_(directory), frame_idx_(0), first_frame_(first_frame), color_as_bgr_(color_as_bgr)
  {
    if (!pvt::utils::file::IsDir(directory_))
      PVT_ABORT("Image directory '" << directory_ << "' does not exist!");

    namespace puff = pvt::utils::file::filename_filter;
    filenames_ = pvt::utils::file::ListDirContents(directory_, &puff::HasImageExtension, true, true, true,
                                      &puff::CompareFileLengthsAndNames);

    if (filenames_.empty())
    {
#ifdef PVT_SHOW_DEBUG_OUTPUT
      PVT_LOG_INFO("No image files within '" << directory_ << "', now looking for binary matrix dumps (cvm/cvz).");
#endif // PVT_SHOW_DEBUG_OUTPUT

      load_images_ = false;
      // Check if the directory contains binary mat files.
      filenames_ = pvt::utils::file::ListDirContents(directory_, [](const std::string &f) -> bool {
        const std::string cvmat_ext(".cvm");
        const std::string cvzip_ext(".cvz");
        const std::string ext = pvt::utils::file::GetExtension(f);
        return (cvmat_ext.compare(ext) == 0) || (cvzip_ext.compare(ext) == 0);
      }, true, true, true, &puff::CompareFileLengthsAndNames);
      if (filenames_.empty())
      {
        PVT_EXIT("Image directory '" << directory_ << "' is empty!");
      }
    }
    else
    {
#ifdef PVT_SHOW_DEBUG_OUTPUT
      PVT_LOG_INFO("Image directory '" << directory_ << "' contains " << filenames_.size() << " images.");
#endif // PVT_SHOW_DEBUG_OUTPUT
      load_images_ = true;
    }
  }

  virtual ~ImageDirectorySink()
  {
  }

  void StartStream() override
  {
    frame_idx_ = first_frame_;
  }

  void Terminate() override
  {
    frame_idx_ = first_frame_;
  }

  void GetNextFrame(cv::Mat &frame) override
  {
#ifdef PVT_SHOW_DEBUG_OUTPUT
      PVT_LOG_INFO("Image directory '" << directory_ << "' loading next frame " << (frame_idx_+1) << "/" << filenames_.size());
#endif // PVT_SHOW_DEBUG_OUTPUT
    if (frame_idx_ < filenames_.size())
    {
      cv::Mat loaded;
      if (load_images_)
      {
        loaded = cv::imread(pvt::utils::file::FullFile(directory_, filenames_[frame_idx_]), cv::IMREAD_UNCHANGED);
        if (color_as_bgr_)
        {
          // Default OpenCV behavior
          frame = loaded;
        }
        else
        {
          if (loaded.channels() == 3)
            cv::cvtColor(loaded, frame, CV_BGR2RGB);
          else if (loaded.channels() == 4)
            cv::cvtColor(loaded, frame, CV_BGRA2RGBA);
          else
            frame = loaded;
        }
      }
      else
        frame = pvt::imutils::LoadMat(pvt::utils::file::FullFile(directory_, filenames_[frame_idx_]));

      ++frame_idx_;
    }
    else
    {
      frame = cv::Mat();
    }
  }


  void GetPreviousFrame(cv::Mat &frame) override
  {
#ifdef PVT_SHOW_DEBUG_OUTPUT
      PVT_LOG_INFO("Image directory '" << directory_ << "' loading previous frame " << (frame_idx_-2) << "/" << filenames_.size());
#endif // PVT_SHOW_DEBUG_OUTPUT
    if (frame_idx_ > 1)
    {
      frame_idx_-=2; // Frame index already points to the next frame (which is not yet loaded, so seeking back must decrease by 2)
    }
    else
    {
      PVT_LOG_WARNING("Cannot seek back beyond the first frame for image directory " << directory_);
    }
    GetNextFrame(frame);
  }

  void FastForward(cv::Mat &frame, size_t num_frames) override
  {
    frame_idx_ += (num_frames-1);
    GetNextFrame(frame);
  }


  int IsAvailable() const override
  {
    if (!filenames_.empty() && frame_idx_ < filenames_.size())
      return 1;

#ifdef PVT_SHOW_DEBUG_OUTPUT
      PVT_LOG_INFO("Image directory '" << directory_ << "' is NOT available (about to load fnr " << frame_idx_ << ", total " << filenames_.size() << ")");
#endif // PVT_SHOW_DEBUG_OUTPUT
    return 0;
  }

  int IsFrameAvailable() const override
  {
    return IsAvailable(); // Processing an image directory frame-by-frame, we don't have an image queue (so only check whether images are left to be loaded)
  }

private:
  std::string directory_;
  size_t frame_idx_;
  size_t first_frame_;
  bool color_as_bgr_;
  std::vector<std::string> filenames_; // Relative filenames
  bool load_images_;
};
} // namespace

std::unique_ptr<StreamSink> CreateVideoFileSink(const std::string &video_filename, size_t first_frame, double fps, bool color_as_bgr)
{
  if (fps > 0.0)
    return std::unique_ptr<TimedVideoFileSink>(new TimedVideoFileSink(video_filename, first_frame, fps, color_as_bgr, CreateCircularStreamSinkBuffer<PVT_ICC_CIRCULAR_SINK_BUFFER_CAPACITY>()));
  else
    return std::unique_ptr<VideoFileSink>(new VideoFileSink(video_filename, first_frame, color_as_bgr));
}

//TODO a TimedImageDirectorySink would be nice-to-have (needs additional fps parameter)
std::unique_ptr<StreamSink> CreateImageDirectorySink(const std::string &image_directory, size_t first_frame, bool color_as_bgr)
{
  return std::unique_ptr<ImageDirectorySink>(new ImageDirectorySink(image_directory, first_frame, color_as_bgr));
}

} // namespace icc
} // namespace pvt
