#include "webcam_sink.h"

#include <thread>
#include <mutex>
#include <atomic>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifdef VCP_BEST_DEBUG_FRAMERATE
    #include <chrono>
    #include <iomanip>
#endif // VCP_BEST_DEBUG_FRAMERATE

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/file_utils.h>
#include <vcp_imutils/opencv_compatibility.h>


/**
 * Notes:
 * I tried to implement it without OpenCV once (using FFMPEG to be more
 * flexible/closer to the device). However, there we observed
 * rolling shutter-like effects when streaming from HD webcams.
 * OpenCV didn't have these issues.
 */

#ifdef WITH_FFMPEG
#ifdef __GNUC__
  #define DISABLE_DEPRECATED_WARNING\
    _Pragma("GCC diagnostic push")\
    _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")
  #define ENABLE_DEPRECATED_WARNING\
    _Pragma("GCC diagnostic pop")
#else // __GNUC__
  #define DISABLE_DEPRECATED_WARNING
  #define ENABLE_DEPRECATED_WARNING
#endif

// Required for libav:
#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif
#ifdef __cplusplus
extern "C"
{
#endif
  #include <libavdevice/avdevice.h>
//  #include <libavcodec/avcodec.h>
  #include <libavformat/avformat.h>
  #include <libswscale/swscale.h>
  #include <libavutil/imgutils.h>
#ifdef __cplusplus
}
#endif
// av_frame_alloc was added in lavc 55.28.1 (not available on my Ubuntu 14.04)
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
  #define av_frame_alloc avcodec_alloc_frame
  #define USE_DEPRECATED_PICTURE_API
#endif
#endif // WITH_FFMPEG

namespace vcp
{
namespace best
{
namespace webcam
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::webcam"


std::ostream &operator<< (std::ostream &out, const WebcamSinkParams &p)
{
  out << p.sink_label << ", device " << p.device_number;
  return out;
}


#ifdef WITH_FFMPEG
#error "This needs to be properly investigated - currently, we observe "rolling shutter"-like effects for HD webcams"
  class WebcamSink : public StreamSink
  {
  public:
    WebcamSink(std::unique_ptr<SinkBuffer> sink_buffer,
               int device_number, const cv::Size &frame_resolution=cv::Size(0,0),
               double fps=-1.0, bool color_as_bgr) : StreamSink(),
      continue_capture_(false), available_(false), image_queue_(std::move(sink_buffer)),
      capture_(nullptr), device_number_(device_number), frame_resolution_(frame_resolution), fps_(fps),
      color_as_bgr_(color_as_bgr), video_stream_index_(-1)
    {
      Init();
    }

    virtual ~WebcamSink()
    {
      Terminate();
      if (codec_context_)
        avcodec_free_context(&codec_context_);
#if LIBAVCODEC_VERSION_INT < 3747941
      if (codec_context_orig_)
        avcodec_close(codec_context_orig_);
#endif
      if (picture_)
        av_free(picture_);
      if (picture_bgr_)
        av_free(picture_bgr_);
      if (buffer_picture_bgr_)
        av_free(buffer_picture_bgr_);
      if (convert_context_)
        sws_freeContext(convert_context_);
      if (format_context_)
        avformat_close_input(&format_context_);
    }

    void StartStream() override
    {
      continue_capture_ = true;
      stream_thread_ = std::thread(&WebcamSink::Receive, this);
    }

    void Terminate() override
    {
      if (continue_capture_)
      {
        continue_capture_ = false;
        stream_thread_.join();
      }
    }

    void GetNextFrame(cv::Mat &frame) override
    {
      image_queue_mutex_.lock();
      if (image_queue_->Empty())
      {
        frame = cv::Mat();
      }
      else
      {
        // Retrieve oldest image in queue.
        frame = image_queue_->Front().clone();
        image_queue_->PopFront();
      }
      image_queue_mutex_.unlock();
    }


    int IsAvailable() const override
    {
      return available_;
    }

  private:
    std::atomic<bool> continue_capture_;
    std::atomic<bool> available_;
    std::unique_ptr<SinkBuffer> image_queue_;
    std::unique_ptr<cv::VideoCapture> capture_;
    int device_number_;
    cv::Size frame_resolution_;
    double fps_;
    bool color_as_bgr_;
    int video_stream_index_;

    std::thread stream_thread_;
    std::mutex image_queue_mutex_;


    AVCodecContext  *codec_context_;
#if LIBAVCODEC_VERSION_INT < 3747941
    AVCodecContext *codec_context_orig_;
#endif
    AVFormatContext *format_context_;
    AVCodec * codec_;
    AVInputFormat *input_format_;
//      AVFrame *pFrame = NULL, *pFrameRGB = NULL;
//    AVFrame *av_frame_ = NULL;
    AVFrame *picture_;
    AVFrame *picture_bgr_;
    uint8_t *buffer_picture_bgr_;
    struct SwsContext *convert_context_;

    void Init()
    {
      codec_context_ = nullptr;
#if LIBAVCODEC_VERSION_INT < 3747941
      codec_context_orig_ = nullptr;
#endif
      format_context_ = nullptr;
      codec_ = nullptr;
      input_format_ = nullptr;
  //      AVFrame *pFrame = NULL, *pFrameRGB = NULL;
  //    AVFrame *av_frame_ = NULL;
      picture_ = nullptr;
      picture_bgr_ = nullptr;
      buffer_picture_bgr_ = nullptr;
      convert_context_ = nullptr;

      avdevice_register_all();
      avcodec_register_all();
//      av_register_all();


      AVDictionary *options = NULL;
      if (IsValidSize(frame_resolution_))
      {
        const std::string video_size_opt = vcp::utils::string::ToStr(frame_resolution_.width) + "x" + vcp::utils::string::ToStr(frame_resolution_.height);
        av_dict_set(&options, "video_size", video_size_opt.c_str(), 0);
      }
      if (fps_ > 0.0)
      {
        const std::string fps_opt = vcp::utils::string::ToStrPrec(fps_, 1);
        av_dict_set(&options, "framerate", fps_opt.c_str(), 0);
      }

#ifdef _WIN32
      const std::string input_format_str("dshow");
#elif defined(__linux__) || defined(__unix__)
      const std::string input_format_str("v4l2");
#else
  #error "I don't know the AVInputFormat for your platform"
#endif
      input_format_ = av_find_input_format(input_format_str.c_str());
      if (!input_format_) //data->iformat
      {
        VCP_ERROR("Could not find input format: " << input_format_str);
      }

      format_context_ = avformat_alloc_context();
      if(!format_context_)
      {
        VCP_ERROR("Could not allocate AVFormatContext");
      }

      if(avformat_open_input(&format_context_, "/dev/video0", input_format_, options == NULL ? NULL : &options) != 0)
      {
        VCP_ERROR("Could not open input device '" << "TODO" << "'");
      }
//      if(avformat_find_stream_info(data->pFormatCtx, &options) < 0)
//      {
//          I3D_LOG(i3d::error) << "initUVCRGB(): could not find stream info of " << filenameSrc;
//          return false;
//      }

//      if (av_find_stream_info(format_context_) < 0)
      if (avformat_find_stream_info(format_context_, NULL) < 0)
        VCP_ERROR("Cannot find stream information for the format context");

      av_dump_format(format_context_, 0, "/dev/video0", 0); //TODO replace by device_name string
      for(unsigned i = 0; i < format_context_->nb_streams; i++)
      {
#if LIBAVCODEC_VERSION_INT < 3747941
        if(format_context_->streams[i]->codec->coder_type == AVMEDIA_TYPE_VIDEO)
#else
        if(format_context_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
#endif
        {
          video_stream_index_ = static_cast<int>(i);
          break;
        }
      }


//      if(data->video_stream_index == -1)
//      {
//          I3D_LOG(i3d::error) << "initUVCRGB(): couldn't find a stream of type AVMEDIA_TYPE_VIDEO in " << filenameSrc.c_str();
//          return false;
//      }
      AVCodecID codec_id;
#if LIBAVCODEC_VERSION_INT < 3747941
      codec_context_orig_ = format_context_->streams[video_stream_index_]->codec;
      codec_id = codec_context_orig_->codec_id;
#else
      // Get the stream's codec's parameters and find a matching decoder
      AVCodecParameters* cparams = format_context_->streams[video_stream_index_]->codecpar;
      codec_id = cparams->codec_id;
#endif
      codec_ = avcodec_find_decoder(codec_id);

      if(!codec_)
        VCP_ERROR("Cannot find a suitable decoder for the codec");

#if LIBAVCODEC_VERSION_INT < 3747941
      // Copy context
      codec_context_ = avcodec_alloc_context3(codec_);
      if (avcodec_copy_context(codec_context_, codec_context_orig_) != 0)
        VCP_ERROR("Cannot copy codec context");
#else
      // Create a context for the codec, using the existing parameters
      codec_context_ = avcodec_alloc_context3(codec_);
      if (avcodec_parameters_to_context(codec_context_, cparams) < 0)
        VCP_ERROR("Cannot create AV codec context from AVCodecParameters");
#endif

      if(avcodec_open2(codec_context_, codec_, NULL) < 0)
        VCP_ERROR("Cannot open decoder for codec");

      VCP_LOG_INFO("Codec says frames are " << codec_context_->width << "x" << codec_context_->height);
      frame_resolution_ = cv::Size(codec_context_->width, codec_context_->height);

      picture_ = av_frame_alloc();
      picture_bgr_ = av_frame_alloc();
      if(!picture_ || !picture_bgr_)
        VCP_ERROR("Could not allocate AVFrame");


#ifdef USE_DEPRECATED_PICTURE_API
      // Required on my 14.04 standard installation
      int bytes = avpicture_get_size(AV_PIX_FMT_RGB24, frame_resolution_.width, frame_resolution_.height);
#else
      int bytes = av_image_get_buffer_size(AV_PIX_FMT_RGB24, frame_resolution_.width, frame_resolution_.height, 32); // Alignment is recommended to be 32, see https://stackoverflow.com/a/35682306
#endif
      buffer_picture_bgr_ = (uint8_t *)av_malloc(bytes*sizeof(uint8_t));
      if (!buffer_picture_bgr_)
        VCP_ERROR("Cannot allocate buffer for AVFrame");

#ifdef USE_DEPRECATED_PICTURE_API
      avpicture_fill((AVPicture *)picture_bgr_, buffer_picture_bgr_, AV_PIX_FMT_RGB24, frame_resolution_.width, frame_resolution_.height);
#else
      av_image_fill_arrays(picture_bgr_->data, picture_bgr_->linesize, buffer_picture_bgr_, AV_PIX_FMT_RGB24, frame_resolution_.width, frame_resolution_.height, 32);
#endif

      convert_context_ = sws_getCachedContext(NULL, codec_context_->width, codec_context_->height, codec_context_->pix_fmt,
                                              codec_context_->width, codec_context_->height, AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL,NULL);
      if (!convert_context_)
        VCP_ERROR("Cannot initialize SWS conversion context");
    }

    void Receive()
    {
      AVPacket packet;
      int result, frame_finished;
      available_ = true;
      while (continue_capture_)
      {
        result = av_read_frame(format_context_, &packet);
        if (result >= 0)
        {
          if (packet.stream_index == video_stream_index_)
          {
            DISABLE_DEPRECATED_WARNING
            avcodec_decode_video2(codec_context_, picture_, &frame_finished, &packet);
            ENABLE_DEPRECATED_WARNING
            if (frame_finished)
            {
              cv::Mat cvframe;
              int slice_height = sws_scale(convert_context_, picture_->data, picture_->linesize, 0,
                                           picture_->height, picture_bgr_->data, picture_bgr_->linesize);
              if (slice_height > 0)
                cvframe = cv::Mat(picture_->height, picture_->width, CV_8UC3, picture_bgr_->data[0], picture_bgr_->linesize[0]);

              if (color_as_bgr_)
              {
                VCP_LOG_FAILURE("Color conversion not yet implemented!!!");
              }
              // An empty frame denotes a conversion/stream error
              image_queue_mutex_.lock();
              image_queue_->PushBack(cvframe.clone());
              image_queue_mutex_.unlock();
            }
          }
#ifdef USE_DEPRECATED_PICTURE_API
          av_free_packet(&packet);
#else
          av_packet_unref(&packet);
#endif
        }
      }
      available_ = false;
    }
  };
#else // Non-ffmpeg version below:

class WebcamSink : public StreamSink
{
public:
  WebcamSink(std::unique_ptr<SinkBuffer> sink_buffer,
             const WebcamSinkParams &params) : StreamSink(),
    continue_capture_(false), image_queue_(std::move(sink_buffer)),
    capture_(nullptr), params_(params)
  {
#ifdef VCP_BEST_DEBUG_FRAMERATE
    previous_frame_timepoint_ = std::chrono::high_resolution_clock::now();
    ms_between_frames_ = -1.0;
#endif // VCP_BEST_DEBUG_FRAMERATE
  }

  virtual ~WebcamSink()
  {
    CloseDevice();
  }

  bool OpenDevice() override
  {
    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Opening webcam: " << params_);

    if (capture_ && capture_->isOpened())
    {
      VCP_LOG_FAILURE("Webcam #" << params_.device_number << " already opened - ignoring OpenDevice() call");
      return false;
    }

    if (params_.device_number < 0)
    {
#if defined(__linux__) || defined(__unix__)
      const auto devices = vcp::utils::file::ListDirContents("/dev", [](const std::string &f) -> bool {
        if (vcp::utils::string::StartsWith(f, "video"))
        {
          // We need to skip RealSense devices, as we cannot open them with default OpenCV settings.
          // Similarly, we skip Azure Kinect devices, as we had some troubles with frequently failing
          // color streams.
          const std::string name = vcp::utils::file::SlurpAsciiFile(
                vcp::utils::file::FullFile("/sys/class/video4linux/", vcp::utils::file::FullFile(f, "name")));
          // Return false for the directory list if the hardware info contains "RealSense" or "Azure Kinect"
          return name.find("RealSense") == std::string::npos && name.find("Azure Kinect") == std::string::npos;
        }
        return false;
      }, true, true, true, &vcp::utils::file::filename_filter::CompareFileLengthsAndNames);
      if (devices.empty())
      {
        VCP_LOG_FAILURE("No webcam is connected to your PC!");
        return false;
      }

      params_.device_number = std::atoi(devices[0].substr(5).c_str()); // Strip the beginning 'video'
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Trying to open webcam /dev/" << devices[0]);
#else
      VCP_LOG_FAILURE("Searching for a webcam is only supported on unix-based operating systems!");
      return false
#endif
    }

    if (capture_)
      capture_->open(params_.device_number);
    else
      capture_ = std::unique_ptr<cv::VideoCapture>(new cv::VideoCapture(params_.device_number));

    if (!capture_->isOpened())
    {
      VCP_LOG_FAILURE("Cannot open webcam: " << params_.device_number);
      return false;
    }

    // Set the requested image size:
    if (params_.resolution.width > 0 && params_.resolution.height > 0)
    {
#if CV_MAJOR_VERSION < 3
      capture_->set(CV_CAP_PROP_FRAME_WIDTH, params_.resolution.width);
      capture_->set(CV_CAP_PROP_FRAME_HEIGHT, params_.resolution.height);
#else
      capture_->set(cv::CAP_PROP_FRAME_WIDTH, params_.resolution.width);
      capture_->set(cv::CAP_PROP_FRAME_HEIGHT, params_.resolution.height);
#endif
    }

    // Set frame rate if requested:
    if (params_.fps > 0.0)
    {
#if CV_MAJOR_VERSION < 3
      capture_->set(CV_CAP_PROP_FPS, params_.fps);
#else
      capture_->set(cv::CAP_PROP_FPS, params_.fps);
#endif
    }
    return true;
  }

  bool CloseDevice() override
  {
    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Closing webcam #" << params_.device_number);
    capture_.reset();
    return true;
  }

  bool StartStreaming() override
  {
    if (!capture_ || !capture_->isOpened())
      OpenDevice();

    if (continue_capture_)
    {
      VCP_LOG_FAILURE("Streaming thread already running, ignoring StartStreaming() call.");
      return false;
    }

    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Starting streaming thread for webcam #" << params_.device_number);
    continue_capture_ = true;
    stream_thread_ = std::thread(&WebcamSink::Receive, this);
    return true;
  }


  bool StopStreaming() override
  {
    if (continue_capture_)
    {
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Waiting for streaming thread to disconnect from webcam #" << params_.device_number);
      continue_capture_ = false;
      stream_thread_.join();
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Disconnected from webcam #" << params_.device_number);
    }
    return true;
  }


  std::vector<cv::Mat> Next() override
  {
    std::vector<cv::Mat> frames;
    cv::Mat frame;
    image_queue_mutex_.lock();
    if (image_queue_->Empty())
    {
      frame = cv::Mat();
    }
    else
    {
      // Retrieve oldest image in queue.
      frame = image_queue_->Front().clone();
      image_queue_->PopFront();
    }
    image_queue_mutex_.unlock();
    frames.push_back(frame);
    return frames;
  }


  int IsDeviceAvailable() const override
  {
    if (capture_ && capture_->isOpened())
      return 1;
    return 0;
  }

  int IsFrameAvailable() const override
  {
    image_queue_mutex_.lock();
    const bool empty = image_queue_->Empty();
    image_queue_mutex_.unlock();
    if (empty)
      return 0;
    return 1;
  }

  size_t NumAvailableFrames() const override
  {
    return static_cast<int>(IsFrameAvailable());
  }

  size_t NumStreams() const override
  {
    return 1;
  }

  FrameType FrameTypeAt(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return params_.frame_type;
  }

  std::string StreamLabel(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return params_.sink_label;
  }

  SinkParams SinkParamsAt(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return params_;
  }

  size_t NumDevices() const override
  {
    return 1;
  }

private:
  std::atomic<bool> continue_capture_;
  std::unique_ptr<SinkBuffer> image_queue_;
  std::unique_ptr<cv::VideoCapture> capture_;
  WebcamSinkParams params_;

#ifdef VCP_BEST_DEBUG_FRAMERATE
  std::chrono::high_resolution_clock::time_point previous_frame_timepoint_;
  double ms_between_frames_;
#endif // VCP_BEST_DEBUG_FRAMERATE

  std::thread stream_thread_;
  mutable std::mutex image_queue_mutex_;

  void Receive()
  {
    while (continue_capture_)
    {
      // Get next frame
      cv::Mat retrieved, converted;
      if (capture_->grab())
      {
        capture_->retrieve(retrieved);

        if (params_.color_as_bgr || retrieved.empty())
          converted = retrieved;
        else
          cv::cvtColor(retrieved, converted, CV_BGR2RGB);

        // Push into queue
        image_queue_mutex_.lock();
        image_queue_->PushBack(converted.clone());
        image_queue_mutex_.unlock();

#ifdef VCP_BEST_DEBUG_FRAMERATE
        const std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(now - previous_frame_timepoint_);
        previous_frame_timepoint_ = now;
        const double ms_ema_alpha = 0.1;

        if (ms_between_frames_ < 0.0)
          ms_between_frames_ = duration.count();
        else
          ms_between_frames_ = ms_ema_alpha * duration.count() + (1.0 - ms_ema_alpha) * ms_between_frames_;

        VCP_LOG_DEBUG_DEFAULT("WebcamSink #" << params_.device_number << " received frame after " << std::fixed << std::setw(5) << duration.count() << " ms, "
                            << std::setw(5) << (1000.0 / ms_between_frames_) << " fps");
#endif // VCP_BEST_DEBUG_FRAMERATE
      }
    }
    capture_.reset();
  }
};
#endif

bool IsWebcamSink(const std::string &type_param)
{
  std::string type(type_param);
  vcp::utils::string::ToLower(type);
  if (type.compare("webcam") == 0
      || type.compare("web_cam") == 0
      || type.compare("web-cam") == 0)
  {
    return true;
  }
  return false;
}


WebcamSinkParams WebcamSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param)
{
  std::vector<std::string> configured_keys = config.ListConfigGroupParameters(cam_param);
  const SinkParams sink_params = ParseBaseSinkParamsFromConfig(config, cam_param, configured_keys);

  int device;
  if (config.SettingExists(cam_param + ".device"))
  {
    device = config.GetInteger(cam_param + ".device");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "device"), configured_keys.end());
  }
  else if (config.SettingExists(cam_param + ".device_number"))
  {
    device = config.GetInteger(cam_param + ".device_number");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "device_number"), configured_keys.end());
  }
  else if (config.SettingExists(cam_param + ".number"))
  {
    device = config.GetInteger(cam_param + ".number");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "number"), configured_keys.end());
  }
  else
  {
    VCP_ERROR("Cannot find device parameter for '" << cam_param << "'. Use either 'device', 'device_number' or 'number' to specify it.");
  }

  cv::Size resolution;
  const std::string key_width = cam_param + ".width";
  const std::string key_height = cam_param + ".height";
  if (config.SettingExists(key_width) || config.SettingExists(key_height))
  {
    if (config.SettingExists(key_width) && config.SettingExists(key_height))
    {
      resolution.width = config.GetInteger(key_width);
      resolution.height = config.GetInteger(key_height);
    }
    else
    {
      VCP_LOG_FAILURE("You have to configure both the width and height, not only one!");
    }
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "width"), configured_keys.end());
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "height"), configured_keys.end());
  }

  const double frame_rate = GetOptionalDoubleFromConfig(config, cam_param, "fps", -1.0);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "fps"), configured_keys.end());

  WarnOfUnusedParameters(cam_param, configured_keys);

  return WebcamSinkParams(sink_params, device, resolution, frame_rate);
}


std::unique_ptr<StreamSink> CreateBufferedWebcamSink(
    const WebcamSinkParams &params,
    std::unique_ptr<SinkBuffer> sink_buffer)
{
  return std::unique_ptr<WebcamSink>(new WebcamSink(std::move(sink_buffer), params));
}

} // namespace webcam
} // namespace best
} // namespace vcp
