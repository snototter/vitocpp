#include "http_mjpeg_sink.h"

#include <thread>
#include <mutex>
#include <atomic>

// TODO add DEBUG FRAMERATE code

#if CV_VERSION_MAJOR < 3
    #include <opencv2/highgui/highgui.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/highgui.hpp>
    #include <opencv2/imgproc.hpp>
#endif

#include "curl_file_handling.h"
#include <vcp_utils/vcp_error.h>

namespace vcp
{
namespace best
{
namespace ipcam
{
namespace http
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::ipcam::http"
/**
 * @brief The MJPEG over HTTP stream sink
 */
class HttpMjpegSink: public StreamSink
{
public:
  HttpMjpegSink(const IpCameraSinkParams &params, std::unique_ptr<SinkBuffer> sink_buffer) : StreamSink(),
    params_(params), mjpg_stream_(nullptr), mjpg_multi_handle_(nullptr), continue_stream_(false),
    is_stream_available_(false), image_queue_(std::move(sink_buffer))
  {
    VCP_LOG_DEBUG("HttpMjpegSink::HttpMjpegSink()");
    if (!(params.application_protocol == IpApplicationProtocol::HTTP
          && params.stream_encoding == IpStreamEncoding::MJPEG))
      VCP_ERROR("Invalid protocol/encoding ("
                << params.application_protocol << "/" << params.stream_encoding
                << ") for HTTP/MJPEG sink.");
  }

  virtual ~HttpMjpegSink()
  {
    VCP_LOG_DEBUG("HttpMjpegSink::~HttpMjpegSink()");
    CloseDevice();
  }


  std::vector<cv::Mat> Next() override
  {
    std::vector<cv::Mat> frames;
    image_queue_mutex_.lock();
    if (image_queue_->Empty())
    {
      frames.push_back(cv::Mat());
    }
    else
    {
      // Retrieve oldest image in queue.
      frames.push_back(image_queue_->Front().clone());
      image_queue_->PopFront();
    }
    image_queue_mutex_.unlock();
    return frames;
  }


  size_t NumAvailableFrames() const override
  {
    return static_cast<size_t>(IsFrameAvailable());
  }


  int IsDeviceAvailable() const override
  {
    if (is_stream_available_)
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

  bool OpenDevice() override
  {
    VCP_LOG_DEBUG("HttpMjpegSink::OpenDevice()");
    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Opening IP camera connection: " << params_);

    if (mjpg_stream_)
    {
      VCP_LOG_FAILURE("HTTP/MJPEG stream is already initialized: " << params_);
      return false;
    }
    if (params_.transport_protocol != IpTransportProtocol::TCP)
      VCP_LOG_WARNING("HTTP/MJPEG streams can only use TCP (curl default): " << params_);

    mjpg_stream_ = curl::url_fopen(&mjpg_multi_handle_, params_.stream_url.c_str(), "r");
    return mjpg_stream_ != nullptr;
  }

  bool CloseDevice() override
  {
    StopStreaming();

    if (mjpg_stream_)
    {
      VCP_LOG_DEBUG("HttpMjpegSink::CloseDevice()");
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Closing IP camera connection: '" << params_ << "'");
      curl::url_fclose(&mjpg_multi_handle_, mjpg_stream_);
      mjpg_stream_ = nullptr;
    }
    return true;
  }

  bool StartStreaming() override
  {
    VCP_LOG_DEBUG("HttpMjpegSink::StartStreaming()");
    if (!mjpg_stream_)
    {
      VCP_LOG_FAILURE("IP camera has not been opened yet: " << params_);
      return false;
    }

    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Starting IP camera stream " << params_);

    continue_stream_ = true;
    stream_thread_ = std::thread(&HttpMjpegSink::Receive, this);
    return true;
  }

  bool StopStreaming() override
  {
    if (continue_stream_)
    {
      VCP_LOG_DEBUG("HttpMjpegSink::StopStreaming()");
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Stopping IP camera stream: " << params_);
      // Stop receiver thread.
      continue_stream_ = false;
      stream_thread_.join();
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Successfully stopped IP camera stream: " << params_);
    }
    return true;
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

  void SetVerbose(bool verbose) override
  {
    params_.verbose = verbose;
  }

  SinkType GetSinkType() const override
  {
    return SinkType::IPCAM_MONOCULAR;
  }

private:
  IpCameraSinkParams params_;
  curl::URL_FILE *mjpg_stream_;
  CURLM *mjpg_multi_handle_;
  bool continue_stream_;

  std::thread stream_thread_;
  std::atomic<bool> is_stream_available_;
  mutable std::mutex image_queue_mutex_;
  std::unique_ptr<SinkBuffer> image_queue_;

  void Receive()
  {
    is_stream_available_ = true;
    size_t frame_drop_cnt = 0;
    while (continue_stream_)
    {
      char garbage[80];
      // Skip "--<boundary>"
      curl::url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_);
      // Skip "Content-type:...\n"
      curl::url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_);
      // Skip "Content-Length: "
      curl::url_fread(mjpg_multi_handle_, (void *)garbage, sizeof(char), 16, mjpg_stream_);
      // Read the frame size
      curl::url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_);
      int jpeg_bytes = atoi(garbage);

      if (jpeg_bytes == 0)
      {
        //TODO camera reboot should be detected here!
        VCP_LOG_FAILURE_NSEC("IP camera received no bytes: " << params_, 0.25);
        ++frame_drop_cnt;
        if (frame_drop_cnt > 1000)
        {
          is_stream_available_ = false;
        }
        continue;
      }
      frame_drop_cnt = 0;

      // Skip empty row before frame data
      curl::url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_);
      // Create space to store the frame
      uchar *frame_data = new uchar[jpeg_bytes];

      if (!frame_data)
      {
        VCP_LOG_FAILURE("Cannot allocate frame buffer: " << params_);
        continue;
      }

      // Read the frame
      curl::url_fread(mjpg_multi_handle_, (void *)frame_data, sizeof(uchar), jpeg_bytes, mjpg_stream_);

      if(!frame_data)
      {
        VCP_LOG_FAILURE("Cannot retrieve frame data from URL handle: " << params_);
        continue;
      }

      // Skip empty row before the next delimiter
      curl::url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_);
      // Finished reading a frame, now convert!

      cv::Mat buf(1, jpeg_bytes, CV_8UC1, (void *)frame_data);
      cv::Mat decoded = cv::imdecode(buf, CV_LOAD_IMAGE_UNCHANGED);
      cv::Mat frame;
      if (params_.color_as_bgr)
        frame = decoded;
      else
        cv::cvtColor(decoded, frame, CV_BGR2RGB);

      // Apply basic image transformations if needed.
      const cv::Mat img = imutils::ApplyImageTransformations(frame, params_.transforms);
      image_queue_mutex_.lock();
      image_queue_->PushBack(img.clone());
      image_queue_mutex_.unlock();
      delete[] frame_data;
    }
  }
};

std::unique_ptr<StreamSink> CreateBufferedHttpMjpegSink(const IpCameraSinkParams &params, std::unique_ptr<SinkBuffer> sink_buffer)
{
  return std::unique_ptr<HttpMjpegSink>(new HttpMjpegSink(params, std::move(sink_buffer)));
}

} // namespace http
} // namespace ipcam
} // namespace best
} // namespace vcp
