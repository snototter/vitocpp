#include "http_mjpeg_sink.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <vcp_utils/file_utils.h>

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
#include <vcp_utils/string_utils.h>
#include <vcp_imutils/opencv_compatibility.h>

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
      VCP_LOG_INFO_DEFAULT("Opening IP camera connection: " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));

    if (mjpg_stream_)
    {
      VCP_LOG_FAILURE("HTTP/MJPEG stream is already initialized: " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));
      return false;
    }
    if (params_.transport_protocol != IpTransportProtocol::TCP)
      VCP_LOG_WARNING("HTTP/MJPEG streams can only use TCP (curl default): " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));


    // Load calibration if available
    if (params_.rectify || vcp::utils::file::Exists(params_.calibration_file))
    {
      if (params_.calibration_file.empty() || !vcp::utils::file::Exists(params_.calibration_file))
      {
        VCP_LOG_FAILURE("To undistort & rectify the http/mjpeg stream " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url) << ", the calibration file '"
                  << params_.calibration_file << "' must exist!");
        return false;
      }

      const auto intrinsics = calibration::LoadIntrinsicsFromFile(params_.calibration_file);
      if (intrinsics.size() != 1)
      {
        VCP_LOG_FAILURE("Loaded invalid number of " << intrinsics.size()
                        << " intrinsic calibrations from " << params_.calibration_file << ".");
        return false;
      }

      intrinsics_ = intrinsics[0];

      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Loaded intrinsic calibration for http/mjpeg stream: " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url) << ".");
    }

    mjpg_stream_ = curl::url_fopen(&mjpg_multi_handle_, params_.stream_url.c_str(), "r", 5000L, 0L);
    return mjpg_stream_ != nullptr;
  }

  bool CloseDevice() override
  {
    StopStreaming();

    if (mjpg_stream_)
    {
      VCP_LOG_DEBUG("HttpMjpegSink::CloseDevice()");
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Closing IP camera connection: '"
                             << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url) << "'");
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
      VCP_LOG_FAILURE("IP camera has not been opened yet: " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));
      return false;
    }

    if (params_.verbose)
      VCP_LOG_INFO_DEFAULT("Starting IP camera stream " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));

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
        VCP_LOG_INFO_DEFAULT("Stopping IP camera stream: " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));
      // Stop receiver thread.
      continue_stream_ = false;
      stream_thread_.join();
      if (params_.verbose)
        VCP_LOG_INFO_DEFAULT("Successfully stopped IP camera stream: " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));
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
    VCP_LOG_FAILURE("GetSinkType() should not be called for an HttpMjpegSink, use SinkParamsAt(stream_index).sink_type instead!");
    return SinkType::IPCAM_GENERIC;
  }

  vcp::best::calibration::StreamIntrinsics IntrinsicsAt(size_t stream_index) const override
  {
    VCP_UNUSED_VAR(stream_index);
    return intrinsics_;
  }

  bool SetExtrinsicsAt(size_t stream_index, const cv::Mat &R, const cv::Mat &t) override
  {
    VCP_UNUSED_VAR(stream_index);
    return extrinsics_.SetExtrinsics(R, t, intrinsics_);
  }

  void ExtrinsicsAt(size_t stream_index, cv::Mat &R, cv::Mat &t) const override
  {
    VCP_UNUSED_VAR(stream_index);
    R = extrinsics_.R().clone();
    t = extrinsics_.t().clone();
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

  calibration::StreamIntrinsics intrinsics_;
  calibration::StreamExtrinsics extrinsics_;

  void Receive()
  {
    is_stream_available_ = true;
    size_t frame_drop_cnt = 0;
//FIXME add timeout parameter (use a stopwatch, reset after each received frame)
    while (continue_stream_)
    {
      char garbage[80];
      // Skip "--<boundary>"
      if (!curl::url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_))
        continue;
      // Skip "Content-type:...\n"
      if (!curl::url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_))
        continue;
      // Skip "Content-Length: "
      if (!curl::url_fread(mjpg_multi_handle_, (void *)garbage, sizeof(char), 16, mjpg_stream_))
        continue;
      // Read the frame size
      if (!curl::url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_))
        continue;
      int jpeg_bytes = atoi(garbage);

      if (jpeg_bytes == 0)
      {
        VCP_LOG_FAILURE_NSEC("IP camera received no bytes: " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url), 0.25);
        ++frame_drop_cnt;
        if (frame_drop_cnt > 1000)
        {
          VCP_ERROR("More than 1000 (" << frame_drop_cnt << ") dropped frames, aborting: " << params_);
          is_stream_available_ = false;
        }
        continue;
      }
      frame_drop_cnt = 0;
//FIXME collect bytes, then search for 0xff0xd8 and d9 markers!
      // Skip empty row before frame data
      curl::url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_);
      // Create space to store the frame
      uchar *frame_data = new uchar[jpeg_bytes];

      if (!frame_data)
      {
        VCP_LOG_FAILURE("Cannot allocate frame buffer: " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));
        continue;
      }
      // Read the frame
      curl::url_fread(mjpg_multi_handle_, (void *)frame_data, sizeof(uchar), jpeg_bytes, mjpg_stream_);

      if(!frame_data)
      {
        VCP_LOG_FAILURE("Cannot retrieve frame data from URL handle: " << vcp::utils::string::ObscureUrlAuthentication(params_.stream_url));
        continue;
      }
      // Skip empty row before the next delimiter
      curl::url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_);
      // Finished reading a frame, now convert!
      cv::Mat buf(1, jpeg_bytes, CV_8UC1, (void *)frame_data);
      cv::Mat decoded = cv::imdecode(buf, COMPAT_CV_LOAD_IMAGE_UNCHANGED);
      cv::Mat frame;
      if (params_.color_as_bgr)
        frame = decoded;
      else
        frame = FlipChannels(decoded);
      // Rectify if needed
      if (params_.rectify)
      {
        cv::Mat tmp = frame.clone();
        frame = intrinsics_.UndistortRectify(tmp);
      }
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
