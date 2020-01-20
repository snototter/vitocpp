#include "http_mjpeg_sink.h"

#include <thread>
#include <mutex>
#include <atomic>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "curl_file_handling.h"

#include <vcp_utils/vcp_error.h>

namespace vcp
{
namespace best
{
namespace ipcam
{
/**
 * @brief The MJPEG over HTTP stream sink
 */
class HttpMjpegSink: public StreamSink
{
public:
  HttpMjpegSink(const std::string &stream_url, std::unique_ptr<SinkBuffer> sink_buffer) : StreamSink(),
    mjpg_stream_(nullptr), mjpg_multi_handle_(nullptr), continue_stream_(false),
    stream_url_(stream_url), is_stream_available_(false), image_queue_(std::move(sink_buffer))
  {
  }

  virtual ~HttpMjpegSink()
  {
    CloseStream();
  }

  void StartStream() override
  {
    InitStream(stream_url_);
    stream_thread_ = std::thread(&HttpMjpegSink::Receive, this);
  }

  void Terminate() override
  {
    CloseStream();
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

private:
  URL_FILE *mjpg_stream_;
  CURLM *mjpg_multi_handle_;
  bool continue_stream_;
  std::string stream_url_;

  std::thread stream_thread_;
  std::atomic<bool> is_stream_available_;
  mutable std::mutex image_queue_mutex_;
  std::unique_ptr<SinkBuffer> image_queue_;

  void InitStream(const std::string &stream_url)
  {
    if (mjpg_stream_)
    {
      VCP_ERROR("HttpMJPEGSink on '" << stream_url << "' already initialized");
    }

    mjpg_stream_ = url_fopen(&mjpg_multi_handle_, stream_url.c_str(), "r");

    if (!mjpg_stream_)
    {
      PVT_EXIT("Cannot open MJPEG stream to " << stream_url);
    }
    continue_stream_ = true;
  }

  void Receive()
  {
    is_stream_available_ = true;
    size_t frame_drop_cnt = 0;
    while (continue_stream_)
    {
      char garbage[80];
      // Skip "--<boundary>"
      url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_);
      // Skip "Content-type:...\n"
      url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_);
      // Skip "Content-Length: "
      url_fread(mjpg_multi_handle_, (void *)garbage, sizeof(char), 16, mjpg_stream_);
      // Read the frame size
      url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_);
      int jpeg_bytes = atoi(garbage);

      if (jpeg_bytes == 0)
      {
        //TODO detect camera reboot here!!!!!
        PVT_LOG_FAILURE("No bytes have been received");
        ++frame_drop_cnt;
        if (frame_drop_cnt > 10000)
        {
          //continue_stream_ = false; // Otherwise we'll end up in a resource deadlock
          //CloseStream();
          is_stream_available_ = false;
        }
        continue;
      }
      frame_drop_cnt = 0;

      // Skip empty row before frame data
      url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_);
      // Create space to store the frame
      uchar *frame_data = new uchar[jpeg_bytes];

      if (!frame_data)
      {
        PVT_LOG_FAILURE("Cannot allocate frame buffer");
        continue;
      }

      // Read the frame
      url_fread(mjpg_multi_handle_, (void *)frame_data, sizeof(uchar), jpeg_bytes, mjpg_stream_);

      if(!frame_data)
      {
        PVT_LOG_FAILURE("Cannot retrieve frame data from URL handle");
        continue;
      }

      // Skip empty row before the next delimiter
      url_fgets(mjpg_multi_handle_, garbage, sizeof(garbage), mjpg_stream_);
      // Finished reading a frame, now convert!

      cv::Mat buf(1, jpeg_bytes, CV_8UC1, (void *)frame_data);
      cv::Mat frame = cv::imdecode(buf, CV_LOAD_IMAGE_COLOR);

      image_queue_mutex_.lock();
      image_queue_->PushBack(frame.clone());
      image_queue_mutex_.unlock();
      delete[] frame_data;
    }
  }

  void CloseStream()
  {
    if (continue_stream_)
    {
      // Stop receiver thread.
      continue_stream_ = false;

      stream_thread_.join();
    }
    if (mjpg_stream_)
    {
      PVT_LOG_TIMED("Closing Stream");
      // Close stream.
      url_fclose(&mjpg_multi_handle_, mjpg_stream_);
      mjpg_stream_ = nullptr;
      PVT_LOG_TIMED("Stream closed!");
    }
  }
};

std::unique_ptr<StreamSink> CreateBufferedHttpMjpegSink(const std::string &stream_url, std::unique_ptr<SinkBuffer> sink_buffer)
{
  return std::unique_ptr<HttpMjpegSink>(new HttpMjpegSink(stream_url, std::move(sink_buffer)));
}

} // namespace ipcam
} // namespace best
} // namespace vcp
