#ifndef __VCP_BEST_LIVEVIEW_H__
#define __VCP_BEST_LIVEVIEW_H__

#include <string>
#include <memory>
#include <opencv2/core/core.hpp>
#include "sink_buffer.h"


namespace vcp
{
namespace best
{
namespace liveview
{

struct LiveViewParams
{
  std::string window_title;
  cv::Size max_size;
  int wait_ms;

  LiveViewParams(
      const std::string &window_title,
      const cv::Size &max_size,
      const int wait_ms=10)
    : window_title(window_title),
      max_size(max_size),
      wait_ms(wait_ms)
  {}
};


/**
 * @brief Abstract base class for live views which run in a separate thread.
 */
class LiveView
{
protected:
  LiveView(const LiveViewParams &params) : params_(params) {}
  LiveViewParams params_;

public:
  virtual ~LiveView() {}

  /** @brief Start displaying. */
  virtual bool Start() = 0;

  /** @brief Stop displaying. */
  virtual bool Stop() = 0;

  /** @brief Enqueue a single image to be displayed.
   * If the image is larger than the configured max_size,
   * it will be resized.
   */
  virtual void PushImageRequest(const cv::Mat &image) = 0;

  /** @brief The given images will be converted to a collage. */
  virtual void PushCollageRequest(const std::vector<cv::Mat> &images) = 0;

  /** @brief Change the time (in ms) to wait after displaying the image(s) (to let the display thread finish rendering). */
  virtual void SetWaitMs(int wait_ms) = 0;

  /** @brief Returns the result of the last cv::waitKey(). */
  virtual int GetLastUserInput() const = 0;

  /** @brief Returns the frame rate of the incoming view requests, NOT the display frame rate! */
  virtual double GetRequestFrameRate() const = 0;

  /** @brief Returns the current display frame rate. */
  virtual double GetDisplayFrameRate() const = 0;
};


/** @brief Creates a LiveView to display your stream (starts a separate thread) using the given stream buffer (takes ownership). */
std::unique_ptr<LiveView> CreateBufferedLiveView(const LiveViewParams &params, std::unique_ptr<SinkBuffer> stream_buffer);

/**
 * @brief Creates a LiveView, specify size of the image queue as template parameter.
 */
template <int BufferCapacity>
std::unique_ptr<LiveView> CreateLiveView(const LiveViewParams &params)
{
  return CreateBufferedLiveView(params, std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()));
}

} // liveview
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_LIVEVIEW_H__
