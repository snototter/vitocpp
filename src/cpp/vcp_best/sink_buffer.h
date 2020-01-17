#ifndef __VCP_BEST_SINK_BUFFER_H__
#define __VCP_BEST_SINK_BUFFER_H__

#include <memory>
#include <vcp_utils/circular_buffer.h>
#include <opencv2/core/core.hpp>


// Default capacity (number of frames) for vcp::best:*Sink classes.
// Increase if you want to have a "real" buffer "feeling" (but be aware
// that your streams may "run behind"). Setting this to 2 is recommended
// for live streaming/always processing the most recent frames.
#ifndef VCP_BEST_STREAM_BUFFER_CAPACITY
    #define VCP_BEST_STREAM_BUFFER_CAPACITY 2
#endif

namespace vcp
{
namespace best
{
/**
 * @brief Interface for SinkBuffers to simplify templatization of
 *   circular_buffer as image queue.
 */
class SinkBuffer
{
public:
  virtual ~SinkBuffer() {}


  /** @brief Append image to buffer. */
  virtual void PushBack(const cv::Mat &frame) = 0;


  /** @brief Access most recently inserted element. */
  virtual const cv::Mat &Back() const = 0;


  /** @brief Access oldest elment in buffer. */
  virtual const cv::Mat &Front() const = 0;


  /** @brief Remove most recently inserted element. */
  virtual void PopBack() = 0;


  /** @brief Remove oldest element. */
  virtual void PopFront() = 0;


  /** @brief Returns true if the buffer is empty. */
  virtual bool Empty() const = 0;


  /** @brief Returns the number of images currently available. */
  virtual size_t Size() const = 0;


  /** @brief Returns the buffer capacity (or -1 if unknown/cannot be determined). */
  virtual int Capacity() const = 0;


protected:
  SinkBuffer() {}
};


/**
 * @brief Uses a circular buffer to implement a SinkBuffer.
 *
 * For documentation of the functionality, @see SinkBuffer.
 */
template<int BufferCapacity>
class CircularSinkBuffer : public SinkBuffer
{
public:
  CircularSinkBuffer() : SinkBuffer() {}
  virtual ~CircularSinkBuffer() {}

  void PushBack(const cv::Mat &frame) override
  {
    image_queue_.push_back(frame);
  }

  const cv::Mat &Back() const override
  {
    return image_queue_.back();
  }

  const cv::Mat &Front() const override
  {
    return image_queue_.front();
  }

  void PopBack() override
  {
    image_queue_.pop_back();
  }

  void PopFront() override
  {
    image_queue_.pop_front();
  }

  bool Empty() const override
  {
    return image_queue_.empty();
  }

  size_t Size() const override
  {
    return image_queue_.size();
  }

  int Capacity() const override
  {
    return BufferCapacity;
  }

private:
  vcp::utils::circular_buffer<cv::Mat, BufferCapacity> image_queue_;
};


/**
 * @brief Convenience utility to instantiate a CircularSinkBuffer with the templated capacity.
 */
template<int BufferCapacity>
std::unique_ptr<SinkBuffer> CreateCircularStreamSinkBuffer()
{
  return std::unique_ptr<CircularSinkBuffer<BufferCapacity> >(new CircularSinkBuffer<BufferCapacity>());
}

} // namespace best
} // namespace vcp
#endif // __VCP_BEST_SINK_BUFFER_H__
