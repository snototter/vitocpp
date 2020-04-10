#include "rtsp_media_sink.h"

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
    #include <opencv2/highgui/highgui.hpp>
#else
    #include <opencv2/core.hpp>
    #include <opencv2/highgui.hpp>
#endif

// Use this to dump incoming rtsp frames (which 'should be' split into NAL units by live555).
//#define DEBUG_RTSP_H264_DECODING
#ifdef DEBUG_RTSP_H264_DECODING
  #include <iomanip>
#endif

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
  #include <libavcodec/avcodec.h>
  #include <libavformat/avformat.h>
  #include <libswscale/swscale.h>
  #include <libavutil/imgutils.h>
#ifdef __cplusplus
}
#endif

#include <vcp_utils/vcp_error.h>

// av_frame_alloc was added in lavc 55.28.1 (not available on Ubuntu 14.04)
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
  //#define av_frame_alloc avcodec_alloc_frame
  //#define USE_DEPRECATED_PICTURE_API
  #error "We no longer support the deprecated libav API - we need the loop based decoding (send packets to the decoder, retrieve (potentially) multiple frames, decoupled from the actual network bitstream)"
#endif

// Define the size of the buffer that we'll use (10MB)
#define RTSP_MEDIA_SINK_RECEIVE_BUFFER_SIZE 10485760

namespace vcp
{
namespace best
{
namespace ipcam
{
namespace rtsp
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::best::ipcam::rtsp"
//-----------------------------------------------------------------------------
// MediaSink base class
RtspMediaSink::~RtspMediaSink()
{
  if (receive_buffer_)
    delete[] receive_buffer_;
}

RtspMediaSink::RtspMediaSink(UsageEnvironment &env, MediaSubsession &subsession, const IpCameraSinkParams &params, void (*callback_frame_received)(const cv::Mat &, void *), void *callback_user_data)
  : MediaSink(env), receive_buffer_(nullptr), subsession_(subsession),
    num_received_frames_(0), has_been_rtcp_synchronized_(false),
    frame_width_(params.frame_width), frame_height_(params.frame_height),
    color_as_rgb_(params.color_as_bgr), stream_id_(params.stream_url),
    callback_frame_received_(callback_frame_received), callback_user_data_(callback_user_data)
{
  receive_buffer_ = new u_int8_t[RTSP_MEDIA_SINK_RECEIVE_BUFFER_SIZE + AV_INPUT_BUFFER_PADDING_SIZE];
}

Boolean RtspMediaSink::continuePlaying()
{
  // Sanity check (according to RTSP demo code, this should never happen).
  if (fSource == NULL)
    return False;

  // Request the next data frame from our input source.
  // "afterGettingFrame()" will be invoked once the frame arrived.
  fSource->getNextFrame(receive_buffer_, RTSP_MEDIA_SINK_RECEIVE_BUFFER_SIZE,
                        afterGettingFrame, this,
                        onSourceClosure, this);
  return True;
}

void RtspMediaSink::afterGettingFrame(void *client_data, unsigned frame_size, unsigned num_truncated_bytes, struct timeval presentation_time, unsigned duration_in_microseconds)
{
  // The corresponding instance is passed as client_data, so we need to cast.
  RtspMediaSink *sink = static_cast<RtspMediaSink*>(client_data);
  sink->DecodeFrame(frame_size, num_truncated_bytes, presentation_time, duration_in_microseconds);
}



//-----------------------------------------------------------------------------
// MJPEG decoding media sink.
class RtspMjpegMediaSink : public RtspMediaSink
{
public:
  RtspMjpegMediaSink(UsageEnvironment &env, MediaSubsession &subsession, const IpCameraSinkParams &params, void (*callback_frame_received)(const cv::Mat &, void *), void *callback_user_data)
    : RtspMediaSink(env, subsession, params, callback_frame_received, callback_user_data)
  {
  }

  virtual ~RtspMjpegMediaSink()
  {
  }

protected:
  void DecodeFrame(unsigned /*frame_size*/, unsigned num_truncated_bytes, struct timeval /*presentation_time*/, unsigned /*duration_in_microseconds*/) //, cv::Mat &decoded_frame) override
  {
    // We've just received a frame of data.
    if (num_truncated_bytes > 0)
    {
      VCP_LOG_FAILURE("Received corrupt JPEG frame from '" << stream_id_ << "'.");
    }
    else
    {
      cv::Mat buf(frame_height_, frame_width_, CV_8UC3, (void *)receive_buffer_);
      cv::Mat decoded_frame = cv::imdecode(buf, CV_LOAD_IMAGE_COLOR);
      ++num_received_frames_;
      // Notify observer (which stores the image in a queue)
      (*this->callback_frame_received_)(color_as_rgb_ ? FlipChannels(decoded_frame) : decoded_frame, this->callback_user_data_);
    }
    continuePlaying();
  }
};


//-----------------------------------------------------------------------------
// H264 decoding media sink.
class RtspH264MediaSink : public RtspMediaSink
{
public:
  RtspH264MediaSink(UsageEnvironment &env, MediaSubsession &subsession, const IpCameraSinkParams &params, void (*callback_frame_received)(const cv::Mat &, void *), void *callback_user_data)
    : RtspMediaSink(env, subsession, params, callback_frame_received, callback_user_data)
  {
    fSPropParameterSetsStr = subsession.fmtp_spropparametersets();
    fHaveWrittenFirstFrame = false;
    fNeedsMoreBytes = false;

    first_full_frame_arrived = false;

    avcodec_register_all();

    codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if(!codec)
      VCP_ERROR("Cannot find the H264 codec");

    codec_context = avcodec_alloc_context3(codec);
    // For some codecs we need to explicitly set the frame dimension as these are not encoded in the bitstream
    codec_context->width = params.frame_width;
    codec_context->height = params.frame_height;

    if(codec->capabilities & CODEC_CAP_TRUNCATED)
      codec_context->flags |= CODEC_FLAG_TRUNCATED;

    // Force low delay
    if (codec->capabilities & CODEC_CAP_DELAY)
      codec_context->flags |= AV_CODEC_FLAG_LOW_DELAY;

    // live555 should gather the NAL units for us, and tell us, once a "frame" (i.e. NALU) is ready, so
    // there's no need to tell the decoder to watch out for truncated buffers. Thus, we don't need:
//    codec_context->flags2 |= CODEC_FLAG2_CHUNKS;

    if(avcodec_open2(codec_context, codec, NULL) < 0)
      VCP_ERROR("Cannot open the H264 codec");

    picture = av_frame_alloc();
    if (!picture)
      VCP_ERROR("Cannot allocate decoding picture");

    parser = av_parser_init(AV_CODEC_ID_H264);
    if(!parser)
      VCP_ERROR("Cannot create the H264 parser");

    convert_ctx = NULL;

    picture_bgr = av_frame_alloc();
    if (!picture_bgr)
      VCP_ERROR("Cannot allocate BGR frame");

#ifdef USE_DEPRECATED_PICTURE_API
    // Required on my 14.04 standard installation
    int bytes = avpicture_get_size(AV_PIX_FMT_RGB24, frame_width_, frame_height_);
#else
    int bytes = av_image_get_buffer_size(AV_PIX_FMT_RGB24, frame_width_, frame_height_, 32); // Alignment is recommended to be 32, see https://stackoverflow.com/a/35682306
#endif
    buffer_bgr_picture = (uint8_t *)av_malloc(bytes*sizeof(uint8_t));
    if (!buffer_bgr_picture)
      VCP_ERROR("Cannot allocate BGR frame buffer");

#ifdef USE_DEPRECATED_PICTURE_API
    avpicture_fill((AVPicture *)picture_bgr, buffer_bgr_picture, AV_PIX_FMT_RGB24, frame_width_, frame_height_);
#else
    av_image_fill_arrays(picture_bgr->data, picture_bgr->linesize, buffer_bgr_picture, AV_PIX_FMT_RGB24, frame_width_, frame_height_, 32);
#endif

    pkt = av_packet_alloc();
    if (!pkt)
      VCP_ERROR("Cannot allocate AVPacket");

    // Initialize the additional zero padding (for damaged incoming frames)
    std::memset(zero_padding, 0, AV_INPUT_BUFFER_PADDING_SIZE);
  }

  virtual ~RtspH264MediaSink()
  {
    if(parser)
    {
      av_parser_close(parser);
      parser = nullptr;
    }

    if(codec_context)
    {
      avcodec_close(codec_context);
      av_free(codec_context);
      codec_context = nullptr;
    }

    if(picture)
    {
      av_free(picture);
      picture = nullptr;
    }

    if (picture_bgr)
    {
      av_free(picture_bgr);
      picture_bgr = nullptr;
    }

    if (buffer_bgr_picture)
    {
      av_free(buffer_bgr_picture);
      buffer_bgr_picture = nullptr;
    }

    if (convert_ctx)
    {
      sws_freeContext(convert_ctx);
      convert_ctx = nullptr;
    }

    if (pkt)
    {
      av_packet_free(&pkt);
      pkt = nullptr;
    }
  }

protected:

  void DecodeAvPacket()
  {
    int ret = avcodec_send_packet(codec_context, pkt);
    if (ret < 0)
      VCP_ERROR("Error sending a packet for decoding, capture '" << stream_id_ << "'");

    while (ret >= 0)
    {
      ret = avcodec_receive_frame(codec_context, picture);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
      {
        return;
      }
      else if (ret < 0)
      {
        VCP_ERROR("Error sending a packet for decoding, capture '" << stream_id_ << "'");
      }

      // Create converter upon first invocation
      if (!convert_ctx)
      {
        convert_ctx = sws_getContext(picture->width, picture->height, static_cast<AVPixelFormat>(picture->format),
                                     picture->width, picture->height, AV_PIX_FMT_BGR24,
                                     SWS_FAST_BILINEAR, NULL, NULL, NULL); // SWS_BICUBIC
        if (!convert_ctx)
         VCP_ERROR("Cannot create SWS conversion context");
      }

      const int slice_height = sws_scale(convert_ctx, picture->data, picture->linesize, 0,
                                         picture->height, picture_bgr->data, picture_bgr->linesize);
      if (slice_height > 0)
      {
        if (frame_width_ != picture->width || frame_height_ != picture->height)
          VCP_ERROR("Incorrect configuration of capture '" << stream_id_ << "': expected a "
                    << frame_width_ << "x" << frame_height_ << " stream, but received packets for " << picture->width << "x" << picture->height);

        cv::Mat dec(picture->height, picture->width, CV_8UC3, picture_bgr->data[0], picture_bgr->linesize[0]);
       (*this->callback_frame_received_)(color_as_rgb_ ? FlipChannels(dec) : dec, this->callback_user_data_);
        ++num_received_frames_;
#ifdef DEBUG_RTSP_H264_DECODING
        VCP_LOG_INFO_DEFAULT("Decoded frame " << num_received_frames_ << " vs " << codec_context->frame_number << " with delay [# of frames]: " << codec_context->delay);
#endif // DEBUG_RTSP_H264_DECODING
      }
    }
  }

  void DecodeFrame(unsigned frame_size, unsigned num_truncated_bytes, struct timeval /*presentation_time*/, unsigned /*duration_in_microseconds*/) //, cv::Mat &decoded_frame) override
  {
    // See live555/liveMedia/H264or5VideoFileSink.cpp::afterGettingFrame()
    // see also http://lists.live555.com/pipermail/live-devel/2014-September/018709.html
    // and finally https://libav.org/documentation/doxygen/master/decode__video_8c_source.html

#ifdef DEBUG_RTSP_H264_DECODING
    VCP_LOG_DEBUG_DEFAULT("Hex dump of received 'frame' (should be a complete NALU):");
    for (unsigned int i = 0; i < std::min((unsigned)48,frame_size); ++i)
    {
      if (i > 0 && i % 16 == 0)
        std::cout << std::endl;
      std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(receive_buffer_[i]) << " ";
    }
    std::cout << std::endl;
    std::cout.copyfmt(std::ios(NULL)); // restore manipulators
#endif // DEBUG_RTSP_H264_DECODING

    // h264 AVCC encodes the length as little endian at the beginning of a frame:
    // It seems like we don't get AVCC encoded payloads (even if "vlc" would tell us
    // that a stream is h264/avc:
// //    int avcc_frame_size = (receive_buffer_[0]<<0) | (receive_buffer_[1]<<8) | (receive_buffer_[2]<<16) | (receive_buffer_[3]<<24);
// //    VCP_LOG_INFO("If AVCC, frame size is: " << avcc_frame_size << " vs live555 frame_size: " << frame_size);

    const unsigned nal_unit_type = receive_buffer_[0] & 0x1f;
#ifdef DEBUG_RTSP_H264_DECODING
    if (nal_unit_type != 1)
      VCP_LOG_FAILURE_DEFAULT("[No failure] Received NALU: " << nal_unit_type); // Make it a warning, so we see it easier in the console spam
#endif // DEBUG_RTSP_H264_DECODING

    // Every camera is special:
    // - Axis never sends SPS or PPS (only full frames, SPS/PPS is delivered with the SDP description)
    // - Hikvision periodically sends SPS (7), PPS (8) and full frames (5)
    // - Streaming via vlc sends SPS/PPS/full frames very sparsely
    if (nal_unit_type == 5 || nal_unit_type == 7)
      first_full_frame_arrived = true;

    if (!first_full_frame_arrived)
    {
      frame_buffer.clear();
      VCP_LOG_WARNING_NSEC("Skipping incoming packets for capture '" << stream_id_ << "' until we find SPS/PPS.", 0.5);
      continuePlaying();
      return;
    }

    // Needs to be prepended before each "network frame" (i.e. NAL unit) to correctly decode it.
    unsigned char const start_code[4] = {0x00, 0x00, 0x00, 0x01};

    if (!fHaveWrittenFirstFrame && !(nal_unit_type == 7 || nal_unit_type == 8))
    {
      // If we have NAL units encoded in "sprop parameter strings", prepend these to the buffer for parsing/decoding:
      unsigned numSPropRecords;
      SPropRecord* sPropRecords = parseSPropParameterSets(subsession_.fmtp_spropparametersets(), numSPropRecords);
      for (unsigned i = 0; i < numSPropRecords; ++i)
      {
        AppendData(start_code, 4);
        AppendData(sPropRecords[i].sPropBytes, sPropRecords[i].sPropLength);
      }
      delete[] sPropRecords;
      fHaveWrittenFirstFrame = true; // Can be skipped for future frames.
    }
    AppendData(start_code, 4);

    AppendData(receive_buffer_, frame_size);

    // Set end of buffer to 0 (ensures that no overreading happens for damaged MPEG streams)
    AppendData(&zero_padding[0], AV_INPUT_BUFFER_PADDING_SIZE);

    if (num_truncated_bytes > 0)
      VCP_LOG_FAILURE("Capture '" << stream_id_ << "' dropped " << num_truncated_bytes << " bytes - expect decoding errors!");

    int data_size = frame_buffer.size() - AV_INPUT_BUFFER_PADDING_SIZE;
    uint8_t *data = &frame_buffer[0];

    int consumed = 0;
    while(data_size > 0)
    {
      int len = av_parser_parse2(parser, codec_context, &pkt->data, &pkt->size, data, data_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
      if (len < 0)
        VCP_ERROR("Error while parsing h264 stream, capture '" << stream_id_ << "'");
      data += len;
      data_size -= len;
      consumed += len;
      if (pkt->size)
        DecodeAvPacket();
    }
    // Remove the zero padding (at the end).
    frame_buffer.erase(frame_buffer.end() - AV_INPUT_BUFFER_PADDING_SIZE, frame_buffer.end());

    // Remove the bytes consumed by the parser (and fed into the AVPacket, which was then passed to the decoder).
    frame_buffer.erase(frame_buffer.begin(), frame_buffer.begin() + std::min(frame_buffer.size(), static_cast<size_t>(consumed)));

    // Finally, request the next frame of data.
    continuePlaying();
  }

private:
  char const* fSPropParameterSetsStr;
  bool fHaveWrittenFirstFrame;
  bool fNeedsMoreBytes;

  bool first_full_frame_arrived; // We skip "network frames" until the first full image frame (NALU 5) or at least the session description (NALU 7/8) arrived.
  std::vector<uint8_t> frame_buffer; // stores received bytes + sdp format stuff

  AVCodec* codec; // the AVCodec* which represents the H264 decoder
  AVCodecContext* codec_context; // the context; keeps generic state
  AVCodecParserContext* parser; // parser that is used to decode the h264 bitstream
  AVFrame* picture; // will contain a decoded picture
  AVPacket* pkt; // Used to parse the incoming "network frames" into h264 units, will be send to the decoder

  AVFrame *picture_bgr; // Used to decode (swscale the decoded h264 "picture" into BGR format)
  uint8_t *buffer_bgr_picture; // Buffer to store the "picture_bgr" data.
  struct SwsContext *convert_ctx;

  uint8_t zero_padding[AV_INPUT_BUFFER_PADDING_SIZE];

  void AppendData(unsigned char const *data, unsigned data_size /*, struct timeval presentation_time*/)
  {
    if (data_size > 0)
      std::copy(data, data + data_size, std::back_inserter(frame_buffer));
  }
};


RtspMediaSink *CreateRtspMjpegMediaSink(UsageEnvironment &env, MediaSubsession &subsession, const IpCameraSinkParams &params, void (*callback_frame_received)(const cv::Mat &, void *), void *callback_user_data)
{
  return new RtspMjpegMediaSink(env, subsession, params, callback_frame_received, callback_user_data);
}

RtspMediaSink *CreateRtspH264MediaSink(UsageEnvironment &env, MediaSubsession &subsession, const IpCameraSinkParams &params, void (*callback_frame_received)(const cv::Mat &, void *), void *callback_user_data)
{
  return new RtspH264MediaSink(env, subsession, params, callback_frame_received, callback_user_data);
}

} // namespace rtsp
} // namespace ipcam
} // namespace best
} // namespace vcp
