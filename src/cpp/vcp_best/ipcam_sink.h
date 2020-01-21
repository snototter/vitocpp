#ifndef __VCP_BEST_IPCAM_SINK_H__
#define __VCP_BEST_IPCAM_SINK_H__

#include <memory>
#include <vector>
#include <utility>

#include "capture.h"

namespace vcp
{
namespace best
{
namespace ipcam
{
/** @brief Supported IP camera application layer protocols. */
enum class IpApplicationProtocol
{
  HTTP = 1,
  RTSP
};

/** @brief Overloaded ostream operator. */
std::ostream &operator<<(std::ostream &stream, const IpApplicationProtocol &s);

/** @brief Maps a string (e.g. from configuration file) to IpApplicationProtocol enum. */
IpApplicationProtocol IpApplicationProtocolFromString(const std::string &protocol);

/** @brief Returns the string representation. */
std::string IpApplicationProtocolToString(const IpApplicationProtocol &p);


/** @brief Supported IP camera transport layer protocols. */
enum class IpTransportProtocol
{
  TCP = 1,
  UDP
};

/** @brief Overloaded ostream operator. */
std::ostream &operator<<(std::ostream &stream, const IpTransportProtocol &s);

/** @brief Maps a string (e.g. from configuration file) to IpTransportProtocol enum. */
IpTransportProtocol IpTransportProtocolFromString(const std::string &protocol);

/** @brief Returns the string representation. */
std::string IpTransportProtocolToString(const IpTransportProtocol &p);



/** @brief Supported stream encodings. */
enum class IpStreamEncoding
{
  H264 = 1,
  MJPEG
};

/** @brief Overloaded ostream operator. */
std::ostream &operator<<(std::ostream &stream, const IpStreamEncoding &s);

/** @brief Maps a string (e.g. from configuration file) to IpStreamEncoding enum. */
IpStreamEncoding IpStreamEncodingFromString(const std::string &stream_type);

/** @brief Returns the string representation. */
std::string IpStreamEncodingToString(const IpStreamEncoding &s);



/** @brief Supported IP cameras. */
enum class IpCameraType
{
  Generic = 1, /**< Basically, all IP cameras are "generic" sinks, they only differ regarding their streaming URLs. */
  Axis
//  Mobotix,
//  Hikvision
};

/** @brief Overloaded ostream operator. */
std::ostream &operator<<(std::ostream &stream, const IpCameraType &cam);

/** @brief Maps a string (e.g. from configuration file) to IpCameraType enum. */
IpCameraType IpCameraTypeFromString(const std::string &camera_type);

/** @brief Returns the string representation. */
std::string IpCameraTypeToString(const IpCameraType &c);



/** @brief Configuration parameters for a single IP camera. */
struct IpCameraSinkParams : SinkParams
{
  std::string host;
  std::string user;
  std::string password;
  IpCameraType ipcam_type;
  IpApplicationProtocol protocol;
  IpStreamEncoding stream_encoding;
  int frame_width;
  int frame_height;
  int frame_rate; // Must be an int (e.g. VAPIX only supports integer frame rates)
  std::string stream_url;

  IpCameraSinkParams(
      const SinkParams &sink_params,
      const std::string host,
      const std::string user=std::string(),
      const std::string pwd=std::string(),
      const IpCameraType ipcam_type=IpCameraType::Generic,
      const IpApplicationProtocol protocol=IpApplicationProtocol::RTSP,
      const IpStreamEncoding stream_encoding=IpStreamEncoding::H264,
      const int frame_width=-1,
      const int frame_height=-1,
      const int frame_rate=-1,
      const std::string &custom_url=std::string())
    : SinkParams(sink_params),
      host(host), user(user), password(pwd),
      ipcam_type(ipcam_type), protocol(protocol), stream_encoding(stream_encoding),
      frame_width(frame_width), frame_height(frame_height), frame_rate(frame_rate),
      stream_url(custom_url)
  {}
};

std::ostream &operator<< (std::ostream &out, const IpCameraSinkParams &p);


///** @brief Configuration parameters for a single IP camera. */
//struct IpCameraDeviceParams
//{
//  std::string host;
//  std::string user;
//  std::string password;
//  IpCameraType ipcam_type;
//  IpApplicationProtocol protocol;
//  IpStreamEncoding stream_encoding;
//  int frame_width;
//  int frame_height;
//  int frame_rate; // Must be an int (e.g. VAPIX only supports integer frame rates)
//  std::string custom_url;

//  IpCameraDeviceParams(
//      const std::string host,
//      const std::string user=std::string(),
//      const std::string pwd=std::string(),
//      const IpCameraType ipcam_type=IpCameraType::Generic,
//      const IpApplicationProtocol protocol=IpApplicationProtocol::RTSP,
//      const IpStreamEncoding stream_encoding=IpStreamEncoding::H264,
//      const int frame_width=-1,
//      const int frame_height=-1,
//      const int frame_rate=-1)
//    : host(host), user(user), password(pwd),
//      ipcam_type(ipcam_type), protocol(protocol), stream_encoding(stream_encoding),
//      frame_width(frame_width), frame_height(frame_height), frame_rate(frame_rate)
//  {}


//  std::string GetStreamingUrl() const;
//};
//std::ostream &operator<< (std::ostream &out, const IpCameraDeviceParams &p);

///** @brief Configuration parameters to stream from an IP camera. */
//struct MonocularIpCameraSinkParams : public SinkParams
//{
//  IpCameraDeviceParams ipcam_params;

//  MonocularIpCameraSinkParams(
//      const SinkParams &sink_params,
//      const IpCameraDeviceParams &ipcam_params)
//    : SinkParams(sink_params),
//      ipcam_params(ipcam_params)
//  {}

//  friend std::ostream &operator<< (std::ostream &out, const MonocularIpCameraSinkParams &p);
//};


///** @brief Configuration parameters to stream from two IP cameras which define a stereo setup. */
//struct StereoIpCameraSinkParams : public SinkParams
//{
//  IpCameraDeviceParams ipcam_params1;
//  IpCameraDeviceParams ipcam_params2;

//  StereoIpCameraSinkParams(
//      const SinkParams &sink_params,
//      const IpCameraDeviceParams &ipcam_params1,
//      const IpCameraDeviceParams &ipcam_params2)
//    : SinkParams(sink_params),
//      ipcam_params1(ipcam_params1),
//      ipcam_params2(ipcam_params2)
//  {}

//  friend std::ostream &operator<< (std::ostream &out, const StereoIpCameraSinkParams &p);
//};


//FIXME
//struct IpStereoParams
//{
//  IpCameraParams left;
//  IpCameraParams right;
//};




IpCameraSinkParams MonocularIpCameraSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param);
std::pair<IpCameraSinkParams, IpCameraSinkParams> StereoIpCameraSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param);
//FIXME IpStereoParams IpStereoParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &setting);



std::unique_ptr<StreamSink> CreateIpCameraSink(const std::vector<IpCameraSinkParams> &params);


//std::unique_ptr<StreamSink> CreateStereoIpCameraSink(const std::vector<StereoIpCameraSinkParams> &params);


/** @brief Given a configuration file's "camera_type" parameter, returns true if the camera is an IP cam (generic, axis, ...). */
bool IsIpCamera(const std::string &camera_type);


/** @brief Check if the configuration belongs to a monocular IP camera. */
bool IsMonocularIpCamera(const std::string &camera_type);


/** @brief Check if the configuration belongs to a stereo IP camera (i.e. two seperate physical devices whose frames are concatenated by the StreamSink). */
bool IsStereoIpCamera(const std::string &camera_type);

} // namespace ipcam
} // namespace best
} // namespace vcp

#endif // __VCP_BEST_IPCAM_SINK_H__
