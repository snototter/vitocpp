#ifndef __VCP_BEST_IPCAM_SINK_H__
#define __VCP_BEST_IPCAM_SINK_H__

#include <memory>
#include <vector>

#include "capture.h"

namespace vcp
{
namespace best
{
namespace ipcam
{
enum class IpProtocol
{
  HTTP = 1,
  RTSP
};
std::ostream &operator<<(std::ostream &stream, const IpProtocol &s);

enum class IpStreamType
{
  H264 = 1,
  MJPEG
};
std::ostream &operator<<(std::ostream &stream, const IpStreamType &s);

enum class IpCameraType
{
  Generic = 1,
  Axis,
  Mobotix,
  Hikvision
  // TODO maybe add squire TPU (TrafficPTZ Ultimo)
};
std::ostream &operator<<(std::ostream &stream, const IpCameraType &cam);


struct IpCameraParams
{
  IpCameraParams() : host(""), user(""), password(""), camera_type(IpCameraType::Generic),
    protocol(IpProtocol::HTTP), stream_type(IpStreamType::H264),
    frame_width(-1), frame_height(-1), frame_rate(-1), custom_url("")
  {}

  std::string host; // IP or hostname
  std::string user;
  std::string password;

  IpCameraType camera_type;
  IpProtocol protocol;
  IpStreamType stream_type;

  int frame_width;
  int frame_height;
  int frame_rate; // Must be an int for Axis (VAPIX Streaming API)

  std::string custom_url; // Used to connect to a generic IP camera (you just provide IpProtocol, IpStreamType and this URL).

  friend std::ostream &operator<< (std::ostream &out, const IpCameraParams &p);
};

struct IpStereoParams
{
  IpCameraParams left;
  IpCameraParams right;
};

// Convenience wrappers/lookups
IpProtocol IpProtocolFromString(const std::string &protocol);
std::string IpProtocolToString(const IpProtocol &p);

IpStreamType IpStreamTypeFromString(const std::string &stream_type);
std::string IpStreamTypeToString(const IpStreamType &s);

IpCameraType IpCameraTypeFromString(const std::string &camera_type);
std::string IpCameraTypeToString(const IpCameraType &c);

IpCameraParams IpCameraParamsFromConfigSetting(const vcp::config::ConfigParams &config, const std::string &setting);
IpStereoParams IpStereoParamsFromConfigSetting(const vcp::config::ConfigParams &config, const std::string &setting);


class MonoIpCameraSinkParams : public SinkParams
{
public:
  MonoIpCameraSinkParams(const std::string &lbl, const std::string &calib_file, const StreamType &type,
                   const ipcam::IpCameraParams &params)
    : SinkParams(lbl, calib_file, type),
      device_params(params) {}
  virtual ~MonoIpCameraSinkParams() {}

  ipcam::IpCameraParams device_params;

private:
  MonoIpCameraSinkParams();
};


class StereoIpCameraSinkParams : public SinkParams
{
public:
  StereoIpCameraSinkParams(const std::string &lbl, const std::string &calib_file, const StreamType &type,
                      const ipcam::IpCameraParams &cam1, const ipcam::IpCameraParams &cam2)
    : SinkParams(lbl, calib_file, type),
      cam1_params(cam1), cam2_params(cam2) {}
  virtual ~StereoIpCameraSinkParams() {}

  ipcam::IpCameraParams cam1_params;
  ipcam::IpCameraParams cam2_params;

private:
  StereoIpCameraSinkParams();
};


MonoIpCameraSinkParams MonoIpCameraSinkParamsFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group);


StereoIpCameraSinkParams StereoIpCameraSinkParamsFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group);


std::unique_ptr<StreamSink> CreateMonoIpCamSink(const std::vector<MonoIpCameraSinkParams> &params);


std::unique_ptr<StreamSink> CreateStereoIpCamSink(const std::vector<StereoIpCameraSinkParams> &params);


/** @brief Given a configuration file's camera "type" parameter, returns true if the camera is an IP cam (generic, axis, ...). */
bool IsIpCamera(const std::string &camera_type);


/** @brief Check if the configuration belongs to a monocular IP camera. */
bool IsMonocularIpCamera(const std::string &camera_type);


/** @brief Check if the configuration belongs to a stereo IP camera (i.e. two seperate physical devices whose frames are concatenated by the StreamSink). */
bool IsStereoIpCamera(const std::string &camera_type);

} // namespace ipcam
} // namespace best
} // namespace vcp

#endif // __VCP_BEST_IPCAM_SINK_H__
