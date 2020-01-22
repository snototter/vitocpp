#include "ipcam_sink.h"

#include <vcp_utils/vcp_logging.h>
#include <sstream>
#include <set>
#include <iomanip>

#ifdef VCP_BEST_WITH_IPCAM_HTTP
  #include "http_mjpeg_sink.h"
#endif
#ifdef VCP_BEST_WITH_IPCAM_RTSP
  #include "rtsp_sink.h"
#endif
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/string_utils.h>
#include <vcp_imutils/matutils.h>
#include <vcp_math/geometry3d.h>

namespace vcp
{
namespace best
{
namespace ipcam
{
std::string GetAxisRtspUrl(const IpCameraSinkParams &p)
{
  std::string codec;
  if (p.stream_encoding == IpStreamEncoding::MJPEG)
    codec = "jpeg";
  else
    codec = "h264";

  std::stringstream rtsp;
  rtsp << "rtsp://";

  if (!p.user.empty())
    rtsp << p.user << ":" << p.password << "@";

  rtsp << p.host << "/axis-media/media.amp";
  rtsp << "?videocodec=" << codec << "&audio=0";

  if (p.frame_width <= 0 || p.frame_height <= 0)
    VCP_ERROR("You must specify 'frame_width' and 'frame_height' for axis stream: '" << p.sink_label << "'.");
  rtsp << "&resolution=" << p.frame_width << 'x' << p.frame_height;

  if (p.frame_rate > 0)
    rtsp << "&fps=" << p.frame_rate;

  // Append &clock=1 if you want a clock overlay

  // If you increase the Axis parameter videokeyframeinterval, this causes more frequent
  // I-Frames, but leads to more dropped frames/frames which can't be decoded
  // (seems like a general cam problem 2015-12-01). 16 worked like a charm with our Axis IP cams.
  if (p.stream_encoding == IpStreamEncoding::H264)
    rtsp << "&videokeyframeinterval=16";
  return rtsp.str();
}


std::string GetAxisHttpUrl(const IpCameraSinkParams &p)
{
  if (p.stream_encoding != IpStreamEncoding::MJPEG)
    VCP_ERROR("Axis can only stream MJPEG over HTTP, either change stream type or protocol!");

  std::stringstream http;
  http << "http://";

  if (!p.user.empty())
    http << p.user << ':' << p.password << '@';

  http << p.host << "/axis-cgi/mjpg/video.cgi";

  if (p.frame_width > 0 && p.frame_height > 0)
  {
    http << "?resolution=" << p.frame_width << 'x' << p.frame_height;
    if (p.frame_rate > 0)
      http << "&fps=" << p.frame_rate;
  }
  else
  {
    if (p.frame_rate > 0)
      http << "?fps=" << p.frame_rate;
  }
  return http.str();
}


std::string GetAxisUrl(const IpCameraSinkParams &p)
{
  if (p.application_protocol == IpApplicationProtocol::HTTP)
    return GetAxisHttpUrl(p);
  if (p.application_protocol == IpApplicationProtocol::RTSP)
    return GetAxisRtspUrl(p);
  VCP_ERROR("Protocol type for Axis streaming must be HTTP or RTSP");
}


//std::string GetMobotixUrl(const pvt::icc::ipcam::IpCameraParams &p)
//{
//  if (p.protocol == ipcam::IpApplicationProtocol::HTTP && p.stream_type == ipcam::IpStreamEncoding::MJPEG)
//  {
//    std::stringstream url;
//    url << "http://";
//    if (!p.user.empty())
//      url << p.user << ":" << p.password << "@";
//    url << p.host << "/cgi-bin/faststream.jpg?stream=full&needlength&fps=" << std::setprecision(1) << static_cast<double>(p.frame_rate);
//    return url.str();
//  }
//  PVT_ABORT("Currently, we only support (or have tested) MJPEG over HTTP for Mobotix cameras. You requested " << p.stream_type << " over " << p.protocol);
//}


//std::string GetHikvisionUrl(const pvt::icc::ipcam::IpCameraParams &p)
//{
//  if (p.protocol == ipcam::IpApplicationProtocol::RTSP && p.stream_type == ipcam::IpStreamEncoding::H264)
//  {
//    std::stringstream url;
//    url << "rtsp://";
//    if (!p.user.empty())
//      url << p.user << ":" << p.password << "@";
//    url << p.host << "/Streaming/Channels/1";
//    return url.str();
//  }
//  PVT_ABORT("Currently, we only support H264 over RTSP for Hikvision cameras (use camera's H264+ setting).");
//}


std::ostream &operator<<(std::ostream &stream, const IpApplicationProtocol &s)
{
  stream << IpApplicationProtocolToString(s);
  return stream;
}


std::string IpApplicationProtocolToString(const IpApplicationProtocol &p)
{
  switch(p)
  {
    case IpApplicationProtocol::HTTP:
      return "http";
    case IpApplicationProtocol::RTSP:
      return "rtsp";
    default:
      VCP_ERROR("IP application protocol '" << static_cast<int>(p) << "' is not yet mapped.");
  }
}


IpApplicationProtocol IpApplicationProtocolFromString(const std::string &protocol)
{
  std::string lower(protocol);
  vcp::utils::string::ToLower(lower);
  if (lower.compare("http") == 0)
    return IpApplicationProtocol::HTTP;
  if (lower.compare("rtsp") == 0)
    return IpApplicationProtocol::RTSP;

  VCP_ERROR("IP application protocol '" << protocol << "' not yet mapped.");
}



std::ostream &operator<<(std::ostream &stream, const IpTransportProtocol &s)
{
  stream << IpTransportProtocolToString(s);
  return stream;
}


std::string IpTransportProtocolToString(const IpTransportProtocol &p)
{
  switch(p)
  {
    case IpTransportProtocol::TCP:
      return "tcp";
    case IpTransportProtocol::UDP:
      return "udp";
    default:
      VCP_ERROR("IP transport protocol '" << static_cast<int>(p) << "' is not yet mapped.");
  }
}


IpTransportProtocol IpTransportProtocolFromString(const std::string &protocol)
{
  std::string lower(protocol);
  vcp::utils::string::ToLower(lower);
  if (lower.compare("tcp") == 0)
    return IpTransportProtocol::TCP;
  if (lower.compare("udp") == 0)
    return IpTransportProtocol::UDP;

  VCP_ERROR("IP transport protocol '" << protocol << "' not yet mapped.");
}


std::ostream &operator<<(std::ostream &stream, const IpStreamEncoding &s)
{
  stream << IpStreamEncodingToString(s);
  return stream;
}



IpStreamEncoding IpStreamEncodingFromString(const std::string &stream_type)
{
  std::string lower(stream_type);
  vcp::utils::string::ToLower(lower);
  if (lower.compare("h264") == 0)
    return IpStreamEncoding::H264;
  if (lower.compare("mjpeg") == 0
      || lower.compare("mjpg") == 0)
    return IpStreamEncoding::MJPEG;

  VCP_ERROR("IpStreamEncoding '" << stream_type << "' is not yet mapped.");
}


std::string IpStreamEncodingToString(const IpStreamEncoding &s)
{
  switch(s)
  {
    case IpStreamEncoding::H264:
      return "h264";
    case IpStreamEncoding::MJPEG:
      return "mjpeg";
    default:
      VCP_ERROR("IpStreamEncoding '" << static_cast<int>(s) << "' is not yet mapped.");
  }
}


std::ostream &operator<<(std::ostream &stream, const IpCameraType &cam)
{
  stream << IpCameraTypeToString(cam);
  return stream;
}


IpCameraType IpCameraTypeFromString(const std::string &camera_type)
{
  std::string lower(camera_type);
  vcp::utils::string::ToLower(lower);
  if (vcp::utils::string::StartsWith(lower, "ipcam"))
    return IpCameraType::Generic;
  if (vcp::utils::string::StartsWith(lower, "axis"))
    return IpCameraType::Axis;
//  if (vcp::utils::string::StartsWith(lower, "mobotix"))
//    return IpCameraType::Mobotix;
//  if (vcp::utils::string::StartsWith(lower, "hikvision"))
//    return IpCameraType::Hikvision;

  VCP_ERROR("IpCameraType '" << camera_type << "' is not yet mapped.");
}


std::string IpCameraTypeToString(const IpCameraType &c)
{
  switch(c)
  {
    case IpCameraType::Generic:
      return "ipcam";
    case IpCameraType::Axis:
      return "axis";
//    case IpCameraType::Mobotix:
//      return "mobotix";
//    case IpCameraType::Hikvision:
//      return "hikvision";
    default:
      VCP_ERROR("IpCameraType '" << static_cast<int>(c) << "' is not yet mapped.");
  }
}


std::ostream &operator<< (std::ostream &out, const IpCameraSinkParams &p)
{
  out << p.host << " " << IpApplicationProtocolToString(p.application_protocol) << "/" << IpStreamEncodingToString(p.stream_encoding)
      << " (" << IpTransportProtocolToString(p.transport_protocol) << ")";

  if (p.frame_width > 0 && p.frame_height > 0)
    out << ", " << p.frame_width << " x " << p.frame_height;
  if (p.frame_rate > 0)
    out << ", @" << p.frame_rate << " fps";
  out << ", " << vcp::utils::string::ObscureUrlAuthentication(p.stream_url);
  return out;
}

std::string GetStreamingUrl(const IpCameraSinkParams &p)
{
  // If you need to search for a new streaming URL, you might find it at https://www.ispyconnect.com/man.aspx?n=Siqura
  switch(p.ipcam_type)
  {
    case ipcam::IpCameraType::Generic:
      return p.stream_url;
    case ipcam::IpCameraType::Axis:
      return GetAxisUrl(p);
//    case ipcam::IpCameraType::Mobotix:
//      return GetMobotixUrl(this);
//    case ipcam::IpCameraType::Hikvision:
//      return GetHikvisionUrl(this);
    default:
      VCP_ERROR("URL lookup for IpCameraType '" << p.ipcam_type << "' is not yet implemented.");
  }
}

// Internal helper
IpCameraSinkParams ParseIpCameraSinkParams(const vcp::config::ConfigParams &config,
                                           const std::string &cam_param,
                                           const std::string &postfix,
                                           std::vector<std::string> &configured_keys)
{
  SinkParams sink_params = ParseBaseSinkParamsFromConfig(config, cam_param, configured_keys);
  // Currently, we only support monocular devices (TODO this may change with mobotix
  // which iirc streams concatenated images - gotta check!)
  sink_params.frame_type = FrameType::MONOCULAR;

  const std::string host = config.GetString(cam_param + ".host" + postfix);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "host" + postfix), configured_keys.end());

  // In a stereo setup, user/pwd can be given once ("user"/"password") or for each device separately ("user_left"/"password_left")
  std::string user = std::string();
  if (config.SettingExists(cam_param + ".user" + postfix))
  {
    user = config.GetString(cam_param + ".user" + postfix);
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "user" + postfix), configured_keys.end());
  }
  else if (config.SettingExists(cam_param + ".user"))
  {
    user = config.GetString(cam_param + ".user");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "user"), configured_keys.end());
  }

  std::string pwd = std::string();
  if (config.SettingExists(cam_param + ".password" + postfix))
  {
    pwd = config.GetString(cam_param + ".password" + postfix);
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "password" + postfix), configured_keys.end());
  }
  else if (config.SettingExists(cam_param + ".pwd" + postfix))
  {
    pwd = config.GetString(cam_param + ".pwd" + postfix);
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "pwd" + postfix), configured_keys.end());
  }
  else if (config.SettingExists(cam_param + ".password"))
  {
    pwd = config.GetString(cam_param + ".password");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "password"), configured_keys.end());
  }
  else if (config.SettingExists(cam_param + ".pwd"))
  {
    pwd = config.GetString(cam_param + ".pwd");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "pwd"), configured_keys.end());
  }

  const std::string stream_url =
      config.SettingExists(cam_param + ".stream_url" + postfix) ? config.GetString(cam_param + ".stream_url" + postfix) : std::string();
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "stream_url" + postfix), configured_keys.end());

  const IpCameraType ipcam_type = IpCameraTypeFromString(GetSinkTypeStringFromConfig(config, cam_param, &configured_keys));

  const IpApplicationProtocol app_protocol =
      config.SettingExists(cam_param + ".application_protocol") ?
        IpApplicationProtocolFromString(config.GetString(cam_param + ".application_protocol"))
      : IpApplicationProtocol::RTSP;
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "application_protocol"), configured_keys.end());

  const IpTransportProtocol transport_protocol =
      config.SettingExists(cam_param + ".transport_protocol") ?
        IpTransportProtocolFromString(config.GetString(cam_param + ".transport_protocol"))
      : IpTransportProtocol::TCP;
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "transport_protocol"), configured_keys.end());

  const IpStreamEncoding stream_encoding =
      config.SettingExists(cam_param + ".encoding") ?
        IpStreamEncodingFromString(config.GetString(cam_param + ".encoding"))
      : IpStreamEncoding::H264;
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "encoding"), configured_keys.end());

  const int frame_width =
      config.SettingExists(cam_param + ".frame_width") ?
        config.GetInteger(cam_param + ".frame_width") : -1;
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "frame_width"), configured_keys.end());

  const int frame_height =
      config.SettingExists(cam_param + ".frame_height") ?
        config.GetInteger(cam_param + ".frame_height") : -1;
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "frame_height"), configured_keys.end());

  int frame_rate = -1;
  if (config.SettingExists(cam_param + ".frame_rate"))
  {
    frame_rate = config.GetInteger(cam_param + ".frame_rate");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "frame_rate"), configured_keys.end());
  }
  else if (config.SettingExists(cam_param + ".fps"))
  {
    frame_rate = config.GetInteger(cam_param + ".fps");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "fps"), configured_keys.end());
  }

  IpCameraSinkParams params(sink_params, host, user, pwd, ipcam_type, app_protocol, transport_protocol, stream_encoding, frame_width, frame_height, frame_rate, stream_url);
  params.stream_url = stream_url.empty() ? GetStreamingUrl(params) : stream_url;

  return params;
}


IpCameraSinkParams MonocularIpCameraSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param)
{
  std::vector<std::string> configured_keys = config.ListConfigGroupParameters(cam_param);
  IpCameraSinkParams params = ParseIpCameraSinkParams(config, cam_param, std::string(), configured_keys);
  WarnOfUnusedParameters(cam_param, configured_keys);
  return params;
}


std::pair<IpCameraSinkParams, IpCameraSinkParams> StereoIpCameraSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param)
{
  std::vector<std::string> configured_keys = config.ListConfigGroupParameters(cam_param);
  IpCameraSinkParams left = ParseIpCameraSinkParams(config, cam_param, "_left", configured_keys);
  IpCameraSinkParams right = ParseIpCameraSinkParams(config, cam_param, "_right", configured_keys);
  left.sink_label = left.sink_label + "-left";
  right.sink_label = right.sink_label + "-right";

  WarnOfUnusedParameters(cam_param, configured_keys);
  return std::make_pair(left, right);
}


/**
 * @brief The GenericIpCameraSink class
 *
 * Currently, we support HTTP/MJPEG, RTSP/H264, RTSP/MJPEG streams.
 * Streams may be re-ordered by their protocol, i.e. Next() returns frames of HTTP
 * cameras first, then RTSP.
 * Anyways, you should be aware that mixing both HTTP and RTSP sources is not the
 * best idea.
 *
 * Axis streaming URLs that worked (test yourself with vlc <url>):
 * * http://user:pwd@192.168.0.50/axis-cgi/mjpg/video.cgi?resolution=320x240&fps=10
 * * rtsp://user:pwd@192.168.0.47/axis-media/media.amp?videocodec=h264&resolution=480x270&fps=25
 * * rtsp://user:pwd@192.168.0.91/axis-media/media.amp?videocodec=jpeg&resolution=480x270&fps=25
 *
 * Our Axis didn't support MPEG4 over RTSP (so we didn't implement it):
 */
class GenericIpCameraSink : public StreamSink
{
public:

  GenericIpCameraSink(const std::vector<IpCameraSinkParams> &params)
    : StreamSink()
  {
    // Group parameters by their protocol
    for (const auto &p : params)
    {
      switch(p.application_protocol)
      {
        case IpApplicationProtocol::HTTP:
          params_http_.push_back(p);
          break;

        case IpApplicationProtocol::RTSP:
          params_rtsp_.push_back(p);
          break;

        default:
          VCP_ERROR("IP camera protocol '" << p.application_protocol << "' is not yet supported.");
      }
    }
  }

  virtual ~GenericIpCameraSink()
  {
    CloseDevice();
  }


  int IsDeviceAvailable() const override
  {
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      if (!sinks_[i]->IsDeviceAvailable())
        return 0;
    }
    return 1;
  }


  int IsFrameAvailable() const override
  {
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      if (!sinks_[i]->IsFrameAvailable())
        return 0;
    }
    return 1;
  }


  size_t NumAvailableFrames() const override
  {
    size_t num = 0;
    for (const auto &s : sinks_)
      num += static_cast<size_t>(s->IsFrameAvailable());
    return num;
  }


  size_t NumStreams() const override
  {
    return params_http_.size() + params_rtsp_.size();
  }


  FrameType FrameTypeAt(size_t stream_index) const override
  {
    if (stream_index < params_http_.size())
      return params_http_[stream_index].frame_type;
    else
      return params_rtsp_[stream_index - params_http_.size()].frame_type;
  }


  SinkParams SinkParamsAt(size_t stream_index) const override
  {
    if (stream_index < params_http_.size())
      return params_http_[stream_index];
    else
      return params_rtsp_[stream_index - params_http_.size()];
  }

  size_t NumDevices() const override
  {
    std::set<std::string> unique_hosts;
    for (const auto &p : params_http_)
      unique_hosts.insert(p.host);
    for (const auto &p : params_rtsp_)
      unique_hosts.insert(p.host);
    return unique_hosts.size();
  }


  std::string StreamLabel(size_t stream_index) const override
  {
    if (stream_index < params_http_.size())
      return params_http_[stream_index].sink_label;
    else
      return params_rtsp_[stream_index - params_http_.size()].sink_label;
  }


  bool OpenDevice() override
  {
    for (const auto &p : params_http_)
    {
#ifdef VCP_BEST_WITH_IPCAM_HTTP
      if (p.stream_encoding != IpStreamEncoding::MJPEG)
        VCP_ERROR("Currently, we only support MJPEG over HTTP, not '" << p.stream_encoding << "'.");

      if (p.stream_url.empty())
        VCP_ERROR("Invalid/empty streaming URL for IP camera: " << p);

      sinks_.push_back(http::CreateHttpMjpegSink<VCP_BEST_STREAM_BUFFER_CAPACITY>(p));
      // TODO nice-to-have would be a camera-specific start-up routine (e.g. enabling/disabling overlays, etc.)
#else // VCP_BEST_WITH_IPCAM_HTTP
      VCP_ERROR("You need to compile VCP with HTTP streaming enabled!");
#endif // VCP_BEST_WITH_IPCAM_HTTP
    }


    if (!params_rtsp_.empty())
    {
#ifdef VCP_BEST_WITH_IPCAM_RTSP
      sinks_.push_back(std::move(rtsp::CreateMultiRtspStreamSink<VCP_BEST_STREAM_BUFFER_CAPACITY>(params_rtsp_)));
#else // VCP_BEST_WITH_IPCAM_RTSP
      VCP_ERROR("You need to compile VCP with RTSP streaming enabled!");
#endif // VCP_BEST_WITH_IPCAM_RTSP
    }

    // Finally, open the actual streams/devices.
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success &= sinks_[i]->OpenDevice();
    return success;
  }

  bool CloseDevice() override
  {
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success &= sinks_[i]->CloseDevice();
    return success;
  }

  bool StartStreaming() override
  {
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success &= sinks_[i]->StartStreaming();
    return success;
  }

  bool StopStreaming() override
  {
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success &= sinks_[i]->StopStreaming();
    return success;
  }

  std::vector<cv::Mat> Next() override
  {
    std::vector<cv::Mat> frames;
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      // HttpMjpegSinks yield one frame each.
      // Fort RTSP, however, we have one "multi-sink"
      // which sets up the streaming environment and does
      // all the nasty stuff to get the decoded images.
      // Then, this "multi-sink" returns all frames in a vector.
      //FIXME simplify rtsp sink: no image concatenation anymore!
      const std::vector<cv::Mat> query = sinks_[i]->Next();
      frames.insert(frames.end(), query.begin(), query.end());
    }
    return frames;
  }


private:
  std::vector<IpCameraSinkParams> params_http_;
  std::vector<IpCameraSinkParams> params_rtsp_;
  std::vector<std::unique_ptr<StreamSink>> sinks_;
};


/*
class GenericStereoIpCamSink : public StreamSink
{
public:
  virtual ~GenericStereoIpCamSink() {}

  GenericStereoIpCamSink(const std::vector<StereoIpCameraSinkParams> &params, std::string (*GetUrlFn)(const pvt::icc::ipcam::IpCameraParams &)=GetIpCameraUrl)
    : StreamSink()
  {
    // Check protocol
    num_stereo_cameras_ = params.size();
    if (params[0].cam1_params.protocol == pvt::icc::ipcam::IpApplicationProtocol::HTTP)
    {
#ifdef VCP_BEST_WITH_IPCAM_HTTP
      is_rtsp_ = false;
      if (params[0].cam1_params.stream_type != pvt::icc::ipcam::IpStreamEncoding::MJPEG)
        PVT_ABORT("Cannot stream '" << ipcam::IpStreamEncodingToString(params[0].cam1_params.stream_type) << "' over HTTP!");

      for (const auto &p : params)
      {
        if ((p.cam1_params.protocol != params[0].cam1_params.protocol) ||
            (p.cam2_params.protocol != params[0].cam1_params.protocol))
          PVT_ABORT("All IP stereo cameras must use the same streaming protocol!");

        const std::string url_left = GetUrlFn(p.cam1_params);
        const std::string url_right = GetUrlFn(p.cam2_params);

        PVT_LOG_INFO("Connecting to '" << p.cam1_params.camera_type << "' stereo setup at: "
                     << pvt::utils::string::ObscureUrlAuthentication(url_left) << " and "
                     << pvt::utils::string::ObscureUrlAuthentication(url_right));
        sinks_.push_back(pvt::icc::CreateHttpMjpegSink<PVT_ICC_CIRCULAR_SINK_BUFFER_CAPACITY>(url_left));
        sinks_.push_back(pvt::icc::CreateHttpMjpegSink<PVT_ICC_CIRCULAR_SINK_BUFFER_CAPACITY>(url_right));
      }
#else // VCP_BEST_WITH_IPCAM_HTTP
      PVT_ABORT("You need to compile PVT with HTTP streaming enabled!");
#endif // VCP_BEST_WITH_IPCAM_HTTP
    }
    else if (params[0].cam1_params.protocol == pvt::icc::ipcam::IpApplicationProtocol::RTSP)
    {
#ifdef VCP_BEST_WITH_IPCAM_RTSP
      is_rtsp_ = true;
      // Due to whatever reason, RTSP streaming with live555 only works with one
      // sink (but multiple clients).
      // So we just set up a single sink and store the frame splits.
      int split_column = 0;
      std::vector<pvt::icc::RtspStreamParams> cam_params;
      for (const auto &p : params)
      {
        // Left camera
        if ((p.cam1_params.protocol != params[0].cam1_params.protocol) ||
            (p.cam2_params.protocol != params[0].cam1_params.protocol))
          PVT_ABORT("All IP cameras must use the same streaming protocol!");

        // RTSP supports MJPEG and H264
        pvt::icc::RtspStreamType stream_type;
        if (p.cam1_params.stream_type == pvt::icc::ipcam::IpStreamEncoding::MJPEG)
          stream_type = pvt::icc::RtspStreamType::MJPEG;
        else if (p.cam1_params.stream_type == pvt::icc::ipcam::IpStreamEncoding::H264)
          stream_type = pvt::icc::RtspStreamType::H264;
        else
          PVT_ABORT("Stream type '" << ipcam::IpStreamEncodingToString(p.cam1_params.stream_type) << "' not supported!");

        pvt::icc::RtspStreamParams cam_param_left;
        cam_param_left.stream_url = GetUrlFn(p.cam1_params);
        cam_param_left.stream_type = stream_type;
        cam_param_left.frame_width = p.cam1_params.frame_width;
        cam_param_left.frame_height = p.cam1_params.frame_height;
        // UDP is preferred but our Axis cams will kill the RTSP over UDP streams
        // after a few seconds/minutes. Thus, we accept the TCP overhead (as long
        // as we can reliably stream our data).
        cam_param_left.protocol = pvt::icc::RtspProtocol::TCP;


        // Right camera
        if (p.cam2_params.stream_type != params[0].cam1_params.stream_type)
          PVT_ABORT("All IP cameras must use the same stream type (Video encoding)!");

        pvt::icc::RtspStreamParams cam_param_right;
        cam_param_right.stream_url = GetUrlFn(p.cam2_params);
        cam_param_right.stream_type = stream_type;
        cam_param_right.frame_width = p.cam2_params.frame_width;
        cam_param_right.frame_height = p.cam2_params.frame_height;
        cam_param_right.protocol = cam_param_left.protocol;


        PVT_LOG_INFO("Connecting to '" << p.cam1_params.camera_type << "' stereo setup at: "
                     << pvt::utils::string::ObscureUrlAuthentication(cam_param_left.stream_url) << " and "
                     << pvt::utils::string::ObscureUrlAuthentication(cam_param_right.stream_url));
        cam_params.push_back(cam_param_left);
        cam_params.push_back(cam_param_right);
        split_column += (cam_param_left.frame_width + cam_param_right.frame_width);
        multi_rtsp_splits_.push_back(split_column);
      }
      sinks_.push_back(pvt::icc::CreateMultiRtspStreamSink<PVT_ICC_CIRCULAR_SINK_BUFFER_CAPACITY>(cam_params));
#else // VCP_BEST_WITH_IPCAM_RTSP
      PVT_ABORT("You need to compile PVT with RTSP streaming enabled!");
#endif // VCP_BEST_WITH_IPCAM_RTSP
    }
    else
      VCP_ERROR("Protocol not supported/implemented for our Axis stereo captures!");
  }


  int IsDeviceAvailable() const override
  {
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      if (!sinks_[i]->IsDeviceAvailable())
        return 0;
    }
    return 1;
  }


  int IsFrameAvailable() const override
  {
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      if (!sinks_[i]->IsFrameAvailable())
        return 0;
    }
    return 1;
  }

  bool OpenDevice() override
  {
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success &= sinks_[i]->OpenDevice();
    return success;
  }

  bool CloseDevice() override
  {
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success &= sinks_[i]->CloseDevice();
    return success;
  }

  bool StartStreaming() override
  {
    bool success = true;
    for (size_t i = 0; i < sinks_.size(); ++i)
      success &= sinks_[i]->StartStreaming();
    return success;
  }

  std::vector<cv::Mat> Next() override
  {
    std::vector<cv::Mat> frames;
    frames.reserve(num_stereo_cameras_);

    if (is_rtsp_)
    {
      // Currently, there should only be 1 MultiRtspSink - otherwise, the streaming
      // gets pretty unreliable (as tested in our setups).
      // Since this might change (e.g. if we update live555 or switch to a different
      // streaming library), we still loop over all sinks.
      for (size_t i = 0; i < sinks_.size(); ++i)
      {
        cv::Mat frame;
        sinks_[i]->GetNextFrame(frame);

        if (multi_rtsp_splits_.size() < 2)
        {
          frames.push_back(frame);
        }
        else
        {
          // We use a multi-rtsp cue with multiple streams.
          // Thus, we have to split the incoming frame.
          int from = 0;
          for (size_t j = 0; j < multi_rtsp_splits_.size(); ++j)
          {
            if (frame.empty())
            {
              frames.push_back(frame);
            }
            else
            {
              int to = multi_rtsp_splits_[j];
              const cv::Rect roi(from, 0, to-from, frame.rows);
              cv::Mat split = frame(roi).clone();
              frames.push_back(split);
              from = to;
            }
          }
        }
      }
    }
    else
    {
      // With HTTP streams, we have 1 camera per sink, so the first
      // stereo camera consists of sinks[0] and sinks[1], etc.

      for (size_t i = 0; i < sinks_.size(); i+=2)
      {
        // Concatenate left and right frame
        cv::Mat left, right;
        sinks_[i]->GetNextFrame(left);
        sinks_[i+1]->GetNextFrame(right);
        frames.push_back(pvt::imutils::ColumnStack(left, right));
      }
    }

    return frames;
  }

private:
  size_t num_stereo_cameras_;
  bool is_rtsp_;
  std::vector<int> multi_rtsp_splits_;
  std::vector<std::unique_ptr<StreamSink>> sinks_;
};
//FIXME!
*/




std::unique_ptr<StreamSink> CreateIpCameraSink(const std::vector<IpCameraSinkParams> &params)
{
  if (params.empty())
    return nullptr;
  return std::unique_ptr<GenericIpCameraSink>(new GenericIpCameraSink(params));
}


//FIXMEstd::unique_ptr<StreamSink> CreateStereoIpCamerSink(const std::vector<StereoIpCameraSinkParams> &params)
//{
//  return std::unique_ptr<GenericStereoIpCamSink>(new GenericStereoIpCamSink(params, GetIpCameraUrl));
//}


bool IsGenericIpCameraMonocular(const std::string &camera_type)
{
  std::string lower(camera_type);
  vcp::utils::string::ToLower(lower);
  return lower.compare("ipcam") == 0 || lower.compare("ipcamera") == 0
      || lower.compare("ipcam-monocular") == 0 || lower.compare("ipcamera-monocular") == 0
      || lower.compare("ipcam-mono") == 0 || lower.compare("ipcamera-mono") == 0;
}

bool IsGenericIpCameraStereo(const std::string &camera_type)
{
  std::string lower(camera_type);
  vcp::utils::string::ToLower(lower);
  return lower.compare("ipcam-stereo") == 0 || lower.compare("ipcamera-stereo") == 0;
}

bool IsAxisMonocular(const std::string &camera_type)
{
  std::string lower(camera_type);
  vcp::utils::string::ToLower(lower);
  return lower.compare("axis") == 0 || lower.compare("axis-mono") == 0 || lower.compare("axis-monocular") == 0;
}

bool IsAxisStereo(const std::string &camera_type)
{
  std::string lower(camera_type);
  vcp::utils::string::ToLower(lower);
  return lower.compare("axis-stereo") == 0;
}

//bool IsMobotix(const std::string &camera_type)
//{
//  return camera_type.compare("mobotix") == 0;
//}
//bool IsHikvision(const std::string &camera_type) { return camera_type.compare("hikvision") == 0; }

bool IsMonocularIpCamera(const std::string &camera_type)
{
  return IsGenericIpCameraMonocular(camera_type) ||
      IsAxisMonocular(camera_type);
//      || IsMobotix(camera_type)
//      || IsHikvision(camera_type);
}


bool IsStereoIpCamera(const std::string &camera_type)
{
  return IsGenericIpCameraStereo(camera_type) ||
      IsAxisStereo(camera_type);
}


bool IsIpCamera(const std::string &camera_type)
{
  return IsMonocularIpCamera(camera_type) || IsStereoIpCamera(camera_type);
}

} // namespace ipcam
} // namespace best
} // namespace vcp
