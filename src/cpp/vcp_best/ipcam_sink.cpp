#include "ipcam_sink.h"

#include <vcp_utils/vcp_logging.h>
#include <sstream>
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
std::string GetAxisRtspUrl(const IpCameraDeviceParams &p)
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
  rtsp << "?videocodec=" << codec << "&resolution=" << p.frame_width << "x" << p.frame_height << "&fps=" << p.frame_rate << "&audio=0";
  // Append &clock=1 if you want a clock overlay

  // If you increase the Axis parameter videokeyframeinterval, this causes more frequent
  // I-Frames, but leads to more dropped frames/frames which can't be decoded
  // (seems like a general cam problem 2015-12-01). 16 worked like a charm with our Axis IP cams.
  if (p.stream_encoding == IpStreamEncoding::H264)
    rtsp << "&videokeyframeinterval=16";
  return rtsp.str();
}


std::string GetAxisHttpUrl(const IpCameraDeviceParams &p)
{
  if (p.stream_encoding != IpStreamEncoding::MJPEG)
    VCP_ERROR("Axis can only stream MJPEG over HTTP, either change stream type or protocol!");

  std::stringstream http;
  http << "http://";

  if (!p.user.empty())
    http << p.user << ':' << p.password << '@';

  http << p.host << "/axis-cgi/mjpg/video.cgi";
  http << "?resolution=" << p.frame_width << 'x' << p.frame_height << "&fps=" << p.frame_rate;
  return http.str();
}


std::string GetAxisUrl(const IpCameraDeviceParams &p)
{
  if (p.protocol == IpProtocol::HTTP)
    return GetAxisHttpUrl(p);
  if (p.protocol == IpProtocol::RTSP)
    return GetAxisRtspUrl(p);
  VCP_ERROR("Protocol type for Axis streaming must be HTTP or RTSP");
}


//std::string GetMobotixUrl(const pvt::icc::ipcam::IpCameraParams &p)
//{
//  if (p.protocol == ipcam::IpProtocol::HTTP && p.stream_type == ipcam::IpStreamEncoding::MJPEG)
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
//  if (p.protocol == ipcam::IpProtocol::RTSP && p.stream_type == ipcam::IpStreamEncoding::H264)
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


std::ostream &operator<<(std::ostream &stream, const IpProtocol &s)
{
  stream << IpProtocolToString(s);
  return stream;
}


std::string IpProtocolToString(const IpProtocol &p)
{
  switch(p)
  {
    case IpProtocol::HTTP:
      return "http";
    case IpProtocol::RTSP:
      return "rtsp";
    default:
      VCP_ERROR("IP protocol '" << static_cast<int>(p) << "' is not yet mapped.");
  }
}


IpProtocol IpProtocolFromString(const std::string &protocol)
{
  std::string lower(protocol);
  vcp::utils::string::ToLower(lower);
  if (lower.compare("http") == 0)
    return IpProtocol::HTTP;
  if (lower.compare("rtsp") == 0)
    return IpProtocol::RTSP;

  VCP_ERROR("Protocol '" << protocol << "' not yet mapped.");
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
  if (lower.compare("mjpeg") == 0)
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


std::ostream &operator<< (std::ostream &out, const IpCameraDeviceParams &p)
{
  out << "IP Camera: " << p.host << IpProtocolToString(p.protocol) << "/" << IpStreamEncodingToString(p.stream_encoding);
  if (p.frame_width > 0 && p.frame_height > 0)
    out << ", " << p.frame_width << " x " << p.frame_height;
  if (p.frame_rate > 0)
    out << ", @" << p.frame_rate << " fps";
  return out;
}

std::string IpCameraDeviceParams::GetStreamingUrl() const
{
  // If you need to find a new streaming URL, you might find it at https://www.ispyconnect.com/man.aspx?n=Siqura
  switch(ipcam_type)
  {
    case ipcam::IpCameraType::Generic:
      return custom_url;
    case ipcam::IpCameraType::Axis:
      return GetAxisUrl(*this);
//    case ipcam::IpCameraType::Mobotix:
//      return GetMobotixUrl(this);
//    case ipcam::IpCameraType::Hikvision:
//      return GetHikvisionUrl(this);
    default:
      VCP_ERROR("URL lookup for IpCameraType '" << ipcam_type << "' is not yet implemented.");
  }
}


IpCameraDeviceParams ParseIpCameraDeviceParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &prefix, const std::string &postfix, std::vector<std::string> &configured_keys)
{
  const std::string host = config.GetString(prefix + ".host" + postfix);
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "host" + postfix), configured_keys.end());


  // In a stereo setup, user/pwd can be given once ("user"/"password") or for each device separately ("user_left"/"password_left")
  std::string user = std::string();
  if (config.SettingExists(prefix + ".user" + postfix))
  {
    user = config.GetString(prefix + ".user" + postfix);
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "user" + postfix), configured_keys.end());
  }
  else if (config.SettingExists(prefix + ".user"))
  {
    user = config.GetString(prefix + ".user");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "user"), configured_keys.end());
  }

  std::string pwd = std::string();
  if (config.SettingExists(prefix + ".password" + postfix))
  {
    user = config.GetString(prefix + ".password" + postfix);
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "password" + postfix), configured_keys.end());
  }
  else if (config.SettingExists(prefix + ".pwd" + postfix))
  {
    user = config.GetString(prefix + ".pwd" + postfix);
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "pwd" + postfix), configured_keys.end());
  }
  else if (config.SettingExists(prefix + ".password"))
  {
    user = config.GetString(prefix + ".password");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "password"), configured_keys.end());
  }
  else if (config.SettingExists(prefix + ".pwd"))
  {
    user = config.GetString(prefix + ".pwd");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "pwd"), configured_keys.end());
  }

  const IpCameraType ipcam_type = IpCameraTypeFromString(GetSinkTypeStringFromConfig(config, prefix, &configured_keys));

  const IpProtocol protocol =
      config.SettingExists(prefix + ".protocol") ? IpProtocolFromString(config.GetString(prefix + ".protocol")) : IpProtocol::RTSP;
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "protocol"), configured_keys.end());

  const IpStreamEncoding stream_encoding =
      config.SettingExists(prefix + ".encoding") ? IpStreamEncodingFromString(config.GetString(prefix + ".encoding")) : IpStreamEncoding::H264;
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "encoding"), configured_keys.end());

  const int frame_width =
      config.SettingExists(prefix + ".frame_width") ? config.GetInteger(prefix + ".frame_width") : -1;
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "frame_width"), configured_keys.end());

  const int frame_height =
      config.SettingExists(prefix + ".frame_height") ? config.GetInteger(prefix + ".frame_height") : -1;
  configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "frame_height"), configured_keys.end());

  int frame_rate = -1;
  if (config.SettingExists(prefix + ".frame_rate"))
  {
    frame_rate = config.GetInteger(prefix + ".frame_rate");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "frame_rate"), configured_keys.end());
  }
  else if (config.SettingExists(prefix + ".fps"))
  {
    frame_rate = config.GetInteger(prefix + ".fps");
    configured_keys.erase(std::remove(configured_keys.begin(), configured_keys.end(), "fps"), configured_keys.end());
  }
  return IpCameraDeviceParams(host, user, pwd, ipcam_type, protocol, stream_encoding, frame_width, frame_height, frame_rate);
}

MonocularIpCameraSinkParams MonocularIpCameraSinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param)
{
  std::vector<std::string> configured_keys = config.ListConfigGroupParameters(cam_param);
  const SinkParams sink_params = ParseBaseSinkParamsFromConfig(config, cam_param, configured_keys);
  const IpCameraDeviceParams ipcam_params = ParseIpCameraDeviceParamsFromConfig(config, cam_param, std::string(), configured_keys);

  WarnOfUnusedParameters(cam_param, configured_keys);

  return MonocularIpCameraSinkParams(sink_params, ipcam_params);
}
//FIXME stereo: parseipcam... postfix = '_left'
//IpCameraParams IpCameraParamsFromConfigSetting(const pvt::config::ConfigParams &config, const std::string &setting)
//{
//  vcp::best::ipcam::IpCameraParams p;
//  // Get the camera type:
//  p.camera_type = ipcam::IpCameraTypeFromString(config.GetString(setting + ".type"));

//  // How should we retrieve the IP camera's stream?
//  p.protocol = ipcam::IpProtocolFromString(config.GetString(setting + ".protocol"));
//  p.stream_type = ipcam::IpStreamEncodingFromString(config.GetString(setting + ".stream_type"));


//  if (p.camera_type == ipcam::IpCameraType::Generic)
//  {
//    p.custom_url = config.GetString(setting + ".custom_url");
//  }
//  else
//  {
//    const std::string k_host = setting + ".host";
//    p.host = config.GetString(k_host);

//    const std::string k_usr = setting + ".user";
//    if (config.SettingExists(k_usr))
//    {
//      p.user = config.GetString(k_usr);

//      // If there's a user, there must be a password!
//      p.password = config.GetString(setting + ".password");
//    }
//  }

//  // Details about the stream:
//  p.frame_height = config.GetInteger(setting + ".frame_height");
//  p.frame_width = config.GetInteger(setting + ".frame_width");
//  p.frame_rate = config.GetInteger(setting + ".frame_rate");

//  return p;
//}


//IpStereoParams IpStereoParamsFromConfigSetting(const pvt::config::ConfigParams &config, const std::string &setting)
//{
//  pvt::icc::ipcam::IpStereoParams p;
//  // Get the camera type:
//  p.left.camera_type = ipcam::IpCameraTypeFromString(config.GetString(setting + ".type"));
//  p.right.camera_type = p.left.camera_type;

//  if (p.left.camera_type == ipcam::IpCameraType::Generic)
//  {
//    p.left.custom_url = config.GetString(setting + ".custom_url_left");
//    p.right.custom_url = config.GetString(setting + ".custom_url_right");
//  }
//  else
//  {
//    p.left.host = config.GetString(setting + ".host_left");

//    // left camera
//    const std::string k_usr_left = setting + ".user_left";
//    if (config.SettingExists(k_usr_left))
//    {
//      p.left.user = config.GetString(k_usr_left);

//      // If there's a user, there must be a password!
//      p.left.password = config.GetString(setting + ".password_left");
//    }


//    // right camera
//    p.right.host = config.GetString(setting + ".host_right");
//    const std::string k_usr = setting + ".user_right";
//    if (config.SettingExists(k_usr))
//    {
//      p.right.user = config.GetString(k_usr);
//      p.right.password = config.GetString(setting + ".password_right");
//    }
//  }

//  // For now, the remaining params (streaming type, etc) are the same for both cameras in the stereo setup!
//  // IF YOU WANT TO CHANGE THAT (i.e. allow different streaming types for
//  // the two cameras of the stereo setup), ALSO CHANGE THE C'TOR OF
//  // pvt::tools::capture::axis::AxisStereoCapture !!!!

//  // How should we retrieve the IP camera's stream?
//  p.left.protocol = ipcam::IpProtocolFromString(config.GetString(setting + ".protocol"));
//  p.right.protocol = p.left.protocol;
//  p.left.stream_type = ipcam::IpStreamEncodingFromString(config.GetString(setting + ".stream_type"));
//  p.right.stream_type = p.left.stream_type;

//  // Details about the stream:
//  p.left.frame_height = config.GetInteger(setting + ".frame_height");
//  p.right.frame_height = p.left.frame_height;
//  p.left.frame_width = config.GetInteger(setting + ".frame_width");
//  p.right.frame_width = p.left.frame_width;
//  p.left.frame_rate = config.GetInteger(setting + ".frame_rate");
//  p.right.frame_rate = p.left.frame_rate;

//  return p;
//}





/**
 * @brief The GenericMonocularIpCamSink class
 *
 * This was adapted from our initial Axis-only streaming class.
 * Currently, we support HTTP/MJPEG, RTSP/H264, RTSP/MJPEG streams.
 *
 * To use a different camera brand, specify the custom url (@see CreateGenericIpCamSink()).
 *
 * Axis streaming URLs that worked (test yourself with vlc <url>):
 * * http://root:root@192.168.0.50/axis-cgi/mjpg/video.cgi?resolution=320x240&fps=10
 * * rtsp://root:root@192.168.0.50/axis-media/media.amp?videocodec=h264&resolution=480x270&fps=25
 * * rtsp://root:root@192.168.0.50/axis-media/media.amp?videocodec=jpeg&resolution=480x270&fps=25
 *
 * Our Axis didn't support MPEG4 over RTSP (so we didn't implement it):
 * * rtsp://root:root@192.168.0.50/axis-media/media.amp?videocodec=mpeg4&resolution=480x270&fps=25
 */
class GenericMonocularIpCamSink : public StreamSink
{
public:
  virtual ~GenericMonocularIpCamSink() {}

  GenericMonocularIpCamSink(const std::vector<MonocularIpCameraSinkParams> &params)
    : StreamSink()
  {
    // Check protocol
    if (params[0].ipcam_params.protocol == IpProtocol::HTTP)
    {
#ifdef VCP_BEST_WITH_IPCAM_HTTP
      if (params[0].ipcam_params.stream_encoding != IpStreamEncoding::MJPEG)
        VCP_ERROR("Currently, we only support MJPEG over HTTP, not " << params[0].ipcam_params.stream_encoding << ".");

      for (const auto &p : params)
      {
        if (p.ipcam_params.protocol != params[0].ipcam_params.protocol)
          VCP_ERROR("All IP cameras must use the same streaming protocol!");

        const std::string url = p.ipcam_params.GetStreamingUrl();
        if (url.empty())
          VCP_ERROR("IP camera URL must not be empty!");

        if (p.verbose)
          VCP_LOG_INFO_DEFAULT("Connecting to '" << p.ipcam_params.ipcam_type
                               << "' at: " << vcp::utils::string::ObscureUrlAuthentication(url));
        sinks_.push_back(http::CreateHttpMjpegSink<VCP_BEST_STREAM_BUFFER_CAPACITY>(url));

        // TODO nice-to-have would be a camera-specific start-up routine (e.g. enabling/disabling overlays, etc.)
      }
#else // VCP_BEST_WITH_IPCAM_HTTP
      VCP_ERROR("You need to compile VCP with HTTP streaming enabled!");
#endif // VCP_BEST_WITH_IPCAM_HTTP
    }
    else if (params[0].ipcam_params.protocol == IpProtocol::RTSP)
    {
#ifdef VCP_BEST_WITH_IPCAM_RTSP
      // We support MJPEG and H264 over RTSP
      int split_column = 0;
      std::vector<rtsp::RtspStreamParams> cam_params;
      for (const auto &p : params)
      {
        if (p.ipcam_params.protocol != params[0].ipcam_params.protocol)
          VCP_ERROR("All IP cameras must use the same streaming protocol!");

        // FIXME replace by RtspSinkParams.parse/fromX
        rtsp::RtspStreamType stream_type;
        switch(p.ipcam_params.stream_encoding)
        {
          case IpStreamEncoding::MJPEG:
            stream_type = rtsp::RtspStreamType::MJPEG;
            break;
          case IpStreamEncoding::H264:
            stream_type = RtspStreamType::H264;
            break;
          default:
            VCP_ERROR("Streaming '" << p.ipcam_params.stream_encoding << "' over RTSP is not supported.");
        }

        RtspStreamParams cam_param;
        cam_param.stream_url = p.ipcam_params.GetStreamingUrl();
        cam_param.stream_type = stream_type;
        cam_param.frame_width = p.ipcam_params.frame_width;
        cam_param.frame_height = p.ipcam_params.frame_height;
        cam_param.verbosity_level = p.verbose ? 1 : 0;
        // UDP is preferred but our Axis cams will kill the RTSP over UDP streams
        // after a few seconds (up to few minutes at most). Thus, we accept the TCP overhead (as long
        // as we can reliably stream our data).
        cam_param.protocol = RtspProtocol::TCP;

        cam_params.push_back(cam_param);

        if (p.verbose)
          VCP_LOG_INFO_DEFAULT("Connecting to '" << p.ipcam_params.ipcam_type << "' at: " << vcp::utils::string::ObscureUrlAuthentication(cam_param.stream_url));

        split_column += p.ipcam_params.frame_width;
        multi_rtsp_splits_.push_back(split_column);
        multi_rtsp_heights_.push_back(p.ipcam_params.frame_height);
      }
      sinks_.push_back(std::move(rtsp::CreateMultiRtspStreamSink<VCP_BEST_STREAM_BUFFER_CAPACITY>(cam_params)));
#else // VCP_BEST_WITH_IPCAM_RTSP
      VCP_ERROR("You need to compile VCP with RTSP streaming enabled!");
#endif // VCP_BEST_WITH_IPCAM_RTSP
    }
    else
      VCP_ERROR("Protocol '" << params[0].ipcam_params.protocol << "' not supported/implemented for IP camera captures.");
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
    //FIXME
  }

  size_t NumStreams() const override
  {
    //FIXME
  }

  FrameType FrameTypeAt(size_t stream_index) const override
  {
    //FIXME
  }

  std::string StreamLabel(size_t stream_index) const override
  {
    //FIXME
  }

  bool OpenDevice() override
  {
    //FIXME
  }

  bool CloseDevice() override
  {
    //FIXME
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
      success &= sinks_[i]->StartStreaming();
    return success;
  }

  std::vector<cv::Mat> Next() override
  {
    std::vector<cv::Mat> frames;
    frames.reserve(sinks_.size());
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      const std::vector<cv::Mat> sink_frames = sinks_[i]->Next();

      if (multi_rtsp_splits_.size() < 2)
      {
        // This is not a multi-rtsp cue.
        frames.insert(frames.end(), sink_frames.begin(), sink_frames.end());
      }
      else
      {
        // We use a multi-rtsp cue with multiple streams (concatenated
        // into a single cv::Mat).
        // Thus, we have to split the incoming frame.
        if (sink_frames.size() != 1)
          VCP_ERROR("MultiRtspStreamSink returned " << sink_frames.size() << " frames instead of 1!");

        cv::Mat frame = sink_frames[0];
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
            const cv::Rect roi(from, 0, to-from, multi_rtsp_heights_[j]);
            cv::Mat split = frame(roi).clone();
            frames.push_back(split);
            from = to;
          }
        }
      }
    }
    return frames;
  }


private:
  std::vector<int> multi_rtsp_splits_;
  std::vector<int> multi_rtsp_heights_;
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
    if (params[0].cam1_params.protocol == pvt::icc::ipcam::IpProtocol::HTTP)
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
    else if (params[0].cam1_params.protocol == pvt::icc::ipcam::IpProtocol::RTSP)
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




std::unique_ptr<StreamSink> CreateMonocularIpCameraSink(const std::vector<MonocularIpCameraSinkParams> &params)
{
  return std::unique_ptr<GenericMonocularIpCamSink>(new GenericMonocularIpCamSink(params));
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
