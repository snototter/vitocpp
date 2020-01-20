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
std::ostream &operator<<(std::ostream &stream, const IpProtocol &s)
{
  stream << IpProtocolToString(s);
  return stream;
}


std::ostream &operator<<(std::ostream &stream, const IpStreamType &s)
{
  stream << IpStreamTypeToString(s);
  return stream;
}


std::ostream &operator<<(std::ostream &stream, const IpCameraType &cam)
{
  stream << IpCameraTypeToString(cam);
  return stream;
}


std::string IpProtocolToString(const IpProtocol &p)
{
  if (p == IpProtocol::HTTP)
    return "http";
  if (p == IpProtocol::RTSP)
    return "rtsp";
  VCP_ERROR("IP protocol '" << p << "' is not yet supported.");
}


std::string IpStreamTypeToString(const IpStreamType &s)
{
  if (s == IpStreamType::H264)
    return "h264";
  if (s == IpStreamType::MJPEG)
    return "mjpeg";
  VCP_ERROR("IP stream type '" << s << "' is not yet supported.");
}


IpCameraType IpCameraTypeFromString(const std::string &camera_type)
{
  if (camera_type.compare("ipcam") == 0 || camera_type.compare("ipcam-stereo") == 0)
    return IpCameraType::Generic;
  if (camera_type.compare("axis") == 0 || camera_type.compare("axis-stereo") == 0)
    return IpCameraType::Axis;
  if (camera_type.compare("mobotix") == 0)
    return IpCameraType::Mobotix;
  if (camera_type.compare("hikvision") == 0)
    return IpCameraType::Hikvision;

  VCP_ERROR("IP camera_type '" << camera_type << "' is not yet supported.");
}

std::string IpCameraTypeToString(const IpCameraType &c)
{
  if (c == IpCameraType::Generic)
    return "ipcam";
  if (c == IpCameraType::Axis)
    return "axis";
  if (c == IpCameraType::Mobotix)
    return "mobotix";
  if (c == IpCameraType::Hikvision)
    return "hikvision";

  VCP_ERROR("IP camera type '" << c << "' is not yet supported.");
}


std::ostream &operator<< (std::ostream &out, const IpCameraParams &p)
{
  out << "[IP Camera]" << std::endl
      << "  URL: " << p.user << ":" << p.password << "@" << p.host << std::endl
      << "  Protocol/Type: " << IpProtocolToString(p.protocol) << "/" << IpStreamTypeToString(p.stream_type) << std::endl
      << "  Resolution: " << p.frame_width << " x " << p.frame_height << std::endl
      << "  FPS: " << p.frame_rate;
  return out;
}


IpProtocol IpProtocolFromString(const std::string &protocol)
{
  if (protocol.compare("http") == 0)
    return IpProtocol::HTTP;
  if (protocol.compare("rtsp") == 0)
    return IpProtocol::RTSP;

  VCP_ERROR("Protocol '" << protocol << "' not supported!");
}


IpStreamType IpStreamTypeFromString(const std::string &stream_type)
{
  if (stream_type.compare("h264") == 0)
    return IpStreamType::H264;
  if (stream_type.compare("mjpeg") == 0)
    return IpStreamType::MJPEG;

  VCP_ERROR("IpStreamType '" << stream_type << "' has no enum!");
}


IpCameraParams IpCameraParamsFromConfigSetting(const pvt::config::ConfigParams &config, const std::string &setting)
{
  vcp::best::ipcam::IpCameraParams p;
  // Get the camera type:
  p.camera_type = ipcam::IpCameraTypeFromString(config.GetString(setting + ".type"));

  // How should we retrieve the IP camera's stream?
  p.protocol = ipcam::IpProtocolFromString(config.GetString(setting + ".protocol"));
  p.stream_type = ipcam::IpStreamTypeFromString(config.GetString(setting + ".stream_type"));


  if (p.camera_type == ipcam::IpCameraType::Generic)
  {
    p.custom_url = config.GetString(setting + ".custom_url");
  }
  else
  {
    const std::string k_host = setting + ".host";
    p.host = config.GetString(k_host);

    const std::string k_usr = setting + ".user";
    if (config.SettingExists(k_usr))
    {
      p.user = config.GetString(k_usr);

      // If there's a user, there must be a password!
      p.password = config.GetString(setting + ".password");
    }
  }

  // Details about the stream:
  p.frame_height = config.GetInteger(setting + ".frame_height");
  p.frame_width = config.GetInteger(setting + ".frame_width");
  p.frame_rate = config.GetInteger(setting + ".frame_rate");

  return p;
}


IpStereoParams IpStereoParamsFromConfigSetting(const pvt::config::ConfigParams &config, const std::string &setting)
{
  pvt::icc::ipcam::IpStereoParams p;
  // Get the camera type:
  p.left.camera_type = ipcam::IpCameraTypeFromString(config.GetString(setting + ".type"));
  p.right.camera_type = p.left.camera_type;

  if (p.left.camera_type == ipcam::IpCameraType::Generic)
  {
    p.left.custom_url = config.GetString(setting + ".custom_url_left");
    p.right.custom_url = config.GetString(setting + ".custom_url_right");
  }
  else
  {
    p.left.host = config.GetString(setting + ".host_left");

    // left camera
    const std::string k_usr_left = setting + ".user_left";
    if (config.SettingExists(k_usr_left))
    {
      p.left.user = config.GetString(k_usr_left);

      // If there's a user, there must be a password!
      p.left.password = config.GetString(setting + ".password_left");
    }


    // right camera
    p.right.host = config.GetString(setting + ".host_right");
    const std::string k_usr = setting + ".user_right";
    if (config.SettingExists(k_usr))
    {
      p.right.user = config.GetString(k_usr);
      p.right.password = config.GetString(setting + ".password_right");
    }
  }

  // For now, the remaining params (streaming type, etc) are the same for both cameras in the stereo setup!
  // IF YOU WANT TO CHANGE THAT (i.e. allow different streaming types for
  // the two cameras of the stereo setup), ALSO CHANGE THE C'TOR OF
  // pvt::tools::capture::axis::AxisStereoCapture !!!!

  // How should we retrieve the IP camera's stream?
  p.left.protocol = ipcam::IpProtocolFromString(config.GetString(setting + ".protocol"));
  p.right.protocol = p.left.protocol;
  p.left.stream_type = ipcam::IpStreamTypeFromString(config.GetString(setting + ".stream_type"));
  p.right.stream_type = p.left.stream_type;

  // Details about the stream:
  p.left.frame_height = config.GetInteger(setting + ".frame_height");
  p.right.frame_height = p.left.frame_height;
  p.left.frame_width = config.GetInteger(setting + ".frame_width");
  p.right.frame_width = p.left.frame_width;
  p.left.frame_rate = config.GetInteger(setting + ".frame_rate");
  p.right.frame_rate = p.left.frame_rate;

  return p;
}


std::string GetAxisRtspUrl(const pvt::icc::ipcam::IpCameraParams &p)
{
  std::string codec;
  if (p.stream_type == pvt::icc::ipcam::IpStreamType::MJPEG)
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
  if (p.stream_type == pvt::icc::ipcam::IpStreamType::H264)
    rtsp << "&videokeyframeinterval=16";
  return rtsp.str();
}


std::string GetAxisHttpUrl(const pvt::icc::ipcam::IpCameraParams &p)
{
  if (p.stream_type != pvt::icc::ipcam::IpStreamType::MJPEG)
    PVT_ABORT("Axis can only stream MJPEG over HTTP, either change stream type or protocol!");

  std::stringstream http;
  http << "http://";

  if (!p.user.empty())
    http << p.user << ':' << p.password << '@';

  http << p.host << "/axis-cgi/mjpg/video.cgi";
  http << "?resolution=" << p.frame_width << 'x' << p.frame_height << "&fps=" << p.frame_rate;
  return http.str();
}


std::string GetAxisUrl(const pvt::icc::ipcam::IpCameraParams &p)
{
  if (p.protocol == pvt::icc::ipcam::IpProtocol::HTTP)
    return GetAxisHttpUrl(p);
  if (p.protocol == pvt::icc::ipcam::IpProtocol::RTSP)
    return GetAxisRtspUrl(p);
  PVT_ABORT("Protocol type for Axis streaming must be HTTP or RTSP");
}


std::string GetMobotixUrl(const pvt::icc::ipcam::IpCameraParams &p)
{
  if (p.protocol == ipcam::IpProtocol::HTTP && p.stream_type == ipcam::IpStreamType::MJPEG)
  {
    std::stringstream url;
    url << "http://";
    if (!p.user.empty())
      url << p.user << ":" << p.password << "@";
    url << p.host << "/cgi-bin/faststream.jpg?stream=full&needlength&fps=" << std::setprecision(1) << static_cast<double>(p.frame_rate);
    return url.str();
  }
  PVT_ABORT("Currently, we only support (or have tested) MJPEG over HTTP for Mobotix cameras. You requested " << p.stream_type << " over " << p.protocol);
}


std::string GetHikvisionUrl(const pvt::icc::ipcam::IpCameraParams &p)
{
  if (p.protocol == ipcam::IpProtocol::RTSP && p.stream_type == ipcam::IpStreamType::H264)
  {
    std::stringstream url;
    url << "rtsp://";
    if (!p.user.empty())
      url << p.user << ":" << p.password << "@";
    url << p.host << "/Streaming/Channels/1";
    return url.str();
  }
  PVT_ABORT("Currently, we only support H264 over RTSP for Hikvision cameras (use camera's H264+ setting).");
}


//TODO add siqura TPU: https://www.ispyconnect.com/man.aspx?n=Siqura
std::string GetIpCameraUrl(const pvt::icc::ipcam::IpCameraParams &p)
{
  if (p.camera_type == ipcam::IpCameraType::Generic)
    return p.custom_url;
  if (p.camera_type == ipcam::IpCameraType::Axis)
    return GetAxisUrl(p);
  if (p.camera_type == ipcam::IpCameraType::Mobotix)
    return GetMobotixUrl(p);
  if (p.camera_type == ipcam::IpCameraType::Hikvision)
    return GetHikvisionUrl(p);

  PVT_ABORT("URL lookup for this IP camera type is not yet implemented!");
}


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

  GenericMonocularIpCamSink(const std::vector<MonoIpCameraSinkParams> &params, std::string (*GetUrlFn)(const pvt::icc::ipcam::IpCameraParams &)=GetIpCameraUrl)
    : StreamSink()
  {
    // Check protocol
    if (params[0].device_params.protocol == pvt::icc::ipcam::IpProtocol::HTTP)
    {
#ifdef VCP_BEST_WITH_IPCAM_HTTP
      if (params[0].device_params.stream_type != pvt::icc::ipcam::IpStreamType::MJPEG)
        PVT_ABORT("Currently, we only support MJPEG over HTTP, not " << params[0].device_params.stream_type << "!");

      for (const auto &p : params)
      {
        if (p.device_params.protocol != params[0].device_params.protocol)
          PVT_ABORT("All IP cameras must use the same streaming protocol!");

        const std::string url = GetUrlFn(p.device_params);
        if (url.empty())
          PVT_ABORT("IP camera URL must not be empty!");

        PVT_LOG_INFO("Connecting to '" << p.device_params.camera_type << "' at: " << pvt::utils::string::ObscureUrlAuthentication(url));
        sinks_.push_back(pvt::icc::CreateHttpMjpegSink<PVT_ICC_CIRCULAR_SINK_BUFFER_CAPACITY>(url));

        // TODO, if we ever get back the mobotix cameras, we would need to disable the "zeppcam" overlay after each camera reboot, so add:
        //if (p.camera_type == ipcam::IpCameraType::Mobotix) DisableTextOverlay() <- see <plc-prototype project>/src/camera_capture.cpp
      }
#else // VCP_BEST_WITH_IPCAM_HTTP
      PVT_ABORT("You need to compile PVT with HTTP streaming enabled!");
#endif // VCP_BEST_WITH_IPCAM_HTTP
    }
    else if (params[0].device_params.protocol == pvt::icc::ipcam::IpProtocol::RTSP)
    {
#ifdef VCP_BEST_WITH_IPCAM_RTSP
      // We support MJPEG and H264 over RTSP
      int split_column = 0;
      std::vector<pvt::icc::RtspStreamParams> cam_params;
      for (const auto &p : params)
      {
        if (p.device_params.protocol != params[0].device_params.protocol)
          PVT_ABORT("All IP cameras must use the same streaming protocol!");

//        if ((p.device_params.frame_height != params[0].device_params.frame_height) ||
//            (p.device_params.frame_width != params[0].device_params.frame_width))
//          PVT_ABORT("Currently, all IP cameras must use the same resolution!");

        pvt::icc::RtspStreamType stream_type;
        if (p.device_params.stream_type == pvt::icc::ipcam::IpStreamType::MJPEG)
          stream_type = pvt::icc::RtspStreamType::MJPEG;
        else if (p.device_params.stream_type == pvt::icc::ipcam::IpStreamType::H264)
          stream_type = pvt::icc::RtspStreamType::H264;
        else
          PVT_ABORT("Stream type '" << p.device_params.stream_type << "' not supported!");

        pvt::icc::RtspStreamParams cam_param;
        cam_param.stream_url = GetUrlFn(p.device_params);
        cam_param.stream_type = stream_type;
        cam_param.frame_width = p.device_params.frame_width;
        cam_param.frame_height = p.device_params.frame_height;
        // UDP is preferred but our Axis cams will kill the RTSP over UDP streams
        // after a few seconds/minutes. Thus, we accept the TCP overhead (as long
        // as we can reliably stream our data).
        cam_param.protocol = pvt::icc::RtspProtocol::TCP;

        cam_params.push_back(cam_param);

        PVT_LOG_INFO("Connecting to '" << p.device_params.camera_type << "' at: " << pvt::utils::string::ObscureUrlAuthentication(cam_param.stream_url));

        split_column += p.device_params.frame_width;
        multi_rtsp_splits_.push_back(split_column);
        multi_rtsp_heights_.push_back(p.device_params.frame_height);
      }
      sinks_.push_back(std::move(pvt::icc::CreateMultiRtspStreamSink<PVT_ICC_CIRCULAR_SINK_BUFFER_CAPACITY>(cam_params)));
#else // VCP_BEST_WITH_IPCAM_RTSP
      PVT_ABORT("You need to compile PVT with RTSP streaming enabled!");
#endif // VCP_BEST_WITH_IPCAM_RTSP
    }
    else
      PVT_ABORT("Protocol not supported/implemented for our generic IP camera captures!");
  }


  int IsAvailable() const override
  {
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      if (!sinks_[i]->IsAvailable())
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


  void StartStream() override
  {
    for (size_t i = 0; i < sinks_.size(); ++i)
      sinks_[i]->StartStream();
  }

  void GetNextFrame(cv::Mat &frame) override
  {
    if (sinks_.size() != 1)
    {
      PVT_LOG_FAILURE("This IP camera sink receives data from multiple cameras - Do NOT use GetNextFrame(cv::Mat), but retrieve all with GetNextFrame()!");
      frame = cv::Mat();
    }
    else
    {
      sinks_[0]->GetNextFrame(frame);
    }
  }

  std::vector<cv::Mat> GetNextFrame() override
  {
    std::vector<cv::Mat> frames;
    frames.reserve(sinks_.size());
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
//            const cv::Rect roi(from, 0, to-from, frame.rows);
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

  void Terminate() override
  {
    for (size_t i = 0; i < sinks_.size(); ++i)
      sinks_[i]->Terminate();
  }

private:
  std::vector<int> multi_rtsp_splits_;
  std::vector<int> multi_rtsp_heights_;
  std::vector<std::unique_ptr<pvt::icc::StreamSink>> sinks_;
};



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
      if (params[0].cam1_params.stream_type != pvt::icc::ipcam::IpStreamType::MJPEG)
        PVT_ABORT("Cannot stream '" << ipcam::IpStreamTypeToString(params[0].cam1_params.stream_type) << "' over HTTP!");

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
        if (p.cam1_params.stream_type == pvt::icc::ipcam::IpStreamType::MJPEG)
          stream_type = pvt::icc::RtspStreamType::MJPEG;
        else if (p.cam1_params.stream_type == pvt::icc::ipcam::IpStreamType::H264)
          stream_type = pvt::icc::RtspStreamType::H264;
        else
          PVT_ABORT("Stream type '" << ipcam::IpStreamTypeToString(p.cam1_params.stream_type) << "' not supported!");

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


  int IsAvailable() const override
  {
    for (size_t i = 0; i < sinks_.size(); ++i)
    {
      if (!sinks_[i]->IsAvailable())
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


  void StartStream() override
  {
    for (size_t i = 0; i < sinks_.size(); ++i)
      sinks_[i]->StartStream();
  }

  void GetNextFrame(cv::Mat &frame) override
  {
    if (num_stereo_cameras_ != 1)
    {
      frame = cv::Mat();
      PVT_LOG_FAILURE("This Axis stereo sink receives data from multiple stereo setups - Do NOT use GetNextFrame(cv::Mat), but retrieve all with GetNextFrame()!");
    }
    else
    {
      // Concatenate left and right frame
      if (is_rtsp_)
      {
        // In RTSP, we have a single MultiRtspSink
        sinks_[0]->GetNextFrame(frame);
      }
      else
      {
        // In HTTP, we have 2 sinks for a stereo setup, 1 left, 1 right:
        cv::Mat left, right;
        sinks_[0]->GetNextFrame(left);
        sinks_[1]->GetNextFrame(right);
        frame = pvt::imutils::ColumnStack(left, right);
      }
    }
  }


  std::vector<cv::Mat> GetNextFrame() override
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
      // In HTTP streaming, we have 1 camera per sink, so the first
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

  void Terminate() override
  {
    for (size_t i = 0; i < sinks_.size(); ++i)
      sinks_[i]->Terminate();
  }

private:
  size_t num_stereo_cameras_;
  bool is_rtsp_;
  std::vector<int> multi_rtsp_splits_;
  std::vector<std::unique_ptr<pvt::icc::StreamSink>> sinks_;
};



MonoIpCameraSinkParams MonoIpCameraSinkParamsFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group)
{
  const std::string label = GetSinkLabelFromConfig(config, cam_group);
  const std::string calib = GetCalibrationFileFromConfig(config, cam_group);

  return MonoIpCameraSinkParams(label, calib, StreamType::MONO, ipcam::IpCameraParamsFromConfigSetting(config, cam_group));
}


StereoIpCameraSinkParams StereoIpCameraSinkParamsFromConfig(const pvt::config::ConfigParams &config, const std::string &cam_group)
{
  const std::string label = GetSinkLabelFromConfig(config, cam_group);
  const std::string calib = GetCalibrationFileFromConfig(config, cam_group);
  const ipcam::IpStereoParams stereo = ipcam::IpStereoParamsFromConfigSetting(config, cam_group);

  return StereoIpCameraSinkParams(label, calib, StreamType::STEREO,
                                  stereo.left, stereo.right);
}


std::unique_ptr<StreamSink> CreateMonoIpCamSink(const std::vector<MonoIpCameraSinkParams> &params)
{
  return std::unique_ptr<GenericMonocularIpCamSink>(new GenericMonocularIpCamSink(params, GetIpCameraUrl));
}


std::unique_ptr<StreamSink> CreateStereoIpCamSink(const std::vector<StereoIpCameraSinkParams> &params)
{
  return std::unique_ptr<GenericStereoIpCamSink>(new GenericStereoIpCamSink(params, GetIpCameraUrl));
}


bool IsGenericIpCameraMono(const std::string &camera_type) { return camera_type.compare("ipcam") == 0; }
bool IsGenericIpCameraStereo(const std::string &camera_type) { return camera_type.compare("ipcam-stereo") == 0; }
bool IsAxisMono(const std::string &camera_type) { return camera_type.compare("axis") == 0; }
bool IsAxisStereo(const std::string &camera_type) { return camera_type.compare("axis-stereo") == 0; }
bool IsMobotix(const std::string &camera_type) { return camera_type.compare("mobotix") == 0; }
bool IsHikvision(const std::string &camera_type) { return camera_type.compare("hikvision") == 0; }

bool IsMonocularIpCamera(const std::string &camera_type)
{
  return IsGenericIpCameraMono(camera_type) ||
      IsAxisMono(camera_type) ||
      IsMobotix(camera_type) ||
      IsHikvision(camera_type);
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
