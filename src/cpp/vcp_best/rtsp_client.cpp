#include "rtsp_client.h"

#include <map>
#include <mutex>
#include <thread>

#include <liveMedia.hh>
#include <BasicUsageEnvironment.hh>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/file_utils.h>

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

class StreamClientState
{
public:
  StreamClientState() : iter(NULL), session(NULL), subsession(NULL), streamTimerTask(NULL), duration(0.0) {}

  virtual ~StreamClientState()
  {
    if (iter)
      delete iter;
    if (session != NULL)
    {
      // We also need to delete "session", and unschedule "streamTimerTask" (if set)
      UsageEnvironment& env = session->envir(); // alias

      env.taskScheduler().unscheduleDelayedTask(streamTimerTask);
      Medium::close(session);
    }
  }

// deliberately public:
  MediaSubsessionIterator *iter;
  MediaSession *session;
  MediaSubsession *subsession;
  TaskToken streamTimerTask;
  double duration;
};

class VCPRtspClient: public RTSPClient
{
public:
  VCPRtspClient(UsageEnvironment& env, const IpCameraSinkParams &params,
                void (*frame_callback)(const cv::Mat &, void *), void *callback_param,
                char const* applicationName = NULL, portNumBits tunnelOverHTTPPortNum = 0)
    : RTSPClient(env, params.stream_url.c_str(),
#ifdef VCP_LOG_LEVEL_DEBUG
                 // live555 is *very* verbose, so we only enable it in debug mode
                 params.verbose ? 1 : 0,
#else // VCP_LOG_LEVEL_DEBUG
                 0,
#endif // VCP_LOG_LEVEL_DEBUG
                 applicationName, tunnelOverHTTPPortNum, -1),
      params_(params)
  {
    callback_fx_ = frame_callback;
    callback_user_params_ = callback_param;

    single_medium_subsession_ = "video";
  }

  virtual ~VCPRtspClient() { VCP_LOG_FAILURE("FOOOOOOFUCKER");}

// deliberately public:
  size_t id_;
  StreamClientState scs_;
  IpCameraSinkParams params_;

  // Medium name of the only subsession (video) we want to initialize.
  // Once the "video" stream has been set up, this field will be reset
  // to indicate that we don't have to set up anything else for this
  // sink.
  std::string single_medium_subsession_;

  void (*callback_fx_)(const cv::Mat &, void *);
  void *callback_user_params_;

  int VerbosityLevel() const { return fVerbosityLevel; }
};

//-----------------------------------------------------------------------------
// Forward declarations of RTSP response handlers
void shutdownStream(RTSPClient* rtsp_client);
void continueAfterDESCRIBE(RTSPClient* rtsp_client, int result_code, char *result_string);
void setupNextSubsession(RTSPClient* rtsp_client);
void continueAfterSETUP(RTSPClient* rtsp_client, int result_code, char* result_string);
void continueAfterPLAY(RTSPClient* rtsp_client, int result_code, char* result_string);
void subsessionAfterPlaying(void *client_data);
void subsessionByeHandler(void *client_data);
void streamTimerHandler(void* client_data);

//-----------------------------------------------------------------------------
// Implementation of the RTSP response handlers
void shutdownStream(RTSPClient* rtsp_client)
{
  VCPRtspClient *vcp_client = dynamic_cast<VCPRtspClient*>(rtsp_client);
  StreamClientState& scs = vcp_client->scs_;

  // First, check whether any subsessions have still to be closed:
  if (scs.session != NULL)
  {
    Boolean someSubsessionsWereActive = False;
    MediaSubsessionIterator iter(*scs.session);
    MediaSubsession* subsession;

    while ((subsession = iter.next()) != NULL)
    {
      if (subsession->sink != NULL)
      {
        Medium::close(subsession->sink);
        subsession->sink = NULL;

        if (subsession->rtcpInstance() != NULL)
          subsession->rtcpInstance()->setByeHandler(NULL, NULL); // in case the server sends a RTCP "BYE" while handling "TEARDOWN"

        someSubsessionsWereActive = True;
      }
    }

    if (someSubsessionsWereActive)
    {
      // Send a RTSP "TEARDOWN" command, to tell the server to shutdown the stream.
      // Don't bother handling the response to the "TEARDOWN".
      rtsp_client->sendTeardownCommand(*scs.session, NULL);
    }
  }

  if (vcp_client->params_.verbose)
    VCP_LOG_INFO_DEFAULT("Closing stream for client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url) << "'");
  Medium::close(rtsp_client);
  VCP_LOG_FAILURE("CLOSED THE MEDIUM");
  // Note that this will also cause this stream's "StreamClientState" structure to get reclaimed.
}

void continueAfterDESCRIBE(RTSPClient* rtsp_client, int result_code, char *result_string)
{
  do
  {
    // alias
    UsageEnvironment &env = rtsp_client->envir();
    VCPRtspClient *vcp_client = static_cast<VCPRtspClient*>(rtsp_client);
    StreamClientState &scs = vcp_client->scs_;

    if (result_code != 0)
    {
      VCP_LOG_FAILURE("Failed to get SDP description for client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                      << "': " << result_string);
      delete[] result_string;
      break;
    }

    char* const sdpDescription = result_string;
    if (vcp_client->params_.verbose)
    {
#ifdef VCP_LOG_LEVEL_DEBUG
      VCP_LOG_INFO("Received SDP for client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url) << "'"
                   << std::endl << sdpDescription);
#else // VCP_LOG_LEVEL_DEBUG
      VCP_LOG_INFO("Received SDP for client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url) << "'.");
#endif // VCP_LOG_LEVEL_DEBUG
    }

    // Create a media session object from this SDP description:
    scs.session = MediaSession::createNew(env, sdpDescription);
    delete[] sdpDescription;
    if (scs.session == NULL)
    {
      VCP_LOG_FAILURE("Failed to create a MediaSession for client '"
                      << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url) << "': " << env.getResultMsg());
      break;
    }
    else if (!scs.session->hasSubsessions())
    {
      VCP_LOG_FAILURE("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url) << "': this session has no media subsessions (i.e., no \"m=\" lines).");
      break;
    }

    // Then, create and set up our data source objects for the session.  We do this by iterating over the session's 'subsessions',
    // calling "MediaSubsession::initiate()", and then sending a RTSP "SETUP" command, on each one.
    // (Each 'subsession' will have its own data source.)
    scs.iter = new MediaSubsessionIterator(*scs.session);
    setupNextSubsession(rtsp_client);
    return;
  }
  while (0);

  // An unrecoverable error occurred with this stream.
  shutdownStream(rtsp_client);
}


void setupNextSubsession(RTSPClient* rtsp_client)
{
  UsageEnvironment& env = rtsp_client->envir();
  VCPRtspClient *vcp_client = dynamic_cast<VCPRtspClient*>(rtsp_client);
  StreamClientState& scs = vcp_client->scs_;

  scs.subsession = scs.iter->next();
  if (scs.subsession != NULL)
  {
    if (strcmp(scs.subsession->mediumName(), vcp_client->single_medium_subsession_.c_str()) != 0)
    {
      if (vcp_client->params_.verbose)
        VCP_LOG_INFO_DEFAULT("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                             << "' ignoring non-video (" << scs.subsession->mediumName() << ") subsession.");
      setupNextSubsession(rtsp_client);
    }
    else
    {
      if (!scs.subsession->initiate())
      {
        // give up on this subsession; go to the next one
        VCP_LOG_FAILURE("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url) << "' failed to initiate the \""
                        << scs.subsession->mediumName() << "\" subsession: " << env.getResultMsg());
        setupNextSubsession(rtsp_client);
      }
      else
      {
        // Once we have set up a video session, we ignore the rest:
        vcp_client->single_medium_subsession_ = "ignore-rest";

        if (vcp_client->params_.verbose)
          VCP_LOG_INFO_DEFAULT("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                               << "' initiated the \"" << scs.subsession->mediumName() << "\" subsession.");

        // Continue setting up this subsession, by sending a RTSP "SETUP" command:
        const Boolean request_tcp_stream = (vcp_client->params_.transport_protocol == IpTransportProtocol::TCP) ? True : False;
        rtsp_client->sendSetupCommand(*scs.subsession, continueAfterSETUP, False, request_tcp_stream);
      }
    }
    return;
  }

  // We've finished setting up all of the subsessions.  Now, send a RTSP "PLAY" command to start the streaming:
  if (scs.session->absStartTime() != NULL)
  {
    // Special case: The stream is indexed by 'absolute' time, so send an appropriate "PLAY" command:
    rtsp_client->sendPlayCommand(*scs.session, continueAfterPLAY, scs.session->absStartTime(), scs.session->absEndTime());
  }
  else
  {
    scs.duration = scs.session->playEndTime() - scs.session->playStartTime();
    rtsp_client->sendPlayCommand(*scs.session, continueAfterPLAY);
  }
}

void continueAfterSETUP(RTSPClient* rtsp_client, int result_code, char* result_string)
{
  do
  {
    VCPRtspClient *vcp_client = dynamic_cast<VCPRtspClient*>(rtsp_client);
    UsageEnvironment& env = rtsp_client->envir();
    StreamClientState& scs = vcp_client->scs_;

    if (scs.subsession == NULL)
    {
      VCP_LOG_FAILURE("continueAfterSETUP called with nullpointer subsession!");
      break;
    }

    if (result_code != 0)
    {
      VCP_LOG_FAILURE("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url) << "' failed to set up the \""
                      << scs.subsession->mediumName() << "\" subsession: " << result_string);
      break;
    }

    if (vcp_client->params_.verbose)
      VCP_LOG_INFO_DEFAULT("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url) << "' set up the \""
                           << scs.subsession->mediumName() << "\" subsession.");

    // Having successfully setup the subsession, create the data sink for it and call "startPlaying()" on it.
    // (This will prepare the data sink to receive data; the actual flow of data from the client won't start happening until later,
    // after we've sent a RTSP "PLAY" command.)
    // Then feed the padded frame (possibly also need an additional 0x00.....01 or something in front) to your favourite decoder (ffmpeg,...)
    switch (vcp_client->params_.stream_encoding)
    {
    case IpStreamEncoding::MJPEG:
      scs.subsession->sink = CreateRtspMjpegMediaSink(env, *scs.subsession, vcp_client->params_, vcp_client->callback_fx_, vcp_client->callback_user_params_);
      break;

    case IpStreamEncoding::H264:
      scs.subsession->sink = CreateRtspH264MediaSink(env, *scs.subsession, vcp_client->params_, vcp_client->callback_fx_, vcp_client->callback_user_params_);
      break;

    default:
      VCP_ERROR("IpStreamEncoding '" << vcp_client->params_.stream_encoding << "' not supported!");
    }

    if (scs.subsession->sink == NULL)
    {
      VCP_LOG_FAILURE("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                      << "' failed to create a data sink for the \"" << scs.subsession->mediumName() << "\" subsession: " << env.getResultMsg());
      break;
    }

    if (vcp_client->params_.verbose)
      VCP_LOG_INFO_DEFAULT("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                           << "' set up a data sink for the \""
                           << scs.subsession->mediumName() << "\" subsession.");

    // a hack to let subsession handler functions get the "RTSPClient" from the subsession
    scs.subsession->miscPtr = rtsp_client;
    scs.subsession->sink->startPlaying(*(scs.subsession->readSource()), subsessionAfterPlaying, scs.subsession);

    // Also set a handler to be called if a RTCP "BYE" arrives for this subsession:
    if (scs.subsession->rtcpInstance() != NULL)
      scs.subsession->rtcpInstance()->setByeHandler(subsessionByeHandler, scs.subsession);
  }
  while (0);
  delete[] result_string;

  // Set up the next subsession, if any:
  setupNextSubsession(rtsp_client);
}

void continueAfterPLAY(RTSPClient* rtsp_client, int result_code, char* result_string)
{
  Boolean success = False;

  do
  {
    UsageEnvironment &env = rtsp_client->envir();
    VCPRtspClient *vcp_client = dynamic_cast<VCPRtspClient*>(rtsp_client);
    StreamClientState &scs = vcp_client->scs_;

    if (result_code != 0)
    {
      VCP_LOG_FAILURE("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                      << "' failed to start playing session: " << result_string);
      break;
    }

    // Set a timer to be handled at the end of the stream's expected duration (if the stream does not already signal its end
    // using a RTCP "BYE").  This is optional.  If, instead, you want to keep the stream active - e.g., so you can later
    // 'seek' back within it and do another RTSP "PLAY" - then you can omit this code.
    // (Alternatively, if you don't want to receive the entire stream, you could set this timer for some shorter value.)
    if (scs.duration > 0)
    {
      unsigned const delaySlop = 0; // number of seconds extra to delay, after the stream's expected duration.  (This is optional.)
      scs.duration += delaySlop;
      unsigned uSecsToDelay = (unsigned)(scs.duration*1000000);
      scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(uSecsToDelay, (TaskFunc*)streamTimerHandler, rtsp_client);
    }

    if (vcp_client->params_.verbose)
        VCP_LOG_INFO_DEFAULT("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                             << "' started playing session.");

    success = True;
  }
  while (0);
  delete[] result_string;

  if (!success)
  {
    // An unrecoverable error occurred with this stream.
    shutdownStream(rtsp_client);
  }
}

void subsessionAfterPlaying(void *client_data)
{
  MediaSubsession* subsession = (MediaSubsession*)client_data;
  RTSPClient* rtsp_client = (RTSPClient*)(subsession->miscPtr);

  // Begin by closing this subsession's stream:
  Medium::close(subsession->sink);
  subsession->sink = NULL;

  // Next, check whether *all* subsessions' streams have now been closed:
  MediaSession& session = subsession->parentSession();
  MediaSubsessionIterator iter(session);
  while ((subsession = iter.next()) != NULL)
  {
    // Is this subsession still active?
    if (subsession->sink != NULL)
      return;
  }

  // All subsessions' streams have now been closed, so shutdown the client:
  shutdownStream(rtsp_client);
}

void subsessionByeHandler(void *client_data)
{
  MediaSubsession* subsession = (MediaSubsession*)client_data;
  RTSPClient* rtsp_client = (RTSPClient*)subsession->miscPtr;
  VCPRtspClient *vcp_client = dynamic_cast<VCPRtspClient*>(rtsp_client);

  if (vcp_client && vcp_client->params_.verbose)
    VCP_LOG_INFO_DEFAULT("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                         << "' received RTCP \"BYE\" on \""
                         << subsession->mediumName() << "\" subsession.");

  // Now act as if the subsession had closed:
  subsessionAfterPlaying(subsession);
}

void streamTimerHandler(void* client_data)
{
  VCPRtspClient* rtsp_client = (VCPRtspClient*)client_data;
  rtsp_client->scs_.streamTimerTask = NULL;

  // Shut down the stream:
  shutdownStream(rtsp_client);
}


class RtspClientEnvironmentImpl : public RtspClientEnvironment
{
public:
  RtspClientEnvironmentImpl() : RtspClientEnvironment(),
    scheduler_(nullptr), environment_(nullptr),
    terminate_event_loop_(0)
  {
  }

  virtual ~RtspClientEnvironmentImpl()
  {
    if (environment_)
    {
      // We can only delete the environment via reclaim (protected destructor...)
      while(!environment_->reclaim())
      {
        VCP_LOG_WARNING_NSEC("Waiting to reclaim live555 UsageEnvironment", 0.5);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      environment_ = nullptr;
    }

    if (scheduler_)
    {
      delete scheduler_;
      scheduler_ = nullptr;
    }
  }


  void OpenSdpFile(const IpCameraSinkParams &params, void (*frame_callback)(const cv::Mat &, void *), void *callback_param)
  {
    InitEnvironment();

    const std::string filename = vcp::utils::string::StartsWith(params.stream_url, "file://") ? params.stream_url.substr(7) : params.stream_url;
    const std::string sdp_description = vcp::utils::file::SlurpAsciiFile(filename);
    if (sdp_description.empty())
    {
      VCP_LOG_FAILURE("Cannot load the session description, check SDP file: '" << filename << "'");
      return;
    }

    VCPRtspClient* vcp_client = new VCPRtspClient(*environment_, params, frame_callback, callback_param);
    if (vcp_client == NULL)
    {
      VCP_LOG_FAILURE("Failed to create a RTSP client for URL \""
                      << vcp::utils::string::ObscureUrlAuthentication(params.stream_url)
                      << "\": " << environment_->getResultMsg());
      return;
    }

    client_list_mutex_.lock();
    const size_t id = open_clients_.size();
    vcp_client->id_ = id;
    open_clients_.push_back(vcp_client);
    client_list_mutex_.unlock();

    // alias
    UsageEnvironment &env = vcp_client->envir();
    StreamClientState &scs = vcp_client->scs_;

    if (vcp_client->params_.verbose)
      VCP_LOG_INFO("Initialize stream from local SDP:" << std::endl << sdp_description);

    // Create a media session object from this SDP description:
    scs.session = MediaSession::createNew(env, sdp_description.c_str());
    if (scs.session == NULL)
    {
      VCP_LOG_FAILURE("Failed to create a MediaSession for client '"
                      << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                      << "': " << env.getResultMsg());
      return;
    }
    else if (!scs.session->hasSubsessions())
    {
      VCP_LOG_FAILURE("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                      << "': this session has no media subsessions (i.e., no \"m=\" lines)");
      return;
    }

    // Then, create and set up our data source objects for the session.
    scs.iter = new MediaSubsessionIterator(*scs.session);
    while ((scs.subsession = scs.iter->next()) != NULL)
    {
      if (strcmp(scs.subsession->mediumName(), vcp_client->single_medium_subsession_.c_str()) != 0)
      {
        if (vcp_client->params_.verbose)
          VCP_LOG_INFO_DEFAULT("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                               << "' ignoring non-video (" << scs.subsession->mediumName() << ") subsession.");
      }
      else
      {
        if (!scs.subsession->initiate(0))
        {
          VCP_LOG_FAILURE("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                          << "': Failed to initiate the \""
                          << scs.subsession->mediumName() << "\" subsession: " << env.getResultMsg());
        }
        else
        {
          // Once we have set up a video session, we ignore the rest:
          vcp_client->single_medium_subsession_ = "ignore-rest";

          if (params.verbose)
            VCP_LOG_INFO("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                         << "' initiated the \"" << scs.subsession->mediumName() << "\" subsession.");

          // It seems like we don't need to send a SETUP command to the server, just startPlaying() the sink.
          // Create the data sink for it and call "startPlaying()" on it.
          switch (vcp_client->params_.stream_encoding)
          {
          case IpStreamEncoding::MJPEG:
            scs.subsession->sink = CreateRtspMjpegMediaSink(
                  env, *scs.subsession, vcp_client->params_,
                  vcp_client->callback_fx_, vcp_client->callback_user_params_);
            break;

          case IpStreamEncoding::H264:
            scs.subsession->sink = CreateRtspH264MediaSink(
                  env, *scs.subsession, vcp_client->params_,
                  vcp_client->callback_fx_, vcp_client->callback_user_params_);
            break;

          default:
            VCP_ERROR("IpStreamEncoding '" << vcp_client->params_.stream_encoding << "' not supported!");
          }

          if (scs.subsession->sink == NULL)
          {
            VCP_LOG_FAILURE("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                            << "' failed to create a data sink for the \""
                            << scs.subsession->mediumName() << "\" subsession: " << env.getResultMsg());
            break;
          }

          if (params.verbose)
            VCP_LOG_INFO_DEFAULT("Client '" << vcp::utils::string::ObscureUrlAuthentication(vcp_client->params_.stream_url)
                                 << "' set up a data sink for the \""
                                 << scs.subsession->mediumName() << "\" subsession.");

          // a hack to let subsession handler functions get the "RTSPClient" from the subsession
          scs.subsession->miscPtr = vcp_client;
          scs.subsession->sink->startPlaying(*(scs.subsession->rtpSource()), NULL, NULL);
        }
      }
    }
  }


  void OpenUrl(const IpCameraSinkParams &params, void (*frame_callback)(const cv::Mat &, void *), void *callback_param) override
  {
    if (vcp::utils::string::StartsWith(params.stream_url, "file://"))
    {
      OpenSdpFile(params, frame_callback, callback_param);
      return;
    }

    InitEnvironment();
    // Begin by creating a "RTSPClient" object.  Note that there is a separate "RTSPClient" object for each stream that we wish
    // to receive (even if more than one stream uses the same "rtsp://" URL).
    VCPRtspClient* rtsp_client = new VCPRtspClient(*environment_, params, frame_callback, callback_param);
    if (rtsp_client == NULL)
    {
      VCP_LOG_FAILURE("Failed to create a RTSP client for URL \""
                      << vcp::utils::string::ObscureUrlAuthentication(params.stream_url)
                      << "\": " << environment_->getResultMsg());
      return;
    }

    client_list_mutex_.lock();
    const size_t id = open_clients_.size();
    rtsp_client->id_ = id;
    open_clients_.push_back(rtsp_client);
    client_list_mutex_.unlock();

    // Next, send a RTSP "DESCRIBE" command, to get a SDP description for the stream.
    // Note that this command - like all RTSP commands - is sent asynchronously; we do not block, waiting for a response.
    // Instead, the following function call returns immediately, and we handle the RTSP response later, from within the event loop:
    rtsp_client->sendDescribeCommand(continueAfterDESCRIBE);
  }

  void DoEventLoop() override
  {
    // Event loop will block until terminate_event_loop_ is set.
    environment_->taskScheduler().doEventLoop(&terminate_event_loop_);

    client_list_mutex_.lock();
    // Clean up all open clients.
    for (size_t i = 0; i < open_clients_.size(); ++i)
    {
      shutdownStream(open_clients_[i]);
      // What happens if a client received a shut down earlier on?
      // We'll probably have a dangling pointer. Unfortunately, there is not much useful documentation on all these little streaming details :/
      // So we assume/trust, that the camera doesn't initiate a disconnect: since we work with surveillance cameras, this is a
      // quite reasonable assumption! ;-)
    }
    client_list_mutex_.unlock();
  }

  void TerminateEventLoop() override
  {
    terminate_event_loop_ = 1;
  }

private:
  TaskScheduler* scheduler_;
  UsageEnvironment* environment_;
  char terminate_event_loop_;
  std::vector<RTSPClient*> open_clients_;
  std::mutex client_list_mutex_;

  void InitEnvironment()
  {
    if (!scheduler_)
      scheduler_ = BasicTaskScheduler::createNew();

    if (!environment_)
      environment_ = BasicUsageEnvironment::createNew(*scheduler_);

    terminate_event_loop_ = 0;
  }
};


std::unique_ptr<RtspClientEnvironment> CreateRtspClientEnvironment()
{
  return std::unique_ptr<RtspClientEnvironmentImpl>(new RtspClientEnvironmentImpl());
}

} // namespace rtsp
} // namespace ipcam
} // namespace best
} // namespace vcp
