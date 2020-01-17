#include <iostream>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/timing_utils.h>
#include <vcp_imvis/collage.h>
#include <vcp_config/config_params.h>
#include <vcp_best/capture.h>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <thread>

// Terminate the streaming demo after X ms (if there would be more incoming data).
#define MAX_STREAMING_TIME_PER_CONFIG 10000
void Stream(const std::string &config_file)
{
  std::cout << std::endl << std::endl << std::endl;

  VCP_INIT_TIC_TOC;
  VCP_LOG_INFO_DEFAULT("Streaming from configuration '" << config_file << "'");
  double elapsed_ms = 0.0;

  const auto config = vcp::config::LoadConfigParamsCpp(config_file);
  std::unique_ptr<vcp::best::Capture> capture = vcp::best::CreateCapture(*config);
  if (!capture->OpenDevices())
  {
    VCP_LOG_FAILURE("Couldn't initialize devices!");
    return;
  }

  if (!capture->StartStreams())
  {
    VCP_LOG_FAILURE_DEFAULT("Couldn't start capturing!");
    return;
  }

  capture->WaitForInitialFrames(5000);

  VCP_TIC;
  while (capture->AreFramesAvailable() && elapsed_ms < MAX_STREAMING_TIME_PER_CONFIG)
  //while (elapsed_ms < MAX_STREAMING_TIME_PER_CONFIG)
  {
    std::vector<cv::Mat> frames = capture->Next();

    VCP_TOC_ASSIGN(elapsed_ms);
    bool skip = false;
    for (const auto &frame : frames)
    {
      if (frame.empty())
        skip = true;
    }
    if (skip)
    {
      VCP_LOG_STREAM_NSEC(std::cerr, nullptr, nullptr, -1, "Invalid frames received, continue polling for " << (MAX_STREAMING_TIME_PER_CONFIG - elapsed_ms)/1000 << " sec.", 0.5);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    cv::Mat collage;
    vcp::imvis::collage::Collage(frames, collage, 2, 0, cv::Size(640, 480));

    cv::imshow("Stream", collage);
    int k = cv::waitKey(-1);
    if ((k & 0xFF) == 27)
      break;
  }
  VCP_LOG_INFO_DEFAULT("Closing stream after " << elapsed_ms/1000 << " sec.");

  // TODO test forgetting to stop/close or doing it twice, ...
  capture->StopStreams();
  capture->StopStreams();
  capture->CloseDevices();
  capture->CloseDevices();
  capture->CloseDevices();
  capture.reset();
}


int main(int argc, char **argv)
{
  VCP_UNUSED_VAR(argc);
  VCP_UNUSED_VAR(argv);

  const std::vector<std::string> configs = {
    "data-best/image_sequence.cfg",
    "data-best/video.cfg",
    "data-best/webcam.cfg"
  };
  for (const auto &c : configs)
    Stream(c);

  return 0;
}
