#include <iostream>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/timing_utils.h>
#include <vcp_best/capture.h>
#include <vcp_imvis/collage.h>
#include <chrono>

// Terminate the streaming demo after X ms (if there would be more incoming data).
#define MAX_STREAMING_TIME_PER_CONFIG 10000
void Stream(const std::string &config_file)
{
  VCP_INIT_TIC_TOC;
//FIXME won't build yet
  VCP_LOG_INFO_DEFAULT("Streaming from configuration '" << config_file << "'");
  VCP_TIC;
  double elapsed_ms = 0.0;

  std::unique_ptr<vcp::best::Capture> capture = vcp::best::CreateCapture(config_file);
  if (!capture->Open())
    VCP_ERROR("Couldn't initialize devices!");

  if (!capture->Start())
    VCP_ERROR("Couldn't start capturing!");

  while (elapsed_ms < MAX_STREAMING_TIME_PER_CONFIG)
  {
    std::vector<cv::Mat> frames = capture->NextFrames();
    cv::Mat collage;
    vcp::imvis::collage::Collage(frames, collage, 2, 0, cv::Size(640, 480));
    cv::imshow("Stream", collage);
    int k = cv::waitKey(10);
    if (k & 0xFF == 27)
      break;
    VCP_TOC_ASSIGN(elapsed_ms);
  }
  capture->Close();
}


int main(int argc, char **argv)
{
  VCP_UNUSED_VAR(argc);
  VCP_UNUSED_VAR(argv);

  const std::vector<std::string> configs = {
    "best/image_sequence.cfg",
    "best/webcam.cfg"
  };
  for (const auto &c : configs)
    Stream(c);

  return 0;
}
