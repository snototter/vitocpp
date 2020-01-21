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
#define MAX_STREAMING_TIME_PER_CONFIG 20000
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

  const std::vector<std::string> frame_labels = capture->FrameLabels();
  VCP_LOG_INFO_DEFAULT(*capture);

  capture->WaitForInitialFrames(5000);

  VCP_TIC;
  std::chrono::high_resolution_clock::time_point tp_query;
  while (capture->AreDevicesAvailable() && elapsed_ms < MAX_STREAMING_TIME_PER_CONFIG)
  {
    std::vector<cv::Mat> frames;
    // Since this demo only displays the streams, we might be too
    // fast (querying for the next set of frames).
    // The following loop briefly delays the main thread if no
    // frames are available (up to 500 ms).
    tp_query = std::chrono::high_resolution_clock::now();
    while (!capture->AreFramesAvailable())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
      const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(std::chrono::high_resolution_clock::now() - tp_query);
      if (duration.count() > 500)
        break;
      VCP_LOG_INFO_DEFAULT("Capture has " << capture->NumAvailableFrames() << "/" << capture->NumStreams() << " available");
    }

    // Grab the frames and check which are available.
    frames = capture->Next();
    VCP_TOC_ASSIGN(elapsed_ms);

    std::vector<cv::Mat> valid;
    for (size_t i = 0; i < frames.size(); ++i)
    {
      if (frames[i].empty())
      {
        VCP_LOG_WARNING_DEFAULT("No frame for " << frame_labels[i]);
      }
      else
      {
        valid.push_back(frames[i]);
      }
    }
    if (valid.empty())
    {
      VCP_LOG_FAILURE_DEFAULT("Invalid frames received, continue polling for " << (MAX_STREAMING_TIME_PER_CONFIG - elapsed_ms)/1000 << " sec.");
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    // If they're available, display them. We make a collage (of resized
    // frames) if there are multiple streams to show.
    cv::Mat collage;
    vcp::imvis::collage::Collage(valid, collage, 2, 0, cv::Size(640, 480));

    // Display and let the user press ESC to exit.
    cv::imshow("Stream", collage);
    int k = cv::waitKey(20);
    if ((k & 0xFF) == 27)
      break;
  }
  VCP_LOG_INFO_DEFAULT("Closing stream after " << elapsed_ms/1000 << " sec." << std::endl
                       << "       Devices are " << (capture->AreDevicesAvailable() ? "still" : "not")
                       << " available.");

  capture.reset();
}


int main(int argc, char **argv)
{
  VCP_UNUSED_VAR(argc);
  VCP_UNUSED_VAR(argv);

  const std::vector<std::string> configs = {
    "data-best/ipcam.cfg"/*,
    "data-best/image_sequence.cfg",
    "data-best/video.cfg",
    "data-best/k4a.cfg",
    "data-best/webcam.cfg"*/
  };
  for (const auto &c : configs)
    Stream(c);

  return 0;
}
