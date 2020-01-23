#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/timing_utils.h>
#include <vcp_imutils/matutils.h>
#include <vcp_imvis/collage.h>
#include <vcp_imvis/pseudocolor.h>
#include <vcp_imvis/drawing.h>
#include <vcp_config/config_params.h>
#include <vcp_best/capture.h>
#include <vcp_utils/string_utils.h>
#include <opencv2/highgui.hpp>


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
    VCP_LOG_FAILURE("Couldn't initialize streaming devices!");
    return;
  }

  if (!capture->StartStreams())
  {
    VCP_LOG_FAILURE_DEFAULT("Couldn't start streams!");
    return;
  }

  const std::vector<std::string> frame_labels = capture->FrameLabels();
  VCP_LOG_INFO_DEFAULT("Successfully started capture:" << std::endl << *capture);

  if (!capture->WaitForInitialFrames(5000))
  {
    VCP_LOG_FAILURE("Didn't receive an initial set of frames - aborting.");
    return;
  }

  VCP_TIC;
  std::chrono::high_resolution_clock::time_point tp_query;
  while (capture->AreAllDevicesAvailable() && elapsed_ms < MAX_STREAMING_TIME_PER_CONFIG)
  {
    std::vector<cv::Mat> frames;
    // Since this demo only displays the streams, we might be too
    // fast (querying for the next set of frames).
    // The following loop briefly delays the main thread if no
    // frames are available (up to 500 ms).
    tp_query = std::chrono::high_resolution_clock::now();
    while (!capture->AreAllFramesAvailable())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
      const auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(std::chrono::high_resolution_clock::now() - tp_query);
      if (duration.count() > 500)
        break;
      VCP_LOG_INFO_DEFAULT("Capture has " << capture->NumAvailableFrames() << "/" << capture->NumStreams() << " available");
    }

    // Grab the frames.
    frames = capture->Next();
    VCP_TOC_ASSIGN(elapsed_ms);

    // Again, the following is only needed for this demo.
    // We filter invalid frames, colorize depth/IR streams, etc.
    std::vector<cv::Mat> valid_raw, valid_vis;
    for (size_t i = 0; i < frames.size(); ++i)
    {
      if (frames[i].empty())
      {
        VCP_LOG_WARNING_DEFAULT("No frame for " << frame_labels[i]);
      }
      else
      {
        cv::Mat vis;
        if (frames[i].depth() != CV_8U)
        {
          vcp::imvis::pseudocolor::Colorize(frames[i], vcp::imvis::pseudocolor::ColorMap::Turbo, vis, 0, 3000);
        }
        else
          vis = frames[i];
        valid_raw.push_back(frames[i]);
        valid_vis.push_back(vis);
      }
    }
    if (valid_raw.empty())
    {
      VCP_LOG_FAILURE_DEFAULT("Invalid frames received, continue polling for " << (MAX_STREAMING_TIME_PER_CONFIG - elapsed_ms)/1000 << " sec.");
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    // Make a collage (of resized frames) if there are multiple streams to show.
    cv::Mat collage;
    const cv::Size fixed_size = cv::Size(800, 600);
    const int num_per_row = 2;
    vcp::imvis::collage::Collage(valid_vis, collage, num_per_row, 0, fixed_size);

    // Overlay the sink label and original frame resolution.
    for (size_t i = 0; i < valid_raw.size(); ++i)
    {
      double minv, maxv;
      cv::minMaxIdx(valid_raw[i], &minv, &maxv);
      std::stringstream overlay;
      overlay << frame_labels[i] << " "
              << vcp::utils::string::ToStr(valid_raw[i].cols) << "x"
              << vcp::utils::string::ToStr(valid_raw[i].rows) << " "
              << vcp::imutils::CVMatDepthToString(valid_raw[i].type(), valid_raw[i].channels()).substr(3) // Skip CV_ prefix
              << " [" << std::setw(5) << std::right << minv << ", " << std::setw(5) << std::right << maxv << "]";
      vcp::imvis::drawing::DrawTextBox(collage, overlay.str(),
          cv::Point((i % num_per_row) * fixed_size.width, (i / num_per_row) * fixed_size.height),
          vcp::imvis::drawing::textanchor::TOP | vcp::imvis::drawing::textanchor::LEFT,
          10, 0.5, cv::Scalar(255, 0, 0), cv::Scalar::all(-1),
          cv::FONT_HERSHEY_PLAIN, 1.5, 2);
    }

    // Display and let the user press ESC to exit.
    cv::imshow("Stream", collage);
    int k = cv::waitKey(20);
    if ((k & 0xFF) == 27)
      break;
  }
  VCP_LOG_INFO_DEFAULT("Closing stream after " << elapsed_ms/1000 << " sec." << std::endl
                       << "       Devices are " << (capture->AreAllDevicesAvailable() ? "still" : "not")
                       << " available.");

  capture.reset();
}


int main(int argc, char **argv)
{
  VCP_UNUSED_VAR(argc);
  VCP_UNUSED_VAR(argv);

  const std::vector<std::string> configs = {
    "data-best/k4a.cfg",
    "data-best/ipcam.cfg",
    /*"data-best/image_sequence.cfg",
    "data-best/video.cfg",
    "data-best/webcam.cfg"*/
  };
  for (const auto &c : configs)
    Stream(c);

  return 0;
}
