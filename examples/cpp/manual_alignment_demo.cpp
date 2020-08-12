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
#include <vcp_best/liveview.h>
#include <vcp_utils/string_utils.h>
#include <opencv2/highgui.hpp>

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>


cv::Mat Extract16U(k4a_image_t &image, k4a_transformation_t &transformation, int dst_width, int dst_height)
{
  cv::Mat extracted;
  if (image != nullptr)
  {
    // OpenCV matrix header to point to the raw or warped depth data.
    cv::Mat tmp;

    k4a_image_t aligned_depth_image = NULL;
    // Official warping example:
    // https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/transformation/main.cpp
    if (k4a_image_create(
            K4A_IMAGE_FORMAT_DEPTH16,
            dst_width, dst_height,
            dst_width * static_cast<int>(sizeof(uint16_t)), &aligned_depth_image)
          != K4A_RESULT_SUCCEEDED)
    {
      VCP_LOG_FAILURE("Cannot allocate K4A image buffer to warp depth to color!");
    }
    else
    {
      if (k4a_transformation_depth_image_to_color_camera(transformation, image, aligned_depth_image)
          != K4A_RESULT_SUCCEEDED)
      {
        VCP_LOG_FAILURE("Cannot align K4A depth image to color image!");
      }
      else
      {
        // Get image buffer and size
        uint8_t* buffer = k4a_image_get_buffer(aligned_depth_image);
        const int rows = k4a_image_get_height_pixels(aligned_depth_image);
        const int cols = k4a_image_get_width_pixels(aligned_depth_image);
        // Create OpenCV Mat header pointing to the buffer (no copy yet!)
        tmp = cv::Mat(rows, cols, CV_16U, static_cast<void*>(buffer), k4a_image_get_stride_bytes(aligned_depth_image));
      }
    }

    extracted = tmp.clone();
    // Now it's safe to free the memory
    k4a_image_release(image);

    image = NULL;
    if (aligned_depth_image)
    {
      k4a_image_release(aligned_depth_image);
      aligned_depth_image = NULL;
    }
  }
  else
  {
    VCP_LOG_WARNING("extract16 called without a valid image!");
  }
  return extracted;
}

cv::Mat WarpDepth(const cv::Mat &depth, k4a_transformation_t &transformation, int dst_width, int dst_height)
{
  cv::Mat dcvt;
  depth.convertTo(dcvt, CV_16U);

  k4a_image_t image;
  if (k4a_image_create_from_buffer(K4A_IMAGE_FORMAT_DEPTH16, dcvt.cols, dcvt.rows, dcvt.step[0], depth.data, dcvt.cols*dcvt.rows*sizeof(uint16_t), NULL, NULL, &image)
      != K4A_RESULT_SUCCEEDED)
  {
    VCP_LOG_FAILURE("CANNOT CREATE K4A FROM OPENCV BUFFER!!!!");
    return cv::Mat();
  }

//Works  ///////RESTORE & check if conversion cv -> k4a -> cv works
//  // Get image buffer and size
//  uint8_t* buffer = k4a_image_get_buffer(image);
//  const int rows = k4a_image_get_height_pixels(image);
//  const int cols = k4a_image_get_width_pixels(image);
//  // Create OpenCV Mat header pointing to the buffer (no copy yet!)
//  cv::Mat tmp = cv::Mat(rows, cols, CV_16U, static_cast<void*>(buffer), k4a_image_get_stride_bytes(image));
//  return tmp.clone();

  cv::Mat warped = Extract16U(image, transformation, dst_width, dst_height);
  double mi, ma;
  cv::minMaxIdx(warped, &mi, &ma);
  VCP_LOG_FAILURE("Min/max warped: " << mi << ", " << ma);
  return warped;
}

// Terminate the streaming demo after X ms (if there would be more incoming data).
#define MAX_STREAMING_TIME_PER_CONFIG 200000
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

  if (!capture->WaitForFrames(5000, true))
  {
    VCP_LOG_FAILURE("Didn't receive an initial set of frames - aborting.");
    return;
  }

  cv::Mat Kc, Kd, R, t;
  Kc = capture->CameraMatrixAt(0);
  Kd = capture->CameraMatrixAt(1);
  capture->StereoTransformation(1, R, t);

  //FIXME memory management (delete trafo, delete img in warpdetph())
  const int warp_dest_width = 400;
  const int warp_dest_height = 400;
  const double sx = warp_dest_width / 1280.0;
  const double sy = warp_dest_height / 720.0;

  k4a_calibration_t calib;
  //  k4a_calibration_camera_t color_camera_calibration; /**< Color camera calibration. */
  calib.color_camera_calibration.intrinsics.type = K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY;
  calib.color_camera_calibration.intrinsics.parameters.param.fx = Kc.at<double>(0, 0) * sx;
  calib.color_camera_calibration.intrinsics.parameters.param.fy = Kc.at<double>(1, 1) * sy;
  calib.color_camera_calibration.intrinsics.parameters.param.cx = Kc.at<double>(0, 2) * sx;
  calib.color_camera_calibration.intrinsics.parameters.param.cy = Kc.at<double>(1, 2) * sy;
  calib.color_camera_calibration.intrinsics.parameters.param.k1 = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.k2 = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.k3 = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.k4 = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.k5 = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.k6 = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.p1 = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.p2 = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.codx = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.cody = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.metric_radius = 0;
  calib.color_camera_calibration.intrinsics.parameter_count = 14;
  calib.color_camera_calibration.resolution_width = warp_dest_width;
  calib.color_camera_calibration.resolution_height = warp_dest_height;


  //  k4a_calibration_camera_t depth_camera_calibration; /**< Depth camera calibration. */
  calib.depth_camera_calibration.intrinsics.type = K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY;
  calib.depth_camera_calibration.intrinsics.parameters.param.fx = Kd.at<double>(0, 0);
  calib.depth_camera_calibration.intrinsics.parameters.param.fy = Kd.at<double>(1, 1);
  calib.depth_camera_calibration.intrinsics.parameters.param.cx = Kd.at<double>(0, 2);
  calib.depth_camera_calibration.intrinsics.parameters.param.cy = Kd.at<double>(1, 2);
  calib.depth_camera_calibration.intrinsics.parameters.param.k1 = 0;
  calib.depth_camera_calibration.intrinsics.parameters.param.k2 = 0;
  calib.depth_camera_calibration.intrinsics.parameters.param.k3 = 0;
  calib.depth_camera_calibration.intrinsics.parameters.param.k4 = 0;
  calib.depth_camera_calibration.intrinsics.parameters.param.k5 = 0;
  calib.depth_camera_calibration.intrinsics.parameters.param.k6 = 0;
  calib.depth_camera_calibration.intrinsics.parameters.param.p1 = 0;
  calib.depth_camera_calibration.intrinsics.parameters.param.p2 = 0;
  calib.depth_camera_calibration.intrinsics.parameters.param.codx = 0;
  calib.depth_camera_calibration.intrinsics.parameters.param.cody = 0;
  calib.depth_camera_calibration.intrinsics.parameters.param.metric_radius = 0;
  calib.depth_camera_calibration.intrinsics.parameter_count = 14;
  calib.depth_camera_calibration.resolution_width = 640;
  calib.depth_camera_calibration.resolution_height = 576;

  calib.color_resolution = K4A_COLOR_RESOLUTION_720P;//THIS MIGHT BE A MAJOR PROBLEM! # FIXME check with downscaled Kc!
  calib.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

  // Default value corresponding to ~120 deg fov
  calib.depth_camera_calibration.metric_radius = 1.7f;
//  calib.depth_camera_calibration.intrinsics.parameters.param.metric_radius = 1.7f;
  calib.color_camera_calibration.metric_radius = 1.7f;
//  calib.color_camera_calibration.intrinsics.parameters.param.metric_radius = 1.7f;

  for (int j = 0; j < 15; ++j)
    VCP_LOG_FAILURE("  COLOR CALIB #" << j << ": " << calib.color_camera_calibration.intrinsics.parameters.v[j]);

  for (int j = 0; j < 15; ++j)
    VCP_LOG_FAILURE("  DEPTH CALIB #" << j << ": " << calib.depth_camera_calibration.intrinsics.parameters.v[j]);



  //  /** Extrinsic transformation parameters.
  //   *
  //   * The extrinsic parameters allow 3D coordinate conversions between depth camera, color camera, the IMU's gyroscope
  //   * and accelerometer. To transform from a source to a target 3D coordinate system, use the parameters stored
  //   * under extrinsics[source][target].
  //   */
  //  k4a_calibration_extrinsics_t extrinsics[K4A_CALIBRATION_TYPE_NUM][K4A_CALIBRATION_TYPE_NUM];
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[0] = R.at<double>(0, 0);
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[1] = R.at<double>(0, 1);
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[2] = R.at<double>(0, 2);
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[3] = R.at<double>(1, 0);
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[4] = R.at<double>(1, 1);
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[5] = R.at<double>(1, 2);
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[6] = R.at<double>(2, 0);
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[7] = R.at<double>(2, 1);
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[8] = R.at<double>(2, 2);

  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[0] = t.at<double>(0);
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[1] = t.at<double>(1);
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[2] = t.at<double>(2);

//  k4a_depth_mode_t depth_mode;             /**< Depth camera mode for which calibration was obtained. */
//  k4a_color_resolution_t color_resolution; /**< Color camera resolution for which calibration was obtained. */

  k4a_transformation_t trafo = k4a_transformation_create(&calib);

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

    cv::Mat warped = WarpDepth(frames[1], trafo, warp_dest_width, warp_dest_height);
    VCP_LOG_FAILURE("WARPED SIZE: " << warped.cols << "x" << warped.rows);
    cv::Mat vis_warped;
    vcp::imvis::pseudocolor::Colorize(warped, vcp::imvis::pseudocolor::ColorMap::Turbo, vis_warped, 0, 3000);
    valid_vis[valid_vis.size()-1] = vis_warped;

    cv::Mat col_resized;
    cv::resize(frames[0], col_resized, cv::Size(warp_dest_width, warp_dest_height));
    cv::addWeighted(col_resized, 0.6, vis_warped, 0.4, 0, col_resized);
    valid_vis.push_back(col_resized);

    // Make a collage (of resized frames) if there are multiple streams to show.
    cv::Mat collage;
    const cv::Size fixed_size = cv::Size(800, 600);
    const int num_per_row = 4;
    vcp::imvis::collage::Collage(valid_vis, collage, num_per_row, 0);// fixed_size);

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

//    viewer->PushImageRequest(collage);
//    VCP_LOG_FAILURE("Viewer fps: " << viewer->GetRequestFrameRate() << " vs " << viewer->GetDisplayFrameRate());
//    int k = viewer->GetLastUserInput();


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

//  const std::vector<std::string> configs = {
//    "data-best/k4a.cfg",
//    "data-best/ipcam.cfg",
//    /*"data-best/image_sequence.cfg",
//    "data-best/video.cfg",
//    "data-best/webcam.cfg"*/
//  };
//  for (const auto &c : configs)
//    Stream(c);
//  Stream("data-best/webcam.cfg");
//  Stream("data-best/realsense.cfg");
  Stream("data-best/k4a-manual-alignment.cfg");

  return 0;
}
