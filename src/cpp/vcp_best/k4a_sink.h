#ifndef __VCP_BEST_K4A_SINK_H__
#define __VCP_BEST_K4A_SINK_H__

#include "sink.h"
#include "sink_buffer.h"
#include <k4a/k4a.h>
#include <vector>
#include <string>
#include <utility>
#include <iostream>

namespace vcp
{
namespace best
{
/** @brief Kinect for Azure RGBD streaming. */
namespace k4a
{
/** @brief String representation of an empty/invalid/unset serial number. Use this if you don't want to specify a particular device. */
extern const std::string kEmptyK4ASerialNumber;


/** @brief Struct representing K4A's color control settings, which can be changed by the user. */
struct K4AColorControlSetting
{
  k4a_color_control_command_t command;
  k4a_color_control_mode_t mode;
  int32_t value;

  friend std::ostream &operator<< (std::ostream &out, const K4AColorControlSetting &s);
};


/** @brief Parametrization of a K4A device. */
struct K4ASinkParams : SinkParams
{
  //FIXME k4a: for multiple cameras, see depth stream offset: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/green_screen/main.cpp

  std::string serial_number; // Set to kEmptyK4ASerialNumber (or empty string) to apply these params to the first connected RealSense device.

  bool write_calibration;     // If set, the (factory set) calibration will be written to the calibration_file

  bool align_depth_to_color;  // Each pixel of the depth map matches the corresponding pixel of the color image.

  int32_t capture_timeout_ms; // Timeout in milliseconds for a single k4a_device_get_capture() call

  k4a_color_resolution_t color_resolution;  // Color camera resolution mode

  k4a_depth_mode_t depth_mode;  // Depth mode
  bool depth_in_meters;         // Convert depth to meters (otherwise the plain uint16 values will be returned)
  bool enable_infrared_stream;  // If true (and depth is enabled), IR will be emitted (too).

  k4a_fps_t camera_fps;         // Target/desired framerate

  bool disable_streaming_indicator;         // Set true if you want to turn off the streaming LED

  int32_t depth_delay_off_color_usec;         // Desired delay between color and depth

  uint32_t subordinate_delay_off_master_usec; // External synchronization timing

  bool synchronized_images_only;          // If true, k4a capture will skip readings with invalid color or depth

  k4a_wired_sync_mode_t wired_sync_mode;  // Set properly for multi-sensor setup (MASTER, SLAVE or STANDALONE)

  // There's no easy way to expose these settings to our configuration files/python:
  // k4a can be set either to 'auto' mode or to 'manual' mode (which requires setting
  // the corresponding value).
  // Thus, we use two different settings vectors: one for those settings which should
  // be set to auto:
  std::vector<K4AColorControlSetting> color_control_auto;
  // ... and one for manual settings:
  std::vector<K4AColorControlSetting> color_control_manual;


  K4ASinkParams(const SinkParams &params,
            const std::string &serial_number=kEmptyK4ASerialNumber,
            const bool write_calibration=false,
            const bool align_depth_to_color=false,
            const int32_t capture_timeout_ms=1000,
            const k4a_color_resolution_t color_resolution=K4A_COLOR_RESOLUTION_1080P,
            const k4a_depth_mode_t &depth_mode=K4A_DEPTH_MODE_NFOV_UNBINNED,
            const bool depth_in_meters=false,
            const bool enable_infrared_stream=false,
            const k4a_fps_t &camera_fps=K4A_FRAMES_PER_SECOND_15,
            const bool disable_streaming_indicator=false,
            const int32_t depth_delay_off_color_usec=0,
            const uint32_t subordinate_delay_off_master_usec=0,
            const bool synchronized_images_only=true,
            const k4a_wired_sync_mode_t &wired_sync_mode=K4A_WIRED_SYNC_MODE_STANDALONE,
            const std::vector<K4AColorControlSetting> &color_control_auto=std::vector<K4AColorControlSetting>(),
            const std::vector<K4AColorControlSetting> &color_control_manual=std::vector<K4AColorControlSetting>())
    : SinkParams(params),
      serial_number(serial_number),
      write_calibration(write_calibration),
      align_depth_to_color(align_depth_to_color),
      capture_timeout_ms(capture_timeout_ms),
      color_resolution(color_resolution),
      depth_mode(depth_mode),
      depth_in_meters(depth_in_meters),
      enable_infrared_stream(enable_infrared_stream),
      camera_fps(camera_fps),
      disable_streaming_indicator(disable_streaming_indicator),
      depth_delay_off_color_usec(depth_delay_off_color_usec),
      subordinate_delay_off_master_usec(subordinate_delay_off_master_usec),
      synchronized_images_only(synchronized_images_only),
      wired_sync_mode(wired_sync_mode),
      color_control_auto(color_control_auto),
      color_control_manual(color_control_manual)
  {}

  bool IsColorStreamEnabled() const;
  bool IsDepthStreamEnabled() const;
  bool IsInfraredStreamEnabled() const;
  bool RequiresWiredSync() const;
};


/** @brief Brief device info returned from \c ListK4ADevices(). */
struct K4ADeviceInfo
{
  std::string serial_number;
  std::string name; // Human readable string
  uint32_t device_number; // Device enumeration used by libk4a to open the device.

  K4ADeviceInfo() : serial_number(kEmptyK4ASerialNumber), name("Unknown"), device_number(0) {}
};


/** @brief Parses the configuration group "cam_param" into a K4ASinkParams configuration. */
K4ASinkParams K4ASinkParamsFromConfig(const vcp::config::ConfigParams &config, const std::string &cam_param);


/**
 * @brief Creates a StreamSink to capture from an Azure Kinect using libk4a.
 * @param sink_buffer A sink buffer which will be used as image queue.
 */
std::unique_ptr<StreamSink> CreateBufferedK4ASink(const K4ASinkParams &params, std::unique_ptr<SinkBuffer> rgb_buffer, std::unique_ptr<SinkBuffer> depth_buffer, std::unique_ptr<SinkBuffer> ir_buffer);

/**
 * @brief Creates a StreamSink to capture from an Azure Kinect device, specify size of the image queue as template parameter.
 */
template <int BufferCapacity>
std::unique_ptr<StreamSink> CreateK4ASink(const K4ASinkParams &params)
{
  return CreateBufferedK4ASink(params,
                               std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()),
                               std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()),
                               std::move(CreateCircularStreamSinkBuffer<BufferCapacity>()));
}

/** @brief Returns a list of connected Azure Kinects. */
std::vector<K4ADeviceInfo> ListK4ADevices(bool warn_if_no_devices=true);

/** @brief Checks if the configuration belongs to a K4A (Kinect Azure) device (using the configuration parameter "cameraX.type"). */
bool IsK4A(const std::string &type_param);

} // namespace k4a
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_K4A_SINK_H__
