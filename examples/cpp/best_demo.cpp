#include <iostream>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/timing_utils.h>
#include <vcp_utils/file_utils.h>
#include <chrono>

// Terminate the streaming demo after X ms (if there would be more incoming data).
#define MAX_STREAMING_TIME_PER_CONFIG 10000
void Stream(const std::string &config_file)
{
  VCP_INIT_TIC_TOC;

  VCP_LOG_INFO_DEFAULT("Streaming from configuration '" << config_file << "'");
  VCP_TIC;
  double elapsed_ms = 0.0;
  //TODO set up capture, stream for up to X seconds, finish

  while (elapsed_ms < MAX_STREAMING_TIME_PER_CONFIG)
  {
    VCP_TOC_ASSIGN(elapsed_ms);
  }
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
