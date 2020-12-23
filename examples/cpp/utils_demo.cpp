#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <fstream>
#include <chrono>
#include <thread>

#include <vcp_utils/string_utils.h>
#include <vcp_utils/file_utils.h>

#define VCP_LOG_LEVEL_INFO
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/stop_watch.h>

int main(int argc, char **argv)
{
  (void)(argc);
  (void)(argv);

  VCP_LOG_DEBUG("Debug message - enable/disable via #define");
  VCP_LOG_DEBUG_DEFAULT("==> log always (without location)");
  VCP_LOG_DEBUG_LOCATION("==> log always (with location)");
  VCP_LOG_INFO("Status message - enable/disable via #define");
  VCP_LOG_WARNING("Status message - enable/disable via #define");
  VCP_LOG_FAILURE("Status message - always enabled!");

  VCP_LOG_INFO("String utilities:");
  std::cout << "  str::endswith should be False? " << vcp::utils::string::EndsWith("abc", 'a') << std::endl
            << "  str::endswith should be True?  " << vcp::utils::string::EndsWith("abc",'c') << std::endl;

  std::cout << "Seconds to string: " << std::endl
            << "    10: " << vcp::utils::string::SecondsToStr(10) << std::endl
            << "   379: " << vcp::utils::string::SecondsToStr(379) << std::endl
            << " 10599: " << vcp::utils::string::SecondsToStr(10599) << std::endl
            << "999999: " << vcp::utils::string::SecondsToStr(999999) << std::endl << std::endl;

  VCP_LOG_INFO("Printing durations: " << vcp::utils::string::ToStr(std::chrono::seconds{10})
               << ", " << vcp::utils::string::ToStr(std::chrono::milliseconds{33})
               << ", " << vcp::utils::string::ToStr(std::chrono::microseconds{15})
               << ", " << vcp::utils::string::ToStr(std::chrono::nanoseconds{3000}));

  VCP_LOG_INFO("Filename utilities:");
  const std::vector<std::string> tokens = {"one", "tokenized/", "path"};
  std::cout << "Fullfile: " << vcp::utils::file::FullFile(tokens) << std::endl;
  std::cout << "Fullfile (initlist): " << vcp::utils::file::FullFile({"some", "path", tokens[0], "foo/", "bar"}) << std::endl;
  std::cout << "Fullfile: " << vcp::utils::file::FullFile("", "foo") << std::endl << std::endl;

  VCP_LOG_INFO("File utilities:");
  const std::string dir = "/tmp/a/b/c/d/efg/Foo";
  std::cout << "Exists tmp directory path: " << vcp::utils::file::Exists(dir) << std::endl;
  std::cout << "Creating tmp dir: " << vcp::utils::file::CreatePath(dir) << std::endl;
  std::cout << "Exists now? " << vcp::utils::file::Exists(dir) << std::endl;

  std::cout << "Is /tmp/.... a directory: " << vcp::utils::file::IsDir(dir) << std::endl;
  const std::string tmp_filename = std::tmpnam(nullptr);
  std::ofstream tmp_file;
  tmp_file.open(tmp_filename);
  tmp_file << "foo";
  tmp_file.close();
  std::cout << "tmp file exists: " << vcp::utils::file::Exists(tmp_filename) << std::endl
            << "tmp file is dir: " << vcp::utils::file::IsDir(tmp_filename) << std::endl << std::endl;

  //--------------------------------------------------------------
  // Directory list
  VCP_LOG_INFO("Directory listing (sort/file filters):");
  auto curr_dir_list = vcp::utils::file::ListDirContents(".");
  std::cout << "List all files in current directory (length sort):" << std::endl;
  for (const auto dirent : curr_dir_list)
    std::cout << "  " << dirent << std::endl;

  curr_dir_list = vcp::utils::file::ListDirContents(".", &vcp::utils::file::filename_filter::IncludeAll, true, true, true, &vcp::utils::file::filename_filter::CompareFilenames);
  std::cout << "List all files in current directory (alphabetical sort):" << std::endl;
  for (const auto dirent : curr_dir_list)
    std::cout << "  " << dirent << std::endl;

  curr_dir_list = vcp::utils::file::ListDirContents(".", &vcp::utils::file::filename_filter::HasImageExtension, false);
  std::cout << "List only image files:" << std::endl;
  for (const auto dirent : curr_dir_list)
    std::cout << "  " << dirent << std::endl;

  curr_dir_list = vcp::utils::file::ListDirContents(".", &vcp::utils::file::filename_filter::IncludeAll, false, false, false);
  std::cout << "List only sub directories (inc . and ..):" << std::endl;
  for (const auto dirent : curr_dir_list)
    std::cout << "  " << dirent << std::endl;


  curr_dir_list = vcp::utils::file::ListOrPeekDirContents(".", 3);
  std::cout << "List at most 3 files in current directory (length sort):" << std::endl;
  for (const auto dirent : curr_dir_list)
    std::cout << "  " << dirent << std::endl;

  // Stop watch test
  vcp::utils::LogWatch<> watch("DemoWatch");
  VCP_LOG_INFO_DEFAULT("StopWatch test, watch is steady: " << watch.IsSteady());
  for (int i = 0; i < 20; ++i)
  {
    const int to_sleep = 100 + i*5;
    watch.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(to_sleep));
    watch.PrintLog();
    VCP_LOG_INFO_DEFAULT("^ should have slept for " << to_sleep << "ms");
  }
  return 0;
}
