#include "file_utils.h"
#include "string_utils.h"
#include "vcp_logging.h"
#include <algorithm>
#include <fstream>
#include <sstream>

// C includes for mkdir/mkpath and IsDir
#include <errno.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif /* HAVE_UNISTD_H */
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#if defined(_WIN16) || defined(_WIN32) || defined(_WIN64) || defined(WINDOWS)
  #error "Include headers for file handling on Windows!"
#else // Unix variants provide stat and dirent
  #include <dirent.h>
  #include <sys/stat.h>
#endif

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::utils::file"
namespace vcp
{
namespace utils
{
namespace file
{
namespace filename_filter
{
bool IncludeAll(const std::string &/*filename*/)
{
  return true;
}


bool HasImageExtension(const std::string &filename)
{
  // Supported image formats based on OpenCV doc note:
  // http://docs.opencv.org/2.4/doc/tutorials/introduction/display_image/display_image.html
  namespace vus = vcp::utils::string;

  // Convert to lower case
  std::string l = filename;
  vus::ToLower(l);

  if (vus::EndsWith(l, ".jpg") || vus::EndsWith(l, ".jpeg") || vus::EndsWith(l, ".jpe") ||
      vus::EndsWith(l, ".bmp") || vus::EndsWith(l, ".pbm") || vus::EndsWith(l, ".pgm") ||
      vus::EndsWith(l, ".ppm") || vus::EndsWith(l, ".png") || vus::EndsWith(l, ".sr") ||
      vus::EndsWith(l, ".ras") || vus::EndsWith(l, ".jp2") || vus::EndsWith(l, ".tif") ||
      vus::EndsWith(l, ".tiff"))
  {
    return true;
  }
  return false;
}


bool CompareFileLengthsAndNames(const std::string &filename1, const std::string &filename2)
{
  // Also check length to properly sort frame0, frame1, frame2, frame10, frame100, etc.
  if (filename1.length() < filename2.length())
    return true;
  if (filename1.length() > filename2.length())
    return false;
  if (filename1.compare(filename2) < 0)
    return true;
  return false;
}

bool CompareFilenames(const std::string &filename1, const std::string &filename2)
{
  if (filename1.compare(filename2) < 0)
    return true;
  return false;
}

} // namespace dir_entry_filters

namespace
{
// mkdir and mkpath are based on J. Leffler's stackoverflow post:
// http://stackoverflow.com/questions/675039/how-can-i-create-directory-tree-in-c-linux
bool MkDir(const char *path, mode_t mode)
{
  struct stat st;
  bool success = true;

  if (stat(path, &st) != 0)
  {
    /* Directory does not exist. EEXIST for race condition */
    if (mkdir(path, mode) != 0 && errno != EEXIST)
      success = false;
  }
  else if (!S_ISDIR(st.st_mode))
  {
    errno = ENOTDIR;
    success = false;
  }

  return success;
}

// mkdir and mkpath are based on J. Leffler's stackoverflow post:
// http://stackoverflow.com/questions/675039/how-can-i-create-directory-tree-in-c-linux
/**
* mkpath - ensure all directories in path exist
* Algorithm takes the pessimistic view and works top-down to ensure
* each directory in path exists, rather than optimistically creating
* the last element and working backwards.
*/
bool MkPath(const char *path, mode_t mode)
{
  char *sp;
  char *copypath = strdup(path);

  bool success = true;
  char *pp = copypath;
  while (success && (sp = strchr(pp, '/')) != 0)
  {
    if (sp != pp)
    {
      // Neither root nor double slash in path
      *sp = '\0';
      success = MkDir(copypath, mode);
      *sp = '/';
    }
    pp = sp + 1;
  }
  if (success)
    success = MkDir(path, mode);
  free(copypath);
  return success;
}

} // namespace

#if defined(WIN32) || defined(_WIN32) || defined(__CYGWIN__)
const char kFileSeparator = '\\';
#else
const char kFileSeparator = '/';
#endif

std::string FullFile(const std::string &p1, const std::string &p2)
{
  std::stringstream path;
  path << p1;
  if (!string::EndsWith(p1, kFileSeparator))
    path << kFileSeparator;
  path << p2;
  return path.str();
}

std::string FullFile(const std::vector<std::string> &path_tokens)
{
  std::stringstream path;
  if (!path_tokens.empty())
  {
    path << path_tokens[0];
    for (size_t i = 1; i < path_tokens.size(); ++i)
    {
      if (!string::EndsWith(path_tokens[i-1], kFileSeparator))
        path << kFileSeparator;
      path << path_tokens[i];
    }
  }
  return path.str();
}

std::string FullFile(std::initializer_list<std::string> path_tokens)
{
  std::stringstream path;
  bool prepend_delim = false;
  for (const auto &token : path_tokens)
  {
    if (prepend_delim)
      path << kFileSeparator;
    path << token;
    prepend_delim = !string::EndsWith(token, kFileSeparator);
  }
  return path.str();
}

bool Exists(const std::string &name)
{
  std::ifstream f(name.c_str());
  const bool status = f.good();
  f.close();
  return status;
}

// taken from http://stackoverflow.com/questions/18100097/portable-way-to-check-if-directory-exists-windows-linux-c
bool IsDir(const std::string &path)
{
  struct stat info;
  if (stat(path.c_str(), &info) != 0)
    return false;
  else if(info.st_mode & S_IFDIR)
    return true;
  else
    return false;
}

bool IsAbsolute(const std::string &path)
{
#if defined(__linux__) || defined(__unix__)
  return path.length() > 0 && path[0] == kFileSeparator;
#else
  #error "Not implemented, may be better to switch to C++14/17 std::filesystem!"
#endif
}

bool CreatePath(const std::string &path)
{
  return MkPath(path.c_str(), 0775);
}

std::vector<std::string> ListDirContents(const std::string &directory,
                                         bool (*EntryFilter)(const std::string &),
                                         bool skip_directories, bool skip_parent_self_links,
                                         bool include_files,
                                         bool (*FilenameComparator)(const std::string &, const std::string &))
{
  std::vector<std::string> contents;

  if (!IsDir(directory))
    return contents;

  // Directory parsing based on:
  // http://stackoverflow.com/questions/306533/how-do-i-get-a-list-of-files-in-a-directory-in-c
#if defined(_WIN16) || defined(_WIN32) || defined(_WIN64) || defined(WINDOWS)
  HANDLE dir;
  WIN32_FIND_DATA file_data;

  if ((dir = FindFirstFile(vcp::utils::file::FullFile(directory, "*").c_str(), &file_data)) != INVALID_HANDLE_VALUE)
  {
    do
    {
      const std::string file_name = file_data.cFileName;

      if (skip_parent_self_links && (file_name.length() > 0 && file_name[0] == '.')
        continue;

      const bool is_directory = (file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
      if (skip_directories && is_directory)
        continue;

      if (!include_files && !is_directory)
        continue;

      const bool include_by_filter = EntryFilter(file_name);
      if (!include_by_filter)
        continue;

      contents.push_back(file_name);
    }
    while (FindNextFile(dir, &file_data));
  }
  FindClose(dir);
#else // We build on a unix variant:
  DIR *dir;
  class dirent *ent;
  class stat st;

  dir = opendir(directory.c_str());
  while ((ent = readdir(dir)) != NULL)
  {
    const std::string file_name = ent->d_name;

    if (skip_parent_self_links && (file_name.length() > 0 && file_name[0] == '.'))
      continue;

    const std::string full_file_name = vcp::utils::file::FullFile(directory , file_name);
    if (stat(full_file_name.c_str(), &st) == -1)
      continue;

    // Skip sub-directories?
    const bool is_directory = (st.st_mode & S_IFDIR) != 0;
    if (skip_directories && is_directory)
      continue;

    // Include only sub-directories?
    if (!include_files && !is_directory)
      continue;

    const bool include_by_filter = EntryFilter(file_name);
    if (!include_by_filter)
      continue;

    contents.push_back(file_name);
  }
  closedir(dir);
#endif

  if (FilenameComparator)
    std::sort(contents.begin(), contents.end(), FilenameComparator);
  return contents;
}


std::vector<std::string> ListOrPeekDirContents(const std::string &directory,
                                               size_t max_num_entries,
                                               bool (*EntryFilter)(const std::string &),
                                               bool skip_directories, bool skip_parent_self_links,
                                               bool include_files,
                                               bool (*FilenameComparator)(const std::string &, const std::string &))
{
  std::vector<std::string> contents;

  if (!IsDir(directory))
    return contents;

  // Directory parsing based on:
  // http://stackoverflow.com/questions/306533/how-do-i-get-a-list-of-files-in-a-directory-in-c
#if defined(_WIN16) || defined(_WIN32) || defined(_WIN64) || defined(WINDOWS)
  HANDLE dir;
  WIN32_FIND_DATA file_data;

  if ((dir = FindFirstFile(vcp::utils::file::FullFile(directory, "*").c_str(), &file_data)) != INVALID_HANDLE_VALUE)
  {
    do
    {
      const std::string file_name = file_data.cFileName;

      if (skip_parent_self_links && (file_name.length() > 0 && file_name[0] == '.')
        continue;

      const bool is_directory = (file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
      if (skip_directories && is_directory)
        continue;

      if (!include_files && !is_directory)
        continue;

      const bool include_by_filter = EntryFilter(file_name);
      if (!include_by_filter)
        continue;

      contents.push_back(file_name);
    }
    while (FindNextFile(dir, &file_data) && contents.size() < max_num_entries);
  }
  FindClose(dir);
#else // We build on a unix variant:
  DIR *dir;
  class dirent *ent;
  class stat st;

  dir = opendir(directory.c_str());
  while ((ent = readdir(dir)) != NULL && contents.size() < max_num_entries)
  {
    const std::string file_name = ent->d_name;

    if (skip_parent_self_links && (file_name.length() > 0 && file_name[0] == '.'))
      continue;

    const std::string full_file_name = vcp::utils::file::FullFile(directory , file_name);
    if (stat(full_file_name.c_str(), &st) == -1)
      continue;

    // Skip sub-directories?
    const bool is_directory = (st.st_mode & S_IFDIR) != 0;
    if (skip_directories && is_directory)
      continue;

    // Include only sub-directories?
    if (!include_files && !is_directory)
      continue;

    const bool include_by_filter = EntryFilter(file_name);
    if (!include_by_filter)
      continue;

    contents.push_back(file_name);
  }
  closedir(dir);
#endif

  if (FilenameComparator)
    std::sort(contents.begin(), contents.end(), FilenameComparator);
  return contents;
}


std::string GetExtension(const std::string &filename)
{
  if (filename.empty())
    return std::string();
  const std::vector<std::string> token = vcp::utils::string::Split(filename, '.');
  if (token.size() < 2) // given filename has no extension, e.g. 'foo'
    return std::string();
  return "." + token[token.size()-1];
}


std::string SlurpAsciiFile(const std::string &filename)
{
  std::ifstream ifs(filename.c_str(), std::ios::in);
  if (!ifs.is_open())
  {
    VCP_LOG_FAILURE("Cannot open file to slurp, check location of '" << filename << "'");
    return std::string();
  }

  std::stringstream sstr;
  sstr << ifs.rdbuf();
  ifs.close();
  return sstr.str();
}

} // namespace file
} // namespace utils
} // namespace vcp

