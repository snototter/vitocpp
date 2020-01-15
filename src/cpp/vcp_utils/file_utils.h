#ifndef __VCP_UTILS_FILE_UTILS_H__
#define __VCP_UTILS_FILE_UTILS_H__

#include <string>
#include <sstream>
#include <vector>
#include <initializer_list>

namespace vcp
{
namespace utils
{
/**
 * @brief Path and filename utilities (e.g. mkdir, listdir, exists, etc.)
 */
namespace file
{
/**
 * @brief Commonly used directory entry filters for vcp::utils::file::ListDirContents().
 */
namespace filename_filter
{
/**
 * @brief Directory entry filter to include everything, i.e. simply returns true.
 */
bool IncludeAll(const std::string &filename);


/**
 * @brief Returns true if the extension corresponds to a file format which can be loaded/decoded using OpenCV!
 *
 * See http://docs.opencv.org/2.4/doc/tutorials/introduction/display_image/display_image.html and more up-to-date lists
 * of supported image formats.
 */
bool HasImageExtension(const std::string &filename);


/**
 * @brief Comparator to sort a container of filename strings by their filename length(!).
 *
 * Comparator to sort files by filename length (alphabetically if two have the same length).
 * This properly sorts image directories with contents:
 *   frame0, frame1, frame2, frame10, frame30, frame100, etc.
 * Used within vcp::utils::file::ListDirContents().
 */
bool CompareFileLengthsAndNames(const std::string &filename1, const std::string &filename2);

/**
 * @brief Comparator to sort a container of filename strings alphabetically.
 *
 * Relies on standard string sorting order (i.e. uppercase before lowercase).
 */
bool CompareFilenames(const std::string &filename1, const std::string &filename2);
} // namespace dir_entry_filters


/** @brief Platform-specific file separator (slash or backslash). */
extern const char kFileSeparator;

/**
 * @brief Similar to MATLAB's fullfile, this function concatenates the given paths (Note: currently only appends missing file separators, no reduction of ../, etc).
 */
std::string FullFile(const std::string &p1, const std::string &p2);

/**
 * @brief Similar to MATLAB's fullfile, this function concatenates the given path tokens (Note: currently only appends missing file separators, no reduction of ../, etc).
 */
std::string FullFile(const std::vector<std::string> &path_tokens);

/**
 * @brief Similar to MATLAB's fullfile, this function concatenates the given path tokens (Note: currently only appends missing file separators, no reduction of ../, etc).
 */
std::string FullFile(std::initializer_list<std::string> path_tokens);

/**
 * @brief Checks if the given file/directory already exists.
 */
bool Exists(const std::string &name);

/**
 * @brief Check if the given path exists and is a directory.
 */
bool IsDir(const std::string &path);

/**
 * @brief Check if the given path is absolute.
 */
bool IsAbsolute(const std::string &path);

/**
 * @brief CreatePath Like mkdir -p. Default permissions 775.
 */
bool CreatePath(const std::string &path);

/**
 * @brief Returns a list of all directory contents for which the given EntryFilter returns true.
 *
 * An entry filter is given the filename, so you can check the extension, prefixes, etc. to
 * decide whether you want to include those directory entries in the output list or not.
 *
 * If you want to list only subdirectories, set skip_directories=false and include_files=false.
 *
 * @param[in] directory The directory to analyze
 * @param[in] EntryFilter the filename filter function used to decide whether to include a given filename (or subdirectory, if skip_directories is false).
 * @param[in] skip_directories Excludes any sub-directory if set true.
 * @param[in] skip_parent_self_links Excludes '.' and '..' if set true.
 * @param[in] include_files Include files if set true
 * @param[in] FilenameComparator Comparator to sort the directory contents list. Set to nullptr if you want an unsorted list.
 */
std::vector<std::string> ListDirContents(const std::string &directory,
                                         bool (*EntryFilter)(const std::string &filename) = &filename_filter::IncludeAll,
                                         bool skip_directories = true,
                                         bool skip_parent_self_links = true,
                                         bool include_files = true,
                                         bool (*FilenameComparator)(const std::string &a, const std::string &b) = &filename_filter::CompareFileLengthsAndNames);

/**
 * @brief Returns a list of at most max_num_entries directory contents for which the given EntryFilter returns true.
 * Note that for large directories, the sorted result might not be what you expect - for efficiency, we iterate over the
 * file system's entries (which might be unsorted) and abort after max_num_entries.
 *
 * @param[in] directory The directory to analyze
 * @param[in] max_num_entries Maximum number of entries to include in the resulting list.
 * @param[in] EntryFilter the filename filter function used to decide whether to include a given filename (or subdirectory, if skip_directories is false).
 * @param[in] skip_directories Excludes any sub-directory if set true.
 * @param[in] skip_parent_self_links Excludes '.' and '..' if set true.
 * @param[in] include_files Include files if set true
 * @param[in] FilenameComparator Comparator to sort the directory contents list. Set to nullptr if you want an unsorted list.
 */
std::vector<std::string> ListOrPeekDirContents(const std::string &directory,
                                               size_t max_num_entries,
                                               bool (*EntryFilter)(const std::string &filename) = &filename_filter::IncludeAll,
                                               bool skip_directories = true,
                                               bool skip_parent_self_links = true,
                                               bool include_files = true,
                                               bool (*FilenameComparator)(const std::string &a, const std::string &b) = &filename_filter::CompareFileLengthsAndNames);

/** @brief Returns the extension (e.g. '.cpp') of the given filename (full path or base name). Returns empty string if there is no dot. */
std::string GetExtension(const std::string &filename);


/** @brief Reads the plain text file into a string. */
std::string SlurpAsciiFile(const std::string &filename);
} // namespace file
} // namespace utils
} // namespace vcp

#endif // __VCP_UTILS_FILE_UTILS_H__
