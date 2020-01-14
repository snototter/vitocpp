#ifndef __VCP_UTILS_STRING_UTILS_H__
#define __VCP_UTILS_STRING_UTILS_H__

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>

// TODO generalize to basic_string and make header-only, e.g. trim: https://github.com/wichtounet/cpp_utils

namespace vcp {
namespace utils {
namespace string {
  /**
   * @brief Checks if the given string ends with another string
   */
  bool EndsWith(const std::string &s, const std::string &end);

  /**
   * @brief Checks if the given string ends with the given character
   */
  bool EndsWith(const std::string &s, char end);


  /** @brief Checks if the given string starts with the prefix. */
  bool StartsWith(const std::string &s, const std::string &prefix);

  /**
   * @brief Convert string to lower case
   */
  void ToLower(std::string &s);

  /**
   * @brief Convert string to upper case
   */
  void ToUpper(std::string &s);

  /**
   * @brief Remove leading and trailing white space
   */
  std::string &Trim(std::string &s);

  /**
   * @brief Remove leading white space
   */
  std::string &LTrim(std::string &s);

  /**
   * @brief Remove trailing white space
   */
  std::string &RTrim(std::string &s);

  //TODO test! (uses strtol and strtod, so both integer and single/double precision types should be covered)
  /** @brief Checks whether the string contains a valid number. */
  bool IsNumeric(const std::string &s);

  /**
   * @brief Tokenize string by given delimiter
   */
  std::vector<std::string> Split(const std::string &s, char delim);

  /**
   * @brief Tokenize string by given delimiter (return tokens within vector elems)
   */
  void Split(const std::string &s, char delim, std::vector<std::string> &elems);

  /** @brief Replaces (the first occurence of) a substring of the given string
    * @param[in] str The string
    * @param[in] search String to search for
    * @param[in] replacement The replacement string
    * @return the string with the replaced part, or an empty string
    */
  std::string Replace(const std::string &str, const std::string &search, const std::string &replacement);

  /** @brief Template to allow conversion from (almost) anything to a string.
    * @param[in] t Object/value to convert to a string (stream operator overloads must be available).
    */
  template <typename T>
  std::string ToStr(const T &t) {
    std::ostringstream os;
    os << t;
    return os.str();
  }

  /** @brief Template to allow conversion from numbers to a string with given precision.
    * @param[in] t Value to convert to a string.
    * @param[in] prec Precision for decimal numbers.
    */
  template <typename T>
  std::string ToStrPrec(const T &t, int prec) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << t;
    return os.str();
  }

  /**
   * @brief Returns a human readable string approximating the given time, e.g. sec2str(3700*24+50) = '1 day 41 mins'
   * @param seconds
   * @return
   */
  std::string SecondsToStr(int seconds);


  /** @brief Clips the given URL to include only protocol and domain (strips path, etc.). */
  std::string ClipUrl(const std::string &url);

  /** @brief Returns the given URL after replacing the user's authentication data. */
  std::string ObscureUrlAuthentication(const std::string &url);

} // namespace string
} // namespace utils
} // namespace vcp

#endif // __VCP_UTILS_STRING_UTILS_H__
