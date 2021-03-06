#include "string_utils.h"
#include <algorithm>
#include <iterator>
#include <fstream>
#include <sstream>
#include "vcp_logging.h"

namespace vcp {
namespace utils {
namespace string {

bool EndsWith(const std::string &s, const std::string &end) {
  if (s.length() >= end.length())
    return (s.compare(s.length() - end.length(), end.length(), end) == 0);
  return false;
}

bool EndsWith(const std::string &s, char end) {
  if (s.length() > 0)
    return (s.at(s.length()-1) == end);
  return false;
}

bool StartsWith(const std::string &s, const std::string &prefix)
{
  if (s.length() >= prefix.length())
    return s.compare(0, prefix.length(), prefix) == 0;
  return false;
}

void ToLower(std::string &s) {
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
}

std::string Lower(const std::string &s) {
  std::string tmp(s);
  ToLower(tmp);
  return tmp;
}

void ToUpper(std::string &s) {
  std::transform(s.begin(), s.end(), s.begin(), ::toupper);
}

std::string Upper(const std::string &s) {
  std::string tmp(s);
  ToUpper(tmp);
  return tmp;
}

// trim from start
std::string LTrim(const std::string &totrim) {
  std::string s(totrim);
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}

// trim from end
std::string RTrim(const std::string &totrim) {
  std::string s(totrim);
  s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
  return s;
}

// trim from both ends
std::string Trim(const std::string &s) {
  return LTrim(RTrim(s));
}

bool IsNumeric(const std::string &s) {
  char *pd, *pl;
  // Check long
  long dummyl = strtol(s.c_str(), &pl, 10);
  VCP_UNUSED_VAR(dummyl);
  // Check double
  double dummyd = strtod(s.c_str(), &pd);
  VCP_UNUSED_VAR(dummyd);
  return (*pd == 0) || (*pl == 0);
}

void Split(const std::string &s, char delim, std::vector<std::string> &elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim))
    elems.push_back(item);
}

std::vector<std::string> Split(const std::string &s, char delim) {
  std::vector<std::string> elems;
  Split(s, delim, elems);
  return elems;
}

std::string Replace(const std::string &str, const std::string &search, const std::string &replacement) {
  size_t start_pos = str.find(search);
  if(start_pos == std::string::npos)
    return str;
  std::string s = str;
  do {
    s.replace(start_pos, search.length(), replacement);
    start_pos = s.find(search);
  } while (start_pos != std::string::npos);
  return s;
}

std::string ToStr(const bool &v)
{
  if (v)
    return "true";
  else
    return "false";
}

std::string ToStr(const std::chrono::hours &h)
{
  std::stringstream ss;
  ss << h.count() << 'h';
  return ss.str();
}

std::string ToStr(const std::chrono::minutes &m)
{
  std::stringstream ss;
  ss << m.count() << "min";
  return ss.str();
}

std::string ToStr(const std::chrono::seconds &s)
{
  std::stringstream ss;
  ss << s.count() << 's';
  return ss.str();
}

std::string ToStr(const std::chrono::milliseconds &ms)
{
  std::stringstream ss;
  ss << ms.count() << "ms";
  return ss.str();
}

std::string ToStr(const std::chrono::microseconds &us)
{
  std::stringstream ss;
  ss << us.count() << "us";
  return ss.str();
}

std::string ToStr(const std::chrono::nanoseconds &ns)
{
  std::stringstream ss;
  ss << ns.count() << "ns";
  return ss.str();
}

std::string SecondsToStr(int seconds)
{
  std::stringstream str;

  const int days = seconds/86400;
  bool needs_space = false;
  if (days > 0)
  {
    seconds -= 86400*days;
    str << days << " day";
    if (days > 1)
      str << "s";
    needs_space = true;
  }

  const int hours = seconds/3600;
  if (hours > 0)
  {
    if (needs_space)
      str << " ";
    str << hours << " hour";
    if (hours > 1)
      str << "s";
    seconds -= 3600*hours;
    needs_space = true;
  }

  const int mins = seconds/60;
  if (mins > 0)
  {
    if (needs_space)
      str << " ";
    str << mins << " minute";
    if (mins > 1)
      str << "s";
  }
  else
  {
    // Only show seconds if we had no larger unit before
    if (hours == 0 && days == 0)
    {
      str << seconds << " second";
      if (seconds > 1)
        str << "s";
    }
  }
  return str.str();
}


/** @brief Sets protocol to URL's protocol (or empty string). */
bool GetUrlProtocol(const std::string &url, std::string &protocol, std::string &remainder)
{
  const size_t protocol_pos = url.find("://");
  if (protocol_pos == std::string::npos)
  {
    protocol = "";
    remainder = url;
  }
  else
  {
    protocol = url.substr(0, protocol_pos+3);
    remainder = url.substr(protocol_pos+3);
  }
  return protocol_pos != std::string::npos;
}

//TODO add SplitUrl(returning scheme, userinfo, host, port, path, query, fragment

std::string ObscureUrlAuthentication(const std::string &url)
{
  std::string protocol, clipped;
  const bool has_protocol = GetUrlProtocol(url, protocol, clipped);

  const size_t at_pos = clipped.find('@');
  if (at_pos == std::string::npos)
    return url;

  const std::string obscured = "<auth>" + clipped.substr(at_pos);
  if (has_protocol)
    return protocol + obscured;
  return obscured;
}


std::string ClipUrl(const std::string &url)
{
  std::string protocol, clipped;
  const bool has_protocol = GetUrlProtocol(url, protocol, clipped);

  const size_t path_del_pos = clipped.find('/');
  if (path_del_pos != std::string::npos)
    clipped = clipped.substr(0, path_del_pos);

  if (has_protocol)
    return protocol + ObscureUrlAuthentication(clipped);
  return ObscureUrlAuthentication(clipped);
}


std::string Remove(const std::string &s, const char c)
{
  std::string removed;
  std::remove_copy(s.begin(), s.end(), std::back_inserter(removed), c);
  return removed;
}

std::string Remove(const std::string &s, std::initializer_list<char> chars)
{
  std::string copy(s);
  for (const auto c : chars)
    copy = Remove(copy, c);
  return copy;
}


std::string Canonic(const std::string &s, bool strip_dashes)
{
  std::string canonic =
      Replace(
        Replace(
          Replace(
            Remove(
              Lower(Trim(s)),
              {'!', ':', '\'', '"', '%', '/', '\\'}),
            "#", "nr"),
          " ", "-"),
        "_", "-");
  if (strip_dashes)
    return Remove(canonic, '-');
  return canonic;
}
} // namespace string
} // namespace utils
} // namespace vcp
