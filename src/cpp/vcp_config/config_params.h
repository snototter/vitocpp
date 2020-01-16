#ifndef __VCP_CONFIG_CONFIGPARAMS_H__
#define __VCP_CONFIG_CONFIGPARAMS_H__

#include <string>
#include <vector>
#include <memory>

namespace vcp
{
namespace config
{
/**
 * @brief A wrapper to access configuration files.
 */
class ConfigParams
{
public:
  virtual ~ConfigParams() {}

  //TODO:
  // * add long long (int64) scalar http://en.cppreference.com/w/cpp/language/types
  // * double precision rectangle
  // * double precision size2d
  // * double precision polygons
  // * load matrices (homography, fundamental, essential, K,R,...) //double* readHomographyParameter(std::string param_name) const;


  /** @brief Returns the scalar integer setting with the given param_name. */
  virtual int GetInteger(const std::string &param_name) const = 0;

  /** @brief Returns the scalar uint setting with the given param_name. */
  virtual unsigned int GetUnsignedInteger(const std::string &param_name) const = 0;

  /** @brief Returns the scalar double setting with the given param_name. */
  virtual double GetDouble(const std::string &param_name) const = 0;

  /** @brief Returns the boolean flag with the given param_name. */
  virtual bool GetBoolean(const std::string &param_name) const = 0;

  /** @brief Returns the string setting with the given param_name. */
  virtual std::string GetString(const std::string &param_name) const = 0;



  /** @brief Returns the integer array with the given param_name. */
  virtual std::vector<int> GetIntegerArray(const std::string &param_name) const = 0;

  /** @brief Returns the double array with the given param_name. */
  virtual std::vector<double> GetDoubleArray(const std::string &param_name) const = 0;

  /** @brief Returns the string array with the given param_name. */
  virtual std::vector<std::string> GetStringArray(const std::string &param_name) const = 0;



  /** @brief Returns a 2-element integer vector. */
  virtual std::vector<int> GetSize2D(const std::string &param_name) const = 0;

  /** @brief Returns a 4-element integer vector. */
  virtual std::vector<int> GetRectangle(const std::string &param_name) const = 0;



  /** @brief Returns a polygon which is a list of points, where each point is a d-dimensional vector.
   *
   * libconfig++ example: ([p1x,p1y], [p2x,p2y], ...) would be a polygon of 2D points.
   */
  virtual std::vector<std::vector<int>> GetIntegerPolygon(const std::string &param_name) const = 0;

  /** @brief Returns a vector of polygons. Each polygon is a list of points (where each point is a d-dimensional vector).
   *
   * libconfig++ example: (([p1x1,p1y1],[p1x2,p1y2],...), ([p2x,p2y],...), ...)
   */
  virtual std::vector<std::vector<std::vector<int>>> GetIntegerPolygons(const std::string &param_name) const = 0;


  /** @brief Allow to build something like a dictionary (required to configure some cameras more easily).
   *
   * libconfig++ example (list of lists): (("key1", 0.1), ("key2", 17.0))
   */
  virtual std::vector<std::pair<std::string, double>> GetDoubleKeyValueList(const std::string &param_name) const = 0;


  /** @brief Allow to build something like a dictionary (required to configure some cameras more easily).
   *
   * libconfig++ example (list of lists): (("key1", 1), ("key2", -30))
   */
  virtual std::vector<std::pair<std::string, int>> GetIntegerKeyValueList(const std::string &param_name) const = 0;


  /** @brief Useful to group detection labels, e.g. we want to get detections for 3 "clusters" [['person', 'athlete', 'cyclist'], ['car', 'truck'], ['dog', 'cat'...]]
   */
  virtual std::vector<std::vector<std::string>> GetNestedStringList(const std::string &param_name) const = 0;


  /** @brief Retrieve a list of integer arrays, e.g. [[1,2],[3,4,5],[42]] Similar to @see GetNestedStringList, intended to use integer IDs instead of names. */
  virtual std::vector<std::vector<int>> GetNestedIntegerList(const std::string &param_name) const = 0;


  /** @brief Returns true, if the setting exists. */
  virtual bool SettingExists(const std::string &param_name) const = 0;


  /** @brief Returns the length of a setting (useful to iterate over vectors/arrays). */
  virtual int SettingLength(const std::string &param_name) const = 0;


  /** @brief Saves the current configuration to disk. */
  virtual bool SaveConfiguration(const std::string &filename) = 0;


  /** @brief Replaces or creates the integer setting with the given param_name. */
  virtual void SetInteger(const std::string &param_name, int value) = 0;


  /** @brief Replaces or creates the double setting with the given param_name. */
  virtual void SetDouble(const std::string &param_name, double value) = 0;


  /** @brief Replaces or creates the boolean flag with the given param_name. */
  virtual void SetBoolean(const std::string &param_name, bool value) = 0;


  /** @brief Replaces or creates the string setting with the given param_name. */
  virtual void SetString(const std::string &param_name, const std::string &value) = 0;


  /** @brief Replaces or creates an array. */
  virtual void SetIntegerArray(const std::string &param_name, const std::vector<int> &values) = 0;


  /** @brief Replaces or creates an array. */
  virtual void SetDoubleArray(const std::string &param_name, const std::vector<double> &values) = 0;


  /** @brief Replaces or creates an array. */
  virtual void SetStringArray(const std::string &param_name, const std::vector<std::string> &values) = 0;


  /** @brief Replaces or creates a size parameter (i.e. 2-element array). */
  virtual void SetSize2D(const std::string &param_name, int width, int height) = 0;


  /** @brief Replaces or creates a rectangle parameter (i.e. 4-element array). */
  virtual void SetRectangle(const std::string &param_name, int left, int top, int width, int height) = 0;

  //TODO doc
  virtual void SetIntegerPolygon(const std::string &param_name, const std::vector<std::vector<int>> &values) = 0;
  virtual void SetIntegerPolygons(const std::string &param_name, const std::vector<std::vector<std::vector<int>>> &values) = 0;

  /** @brief Allows to build "something like" a python dictionary (required to configure some cameras more easily).
   *
   * libconfig++ example (list of lists): (("key1", 1), ("key2", -30))
   */
  virtual void SetIntegerKeyValueList(const std::string &param_name, const std::vector<std::pair<std::string, int>> &values) = 0;

  /** @brief Allows to build "something like" a python dictionary (which is required to configure some sensors more easily in vcp_best).
   *
   * libconfig++ example: (list of lists): (("key1", 1.0), ("key2", -42.123))
   */
  virtual void SetDoubleKeyValueList(const std::string &param_name, const std::vector<std::pair<std::string, double>> &values) = 0;

  /** @brief Returns a string representation of this configuration object (basically the same as would be saved to disk). */
  virtual std::string ToString() const = 0;

  /** @brief Returns a string representation of the corresponding parameter. */
  virtual std::string AsString(const std::string &param_name) const = 0;

  /** @brief Return a list of all parameters under the given parameter name.
   *
   * Can be used to verify a config file, e.g. check if the user provided additional
   * keys (which might indicate a potential typo).
   *
   * Pass empty param_name to query all children of the root node.
   */
  virtual std::vector<std::string> ListConfigGroupParameters(const std::string &param_name) const = 0;

protected:
  ConfigParams() {}
};


/** @brief Loads a libconfig++ configuration file. */
std::unique_ptr<ConfigParams> LoadConfigParamsCpp(const std::string &config_filename);

/** @brief Creates an empty libconfig++ configuration, which you can programatically add settings to. */
std::unique_ptr<ConfigParams> CreateEmptyConfigParamsCpp();

/** @brief Loads the configuration from a JSON file or string, depending on the specified flag. */
std::unique_ptr<ConfigParams> LoadConfigParamsJson(const std::string &input_json, bool is_json_string);

} // namespace config
} // namespace vcp
#endif // __VCP_CONFIG_CONFIGPARAMS_H__
