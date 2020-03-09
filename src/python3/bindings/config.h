#ifndef __VCP_PYTHON_CONFIG_H__
#define __VCP_PYTHON_CONFIG_H__

#include <string>
#include <memory>
#include <vcp_config/config_params.h>
#include <vcp_utils/vcp_error.h>

namespace vcp
{
namespace python
{
namespace config
{

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::python::config"

// Binding an abstract base class didn't work easily (passing around between python and cpp).
// If you're interested in fixing this, start by reading: https://stackoverflow.com/a/20056409
class ConfigWrapper
{
public:
  // TODO Implement addtional getter/setter if somehow needed in python (otherwise, just use cfg['param_name'] to get a string representation and work with that...)
  ConfigWrapper()
  {
    params_ = vcp::config::CreateEmptyConfigParamsCpp();
  }

  // Disable copying
  ConfigWrapper(const ConfigWrapper&) = delete;
  ConfigWrapper &operator=(const ConfigWrapper&) = delete;


  virtual ~ConfigWrapper() {}


  /** @brief Load libconfig++ file. */
  void LoadLibconfigFile(const std::string &filename)
  {
    params_ = vcp::config::LoadConfigParamsCpp(filename);
  }


  /** @brief Load JSON string. */
  void LoadJsonConfigString(const std::string &json_str)
  {
    params_ = vcp::config::LoadConfigParamsJson(json_str, true);
  }


  /** @brief Load JSON file. */
  void LoadJsonConfigFile(const std::string &json_file)
  {
    params_ = vcp::config::LoadConfigParamsJson(json_file, false);
  }

  /** @brief Saves the configuration to disk. */
  bool Save(const std::string &filename)
  {
    return params_->SaveConfiguration(filename);
  }

  /**
   * @brief Returns a const reference to this configuration to be used to set up 'stuff' inside the library.
   * The python module will load/populate a configuration - this abstract method is to be used by other python wrappers to access the configuration.
   */
  const vcp::config::ConfigParams &AsConfigParams() const
  {
    if (!params_)
      VCP_ERROR("Configuration must be set/loaded before using it!");

    return *params_;
  }

  /** @brief String representation for this configuration. */
  std::string ToString() const
  {
    return params_->ToString();
  }

  /** @brief Checks if the given setting exists. */
  bool HasKey(const std::string &param_name) const
  {
    return params_->SettingExists(param_name);
  }

  /** @brief Return the string representation of the given setting. */
  std::string AsString (const std::string &param_name)
  {
    return params_->AsString(param_name);
  }

  /** @brief Return a list of all configured parameters. */
  std::vector<std::string> ListAllParameters() const
  {
    return params_->ListConfigParameters();
  }

  /** @brief Return a list of all first-level children of the given parameter. Pass empty parameter name to retrieve the first level, i.e. the root node's children. */
  std::vector<std::string> ListFirstLevelChildren(const std::string &param_name) const
  {
    return params_->ListConfigGroupParameters(param_name);
  }


  void EnsureAbsolutePaths(const std::vector<std::string> &param_names, const std::string &absolute_base_path, bool verbose)
  {
    params_->EnsureAbsolutePaths(param_names, absolute_base_path, verbose);
  }

  //---------------------------------------------
  // Getter/Setter: Scalars

  /** @brief Returns the string parameter (throws exception upon type mismatch). */
  std::string GetString(const std::string &param_name) const { return params_->GetString(param_name); }
  /** @brief Sets the string parameter. */
  void SetString(const std::string &param_name, const std::string &value) { params_->SetString(param_name, value); }

  int GetInteger(const std::string &param_name) const { return params_->GetInteger(param_name); }
  void SetInteger(const std::string &param_name, int value) { params_->SetInteger(param_name, value); }

  unsigned int GetUnsignedInteger(const std::string &param_name) const { return params_->GetUnsignedInteger(param_name); }

  double GetDouble(const std::string &param_name) const { return params_->GetDouble(param_name); }
  void SetDouble(const std::string &param_name, double value) { params_->SetDouble(param_name, value); }

  bool GetBoolean(const std::string &param_name) const { return params_->GetBoolean(param_name); }
  void SetBoolean(const std::string &param_name, bool value) { params_->SetBoolean(param_name, value); }


  //---------------------------------------------
  // Getter/Setter: Lists/arrays (single, not nested)
  std::vector<int> GetIntegerArray(const std::string &param_name) const { return params_->GetIntegerArray(param_name); }
  void SetIntegerArray(const std::string &param_name, const std::vector<int> &values) { params_->SetIntegerArray(param_name, values); }

  std::vector<double> GetDoubleArray(const std::string &param_name) const { return params_->GetDoubleArray(param_name); }
  void SetDoubleArray(const std::string &param_name, const std::vector<double> &values) { params_->SetDoubleArray(param_name, values); }

  std::vector<std::string> GetStringArray(const std::string &param_name) const { return params_->GetStringArray(param_name); }
  void SetStringArray(const std::string &param_name, const std::vector<std::string> &values) { params_->SetStringArray(param_name, values); }


  //---------------------------------------------
  // Getter/Setter: Nested Lists, dictionary-like, etc.
  //
  // Currently, only getter are supported - for pythonic configurations,
  // use json, dictionaries, or what-not. vcp::config should be used to
  // configure the cvp::best capturing module.
  std::vector<std::vector<std::string>> GetNestedStringArray(const std::string &param_name) const { return params_->GetNestedStringList(param_name); }

  std::vector<std::vector<int>> GetNestedIntegerArray(const std::string &param_name) const { return params_->GetNestedIntegerList(param_name); }


private:
  std::unique_ptr<vcp::config::ConfigParams> params_;
};

} // namespace config
} // namespace python
} // namespace vcp

#endif // __VCP_PYTHON_CONFIG_H__
