
#include "config.h"
#include <vcp_utils/file_utils.h>
#include <vcp_utils/string_utils.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

namespace vcp {
namespace python {
namespace config {

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::python::config"

void ConfigWrapper::EnsureAbsolutePaths(const std::vector<std::string> &param_names, const std::string &absolute_base_path, bool use_exact_keys, bool verbose)
{
  if (use_exact_keys)
  {
    for (const auto &k : param_names)
    {
      if (params_->SettingExists(k) && !vcp::utils::file::IsAbsolute(params_->GetString(k)))
      {
        params_->SetString(k, vcp::utils::file::FullFile(absolute_base_path, params_->GetString(k)));
        if (verbose)
          VCP_LOG_INFO_DEFAULT("Updated config path '" << k << "' to '" << params_->GetString(k) << "'");
      }
    }
  }
  else
  {
    // Grab a list of all known/configured parameters.
    const std::vector<std::string> configured_params = params_->ListConfigParameters();

    // Convert these parameter names to lower case.
    std::vector<std::string> configured_params_lower;
    configured_params_lower.reserve(configured_params.size());
    for (const auto &c : configured_params)
    {
      std::string lower(c);
      vcp::utils::string::ToLower(lower);
      configured_params_lower.push_back(lower);
    }

    // Iterate all given parameter names, convert to
    // lower case, and replace relative by absolute path
    // if the given name partially is contained in
    // the full parameter name.
    for (const auto &pn : param_names)
    {
      std::string lower(pn);
      vcp::utils::string::ToLower(lower);

      for (size_t i = 0; i < configured_params_lower.size(); ++i)
      {
        if (configured_params_lower[i].find(lower) == std::string::npos)
          continue;

        const std::string k = configured_params[i];
        if (vcp::utils::file::IsAbsolute(params_->GetString(k)))
          continue;

        params_->SetString(k, vcp::utils::file::FullFile(absolute_base_path, params_->GetString(k)));
        if (verbose)
          VCP_LOG_INFO("Updated config path '" << k << "' to '" << params_->GetString(k) << "'");
      }
    }
  }
}
} // namespace config
} // namespace python
} // namespace vcp


PYBIND11_MODULE(config, m)
{
  namespace vpc = vcp::python::config;
  m.doc() = "Configuration module to load/process libconfig++-ish configuration\n"
            "files, heavily used by the vcp.best video streaming module.";


  // Class definition
  py::class_<vpc::ConfigWrapper>(m, "Config")
      .def(py::init<>())
      .def("load_libconfig", &vpc::ConfigWrapper::LoadLibconfigFile,
           "Load from libconfig++ file.",
           py::arg("filename"))
      .def("load_jsons", &vpc::ConfigWrapper::LoadJsonConfigString,
           "Load from a JSON string.",
           py::arg("json_str"))
      .def("load_jsonf", &vpc::ConfigWrapper::LoadJsonConfigFile,
           "Load from a JSON file.",
           py::arg("json_str"))
      .def("dumps", &vpc::ConfigWrapper::ToString,
           "Formats the configuration (libconfig++ style) as\n"
           "a nicely indented string.")
      .def("save", &vpc::ConfigWrapper::Save,
           "Saves the configuration as libconfig++ file.",
           py::arg("filename"))
      .def("contains", &vpc::ConfigWrapper::HasKey,
           "Returns True if param_name is in the settings dictionary.",
           py::arg("param_name"))
      .def("__getitem__", &vpc::ConfigWrapper::AsString,
           "Return the string representation of param_name.", py::arg("param_name"))
// List params
      .def("list_parameters", &vpc::ConfigWrapper::ListAllParameters,
           "Return a list of all configured parameters.")
      .def("list_1st_level_children", &vpc::ConfigWrapper::ListFirstLevelChildren,
           "Return a list of all first-level children of\n"
           "the given parameter. Pass an empty parameter name\n"
           "to retrieve the first level, i.e. the root node's\n"
           "children.\n"
           ":returns: a list of parameter names (strings).",
           py::arg("param_name"))
// Get/set scalar
      .def("get_string", &vpc::ConfigWrapper::GetString, "Returns param_name as string.", py::arg("param_name"))
      .def("set_string", &vpc::ConfigWrapper::SetString, "Sets a string parameter.", py::arg("param_name"), py::arg("value"))
      .def("get_int", &vpc::ConfigWrapper::GetInteger, "Returns an integer parameter.", py::arg("param_name"))
      .def("set_int", &vpc::ConfigWrapper::SetInteger, "Sets an integer parameter.", py::arg("param_name"), py::arg("value"))
      .def("get_unsigned_int", &vpc::ConfigWrapper::GetUnsignedInteger, "Returns an unsigned integer parameter.", py::arg("param_name"))
      .def("get_double", &vpc::ConfigWrapper::GetDouble, "Returns a parameter of type double (py: float).", py::arg("param_name"))
      .def("set_double", &vpc::ConfigWrapper::SetDouble, "Sets a parameter of type double (py: float).", py::arg("param_name"), py::arg("value"))
      .def("get_bool", &vpc::ConfigWrapper::GetBoolean, "Returns a boolean flag.", py::arg("param_name"))
      .def("set_bool", &vpc::ConfigWrapper::SetBoolean, "Sets a boolean parameter.", py::arg("param_name"), py::arg("value"))
// Get/set (not-nested) lists
      .def("get_list_int", &vpc::ConfigWrapper::GetIntegerArray, "Returns the list of integers named param_name.", py::arg("param_name"))
      .def("set_list_int", &vpc::ConfigWrapper::SetIntegerArray, "Sets the list of integers named param_name.", py::arg("param_name"), py::arg("lst"))
      .def("get_list_string", &vpc::ConfigWrapper::GetStringArray, "Returns the list of strings named param_name.", py::arg("param_name"))
      .def("set_list_string", &vpc::ConfigWrapper::SetStringArray, "Sets the list of strings named param_name.", py::arg("param_name"), py::arg("lst"))
      .def("get_list_double", &vpc::ConfigWrapper::GetDoubleArray, "Returns the list of doubles (py: float) named param_name.", py::arg("param_name"))
      .def("set_list_double", &vpc::ConfigWrapper::SetDoubleArray, "Sets the list of doubles (py: float) named param_name.", py::arg("param_name"), py::arg("lst"))
// Get nested lists
      .def("get_nested_list_string", &vpc::ConfigWrapper::GetNestedStringArray, "Returns the list of list of strings named param_name.", py::arg("param_name"))
      .def("get_nested_list_int", &vpc::ConfigWrapper::GetNestedIntegerArray, "Returns the list of list of strings named param_name.", py::arg("param_name"))
// Common config value manipulation helper:
      .def("ensure_abs_paths", &vpc::ConfigWrapper::EnsureAbsolutePaths,
           "Replace relative paths by \"<abs_base_path>/<relative_path>\".\n"
           "\n"
           ":params: list of strings, i.e. the parameter names which contain paths\n"
           "         to check. If use_exact_keys is True, a key must fit the \n"
           "         configuration name exactly, e.g. 'camera-17.subgroup.Some_Path'.\n"
           "         Otherwise, the key 'pat' would also match against this parameter name.\n"
           "\n"
           ":use_exact_keys: Flag to toggle case-sensitive and exact search vs.\n"
           "         ignore-case substring matching.\n"
           "\n"
           ":verbose: if True, updated settings will be logged to console.",
           py::arg("params"), py::arg("abs_base_path"),
           py::arg("use_exact_keys")=true,
           py::arg("verbose")=true);
}
