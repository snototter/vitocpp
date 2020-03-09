
#include "config.h"
#include <vcp_utils/file_utils.h>
#include <vcp_utils/string_utils.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;


//-----------------------------------------------------------------------------
// Python module declarations

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
           "         to check. For example, ['file_name'] would make the global\n"
           "         parameter 'file_name' as well as the group parameter 'video1.file_name'\n"
           "         absolute, BUT NOT 'some_file_name'.\n\n"
           ":verbose: if True, updated settings will be logged to console.",
           py::arg("params"), py::arg("abs_base_path"),
           py::arg("verbose")=true);
}
