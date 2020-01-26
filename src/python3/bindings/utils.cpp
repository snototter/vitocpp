#include <vcp_utils/file_utils.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/vcp_logging.h>


#include <pybind11/pybind11.h>
namespace py = pybind11;


//-----------------------------------------------------------------------------
// Wrapper/Helper code

void LogDebug(const std::string &message)
{
  VCP_LOG_DEBUG_DEFAULT(message);
}

void LogInfo(const std::string &message)
{
  VCP_LOG_INFO_DEFAULT(message);
}

void LogWarning(const std::string &message)
{
  VCP_LOG_WARNING_DEFAULT(message);
}

void LogFailure(const std::string &message)
{
  VCP_LOG_FAILURE_DEFAULT(message);
}


//-----------------------------------------------------------------------------
// Python module declarations

PYBIND11_MODULE(utils, m)
{
  m.doc() = "vcp::utils - logging, string and file handling";

  m.def("log_debug", &LogDebug,
        "Log debug message.",
        py::arg("string"));

  m.def("log_info", &LogInfo,
        "Log status/information message.",
        py::arg("string"));

  m.def("log_warning", &LogWarning,
        "Log warning message.",
        py::arg("string"));

  m.def("log_failure", &LogFailure,
        "Log failure message.",
        py::arg("string"));

  namespace vuf = vcp::utils::file;
  m.def("is_image_file", &vuf::filename_filter::HasImageExtension,
        "Checks if the 'filename' has a known/supported image filename extension.",
        py::arg("filename"));

  m.def("compare_file_names_and_lengths", &vuf::filename_filter::CompareFileLengthsAndNames,
        "Comparator to sort files by filename length (and alphabetically if two have the same length).\n"
        "This helps to properly sort image directories with contents similar to:\n"
        "  frame0, frame1, frame2, frame10, frame30, frame100, etc.",
        py::arg("filename1"), py::arg("filename2"));

  m.def("create_path", &vuf::CreatePath,
        "Recursively create path, like `mkdir -p`, with default permissions 775.",
        py::arg("path"));
}
