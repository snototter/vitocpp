#include "cv_core_conversion.h"

namespace vcp
{
namespace python
{
namespace conversion
{

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::python::conversion"

cv::RotatedRect PyObjectToRotatedRect(const py::object &object, bool *is_valid)
{
  if (object.is_none())
  {
    if (is_valid)
      *is_valid = false;
    return cv::RotatedRect();
  }

  if (is_valid)
    *is_valid = true;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  float cx = 0.0;
  float cy = 0.0;
  float w = 0.0;
  float h = 0.0;
  float theta = 0.0;
  if (type.compare("list") == 0)
  {
    py::list list = py::cast<py::list>(object);
    if (py::len(list) < 5)
        VCP_ERROR("A rotated rect must be a 'list' of 5 elements, [cx, cy, w, h, theta]!");

    cx = ExtractNumber<float>(list[0]);
    cy = ExtractNumber<float>(list[1]);
    w = ExtractNumber<float>(list[2]);
    h = ExtractNumber<float>(list[3]);
    theta = ExtractNumber<float>(list[4]);
  }
  else if (type.compare("tuple") == 0)
  {
    py::tuple tuple = py::cast<py::tuple>(object);
    if (py::len(tuple) < 5)
        VCP_ERROR("A rotated rectangle must be a 'tuple' of 5 elements, (cx, cy, w, h, theta)!");

    cx = ExtractNumber<float>(tuple[0]);
    cy = ExtractNumber<float>(tuple[1]);
    w = ExtractNumber<float>(tuple[2]);
    h = ExtractNumber<float>(tuple[3]);
    theta = ExtractNumber<float>(tuple[4]);
  }
  else if (type.compare("ndarray") == 0)
  {
    const py::array_t<float> nd = py::cast<py::array_t<float>>(object);
    auto buffer = nd.request();
    if (buffer.size < 5)
      VCP_ERROR("A rotated rectangle must be a 'buffer' of 5 elements, np.array([cx, cy, w, h, theta])!");

    float *ptr = static_cast<float*>(buffer.ptr);
    cx = ptr[0];
    cy = ptr[1];
    w = ptr[2];
    h = ptr[3];
    theta = ptr[4];
  }
  else
  {
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to cv::RotatedRect");
    if (is_valid)
      *is_valid = false;
    return cv::RotatedRect();
  }
  return cv::RotatedRect(cv::Point2f(cx, cy), cv::Size2f(w, h), theta);
}

cv::Scalar PyObjectToScalar(const py::object &object, bool *is_valid)
{
  if (object.is_none())
  {
    if (is_valid)
      *is_valid = false;
    return cv::Scalar();
  }

  if (is_valid)
    *is_valid = true;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  cv::Scalar value;
  if (type.compare("list") == 0)
  {
    py::list list = py::cast<py::list>(object);
    for (size_t i = 0; i < std::min(static_cast<size_t>(4), py::len(list)); ++i)
      value.val[i] = ExtractNumber<double>(list[i]);
  }
  else if (type.compare("tuple") == 0)
  {
    py::tuple tuple = py::cast<py::tuple>(object);
    for (size_t i = 0; i < std::min(static_cast<size_t>(4), py::len(tuple)); ++i)
      value.val[i] = ExtractNumber<double>(tuple[i]);
  }
  else if (type.compare("ndarray") == 0)
  {
    const py::array_t<double> nd = py::cast<py::array_t<double>>(object);
    auto buffer = nd.request();

    double *ptr = static_cast<double*>(buffer.ptr);
    for (ssize_t i = 0; i < std::min(static_cast<ssize_t>(4), buffer.size); ++i)
      value.val[i] = ptr[i];
  }
  else
  {
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to cv::Scalar");
    if (is_valid)
      *is_valid = false;
    return cv::Scalar();
  }
  return value;
}
} // namespace conversion
} // namespace python
} // namespace vcp
