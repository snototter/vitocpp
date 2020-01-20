#ifndef __VCP_CV_CORE_CONVERSION_H__
#define __VCP_CV_CORE_CONVERSION_H__

#include <pybind11/pybind11.h>
#include <pybind11/cast.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
namespace py = pybind11;

#include <opencv2/core/core.hpp>
#include <vcp_imutils/opencv_compatibility.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/vcp_error.h>

namespace vcp
{
namespace python
{
namespace conversion
{
/** @brief We want to implicitly round/cast real numbers to integers, if needed. pybind throws an exception. */
template <typename _Tp>
_Tp ExtractNumber(const py::object &o)
{
  if (py::isinstance<py::float_>(o))
  {
    const double v = o.cast<double>();
    return static_cast<_Tp>(v);
  }
  else if (py::isinstance<py::int_>(o))
  {
    const int v = o.cast<int>();
    return static_cast<_Tp>(v);
  }

  // Fall back to pybind casting, which may rise an exception (with a meaningful TypeError)
  return o.cast<_Tp>();
}

template <typename _Tp>
cv::Rect_<_Tp> PyObjectToRect(const py::object &object, bool *is_valid)
{
  if (object.is_none())
  {
    if (is_valid)
      *is_valid = false;
    return cv::Rect_<_Tp>();
  }

  if (is_valid)
    *is_valid = true;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  _Tp x = static_cast<_Tp>(0);
  _Tp y = static_cast<_Tp>(0);
  _Tp w = static_cast<_Tp>(0);
  _Tp h = static_cast<_Tp>(0);
  if (type.compare("list") == 0)
  {
    py::list list = py::cast<py::list>(object);
    if (py::len(list) < 4)
        VCP_ERROR("A rectangle must be a 'list' of 4 elements, [l, t, w, h]!");

    x = ExtractNumber<_Tp>(list[0]);
    y = ExtractNumber<_Tp>(list[1]);
    w = ExtractNumber<_Tp>(list[2]);
    h = ExtractNumber<_Tp>(list[3]);
  }
  else if (type.compare("tuple") == 0)
  {
    py::tuple tuple = py::cast<py::tuple>(object);
    if (py::len(tuple) < 4)
        VCP_ERROR("A rectangle must be a 'tuple' of 4 elements, (l, t, w, h)!");

    x = ExtractNumber<_Tp>(tuple[0]);
    y = ExtractNumber<_Tp>(tuple[1]);
    w = ExtractNumber<_Tp>(tuple[2]);
    h = ExtractNumber<_Tp>(tuple[3]);
  }
  else if (type.compare("ndarray") == 0)
  {
    const py::array_t<_Tp> nd = py::cast<py::array_t<_Tp>>(object);
    auto buffer = nd.request();
    if (buffer.size < 4)
      VCP_ERROR("A rectangle must be a 'buffer' of 4 elements, np.array([l, t, w, h])!");

    _Tp *ptr = static_cast<_Tp*>(buffer.ptr);
    x = ptr[0];
    y = ptr[1];
    w = ptr[2];
    h = ptr[3];
  }
  else
  {
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to cv::Rect_<T>");
    if (is_valid)
      *is_valid = false;
    return cv::Rect_<_Tp>();
  }
  return cv::Rect_<_Tp>(x, y, w, h);
}


cv::RotatedRect PyObjectToRotatedRect(const py::object &object, bool *is_valid);


template<typename _Tp, int _Cn>
cv::Vec<_Tp, _Cn> PyObjectToVec(const py::object &object, bool *is_valid)
{
  cv::Vec<_Tp, _Cn> value;

  if (object.is_none())
  {
    if (is_valid)
      *is_valid = false;
    return value;
  }

  if (is_valid)
    *is_valid = true;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  if (type.compare("list") == 0)
  {
    py::list list = py::cast<py::list>(object);
    if (py::len(list) < _Cn)
      VCP_ERROR("List contains too few elements to convert to cv::Vec<T,C>!");

    for (int i = 0; i < _Cn; ++i)
      value[i] = ExtractNumber<_Tp>(list[i]);
  }
  else if (type.compare("tuple") == 0)
  {
    py::tuple tuple = py::cast<py::tuple>(object);
    if (py::len(tuple) < _Cn)
      VCP_ERROR("Tuple contains too few elements to convert to cv::Vec<T,C>!");

    for (int i = 0; i < _Cn; ++i)
      value[i] = ExtractNumber<_Tp>(tuple[i]);
  }
  else if (type.compare("ndarray") == 0)
  {
    const py::array_t<_Tp> nd = py::cast<py::array_t<_Tp>>(object);
    auto buffer = nd.request();
    if (buffer.size < _Cn)
      VCP_ERROR("Buffer (np.array) contains too few elements to convert to cv::Vec<T,C>!");

    _Tp *ptr = static_cast<_Tp*>(buffer.ptr);
    for (int i = 0; i < _Cn; ++i)
      value[i] = ptr[i];
  }
  else
  {
    if (is_valid)
      *is_valid = false;
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to cv::Vec<T,C>!");
  }
  return value;
}


cv::Scalar PyObjectToScalar(const py::object &object, bool *is_valid);


template <typename _Tp>
cv::Point_<_Tp> PyObjectToPoint(const py::object &object, bool *is_valid)
{
  if (object.is_none())
  {
    if (is_valid)
      *is_valid = false;
    return cv::Point_<_Tp>();
  }

  if (is_valid)
    *is_valid = true;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  _Tp x = static_cast<_Tp>(0);
  _Tp y = static_cast<_Tp>(0);
  if (type.compare("list") == 0)
  {
    py::list list = py::cast<py::list>(object);
    if (py::len(list) != 2)
        VCP_ERROR("A 2d point must be a 'list' of 2 elements, [x, y]!");

    x = ExtractNumber<_Tp>(list[0]);
    y = ExtractNumber<_Tp>(list[1]);
  }
  else if (type.compare("tuple") == 0)
  {
    py::tuple tuple = py::cast<py::tuple>(object);
    if (py::len(tuple) != 2)
        VCP_ERROR("A 2d point must be a 'tuple' of 2 elements, (x, y)!");

    x = ExtractNumber<_Tp>(tuple[0]);
    y = ExtractNumber<_Tp>(tuple[1]);
  }
  else if (type.compare("ndarray") == 0)
  {
    const py::array_t<_Tp> nd = py::cast<py::array_t<_Tp>>(object);
    auto buffer = nd.request();
    if (buffer.size != 2)
      VCP_ERROR("A 2d point must be a 'buffer' of 2 elements, np.array([x, y])!");

    _Tp *ptr = static_cast<_Tp*>(buffer.ptr);
    x = ptr[0];
    y = ptr[1];
  }
  else
  {
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to cv::Point_<T>");
    if (is_valid)
      *is_valid = false;
    return cv::Point_<_Tp>();
  }
  return cv::Point_<_Tp>(x, y);
}


template <typename _Tp>
cv::Size_<_Tp> PyObjectToSize(const py::object &object, bool *is_valid)
{
  if (object.is_none())
  {
    if (is_valid)
      *is_valid = false;
    return cv::Size_<_Tp>();
  }

  if (is_valid)
    *is_valid = true;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  _Tp w = static_cast<_Tp>(0);
  _Tp h = static_cast<_Tp>(0);
  if (type.compare("list") == 0)
  {
    py::list list = py::cast<py::list>(object);
    if (py::len(list) != 2)
        VCP_ERROR("A 2d size must be a 'list' of 2 elements, [w, h]!");

    w = ExtractNumber<_Tp>(list[0]);
    h = ExtractNumber<_Tp>(list[1]);
  }
  else if (type.compare("tuple") == 0)
  {
    py::tuple tuple = py::cast<py::tuple>(object);
    if (py::len(tuple) != 2)
        VCP_ERROR("A 2d size must be a 'tuple' of 2 elements, (w, h)!");

    w = ExtractNumber<_Tp>(tuple[0]);
    h = ExtractNumber<_Tp>(tuple[1]);
  }
  else if (type.compare("ndarray") == 0)
  {
    const py::array_t<_Tp> nd = py::cast<py::array_t<_Tp>>(object);
    auto buffer = nd.request();
    if (buffer.size != 2)
      VCP_ERROR("A 2d size must be a 'buffer' of 2 elements, np.array([w, h])!");

    _Tp *ptr = static_cast<_Tp*>(buffer.ptr);
    w = ptr[0];
    h = ptr[1];
  }
  else
  {
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to cv::Size_<T>");
    if (is_valid)
      *is_valid = false;
    return cv::Size_<_Tp>();
  }
  return cv::Size_<_Tp>(w, h);
}

} // namespace conversion
} // namespace python
} // namespace vcp

// We need to partially overload pybind11::detail::type_caster<T>, see https://pybind11.readthedocs.io/en/stable/advanced/cast/custom.html
namespace pybind11
{
namespace detail
{
template <typename _Tp> struct type_caster<cv::Rect_<_Tp>>
{
public:
  /**
   * This macro establishes the name 'cv::Rect' in function signatures and declares a local variable 'value' of type cvrect
   */
  PYBIND11_TYPE_CASTER(cv::Rect_<_Tp>, _("Rect_<T> (i.e. tuple, list or np.array: [l, t, w, h])"));

  /**
   * Conversion part 1 (Python->C++): convert a PyObject into a cv::Rect_<T>
   * instance or return false upon failure. The second argument
   * indicates whether implicit conversions should be applied.
   */
  bool load(handle src, bool)
  {
    bool valid;
    cv::Rect_<_Tp> r = vcp::python::conversion::PyObjectToRect<_Tp>(src.cast<py::object>(), &valid);
    if (valid)
      value = r;
    return valid && !PyErr_Occurred();
  }

  /**
   * Conversion part 2 (C++ -> Python): convert a cv::Rect_<T> instance into
   * a Python object. The second and third arguments are used to
   * indicate the return value policy and parent object (for
   * ``return_value_policy::reference_internal``) and are generally
   * ignored by implicit casters.
   */
  static handle cast(const cv::Rect_<_Tp> &src, return_value_policy /* policy */, handle /* parent */)
  {
    PyObject *t = PyTuple_New(4);
    PyTuple_SetItem(t, 0, std::is_integral<_Tp>::value ? PyLong_FromLong(src.x) : PyFloat_FromDouble(src.x));
    PyTuple_SetItem(t, 1, std::is_integral<_Tp>::value ? PyLong_FromLong(src.y) : PyFloat_FromDouble(src.y));
    PyTuple_SetItem(t, 2, std::is_integral<_Tp>::value ? PyLong_FromLong(src.width) : PyFloat_FromDouble(src.width));
    PyTuple_SetItem(t, 3, std::is_integral<_Tp>::value ? PyLong_FromLong(src.height) : PyFloat_FromDouble(src.height));
    return t;
  }
}; // type_caster for cv::Rect_<T>


template <> struct type_caster<cv::RotatedRect>
{
public:
  PYBIND11_TYPE_CASTER(cv::RotatedRect, _("RotatedRect (i.e. tuple, list or np.array: [cx, cy, w, h, theta])"));

  bool load(handle src, bool)
  {
    bool valid;
    cv::RotatedRect r = vcp::python::conversion::PyObjectToRotatedRect(src.cast<py::object>(), &valid);
    if (valid)
      value = r;
    return valid && !PyErr_Occurred();
  }

  static handle cast(const cv::RotatedRect &src, return_value_policy /* policy */, handle /* parent */)
  {
    PyObject *t = PyTuple_New(5);
    PyTuple_SetItem(t, 0, PyFloat_FromDouble(static_cast<double>(src.center.x)));
    PyTuple_SetItem(t, 1, PyFloat_FromDouble(static_cast<double>(src.center.y)));
    PyTuple_SetItem(t, 2, PyFloat_FromDouble(static_cast<double>(src.size.width)));
    PyTuple_SetItem(t, 3, PyFloat_FromDouble(static_cast<double>(src.size.height)));
    PyTuple_SetItem(t, 4, PyFloat_FromDouble(static_cast<double>(src.angle)));
    return t;
  }
}; // type_caster for cv::RotatedRect


template <typename _Tp, int _Cn> struct type_caster<cv::Vec<_Tp, _Cn> >
{
public:
  typedef cv::Vec<_Tp, _Cn> templated_vec;
  PYBIND11_TYPE_CASTER(templated_vec, _("Vec<T,C> (i.e. tuple, list or np.array: [x0, x1, ...])"));

  bool load(handle src, bool)
  {
    bool valid;
    templated_vec r = vcp::python::conversion::PyObjectToVec<_Tp, _Cn>(src.cast<py::object>(), &valid);
    if (valid)
      value = r;
    return valid && !PyErr_Occurred();
  }

  static handle cast(const templated_vec &src, return_value_policy /* policy */, handle /* parent */)
  {
    PyObject *t = PyTuple_New(_Cn);
    for (int i = 0; i < _Cn; ++i)
    {
      PyTuple_SetItem(t, i, std::is_integral<_Tp>::value ? PyLong_FromLong(static_cast<long>(src.val[i]))
                                                       : PyFloat_FromDouble(static_cast<double>(src.val[i])));
    }
    return t;
  }
}; // type_caster for cv::Vec<_Tp, _Cn>


template <> struct type_caster<cv::Scalar>
{
public:
  PYBIND11_TYPE_CASTER(cv::Scalar, _("Scalar (i.e. tuple, list or np.array: [x0], x1], x2], x3])"));

  bool load(handle src, bool)
  {
    bool valid;
    cv::Scalar r = vcp::python::conversion::PyObjectToScalar(src.cast<py::object>(), &valid);
    if (valid)
      value = r;
    return valid && !PyErr_Occurred();
  }

  static handle cast(const cv::Scalar &src, return_value_policy /* policy */, handle /* parent */)
  {
    PyObject *t = PyTuple_New(4);
    for (size_t i = 0; i < 4; ++i)
      PyTuple_SetItem(t, i, PyFloat_FromDouble(src.val[i]));
    return t;
  }
}; // type_caster for cv::Scalar


template <typename _Tp> struct type_caster<cv::Point_<_Tp>>
{
public:
  PYBIND11_TYPE_CASTER(cv::Point_<_Tp>, _("Point_<T> (i.e. tuple, list or np.array: [x, y])"));

  bool load(handle src, bool)
  {
    bool valid;
    cv::Point_<_Tp> r = vcp::python::conversion::PyObjectToPoint<_Tp>(src.cast<py::object>(), &valid);
    if (valid)
      value = r;
    return valid && !PyErr_Occurred();
  }

  static handle cast(const cv::Point_<_Tp> &src, return_value_policy /* policy */, handle /* parent */)
  {
    PyObject *t = PyTuple_New(2);
    PyTuple_SetItem(t, 0, std::is_integral<_Tp>::value ? PyLong_FromLong(src.x) : PyFloat_FromDouble(src.x));
    PyTuple_SetItem(t, 1, std::is_integral<_Tp>::value ? PyLong_FromLong(src.y) : PyFloat_FromDouble(src.y));
    return t;
  }
}; // type_caster for cv::Point_<T>


template <typename _Tp> struct type_caster<cv::Size_<_Tp>>
{
public:
  PYBIND11_TYPE_CASTER(cv::Size_<_Tp>, _("Size_<T> (i.e. tuple, list or np.array: [w, h])"));

  bool load(handle src, bool)
  {
    bool valid;
    cv::Size_<_Tp> r = vcp::python::conversion::PyObjectToSize<_Tp>(src.cast<py::object>(), &valid);
    if (valid)
      value = r;
    return valid && !PyErr_Occurred();
  }

  static handle cast(const cv::Size_<_Tp> &src, return_value_policy /* policy */, handle /* parent */)
  {
    PyObject *t = PyTuple_New(2);
    PyTuple_SetItem(t, 0, std::is_integral<_Tp>::value ? PyLong_FromLong(src.width) : PyFloat_FromDouble(src.width));
    PyTuple_SetItem(t, 1, std::is_integral<_Tp>::value ? PyLong_FromLong(src.height) : PyFloat_FromDouble(src.height));
    return t;
  }
}; // type_caster for cv::Size_<T>

} // namespace detail
} // namespace pybind11

#endif // __VCP_CV_CORE_CONVERSION_H__
