#ifndef __VCP_PYTHON_CONVERSION_H__
#define __VCP_PYTHON_CONVERSION_H__

#include <pybind11/pybind11.h>
#include <pybind11/cast.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
namespace py = pybind11;

#include <opencv2/core/core.hpp>
#include <vcp_imutils/opencv_compatibility.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/vcp_error.h>

#include <vcp_imvis/drawing.h>
#include <vcp_math/geometry2d.h>
#include <vcp_math/geometry3d.h>

namespace vcp
{
namespace python
{
namespace conversion
{

template<typename _Tp, int _Cn>
py::tuple VecToTuple(const cv::Vec<_Tp,_Cn> &v)
{
  py::list l;
  for (int i = 0; i < _Cn; ++i)
    l.append(v.val[i]);
  return l.cast<py::tuple>();
}


/** @brief Type conversion of 3D box coordinates to Box3d. */
vcp::imvis::drawing::Box3d PyObjectToBox3d(const py::object &object);

/** @brief Type conversion of 2D start/end coordinates to Line2d. */
vcp::math::geo2d::Line2d PyObjectToLine2d(const py::object &object);

/** @brief Type conversion of 3D start/end coordinates to Line3d. */
vcp::math::geo3d::Line3d PyObjectToLine3d(const py::object &object);

/** @brief Extend a cv::Rect object with attributes for visualization. */
typedef struct
{
  cv::Rect box;
  cv::Scalar color;
  std::string caption;
  bool is_dashed;

  bool empty() const { return (box.width == 0) || (box.height == 0); }

  cv::Scalar Color(const cv::Scalar &default_color) const
  {
    if (color[0] < 0.0 || color[1] < 0.0 || color[2] < 0.0)
      return default_color;
    return color;
  }

  void Clear()
  {
    box = cv::Rect();
    color = cv::Scalar::all(-1.0);
    caption = std::string();
    is_dashed = false;
  }
} BoundingBox2d;

BoundingBox2d PyObjectToBoundingBox2d(const py::object &object);


/** @brief Extend a cv::RotatedRect object with attributes for visualization. */
typedef struct
{
  cv::RotatedRect box;
  cv::Scalar color;
  std::string caption;
  bool is_dashed;

  bool empty() const { return (box.size.width <= 0.0001f) || (box.size.height <= 0.0001f); } //TODO use eps_equal!

  cv::Scalar Color(const cv::Scalar &default_color) const
  {
    if (color[0] < 0.0 || color[1] < 0.0 || color[2] < 0.0)
      return default_color;
    return color;
  }

  void Clear()
  {
    box = cv::RotatedRect(cv::Point2f(0.0f, 0.0f), cv::Size2f(0.0f, 0.0f), 0.0f);
    color = cv::Scalar::all(-1.0);
    caption = std::string();
    is_dashed = false;
  }
} RotatedBoundingBox2d;

RotatedBoundingBox2d PyObjectToRotatedBoundingBox2d(const py::object &object);



/** @brief Extend a Box3d object with attributes for visualization. */
typedef struct
{
  vcp::imvis::drawing::Box3d box;
  cv::Scalar color;
  std::string caption;

  bool empty() const { return !box.Valid(); }

  cv::Scalar Color(const cv::Scalar &default_color) const
  {
    if (color[0] < 0.0 || color[1] < 0.0 || color[2] < 0.0)
      return default_color;
    return color;
  }

  void Clear()
  {
    box = vcp::imvis::drawing::Box3d();
    color = cv::Scalar::all(-1.0);
    caption = std::string();
  }
} BoundingBox3d;

BoundingBox3d PyObjectToBoundingBox3d(const py::object &object);



/** @brief Extend the Line2d object with attributes for visualization. */
typedef struct
{
  vcp::math::geo2d::Line2d line;
  cv::Scalar color;
  bool is_dashed;

  bool empty() const { return line.empty(); }

  cv::Scalar Color(const cv::Scalar &default_color) const
  {
    if (color[0] < 0.0 || color[1] < 0.0 || color[2] < 0.0)
      return default_color;
    return color;
  }

  void Clear()
  {
    line = vcp::math::geo2d::Line2d();
    color = cv::Scalar::all(-1.0);
    is_dashed = false;
  }

  cv::Point FromPt() const { return cv::Point(static_cast<int>(line.From().val[0]), static_cast<int>(line.From().val[1])); }
  cv::Point ToPt() const { return cv::Point(static_cast<int>(line.To().val[0]), static_cast<int>(line.To().val[1])); }
} VisLine2d;

VisLine2d PyObjectToVisLine2d(const py::object &object);

} // namespace conversion
} // namespace python
} // namespace vcp

// We need to partially overload pybind11::detail::type_caster<T>, see https://pybind11.readthedocs.io/en/stable/advanced/cast/custom.html
namespace pybind11
{
namespace detail
{
template <> struct type_caster<vcp::imvis::drawing::Box3d>
{
public:
  PYBIND11_TYPE_CASTER(vcp::imvis::drawing::Box3d, _("Box3d (3x8 np.array or list/tuple of eight 3d corners)"));

  bool load(handle src, bool)
  {
    value = vcp::python::conversion::PyObjectToBox3d(src.cast<py::object>());
    return value.Valid() && !PyErr_Occurred();
  }

  static handle cast(const vcp::imvis::drawing::Box3d &src, return_value_policy /* policy */, handle /* parent */)
  {
    PyObject *box = PyDict_New();
    PyObject *top = src.Valid() ? PyTuple_New(4) : NULL;
    PyObject *bottom = src.Valid() ? PyTuple_New(4) : NULL;
    for (size_t i = 0; i < src.TopCorners().size(); ++i)
    {
      PyObject *corner_top = PyTuple_New(3);
      PyObject *corner_bottom = PyTuple_New(3);
      for (size_t dim = 0; dim < 3; ++dim)
      {
        PyTuple_SetItem(corner_top, i, PyFloat_FromDouble(src.TopCorners()[i].val[dim]));
        PyTuple_SetItem(corner_bottom, i, PyFloat_FromDouble(src.BottomCorners()[i].val[dim]));
      }
      PyTuple_SetItem(top, i, corner_top);
      PyTuple_SetItem(bottom, i, corner_bottom);
    }
    PyDict_SetItemString(box, "top_corners", top);
    PyDict_SetItemString(box, "bottom_corners", bottom);
    return box;
  }
}; // type_caster for vcp's Box3d


template <> struct type_caster<vcp::math::geo2d::Line2d>
{
public:
  PYBIND11_TYPE_CASTER(vcp::math::geo2d::Line2d, _("Line2d (i.e. [(x0,y0), (x1,y1)])"));

  bool load(handle src, bool)
  {
    value = vcp::python::conversion::PyObjectToLine2d(src.cast<py::object>());
    return !value.empty() && !PyErr_Occurred();
  }

  static handle cast(const vcp::math::geo2d::Line2d &src, return_value_policy /* policy */, handle /* parent */)
  {
    if (src.empty())
    {
      Py_RETURN_NONE;
    }
    PyObject *line = PyList_New(2);
    PyObject *pt_from = PyTuple_New(2);
    PyObject *pt_to = PyTuple_New(2);
    for (size_t dim = 0; dim < 2; ++dim)
    {
      PyTuple_SetItem(pt_from, dim, PyFloat_FromDouble(src.From().val[dim]));
      PyTuple_SetItem(pt_to, dim, PyFloat_FromDouble(src.To().val[dim]));
    }
    PyList_SetItem(line, 0, pt_from);
    PyList_SetItem(line, 1, pt_to);
    return line;
  }
}; // type_caster for vcp's Line2d



template <> struct type_caster<vcp::math::geo3d::Line3d>
{
public:
  PYBIND11_TYPE_CASTER(vcp::math::geo3d::Line3d, _("Line3d (i.e. [(x0,y0,z0), (x1,y1,z1)])"));

  bool load(handle src, bool)
  {
    value = vcp::python::conversion::PyObjectToLine3d(src.cast<py::object>());
    return !value.empty() && !PyErr_Occurred();
  }

  static handle cast(const vcp::math::geo3d::Line3d &src, return_value_policy /* policy */, handle /* parent */)
  {
    if (src.empty())
    {
      Py_RETURN_NONE;
    }
    PyObject *line = PyList_New(2);
    PyObject *pt_from = PyTuple_New(3);
    PyObject *pt_to = PyTuple_New(3);
    for (size_t dim = 0; dim < 3; ++dim)
    {
      PyTuple_SetItem(pt_from, dim, PyFloat_FromDouble(src.From().val[dim]));
      PyTuple_SetItem(pt_to, dim, PyFloat_FromDouble(src.To().val[dim]));
    }
    PyList_SetItem(line, 0, pt_from);
    PyList_SetItem(line, 1, pt_to);
    return line;
  }
}; // type_caster for vcp's Line3d


template <> struct type_caster<vcp::python::conversion::BoundingBox2d>
{
public:
  PYBIND11_TYPE_CASTER(vcp::python::conversion::BoundingBox2d, _("BoundingBox2d (i.e. ((l,r,w,h), color, caption, is_dashed)"));

  bool load(handle src, bool)
  {
    value = vcp::python::conversion::PyObjectToBoundingBox2d(src.cast<py::object>());
    return !value.empty() && !PyErr_Occurred();
  }

  static handle cast(const vcp::python::conversion::BoundingBox2d &src, return_value_policy policy, handle parent)
  {
    VCP_UNUSED_VAR(src);
    VCP_UNUSED_VAR(policy);
    VCP_UNUSED_VAR(parent);
    VCP_ERROR("Not yet implemented!");
  }
};


template <> struct type_caster<vcp::python::conversion::RotatedBoundingBox2d>
{
public:
  PYBIND11_TYPE_CASTER(vcp::python::conversion::RotatedBoundingBox2d, _("RotatedBoundingBox2d (i.e. ((cx,cy,w,h,angle), color, caption, is_dashed)"));

  bool load(handle src, bool)
  {
    value = vcp::python::conversion::PyObjectToRotatedBoundingBox2d(src.cast<py::object>());
    return !value.empty() && !PyErr_Occurred();
  }

  static handle cast(const vcp::python::conversion::RotatedBoundingBox2d &src, return_value_policy policy, handle parent)
  {
    VCP_UNUSED_VAR(src);
    VCP_UNUSED_VAR(policy);
    VCP_UNUSED_VAR(parent);
    VCP_ERROR("Not yet implemented!");
  }
};


template <> struct type_caster<vcp::python::conversion::BoundingBox3d>
{
public:
  PYBIND11_TYPE_CASTER(vcp::python::conversion::BoundingBox3d, _("BoundingBox3d, i.e. ([(x0,y0,z0), ...], color, caption)"));

  bool load(handle src, bool)
  {
    value = vcp::python::conversion::PyObjectToBoundingBox3d(src.cast<py::object>());
    return !value.empty() && !PyErr_Occurred();
  }

  static handle cast(const vcp::python::conversion::BoundingBox3d &src, return_value_policy policy, handle parent)
  {
    VCP_UNUSED_VAR(src);
    VCP_UNUSED_VAR(policy);
    VCP_UNUSED_VAR(parent);
    VCP_ERROR("Not yet implemented!");
  }
};



template <> struct type_caster<vcp::python::conversion::VisLine2d>
{
public:
  PYBIND11_TYPE_CASTER(vcp::python::conversion::VisLine2d, _("VisLine2d (i.e. ((x0,y0),(x1,y1), is_dashed, color)"));

  bool load(handle src, bool)
  {
    value = vcp::python::conversion::PyObjectToVisLine2d(src.cast<py::object>());
    return !value.empty() && !PyErr_Occurred();
  }

  static handle cast(const vcp::python::conversion::VisLine2d &src, return_value_policy policy, handle parent)
  {
    VCP_UNUSED_VAR(src);
    VCP_UNUSED_VAR(policy);
    VCP_UNUSED_VAR(parent);
    VCP_ERROR("Not yet implemented!");
  }
};
} // namespace detail
} // namespace pybind11

#endif // __VCP_PYTHON_CONVERSION_H__
