#include "vcp_conversion.h"
#include "cv_core_conversion.h"

#include <opencv2/core/core.hpp>

#include <memory>
#include <exception>
#include <string>

namespace vcp
{
namespace python
{
namespace conversion
{

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::python::conversion"

vcp::imvis::drawing::Box3d PyObjectToBox3d(const py::object &object)
{
  vcp::imvis::drawing::Box3d box;

  if (object.is_none())
    return box;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  if (type.compare("list") == 0)
  {
    py::list list = py::cast<py::list>(object);
    if (py::len(list) < 8) // "<" because we may add additional elements for visualization
      VCP_ERROR("List must contain 8 corner points for a 3D box, given list holds " + std::to_string(py::len(list)));

    for (size_t i = 0; i < 4; ++i)
    {
      box.AddTopCorner(PyObjectToVec<double, 3>(list[i], nullptr));
      box.AddBottomCorner(PyObjectToVec<double, 3>(list[i+4], nullptr));
    }
  }
  else if (type.compare("tuple") == 0)
  {
    py::tuple tuple = py::cast<py::tuple>(object);
    if (py::len(tuple) < 8) // "<" because we may add additional elements for visualization
      VCP_ERROR("Tuple must contain 8 corner points for a 3D box, given tuple holds " + std::to_string(py::len(tuple)));

    for (size_t i = 0; i < 4; ++i)
    {
      box.AddTopCorner(PyObjectToVec<double, 3>(tuple[i], nullptr));
      box.AddBottomCorner(PyObjectToVec<double, 3>(tuple[i+4], nullptr));
    }
  }
  else
  {
    //TODO convert ndarray 3x8 to box3d!
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to Box3d");
  }
  return box;
}



vcp::math::geo2d::Line2d PyObjectToLine2d(const py::object &object)
{
  vcp::math::geo2d::Line2d line;
  if (object.is_none())
    return line;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  if (type.compare("list") == 0)
  {
    py::list list = py::cast<py::list>(object);
    if (py::len(list) < 2) // "<" because we may add additional elements for visualization
      VCP_ERROR("List must contain 2 points for a Line2d, given list holds " + std::to_string(py::len(list)));

    const cv::Vec2d vfrom = PyObjectToVec<double, 2>(list[0], nullptr);
    const cv::Vec2d vto = PyObjectToVec<double, 2>(list[1], nullptr);
    line = vcp::math::geo2d::Line2d(vfrom, vto);
  }
  else if (type.compare("tuple") == 0)
  {
    py::tuple tuple = py::cast<py::tuple>(object);
    if (py::len(tuple) < 2) // "<" because we may add additional elements for visualization
      VCP_ERROR("Tuple must contain 2 points for a Line2d, given tuple holds " + std::to_string(py::len(tuple)));

    const cv::Vec2d vfrom = PyObjectToVec<double, 2>(tuple[0], nullptr);
    const cv::Vec2d vto = PyObjectToVec<double, 2>(tuple[1], nullptr);
    line = vcp::math::geo2d::Line2d(vfrom, vto);
  }
  else
  {
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to Line2d (2x2 np.array is not supported to avoid confusion)");
  }
  return line;
}


vcp::math::geo3d::Line3d PyObjectToLine3d(const py::object &object)
{
  vcp::math::geo3d::Line3d line;
  if (object.is_none())
    return line;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  if (type.compare("list") == 0)
  {
    py::list list = py::cast<py::list>(object);
    if (py::len(list) < 2) // "<" because we may add additional elements for visualization
      VCP_ERROR("List must contain 2 points for a Line3d, given list holds " + std::to_string(py::len(list)));

    const cv::Vec3d vfrom = PyObjectToVec<double, 3>(list[0], nullptr);
    const cv::Vec3d vto = PyObjectToVec<double, 3>(list[1], nullptr);
    line = vcp::math::geo3d::Line3d(vfrom, vto);
  }
  else if (type.compare("tuple") == 0)
  {
    py::tuple tuple = py::cast<py::tuple>(object);
    if (py::len(tuple) < 2) // "<" because we may add additional elements for visualization
      VCP_ERROR("Tuple must contain 2 points for a Line3d, given tuple holds " + std::to_string(py::len(tuple)));

    const cv::Vec3d vfrom = PyObjectToVec<double, 3>(tuple[0], nullptr);
    const cv::Vec3d vto = PyObjectToVec<double, 3>(tuple[1], nullptr);
    line = vcp::math::geo3d::Line3d(vfrom, vto);
  }
  //TODO convert 3x2 np array to line3d
  else
  {
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to Line3d.");
  }
  return line;
}


BoundingBox2d PyObjectToBoundingBox2d(const py::object &object)
{
  BoundingBox2d box;
  box.Clear();

  if (object.is_none())
    return box;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  if (type.compare("list") == 0)
  {
    const py::list list = object.cast<py::list>();
    size_t len = py::len(list);
    if (len == 0)
      VCP_ERROR("List must contain at least 1 element to convert to a BoundingBox2d!");

    box.box = PyObjectToRect<int>(list[0], nullptr); // py::cast doesn't work
    if (len > 1 && !list[1].is_none())
      box.color = PyObjectToScalar(list[1], nullptr);
    if (len > 2 && !list[2].is_none())
      box.caption = py::cast<std::string>(list[2]);
    if (len > 3 && !list[3].is_none())
      box.is_dashed = py::cast<bool>(list[3]);
  }
  else if (type.compare("tuple") == 0)
  {
    const py::tuple tuple = object.cast<py::tuple>();
    size_t len = py::len(tuple);
    if (len == 0)
      VCP_ERROR("Tuple must contain at least 1 element to convert to a BoundingBox2d!");

    box.box = PyObjectToRect<int>(tuple[0], nullptr); // py::cast doesn't work
    if (len > 1 && !tuple[1].is_none())
      box.color = PyObjectToScalar(tuple[1], nullptr);
    if (len > 2 && !tuple[2].is_none())
      box.caption = py::cast<std::string>(tuple[2]);
    if (len > 3 && !tuple[3].is_none())
      box.is_dashed = py::cast<bool>(tuple[3]);
  }
  else if (type.compare("ndarray") == 0)
  {
    box.box = PyObjectToRect<int>(object, nullptr); // py::cast doesn't work
  }
  else
  {
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to BoundingBox2d!");
  }
  return box;
}



RotatedBoundingBox2d PyObjectToRotatedBoundingBox2d(const py::object &object)
{
  RotatedBoundingBox2d box;
  box.Clear();

  if (object.is_none())
    return box;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  if (type.compare("list") == 0)
  {
    const py::list list = object.cast<py::list>();
    size_t len = py::len(list);
    if (len == 0)
      VCP_ERROR("List must contain at least 1 element to convert to a RotatedBoundingBox2d!");

    // 5 elements => only cx,cy,w,h,angle are provided
    // less than 5 elements, list[0] is the 5 element rotated rect
    if (len == 5)
    {
      box.box = PyObjectToRotatedRect(list, nullptr); // py::cast doesn't work
    }
    else
    {
      box.box = PyObjectToRotatedRect(list[0], nullptr); // py::cast doesn't work
      if (len > 1 && !list[1].is_none())
        box.color = PyObjectToScalar(list[1], nullptr);
      if (len > 2 && !list[2].is_none())
        box.caption = py::cast<std::string>(list[2]);
      if (len > 3 && !list[3].is_none())
        box.is_dashed = py::cast<bool>(list[3]);
    }
  }
  else if (type.compare("tuple") == 0)
  {
    const py::tuple tuple = object.cast<py::tuple>();
    size_t len = py::len(tuple);
    if (len == 0)
      VCP_ERROR("Tuple must contain at least 1 element to convert to a RotatedBoundingBox2d!");

    // 5 elements => only cx,cy,w,h,angle are provided
    // less than 5 elements, list[0] is the 5 element rotated rect
    if (len == 5)
    {
      box.box = PyObjectToRotatedRect(tuple, nullptr); // py::cast doesn't work
    }
    else
    {
      box.box = PyObjectToRotatedRect(tuple[0], nullptr); // py::cast doesn't work
      if (len > 1 && !tuple[1].is_none())
        box.color = PyObjectToScalar(tuple[1], nullptr);
      if (len > 2 && !tuple[2].is_none())
        box.caption = py::cast<std::string>(tuple[2]);
      if (len > 3 && !tuple[3].is_none())
        box.is_dashed = py::cast<bool>(tuple[3]);
    }
  }
  else if (type.compare("ndarray") == 0)
  {
    box.box = PyObjectToRotatedRect(object, nullptr); // py::cast doesn't work
  }
  else
  {
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to RotatedBoundingBox2d!");
  }
  return box;
}



BoundingBox3d PyObjectToBoundingBox3d(const py::object &object)
{
  BoundingBox3d box;
  box.Clear();

  if (object.is_none())
    return box;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  if (type.compare("list") == 0)
  {
    const py::list list = object.cast<py::list>();
    size_t len = py::len(list);
    if (len == 0)
      VCP_ERROR("List must contain at least 1 element (i.e. a list/tuple of 8 corners) to convert to a BoundingBox3d!");

    box.box = PyObjectToBox3d(list[0]);
    if (len > 1 && !list[1].is_none())
      box.color = PyObjectToScalar(list[1], nullptr);
    if (len > 2 && !list[2].is_none())
      box.caption = py::cast<std::string>(list[2]);
  }
  else if (type.compare("tuple") == 0)
  {
    const py::tuple tuple = object.cast<py::tuple>();
    size_t len = py::len(tuple);
    if (len == 0)
      VCP_ERROR("Tuple must contain at least 1 element (i.e. a list/tuple of 8 corners) to convert to a BoundingBox3d!");

    box.box = PyObjectToBox3d(tuple[0]);
    if (len > 1 && !tuple[1].is_none())
      box.color = PyObjectToScalar(tuple[1], nullptr);
    if (len > 2 && !tuple[2].is_none())
      box.caption = py::cast<std::string>(tuple[2]);
  }
//  else if (type.compare("ndarray") == 0)
//  {
//    box.box = PyObjectToRect<int>(object, nullptr); // py::cast doesn't work
//  }
  else
  {
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to BoundingBox3d, it must be a tuple/list, e.g. ([(x0,x1),...])!");
  }
  return box;
}

VisLine2d PyObjectToVisLine2d(const py::object &object)
{
  VisLine2d line;
  line.Clear();

  if (object.is_none())
    return line;

  const std::string type = py::cast<std::string>(object.attr("__class__").attr("__name__"));
  if (type.compare("list") == 0)
  {
    const py::list list = object.cast<py::list>();
    size_t len = py::len(list);
    if (len < 2)
      VCP_ERROR("List must contain at least 2 elements to convert to a Line2d!");

    line.line = vcp::math::geo2d::Line2d(PyObjectToVec<double,2>(list[0], nullptr), PyObjectToVec<double,2>(list[1], nullptr)); // py::cast does not always work (e.g. if list[i] is a np.array)
    if (len > 2 && !list[2].is_none())
      line.is_dashed = py::cast<bool>(list[2]);
    if (len > 3 && !list[3].is_none())
      line.color = PyObjectToScalar(list[3], nullptr);
  }
  else if (type.compare("tuple") == 0)
  {
    const py::tuple tuple = object.cast<py::tuple>();
    size_t len = py::len(tuple);
    if (len == 0)
      VCP_ERROR("Tuple must contain at least 2 elements to convert to a Line2d!");

    line.line = vcp::math::geo2d::Line2d(PyObjectToVec<double,2>(tuple[0], nullptr), PyObjectToVec<double,2>(tuple[1], nullptr)); // py::cast does not always work (e.g. if list[i] is a np.array)
    if (len > 2 && !tuple[2].is_none())
      line.is_dashed = py::cast<bool>(tuple[2]);
    if (len > 3 && !tuple[3].is_none())
      line.color = PyObjectToScalar(tuple[3], nullptr);
  }
  else
  {
    VCP_LOG_FAILURE("Cannot convert type '" << type << "' to Line2d!");
  }
  return line;
}
} // namespace conversion
} // namespace python
} // namespace vcp
