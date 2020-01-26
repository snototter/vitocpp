//-----------------------------------------------------------------------------
// vcp::ui wrapper code

#include "conversion/cv_core_conversion.h"
#include "conversion/np_cv_conversion.h"

#include <vcp_ui/rect_selection.h>
#include <vcp_ui/point_selection.h>
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/string_utils.h>

namespace vcp
{
namespace python
{
namespace ui
{
vcp::ui::PointMarker MarkerFromString(const std::string &s)
{
  std::string lower = vcp::utils::string::Lower(s);
  if (lower.compare("dot") == 0)
    return vcp::ui::PointMarker::DOT;
  if (lower.compare("cross") == 0)
    return vcp::ui::PointMarker::CROSS;
  VCP_ERROR("Invalid marker type: '" << s << "'");
}

py::tuple SelectRectangle(const cv::Mat &image, const cv::Scalar &rect_color, const std::string &window_name)
{
  VCP_LOG_INFO_DEFAULT(vcp::ui::RectSelectionUsage());
  cv::Rect rect;
  const bool valid = vcp::ui::SelectRectangle(rect, image, rect_color, window_name);
  return py::make_tuple(valid, rect);
}

py::tuple SelectPoint(const cv::Mat &image, const cv::Scalar &point_color, const std::string &marker, int thickness, const std::string &window_name)
{
  VCP_LOG_INFO_DEFAULT(vcp::ui::PointSelectionUsage());
  cv::Point pt;
  const bool valid = vcp::ui::SelectPoint(pt, image, point_color, window_name, MarkerFromString(marker), thickness);
  return py::make_tuple(valid, pt);
}

std::vector<cv::Point> SelectPoints(const cv::Mat &image, const cv::Scalar &point_color, const std::string &marker, int thickness, const std::string &window_name)
{
  VCP_LOG_INFO_DEFAULT(vcp::ui::MultiplePointsSelectionUsage());
  return vcp::ui::SelectPoints(image, point_color, window_name, MarkerFromString(marker), thickness);
}

} // namespace ui
} // namespace python
} // namespace pvt


//-----------------------------------------------------------------------------
// Python module declarations


PYBIND11_MODULE(ui_basics, m)
{
  m.doc() = "Basic UI tools to select points/rectangles on an image.\n"
            "For more sophisticated image viewing/input capabilities\n"
            "check the 'iminspect' python package instead.";

  m.def("select_rectangle", &vcp::python::ui::SelectRectangle,
        "Shows the given image in an opencv/highgui window and let's you\n"
        "select a rectangular region.\n\n"
        ":param:  image Input image\n"
        ":param:  color Color of the overlay rectangle\n"
        ":param:  window_name Window title\n\n"
        ":return: tuple(bool, rect) where the boolean flag indicates whether the user selected (and confirmed)\n"
        "         a valid rectangle (true), or not (false). If true, the rectangle is returned as a 4-element\n"
        "         tuple holding (left, top, width, hight).",
        py::arg("image"),
        py::arg("rect_color") = cv::Scalar(0, 0, 255),
        py::arg("window_name") = "Select rectangle by click & drag");

  m.def("select_point", &vcp::python::ui::SelectPoint,
        "Shows the given image in an opencv/highgui window and let's you\n"
        "select a single point.\n\n"
        ":param:  image Input image\n"
        ":param:  point_color Color of the overlay point\n"
        ":param:  marker, either 'dot' or 'cross'\n"
        ":param:  thickness Radius if 'dot', else line thickness\n"
        ":param:  window_name Window title\n\n"
        ":return: tuple(bool, point) where the boolean flag indicates whether the user selected (and confirmed)\n"
        "         a valid point (true), or not (false). If true, the point is returned as a 2-element\n"
        "         tuple holding (x, y).",
        py::arg("image"),
        py::arg("point_color") = cv::Scalar(255, 0, 255),
        py::arg("marker") = "dot",
        py::arg("thickness") = 5,
        py::arg("window_name") = "Select point");

  m.def("select_points", &vcp::python::ui::SelectPoints,
        "Shows the given image in an opencv/highgui window and let's you\n"
        "select multiple points.\n\n"
        ":param:  image Input image\n"
        ":param:  point_color Color of the overlay point\n"
        ":param:  marker, either 'dot' or 'cross'\n"
        ":param:  thickness Radius if 'dot', else line thickness\n"
        ":param:  window_name Window title\n\n"
        ":return: list<point> which contains all selected points (i.e. list of (x,y) tuples if confirmed, else it's an empty list)",
        py::arg("image"),
        py::arg("point_color") = cv::Scalar(255, 0, 255),
        py::arg("marker") = "dot",
        py::arg("thickness") = 5,
        py::arg("window_name") = "Select point");
}

