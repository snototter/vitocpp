#include "conversion/np_cv_conversion.h"
#include "conversion/cv_core_conversion.h"

#include <vcp_imutils/imutils.h>
#include <vcp_imutils/matutils.h>
#include <vcp_imutils/imabstraction.h>
#include <vcp_math/geometry2d.h>


//-----------------------------------------------------------------------------
// Wrapper/Helper code

namespace vcp
{
namespace python
{
namespace imutils
{
cv::Mat CartoonifyWrapper(const cv::Mat &image, int num_pyramid_levels, int num_bilateral_filters, int diameter_pixel_neighborhood, double sigma_color, double sigma_space, int kernel_size_median, int edge_block_size, bool is_rgb)
{
  cv::Mat vis = image.clone();
  vcp::imutils::Cartoonify(vis, num_pyramid_levels, num_bilateral_filters, diameter_pixel_neighborhood, sigma_color, sigma_space, kernel_size_median, edge_block_size, is_rgb);
  return vis;
}


py::tuple RotateAnnotatedImageWrapper(const cv::Mat &image, const std::vector<cv::Rect> &bounding_boxes, double theta, bool crop)
{
  cv::Mat img = image.clone();
  const std::vector<cv::RotatedRect> rotated_boxes = vcp::imutils::RotateAnnotatedImage(img, bounding_boxes, theta, crop);
  return py::make_tuple(img, rotated_boxes);
}


cv::Mat ConvertTo8U(const cv::Mat &image)
{
  cv::Mat cv8u;
  vcp::imutils::ConvertTo8U(image, cv8u);
  return cv8u;
}


cv::Mat RotateImageWrapper(const cv::Mat &image, double theta, bool crop)
{
  cv::Mat rotated = image.clone();
  vcp::imutils::RotateImage(rotated, theta, crop);
  return rotated;
}

cv::Mat RotateAnchoredImageWrapper(const cv::Mat &image, const cv::Point &anchor, double theta, int border_mode)
{
  cv::Mat rotated = image.clone();
  vcp::imutils::RotateImage(rotated, anchor, theta, border_mode);
  return rotated;
}


py::tuple FuzzyResizeWrapper(const cv::Mat &image, double scaling_factor)
{
  cv::Mat res = vcp::imutils::FuzzyResize(image, scaling_factor);
  return py::make_tuple(res, scaling_factor);
}

py::tuple ResizeAspectAwareWrapper(const cv::Mat &image, const cv::Size &new_size, const cv::Scalar &padding_value, bool center_output)
{
  cv::Rect location;
  cv::Mat res = vcp::imutils::ResizeKeepAspectRatio(image, new_size, padding_value, center_output, &location);
  return py::make_tuple(res, location);
}

cv::Mat ApplyImageTransformations(const cv::Mat &image, const py::args &transforms)
{
  std::vector<vcp::imutils::ImgTransform> ts;
  for (size_t i = 0; i < transforms.size(); ++i)
    ts.push_back(vcp::imutils::ImgTransformFromString(transforms[i].cast<std::string>()));

  return vcp::imutils::ApplyImageTransformations(image, ts);
}

} // namespace imutils
} // namespace python
} // namespace vcp


//-----------------------------------------------------------------------------
// Python module declarations

PYBIND11_MODULE(imutils_cpp, m)
{
  namespace vpmu = vcp::python::imutils;
  m.doc() = "Python/C++ bindings for vcp::imutils";

  //################################################################################
  // Geometric image manipulations

  m.def("rotate_image", &vpmu::RotateImageWrapper,
        "Rotates the image by 'theta' radians around the center.\n"
        "Positive angle rotates counter-clockwise.\n"
        ":param image:\n"
        ":param theta: rotation in radians\n"
        ":param crop: if true, the returned image is cropped to\n"
        "             contain only valid (i.e. no padded) pixels\n"
        ":return: numpy.array",
        py::arg("image"), py::arg("theta"), py::arg("crop")=false);


  m.def("rotate_image_anchor", &vpmu::RotateAnchoredImageWrapper,
        "Rotates the image by 'theta' radians around the given anchor point.\n"
        "Positive angle rotates counter-clockwise.\n"
        ":param image:\n"
        ":param anchor: 2D point defining the center of rotation as (x,y)\n"
        ":param theta: rotation in radians\n"
        ":param border_mode: see OpenCV's border types, e.g. cv2.BORDER_CONSTANT\n"
        ":return: numpy.array",
        py::arg("image"), py::arg("anchor"), py::arg("theta"),
        py::arg("border_mode")=0);


  m.def("rotate_points", &vcp::math::geo2d::RotateVecs,
        "Rotate the given points (2d vectors).\n\n"
        ":param points: list of 2d points, [(x0,y0), (x1,y1), ...].\n"
        ":param rotation_center: 2d rotation center.\n"
        ":param theta:           rotation angle in radians.\n"
        ":return: list of rotated points [(x0,y0),...]",
        py::arg("points"),
        py::arg("rotation_center"),
        py::arg("theta"));


  m.def("flip_points", &vcp::imutils::FlipVecs,
        "Mirror/flip a list of points.\n\n"
        ":param points: list of 2d points, [(x0,y0), (x1,y1), ...].\n"
        ":param image_size: tuple specifying the (width, height)\n"
        "                   of the image\n"
        ":param flip_horizontally: True/False\n"
        ":param flip_vertically: True/False\n",
        py::arg("points"),
        py::arg("image_size"),
        py::arg("flip_horizontally"),
        py::arg("flip_vertically"));


  //################################################################################
  // Input/Output
  m.def("save_mat", &vcp::imutils::DumpMat,
        "Save matrix to disk (converts to cv::Mat first) in a binarized format.\n"
        ":param mat: the matrix/image\n"
        ":param filename:\n"
        ":param zip: if true, the file will be zipped to save disk space\n"
        ":return: true if stored successfully\n\n"
        "Choose zipping wisely:\n"
        "* Storing a 1920x1080 CV_64FC1 matrix uncompressed (zip = False)\n"
        "  takes ~10ms but needs ~17 MB.\n"
        "* Storing it compressed (zip = True) takes ~110ms but only needs 400-500 KB.",
        py::arg("mat"), py::arg("filename"), py::arg("zip")=false);


  m.def("load_mat", &vcp::imutils::LoadMat,
        "Load a cv::Mat from disk, which was stored via save_mat().\n"
        ":param filename:\n"
        ":return: the loaded mat if successful",
        py::arg("filename"));


  //################################################################################
  // Image abstraction
  m.def("cartoonify", &vpmu::CartoonifyWrapper,
        "Applies bilateral filtering and edge enhancement to create a cartoon-like\n"
        "effect. Repeatedly (num_bilateral_filter times) applies a small bilateral\n"
        "filter instead of a single large filter.\n\n"
        ":param is_rgb: Specify whether the input is RGB (true) or BGR (false)\n"
        "Other parameters need to be documented - for now, happy experimenting",
        py::arg("image"),
        py::arg("num_pyramid_levels") = 3,
        py::arg("num_bilateral_filters") = 5,
        py::arg("diameter_pixel_neighborhood") = 7,
        py::arg("sigma_color") = 9.0,
        py::arg("sigma_space") = 7.0,
        py::arg("kernel_size_median") = 7,
        py::arg("edge_block_size") = 9,
        py::arg("is_rgb") = false);

  m.def("pixelate", &vcp::imutils::Pixelate,
        "Pixelate the image into blocks of size w x h.\n"
        ":param image: numpy ndarray.\n"
        ":param w: block width in pixels.\n"
        ":param h: block height in pixels. If h==-1, then\n"
        "          the blocks will be squares of size w x w.\n"
        ":return: the pixelated image as numpy ndarray.",
        py::arg("image"),
        py::arg("w"),
        py::arg("h")=-1);


  //################################################################################
  // Conversion
  m.def("convert_to_uint8", &vpmu::ConvertTo8U,
        "Tries to convert the given image to uint8 buffer.\n"
        "* 8S input will be shifted [x+127]\n"
        "* 8U = 8U\n"
        "* 32F/64F will be scaled by 255 (thus, assuming they\n"
        "  are already normalized to [0,1])",
        py::arg("image"));


  m.def("grayscale", &vcp::imutils::Grayscale,
        "Converts RGB, BGR, or grayscale inputs to single-channel or\n"
        "3-channel grayscale output. Multi-channel output is useful\n"
        "for visualizations (drawing upon it).\n\n"
        ":param image:\n"
        ":param image_is_rgb: true if input is RGB, false if BGR\n"
        ":param output_single_channgel: true if you want a single\n"
        "                               channel output, false for 3-channel\n"
        ":return: grayscale image with 1 or 3 channels",
        py::arg("image"), py::arg("image_is_rgb")=false,
        py::arg("output_single_channel")=false);


  //################################################################################
  // Resizing
  m.def("fuzzy_resize__", &vpmu::FuzzyResizeWrapper,
        "Resizes but rounds the 'scaling_factor' to the closest 1/10th\n"
        "before scaling. Useful if your scaling factor may vary very slightly,\n"
        "to avoid unnecessary resizing.\n"
        ":param image:\n"
        ":param scaling_factor: desired scaling factor (which will be rounded\n"
        "                       to the closest 1/10th)\n"
        ":return: tuple(resized_image, actual_scaling_factor)",
        py::arg("image"), py::arg("scaling_factor"));


  m.def("aspect_aware_resize", &vpmu::ResizeAspectAwareWrapper,
        "Resize image, keep aspect ratio.\n"
        ":param image:\n"
        ":param new_size: desired (width,height)\n",
        ":param padding_value: border pixels will be padded by this constant value\n"
        ":param center_output: true if you want to center the resized image within\n"
        "                      the padded output image\n"
        ":return: tuple(resized_image, location_of_actual_image_content) where the\n"
        "                      latter is a rect (l, t, w, h) defining the valid region",
        py::arg("image"), py::arg("new_size"),
        py::arg("padding_value") = cv::Scalar::all(0.0),
        py::arg("center_output") = false);


  m.def("rotate_rect", &vcp::imutils::RotateRect,
        "Rotates the given rectangle by the given radians,\n"
        "assuming a left-handed (image!) coordinate system.\n\n"
        ":param rect:  tuple, list, np.array (l,r,w,h)\n"
        ":param rotation_center: 2d center of rotation (x,y)\n"
        ":param theta: rotation angle in radians\n"
        ":return: tuple(cx, cy, w, h, angle) where (cx,cy) is\n"
        "         the center and angle is in degrees(!), since\n"
        "         we use OpenCV's RotatedRect data structure.",
        py::arg("rect"),
        py::arg("rotation_center"),
        py::arg("theta"));


  m.def("rotate_annotated_image", &vpmu::RotateAnnotatedImageWrapper,
        "Rotate an image and its annotations (i.e. a list of axis-aligned\n"
        "rectangles) by theta radians.\n\n"
        ":param image:\n"
        ":param rects: a list of rectangles [r1, r2, ...], where each\n"
        "              rectangle is a tuple/list/np.array (l,t,w,h).\n"
        ":param theta: rotation angle in radians. Positive angle\n"
        "              rotates counter-clockwise.\n"
        ":param crop:  If True, the largest center crop which\n"
        "              contains only valid pixels will be returned.\n"
        ":return: tuple(rotated_image, rotated_rects), where\n"
        "              rotated_rects is a list [R1, R2, ...] of rotated\n"
        "              rectangles R=(cx, cy, w, h, angle). Note that the\n"
        "              angle is in degrees(!) as we use OpenCV's RotatedRect.",
        py::arg("image"), py::arg("rects"),
        py::arg("theta"), py::arg("crop") = false);

  m.def("transform", &vpmu::ApplyImageTransformations,
        "Apply basic image transformations, such as:\n"
        "*  Mirroring: fliplr, flipud\n"
        "* Rotation: rot90, rot180, rot270\n"
        "* Histogram equalization: histeq\n"
        "* Discretization: rgb2cn, bgr2cn (color names)\n"
        "* Color space: rgb2hsv, rgb2lab, rgb2gray\n"
        "               as well as their bgr... versions\n\n"
        "See the C++ ImgTransform enum (imutils.h) for supported\n"
        "transformations.\n"
        "You can simply chain above transformations, for example\n"
        "  transform(image, 'rot90', 'histeq', 'rgb2cn')\n\n"
        ":param image: input image as numpy ndarray\n"
        ":*args:       string representations of\n"
        "              the desired transformations.\n"
        ":return: transformed image as numpy ndarray.",
        py::arg("image"));//TODO add surface normal stuff
}

