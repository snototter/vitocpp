#ifndef __VCP_NP_CV_CONVERSION_H__
#define __VCP_NP_CV_CONVERSION_H__

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
namespace py = pybind11;

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
#else
    #include <opencv2/core.hpp>
#endif

namespace vcp
{
namespace python
{
namespace conversion
{
/** @brief Convert a NumPy ndarray (with num_dimensions == 1, 2 or 3) to OpenCV Mat.
 * Optimized for C- and F-contiguous ndarrays - otherwise, this will
 * copy the data element-wise.
 *
 * If you put in a transposed NDArray, WATCH OUT! To correctly transpose a 3D
 * NDArray in NumPy, use:
 *
 *     np.transpose(a, (1,0,2))
 *
 * This will transpose (H,W,L) to (W,H,L), i.e. transpose along the
 * first two axes. If you do
 *
 *     a.transpose(),
 *
 * however, you'll get (L,W,H) !!
 * The latter (L,W,H) is not supported by our conversion.
 */
cv::Mat NDArrayToMat(const py::array &ndarray);
//cv::Mat NDArrayToMat(const py::object &ndarray);


/** @brief Convert an OpenCV Mat to a numpy ndarray.
 * Only 1D and 2D matrices are supported (i.e. images with any number of channels)!
 * The matrix MUST NOT be empty (Otherwise, we would need to change the
 * signature and return a py::object referencing None).
 */
py::array MatToNDArray(const cv::Mat &mat);

} // namespace conversion
} // namespace python
} // namespace vcp


// We need to partially overload pybind11::detail::type_caster<T>, see https://pybind11.readthedocs.io/en/stable/advanced/cast/custom.html
namespace pybind11
{
namespace detail
{
template <> struct type_caster<cv::Mat>
{
public:
  PYBIND11_TYPE_CASTER(cv::Mat, _("Mat (i.e. np.array)"));

  bool load(handle src, bool)
  {
    if (!(src.is_none() || py::isinstance<py::array>(src)))
      return false;

    value = vcp::python::conversion::NDArrayToMat(py::cast<py::array>(src));

    return !PyErr_Occurred();
  }

  static handle cast(const cv::Mat&src, return_value_policy /* policy */, handle /* parent */)
  {
    //FIXME replace array by py::object to return None for empty Mat(s)
    py::array arr = vcp::python::conversion::MatToNDArray(src);
    return arr.release();
  }
}; // pybind11 type_caster for cv::Mat
} // namespace detail
} // namespace pybind11

#endif // __VCP_NP_CV_CONVERSION_H__
