#include "np_cv_conversion.h"

#include <pybind11/pybind11.h>
#include <pybind11/cast.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
namespace py = pybind11;

#include <opencv2/core/core.hpp>
#include <vcp_imutils/opencv_compatibility.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_utils/vcp_error.h>

#include <memory>
#include <string>

namespace vcp
{
namespace python
{
namespace conversion
{
#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::python::conversion"

/** @brief Helper to copy memory from numpy.array to cv::Mat (needed [very rarely] when we have to copy matrices element-wise). */
template<typename T>
void copy_to_mat(const py::array& ndarray, cv::Mat& mat)
{
  long int rows = ndarray.shape(0);
  long int cols = ndarray.shape(1);
  long int chans = ndarray.ndim() == 3 ? ndarray.shape(2) : 1;
  long int stride_row = ndarray.strides(0) / sizeof(T);
  long int stride_col = ndarray.strides(1) / sizeof(T);
  long int stride_chan = ndarray.ndim() == 3 ? ndarray.strides(2) / sizeof(T) : 1;
  //TODO check with negative strides (was size_t instead of py_intptr_t) - seems to work, too - have to check it on paper!

  const T* row_iter = reinterpret_cast<const T*>(ndarray.data());
  for (long int row = 0; row < rows; ++row, row_iter += stride_row)
  {
    T* row_ptr = mat.ptr<T>(row);
    const T* col_iter = row_iter;
    for (long int col = 0; col < cols; ++col, col_iter += stride_col)
    {
      const T* chan_iter = col_iter;
      for (long int chan = 0; chan < chans; ++chan, chan_iter += stride_chan)
      {
        row_ptr[(col * chans) + chan] = *chan_iter;
      }
    }
  }
}

inline py::dtype MatDepthToNDArrayType(int depth)
{
  switch(depth)
  {
  case CV_8U:
    return py::dtype::of<uint8_t>();
  case CV_8S:
    return py::dtype::of<int8_t>();
  case CV_16U:
    return py::dtype::of<uint16_t>();
  case CV_16S:
    return py::dtype::of<int16_t>();
  case CV_32S:
    return py::dtype::of<int32_t>();
  case CV_32F:
    return py::dtype::of<float>();
  case CV_64F:
    return py::dtype::of<double>();
  default:
    VCP_ERROR("Cannot cast cv::Mat depth " + std::to_string(depth) + " to numpy.dtype!");
  }
}

inline int NDArrayTypeToMatDepth(const py::dtype &dtype)
{
  if (dtype.is(py::dtype::of<uint8_t>()))
    return CV_8U;
  if (dtype.is(py::dtype::of<int8_t>()))
    return CV_8S;
  if (dtype.is(py::dtype::of<uint16_t>()))
    return CV_16U;
  if (dtype.is(py::dtype::of<int16_t>()))
    return CV_16S;
  if (dtype.is(py::dtype::of<int32_t>()))
    return CV_32S;
  if (dtype.is(py::dtype::of<float>()))
    return CV_32F;
  if (dtype.is(py::dtype::of<double>()))
    return CV_64F;
  if (dtype.is(py::dtype::of<bool>()))
  {
    VCP_LOG_WARNING("Cannot check if we deal with a packed boolean ndarray. Naively assuming that we can cast np.bool to uint8!");
    return CV_8U;
  }
  VCP_ERROR("Cannot cast the given numpy.dtype ("
           + py::cast<std::string>(py::str(py::cast<py::handle>(dtype)))
           + ") to cv::Mat depth!");
}


cv::Mat NDArrayToMat(const py::array &ndarray)
{
  if (ndarray.ndim() < 1 || ndarray.ndim() > 3)//(ndarray.ndim() != 2 && ndarray.ndim() != 3)
  {
    VCP_ERROR("VCP can only convert 1D np.arrays or 2D images "
               "(i.e. 2D and 3D NumPy ndarrays) to cv::Mat, you tried with array.ndim()=="
               + std::to_string(ndarray.ndim()));
  }

  const py::dtype dtype = ndarray.dtype();
  const int depth = NDArrayTypeToMatDepth(dtype);

  cv::Mat mat;

  if (ndarray.flags() & py::array::c_style)
  {
#ifdef DEBUG_CONVERSION
    VCP_LOG_INFO_DEFAULT("np2cv: Convert C-contiguous NDArray to Mat (row-major)");
#endif
    // Copy single memory chunk, ndarray is row-major
    const size_t rows = ndarray.shape(0);
    const size_t cols = ndarray.ndim() > 1 ? ndarray.shape(1) : 1;
    const size_t channels = ndarray.ndim() == 3 ? ndarray.shape(2) : 1;

    mat.create(rows, cols, CV_MAKE_TYPE(depth, channels));
    if (!mat.isContinuous())
      VCP_ERROR("cv::Mat is not continuous after creation!"); // there must be something wrong!

    std::memcpy(mat.data, ndarray.data(), rows * cols * channels * dtype.itemsize());
  }
  else
  {
    // Nice summary of NumPy memory alignment:
    // https://stackoverflow.com/questions/26998223/what-is-the-difference-between-contiguous-and-non-contiguous-arrays
    // Detailed: https://docs.scipy.org/doc/numpy/reference/arrays.ndarray.html#internal-memory-layout-of-an-ndarray
    if (ndarray.flags() & py::array::f_style)
    {
#ifdef DEBUG_CONVERSION
      VCP_LOG_INFO_DEFAULT("np2cv: Convert F-contiguous NDArray to Mat (column-major)");
#endif
      // NDArray is transposed (column-major), we create a transposed Mat and
      // transpose the result to get the correct result.
      const int rows = ndarray.ndim() > 1 ? ndarray.shape(1) : 1;
      const int cols = ndarray.shape(0);
      const int channels = ndarray.ndim() == 3 ? ndarray.shape(2) : 1;

      mat.create(rows, cols, CV_MAKE_TYPE(depth, channels));
      if (!mat.isContinuous())
        VCP_ERROR("cv::Mat is not continuous after creation!"); // there must be something wrong!

      std::memcpy(mat.data, ndarray.data(), rows * cols * channels * dtype.itemsize());
      mat = mat.t();
    }
    else
    {
#ifdef DEBUG_CONVERSION
      VCP_LOG_INFO_DEFAULT("np2cv: Convert non-contiguous NDArray to Mat (element-wise)");
#endif
      // Flipping a NDArray gives negative strides, let's copy the buffer elementwise:

//      Original:
//      Strides: 3840,3,1
//      Shape: 720,1280,3

//      Flipped:Scalar
//      Strides: 3840,-3,1
//      Shape: 720,1280,3

//      Transpose&flipped, even worse
//      Shape:  1280 x 720 x 3
//      Stride: 3 x -3840 x 1
      const size_t rows = ndarray.shape(0);
      const size_t cols = ndarray.shape(1);
      const size_t channels = ndarray.ndim() == 3 ? ndarray.shape(2) : 1;

      mat.create(rows, cols, CV_MAKE_TYPE(depth, channels));
      switch(depth)
      {
      case CV_8U:
        copy_to_mat<uint8_t>(ndarray, mat);
        break;

      case CV_8S:
        copy_to_mat<int8_t>(ndarray, mat);
        break;

      case CV_16U:
        copy_to_mat<uint16_t>(ndarray, mat);
        break;

      case CV_16S:
        copy_to_mat<int16_t>(ndarray, mat);
        break;

      case CV_32S:
        copy_to_mat<int32_t>(ndarray, mat);
        break;

      case CV_32F:
        copy_to_mat<float>(ndarray, mat);
        break;

      case CV_64F:
        copy_to_mat<double>(ndarray, mat);
        break;

      default:
        VCP_ERROR("Converting non-contiguous numpy.array to cv::Mat with depth '" + std::to_string(depth) + " is not supported!");
      }
    }
  }

  return mat;
}

py::array MatToNDArray(const cv::Mat &mat)
{
  if (mat.empty())
    VCP_ERROR("cv::Mat is empty, cannot convert to numpy.array!");

  // OpenCV Mat documentation: https://docs.opencv.org/2.4/modules/core/doc/basic_structures.html#mat
  // 2D matrices are stored row-major (row-by-row), 3D are stored plane-by-plane, ...
  if (mat.dims > 2)
  {
    VCP_ERROR("VCP only supports converting 1D or 2D cv::Mat to np.array, you requested mat.dims=" + std::to_string(mat.dims));
  }

  const py::dtype dtype = MatDepthToNDArrayType(mat.depth());
  std::vector<int> shape = {mat.rows, mat.cols};
  if (mat.channels() > 1)
  {
    shape.push_back(mat.channels());
  }
  py::array ndarray = py::array(dtype, shape);
  if (ndarray.flags() & py::array::c_style)
  {
    // If ndarray's flag is C_CONTIGUOUS, then we have a single memory chunk
    // (without padded rows). See also boost::numpy::detail::is_c_contiguous()
    // implementation, e.g. https://github.com/ndarray/Boost.NumPy/blob/master/libs/numpy/src/ndarray.cpp
    if (mat.isContinuous())
    {
#ifdef DEBUG_CONVERSION
      VCP_LOG_INFO_DEFAULT("---- Convert continuous MAT to c-contiguous NDArray");
#endif
      std::memcpy(ndarray.mutable_data(), mat.data, mat.total() * mat.elemSize());
    }
    else
    {
#ifdef DEBUG_CONVERSION
      VCP_LOG_INFO_DEFAULT("---- Convert non-continuous MAT to c-contiguous NDArray, need to copy row-by-row");
      for (ssize_t i = 0; i < ndarray.ndim(); ++i)
      {
        VCP_LOG_INFO_DEFAULT("  Dim[" << i << "], shape = " << ndarray.shape(i) << ", stride = " << ndarray.strides(i));
      }
#endif
      // Nice summary of whenever Mat is not continuous: https://stackoverflow.com/questions/33665241/is-opencv-matrix-data-guaranteed-to-be-continuous
      // You can easily test this by passing in a submatrix, i.e. mat(cv::Rect(...)) or a view, e.g. mat.column()
      // Here, we need to copy row by row:
       for (int row = 0; row < mat.rows; ++row)
       {
         void *np_row = ndarray.mutable_data(row, 0);
         const unsigned char *mat_row = mat.ptr(row);
         std::memcpy(np_row, mat_row, mat.cols * mat.elemSize());
       }
    }
  }
  else
  {
    // To my understanding, currently, this cannot happen - unless you use some
    // magic (custom) ndarray allocator which does fancy memory alignment and
    // padding.
    // This is clearly no intended use case for VCP ;-)
    VCP_ERROR("Cannot convert cv::Mat to numpy.array because the pybind11::array() is not C_CONTIGUOUS after creation!");
  }
  return ndarray;
}

} // namespace conversion
} // namespace python
} // namespace vcp
