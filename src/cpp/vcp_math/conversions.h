#ifndef __VCP_MATH_OPENCV_CONVERSIONS_H__
#define __VCP_MATH_OPENCV_CONVERSIONS_H__

#include <opencv2/core/core.hpp>
#include <exception>
#include <vcp_utils/vcp_error.h>

namespace vcp
{
namespace convert
{

//------------------------------------------------------------------------------------------------------cv::Vec
// Convert to cv::Vec
//-------------------------------------------------------------------------------------------------------------
template<typename T>
inline cv::Vec2d ToVec2d(const cv::Point_<T> &pt)
{
  return cv::Vec2d(static_cast<double>(pt.x), static_cast<double>(pt.y));
}


template<typename T>
inline cv::Vec2d ToVec2d(const cv::Mat &pt)
{
  const double *ptr = pt.ptr<T>();
  return cv::Vec2d(ptr[0], ptr[1]);
}


template<typename T>
inline cv::Vec3d ToVec3dUtil(const cv::Mat &pt)
{
  const T *ptr = pt.ptr<T>();
  return cv::Vec3d(ptr[0], ptr[1], ptr[2]);
}

inline cv::Vec3d ToVec3d(const cv::Mat &pt)
{
  switch (pt.depth())
  {
  case CV_8U:  return ToVec3dUtil<uchar>(pt);
  case CV_16U: return ToVec3dUtil<ushort>(pt);
  case CV_16S: return ToVec3dUtil<short>(pt);
  case CV_32S: return ToVec3dUtil<int>(pt);
  case CV_32F: return ToVec3dUtil<float>(pt);
  case CV_64F: return ToVec3dUtil<double>(pt);
  default:
      VCP_ERROR("Cannot convert cv::Mat to cv::Vec3d, wrong matrix element depth");
  }
}


/** @brief More generic than the ToVec2d/3d functions. */
template <typename T, int Dim>
cv::Vec<T,Dim> Mat2Vec(const cv::Mat &m) {
  VCP_CHECK(m.rows == 1 || m.cols == 1);
  VCP_CHECK(m.rows * m.cols == Dim);
  cv::Vec<T,Dim> v;
  for (int i = 0; i < Dim; ++i)
    v.val[i] = m.at<T>(i);
  return v;
}



//------------------------------------------------------------------------------------------------------cv::Mat
// Convert to cv::Mat
//-------------------------------------------------------------------------------------------------------------

template <typename T>
cv::Mat Scalar2Mat(T v)
{
  cv::Mat m = (cv::Mat_<T>(1,1) << v);
  return m;
}


template <typename T, int Dim>
cv::Mat Vec2Mat(const cv::Vec<T,Dim> &v)
{
  cv::Mat m = cv::Mat_<T>(Dim,1);
  for (int i = 0; i < Dim; ++i)
    m.at<T>(i) = v.val[i];
  return m;
}


template <typename T, template <typename...> class Container>
cv::Mat Container2Mat(const Container<T> &c)
{
  cv::Mat m = cv::Mat_<T>(c.size(),1);
  for (int i = 0; i < m.rows; ++i)
    m.at<T>(i) = c[i];
  return m;
}

template <typename T>
void Diagonalize(const cv::Mat &values, int repetitions, cv::Mat &output)
{
  VCP_CHECK(values.cols == 1);
  const int size = repetitions * values.rows;
  output = cv::Mat_<T>(size, size, static_cast<T>(0));
  for (int i = 0; i < size; ++i) {
    output.at<T>(i,i) = values.at<T>(i % values.rows);
  }
}


//----------------------------------------------------------------------------------------------------cv::Point
// Convert to cv::Point
//-------------------------------------------------------------------------------------------------------------

template<typename T>
inline cv::Point ToPoint(const cv::Vec<T, 2> &v)
{
  return cv::Point(static_cast<int>(v[0]), static_cast<int>(v[1]));
}


template<typename T>
inline cv::Point ToPoint(const cv::Point_<T> &pt)
{
  return cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y));
}


template<typename T>
inline cv::Point2f ToPoint2f(const cv::Vec<T, 2> &v)
{
  return cv::Point2f(static_cast<float>(v.val[0]), static_cast<float>(v.val[1]));
}


//-----------------------------------------------------------------------------------------------------cv::Size
// Convert to cv::Size
//-------------------------------------------------------------------------------------------------------------

template<typename T>
inline cv::Size2f ToSize2f(const cv::Size &s)
{
  return cv::Size2f(static_cast<float>(s.width), static_cast<float>(s.height));
}

} // namespace convert
} // namespace vcp

#endif // __VCP_MATH_OPENCV_CONVERSIONS_H__
