#include <iostream>

//#include <Eigen/Core>

#define VCP_VERBOSE_TIMING

#include <opencv2/highgui/highgui.hpp>
#include <vcp_imvis/pseudocolor.h>
#include <vcp_imvis/collage.h>
#include <vcp_utils/vcp_utils.h>
#include <vcp_utils/file_utils.h>
#include <vcp_imutils/opencv_compatibility.h>

#include <werkzeugkiste/timing/tictoc.h>
#include <viren2d/imagebuffer.h>
#include <viren2d/colormaps.h>
#include <viren2d/collage.h>
//#include <viren2d/drawing.h>

inline viren2d::ImageBufferType BufferType(const cv::Mat &mat) {
//  Eigen::Matrix3Xd m;
//  Eigen::Matrix<int, 3, 3, Eigen::RowMajor> m;
  switch(mat.depth()) {
    case CV_8U:
      return viren2d::ImageBufferType::UInt8;

    case CV_16U:
      return viren2d::ImageBufferType::UInt16;

    case CV_16S:
      return viren2d::ImageBufferType::Int16;

    case CV_32S:
      return viren2d::ImageBufferType::Int32;

    case CV_32F:
      return viren2d::ImageBufferType::Float;

    case CV_64F:
      return viren2d::ImageBufferType::Double;

    default:
      throw std::runtime_error("Conversion not yet supported/tested!");

  }
}

viren2d::ImageBuffer ToImageBuffer(cv::Mat &mat, int channels=-1) {
  viren2d::ImageBuffer buf;
  buf.CreateSharedBuffer(
        mat.data, mat.rows, mat.cols,
        mat.channels(), mat.step[0], mat.step[1],
        BufferType(mat));
  if (channels > 0 && channels != mat.channels()) {
    return buf.ToChannels(channels);
  } else {
    return buf;
  }
}

inline cv::Mat ToOpenCV(viren2d::ImageBuffer &buf, bool swap) {
  if (swap && buf.Channels() >= 3) {
    buf.SwapChannels(0, 2);
  }
  cv::Mat cv_buffer(buf.Height(), buf.Width(),
                    CV_MAKETYPE(CV_8U, buf.Channels()),
                    buf.MutableData(), buf.RowStride());
  //FIXME check buffer type!
  return cv_buffer;
}


int main(int argc, char **argv)
{
  VCP_UNUSED_VAR(argc);
  VCP_UNUSED_VAR(argv);

  const std::string kPeaksFile = "peaks.png";
  VCP_LOG_INFO("[Test] PseudoColor");

  VCP_CHECK(vcp::utils::file::Exists(kPeaksFile));

  cv::Mat img = cv::imread(kPeaksFile, COMPAT_CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat img_32f;
  img.convertTo(img_32f, CV_32FC1, 1.0/255.0);

  cv::Mat heatmap_a, heatmap_b;
  werkzeugkiste::timing::tic("colorize-vcp");
  vcp::imvis::pseudocolor::Colorize(img_32f, vcp::imvis::pseudocolor::ColorMap::Viridis, heatmap_a, 0, 1);
  werkzeugkiste::timing::toc_ms("colorize-vcp");
  cv::imshow("PseudoColor 32F [0,1]", heatmap_a);
  werkzeugkiste::timing::tic("colorize-vcp");
  vcp::imvis::pseudocolor::Colorize(img, vcp::imvis::pseudocolor::ColorMap::Viridis, heatmap_b, 50, 200);
  werkzeugkiste::timing::toc_ms("colorize-vcp");
  cv::imshow("PseudoColor 8U [50,200]", heatmap_b);

  // Versus cairo-based viren2d
  auto buf = ToImageBuffer(img_32f, 1);
  werkzeugkiste::timing::tic("colorize-viren2d");
  auto vvis = viren2d::ColorizeScaled(buf, viren2d::ColorMap::Viridis, 0, 1);
  werkzeugkiste::timing::toc_ms("colorize-viren2d");
  auto cv_buffer = ToOpenCV(vvis, true);
  cv::imshow("viren2d PseudoColor 8U [0,1]", cv_buffer);

  buf = ToImageBuffer(img, 1);
  werkzeugkiste::timing::tic("colorize-viren2d");
  vvis = viren2d::ColorizeScaled(buf, viren2d::ColorMap::Viridis, 50, 200);
  werkzeugkiste::timing::toc_ms("colorize-viren2d");
  cv_buffer = ToOpenCV(vvis, true);
  cv::imshow("viren2d PseudoColor 8U [50,200]", cv_buffer);
  cv::waitKey(100);

  vcp::imvis::pseudocolor::Colorize(img_32f, vcp::imvis::pseudocolor::ColorMap::Jet, heatmap_a);
  cv::imshow("Jet", heatmap_a);
  vcp::imvis::pseudocolor::Colorize(img_32f, vcp::imvis::pseudocolor::ColorMap::Thermal, heatmap_b);
  cv::imshow("Thermal", heatmap_b);
  cv::waitKey(100);


  // Collage
  const std::string kFlamingoFile = "flamingo.jpg";
  VCP_CHECK(vcp::utils::file::Exists(kFlamingoFile));
  cv::Mat lena = cv::imread(kFlamingoFile, COMPAT_CV_LOAD_IMAGE_COLOR);

  std::vector<cv::Mat> collage_images;
  collage_images.push_back(lena);
  collage_images.push_back(img);
  collage_images.push_back(img_32f);

  cv::Mat collage;
  vcp::imvis::collage::Collage(collage_images, collage, 2, 5, cv::Size(100, 100), false, cv::Scalar(255, 0, 0));
  // Even though convert_8U param is false, collage will still be CV_8U, because we mixed 32f and 8U images
  cv::imshow("Collage2x2-downsampled", collage);
  cv::waitKey(100);


  collage_images.pop_back();
  collage_images.push_back(heatmap_a);
  collage_images.push_back(img);
  collage_images.push_back(heatmap_b);

//  VCP_TIC;
  werkzeugkiste::timing::tic("collage-vcp");
  vcp::imvis::collage::Collage(collage_images, collage, 3, 10, cv::Size(), false, cv::Scalar(200,200,200));
  werkzeugkiste::timing::toc_ms("collage-vcp");
//  VCP_TOC("vcp Collage");
  // Collage is 8U, but no images were converted
  cv::imshow("Collage3x2", collage);
  cv::waitKey(100);

  std::vector<std::vector<viren2d::ImageBuffer>> cinputs(2);
  cinputs[0].push_back(ToImageBuffer(lena));
  cinputs[0].push_back(ToImageBuffer(img));
  cinputs[0].push_back(ToImageBuffer(heatmap_a));
  cinputs[1].push_back(ToImageBuffer(img));
  cinputs[1].push_back(ToImageBuffer(heatmap_b));
//  VCP_TIC;
  werkzeugkiste::timing::tic("collage-viren2d");
  vvis = viren2d::Collage(cinputs, {-1, -1}, viren2d::Anchor::Center, "white", 4, {10, 10});
  werkzeugkiste::timing::toc_ms("collage-viren2d");
//  VCP_TOC("viren2d Collage");
  cv_buffer = ToOpenCV(vvis, false);
  cv::imshow("Collage viren2d", cv_buffer);


//  collage_images.clear();
//  collage_images.push_back(img_32f);
//  collage_images.push_back(img_32f);
//  collage_images.push_back(img_32f);
//  vcp::imvis::collage::Collage(collage_images, collage, 1, 5, cv::Size(), false, cv::Scalar(100,100,100));
//  cv::imshow("Collage-vertical-32F", collage);
//  std::cout << "Collage depth: " << collage.depth() << " CV_32F: " << CV_32F << ", CV_8U: " << CV_8U << std::endl;


  cv::waitKey();
  return 0;
}
