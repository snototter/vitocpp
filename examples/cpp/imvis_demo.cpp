#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <vcp_imvis/pseudocolor.h>
#include <vcp_imvis/collage.h>
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/file_utils.h>
#include <vcp_imutils/opencv_compatibility.h>

#include <vcp_imvis/poses.h>

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
  vcp::imvis::pseudocolor::Colorize(img_32f, vcp::imvis::pseudocolor::ColorMap::Viridis, heatmap_a, 0, 1);
  cv::imshow("PseudoColor 32F [0,1]", heatmap_a);
  vcp::imvis::pseudocolor::Colorize(img, vcp::imvis::pseudocolor::ColorMap::Viridis, heatmap_b, 50, 200);
  cv::imshow("PseudoColor 8U [80,175]", heatmap_b);
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

  vcp::imvis::collage::Collage(collage_images, collage, 3, 10, cv::Size(), false, cv::Scalar(200,200,200));
  // Collage is 8U, but no images were converted
  cv::imshow("Collage3x2", collage);
  cv::waitKey(100);

  collage_images.clear();
  collage_images.push_back(img_32f);
  collage_images.push_back(img_32f);
  collage_images.push_back(img_32f);
  vcp::imvis::collage::Collage(collage_images, collage, 1, 5, cv::Size(), false, cv::Scalar(100,100,100));
  cv::imshow("Collage-vertical-32F", collage);
  std::cout << "Collage depth: " << collage.depth() << " CV_32F: " << CV_32F << ", CV_8U: " << CV_8U << std::endl;


  cv::waitKey();
  return 0;
}
