#include "conversion/cv_core_conversion.h"
#include "conversion/np_cv_conversion.h"

#include <vcp_bgm/background_model.h>
//#include <vcp_bgm/approx_median_bgm.h>
//#include <vcp_bgm/blockbased_mean_bgm.h>
#include <vcp_bgm/normalized_rgb_bgm.h>

#include <memory>
#include <exception>


namespace vcp
{
namespace python
{
namespace bgm
{
class BackgroundModelWrapper
{
public:
  BackgroundModelWrapper() : bgm_(nullptr) {}

  // Disable copying
  BackgroundModelWrapper(const BackgroundModelWrapper&) = delete;
  BackgroundModelWrapper &operator=(const BackgroundModelWrapper&) = delete;

  virtual ~BackgroundModelWrapper()
  {
    bgm_.reset();
  }

  bool Init(const cv::Mat &image)
  {
    if (!bgm_)
      VCP_ERROR("You must initialize the background model first!");
    return bgm_->Init(image);
  }

  void InitNormalizedRgbBgm(bool report_as_binary=false, float binary_reporting_threshold=0.15f,
                            float update_rate=0.05f, float alpha=0.1f, float beta=1.0f,
                            const cv::Mat &image=cv::Mat())
  {
    bgm_ = std::move(vcp::bgm::CreateNormalizedRgbBgm(vcp::bgm::NormalizedRgbBgmParams(
                                                        report_as_binary, binary_reporting_threshold,
                                                        update_rate, alpha, beta)));
    if (!image.empty())
        bgm_->Init(image);
  }

  //TODO TODO TODO
//  void InitApproxMedianBgm(const cv::Mat &initial_background, float adaptation_step=5.0f, float fg_report_threshold=20.0f)
//  {
//    bgm_ = std::move(pvt::bgm::CreateApproxMedianBgm(initial_background, adaptation_step, fg_report_threshold));
//  }

//  void InitApproxMedianBgmRgb(const cv::Mat &initial_background, float adaptation_step=5.0f, float fg_report_threshold=20.0f)
//  {
//    bgm_ = std::move(pvt::bgm::CreateApproxMedianBgmRgb(initial_background, adaptation_step, fg_report_threshold));
//  }

//  void InitBlockBasedMeanBgm(const cv::Mat &initial_background, const cv::Size &block_size, double block_overlap=0.75, double update_rate=0.05, double fg_report_threshold=5.0)
//  {
//    bgm_ = std::move(pvt::bgm::CreateBlockBasedMeanBgm(initial_background, block_size, 1.0-block_overlap, update_rate, fg_report_threshold, 0)); // Use grayscale images
//  }


  cv::Mat ReportChanges(const cv::Mat &frame, bool update_bgm, const cv::Mat &update_mask)
  {
    if (!bgm_)
      VCP_ERROR("You must initialize the background model first!");
    return bgm_->ReportChanges(frame, update_bgm, update_mask);
  }

  cv::Mat GetBackgroundImage()
  {
    if (!bgm_)
      VCP_ERROR("You must initialize the background model first!");
    return bgm_->BackgroundImage();
  }

private:
  std::unique_ptr<vcp::bgm::BackgroundModel> bgm_;
};

} // namespace bgm
} // namespace python
} // namespace vcp


//-----------------------------------------------------------------------------
// Python module declarations


PYBIND11_MODULE(bgm, m)
{
  namespace pybgm = vcp::python::bgm;
  m.doc() = "Bindings for vcp::bgm, i.e. background models.";

  py::class_<pybgm::BackgroundModelWrapper>(m, "BackgroundModel")
      .def(py::init<>())
//      .def("init_approximate_median_bgm", &pybgm::BackgroundModelWrapper::InitApproxMedianBgm,
//           "Works on/converts to grayscale. TODO needs to be documented - check C++ doc of pvt_bgm for now!",
//           py::arg("initial_background"), py::arg("adaptation_step")=5.0f, py::arg("fg_report_threshold")=20.0f)
//      .def("init_approximate_median_3channel_bgm", &pybgm::BackgroundModelWrapper::InitApproxMedianBgmRgb,
//           "Works on 3 channels. TODO needs to be documented - check C++ doc of pvt_bgm for now!",
//           py::arg("initial_background"), py::arg("adaptation_step")=5.0f, py::arg("fg_report_threshold")=20.0f)
//      .def("init_block_mean_bgm", &pybgm::BackgroundModelWrapper::InitBlockBasedMeanBgm,
//           "TODO needs to be documented - check C++ doc of pvt_bgm for now!",
//           py::arg("initial_background"), py::arg("block_size")=cv::Size(32,32),
//           py::arg("block_overlap")=0.75, py::arg("update_rate")=0.05, py::arg("fg_report_threshold")=5.0)
      .def("normalized_rgb_bgm", &pybgm::BackgroundModelWrapper::InitNormalizedRgbBgm,
           "TODO needs to be documented - check C++ doc of pvt_bgm for now!",
           py::arg("report_as_binary")=false, py::arg("binary_reporting_threshold")=0.15f,
           py::arg("update_rate")=0.05f, py::arg("alpha")=0.1f, py::arg("beta")=1.0f,
           py::arg("initial_background")=cv::Mat())
      .def("init", &pybgm::BackgroundModelWrapper::Init,
           "Initializes the model (if you haven't done so, or need to\n"
           "re-initialize it).\n"
           ":param frame: Image as numpy ndarray.\n"
           ":return: True upon success.",
           py::arg("frame"))
      .def("report_changes", &pybgm::BackgroundModelWrapper::ReportChanges,
           "TODO needs to be documented - check C++ doc of pvt_bgm for now!\n"
           "\nTODOTODO"
           ":param frame: Current image as numpy ndarray.\n"
           ":param update_bgm: Boolean flag whether the model should be\n"
           "              updated or not.\n"
           ":param update_mask: If a numpy ndarray (single channel mask)\n"
           "              is provided, only the HIGHLIGHTED regions will\n"
           "              be updated.\n"
           ":return: The foreground mask (might contain floats, depending on\n"
           "              your chosen background model) as numpy.ndarray.",
           py::arg("frame"),
           py::arg("update_bgm")=false,
           py::arg("update_mask")=cv::Mat())
      .def("background_image", &pybgm::BackgroundModelWrapper::GetBackgroundImage,
           "Returns the internal representation of the background (e.g. mean image).");
}
