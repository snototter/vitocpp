#include "conversion/cv_core_conversion.h"
#include "conversion/np_cv_conversion.h"

#include <vcp_bgm/background_model.h>
#include <vcp_bgm/approx_median_bgm.h>
#include <vcp_bgm/blockbased_mean_bgm.h>
#include <vcp_bgm/normalized_rgb_bgm.h>
#include <vcp_bgm/mog_bgm.h>
#include <vcp_utils/string_utils.h>

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

  std::string Name() const
  {
    if (!bgm_)
      VCP_ERROR("You must initialize the background model first!");
    return bgm_->Parameters().model_name;
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



  void InitApproxMedianBgm(float adaptation_step=5.0f, float fg_report_threshold=20.0f,
                           bool median_on_grayscale=true,
                           const cv::Mat &image=cv::Mat())
  {
    const auto p = vcp::bgm::ApproxMedianBgmParams(adaptation_step, fg_report_threshold);
    if (median_on_grayscale)
      bgm_ = std::move(vcp::bgm::CreateApproxMedianBgmGrayscale(p));
    else
      bgm_ = std::move(vcp::bgm::CreateApproxMedianBgmColor(p));

    if (!image.empty())
        bgm_->Init(image);
  }

  void InitBlockBasedMeanBgm(const cv::Size &block_size, float block_overlap=0.75,
                             float update_rate=0.05, float fg_report_threshold=5.0,
                             const std::string &channel="grayscale",
                             const cv::Mat &image=cv::Mat())
  {
    const std::string lc = vcp::utils::string::Lower(channel);
    vcp::bgm::BlockBasedMeanBgmChannel channel_type;
    if (lc.compare("grayscale") == 0)
      channel_type = vcp::bgm::BlockBasedMeanBgmChannel::GRAYSCALE;
    else if (lc.compare("saturation") == 0)
      channel_type = vcp::bgm::BlockBasedMeanBgmChannel::SATURATION;
    else
      VCP_ERROR("Invalid channel type '" << channel << "' for BlockBasedMeanBgm.");

    bgm_ = std::move(vcp::bgm::CreateBlockBasedMeanBgm(vcp::bgm::BlockBasedMeanBgmParams(
                                               block_size,
                                               1.0f-block_overlap,
                                               update_rate,
                                               fg_report_threshold, channel_type)));
    if (!image.empty())
        bgm_->Init(image);
  }

  void InitGaussianMixtureBgm(int history,
                              bool detect_shadows,
                              double variance_threshold,
                              double complexity_reduction_threshold,
                              const cv::Mat &image=cv::Mat())
  {
    bgm_ = std::move(vcp::bgm::CreateMixtureOfGaussiansBgm(vcp::bgm::MixtureOfGaussiansBgmParams(
                                                             history, detect_shadows,
                                                             variance_threshold, complexity_reduction_threshold)));
    if (!image.empty())
        bgm_->Init(image);
  }

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
      .def("approximate_median_bgm", &pybgm::BackgroundModelWrapper::InitApproxMedianBgm,
           "Approximate median, see McFarlane and Schofield, \"Segmentation\n"
           "and Tracking of Piglets in Images\", MVA 8(3), '95.\n\n"
           ":param adaptation_step: float increment for the median approximation.\n"
           ":param fg_report_threshold: float threshold which changes are\n"
           "                  considered foreground.\n"
           ":param median_on_grayscale: If True, inputs will be converted to\n"
           "                  grayscale. Otherwise, the median will be\n"
           "                  approximated for each channel separately.\n"
           ":param image: numpy ndarray to be used as initial background image,\n"
           "                  leave empty if you want to initialize later on.",
           py::arg("adaptation_step")=5.0f,
           py::arg("fg_report_threshold")=20.0f,
           py::arg("median_on_grayscale")=true,
           py::arg("image")=cv::Mat())
      .def("block_mean_bgm", &pybgm::BackgroundModelWrapper::InitBlockBasedMeanBgm,
           "A more robust mean background model which computes patchwise\n"
           "average.\n\n"
           ":param block_size: (w, h) of each block/patch.\n"
           ":param block_overlap: float in [0, 1] how much the blocks\n"
           "                    should overlap.\n"
           ":param update_rate: float, how fast should the model update?\n"
           ":param fg_report_threshold: float\n"
           ":param channel: string, select which channel to compute the\n"
           "                    average on, e.g. 'grayscale' (converts\n"
           "                    input images to grayscale automatically)\n"
           "                    or 'saturation' (converts to HSV and averages\n"
           "                    on the S channel.\n"
           ":param image: numpy ndarray to be used as initial background image,\n"
           "                    leave empty if you want to initialize later on.",
           py::arg("block_size")=cv::Size(32,32),
           py::arg("block_overlap")=0.75,
           py::arg("update_rate")=0.05,
           py::arg("fg_report_threshold")=5.0,
           py::arg("channel")="grayscale",
           py::arg("image")=cv::Mat())
      .def("gaussian_mixture_bgm", &pybgm::BackgroundModelWrapper::InitGaussianMixtureBgm,
           "Gaussian Mixture-based background model, see Zivkovic & van der Heijden,\n"
           "\"Efficient adaptive density estimation per image pixel for the task of\n"
           "background subtraction\", PRL 27(7), '06.\n\n"
           ":param history: Number of previous frames that affect the model.\n"
           ":param detect_shadows: Bool, should shadows be detected?\n"
           ":param var_thresh: float, Variance threshold for the pixel-model match.\n"
           ":param comp_thresh: float, Complexity reduction threshold.\n",
           ":param image: numpy ndarray to be used as initial background image,\n"
           "                    leave empty if you want to initialize later on.",
           py::arg("history")=500,
           py::arg("detect_shadows")=true,
           py::arg("var_thresh")=16.0,
           py::arg("comp_thresh")=0.05,
           py::arg("image")=cv::Mat())
      .def("normalized_rgb_bgm", &pybgm::BackgroundModelWrapper::InitNormalizedRgbBgm,
           "Normalized RGB, see Reinbacher et al. \"Fast variational\n"
           "multi-view segmentation through backprojection of spatial\n"
           "constraints\", IVC 30(2012).\n\n"
           ":param report_as_binary: If true, mask will be CV_8U and thresholded,\n"
           "                  otherwise CV_32F.\n"
           ":param binary_reporting_threshold: If report_as_binary is True, then the change\n"
           "                  mask will be thresholded at this value.\n"
           ":param update_rate: float How fast the model should adjust.\n"
           ":params alpha, beta: See paper.\n"
           ":param image: numpy ndarray to be used as initial background image,\n"
           "                  leave empty if you want to initialize later on.",
           py::arg("report_as_binary")=false, py::arg("binary_reporting_threshold")=0.15f,
           py::arg("update_rate")=0.05f, py::arg("alpha")=0.1f, py::arg("beta")=1.0f,
           py::arg("image")=cv::Mat())
      .def("name", &pybgm::BackgroundModelWrapper::Name,
           "Returns the name of the underlying background model.")
      .def("init", &pybgm::BackgroundModelWrapper::Init,
           "Initializes the model (if you haven't done so, or need to\n"
           "re-initialize it).\n"
           ":param frame: Image as numpy ndarray.\n"
           ":return: True upon success.",
           py::arg("frame"))
      .def("report_changes", &pybgm::BackgroundModelWrapper::ReportChanges,
           "Applies the background model on the current 'frame' and returns\n"
           "a numpy ndarray masking the foreground/changed regions.\n\n"
           ":param frame: Current image as numpy ndarray.\n"
           ":param update_bgm: Boolean flag whether the model should be\n"
           "              updated or not.\n"
           ":param update_mask: If a numpy ndarray (single channel mask)\n"
           "              is provided, only the **HIGHLIGHTED** regions will\n"
           "              be updated. This parameter may be ignored if the\n"
           "              underlying model doesn't support masked updates.\n"
           "              In such cases, an error message will be logged.\n"
           ":return: The foreground mask (might contain floats, depending on\n"
           "              your chosen background model) as numpy.ndarray.",
           py::arg("frame"),
           py::arg("update_bgm")=false,
           py::arg("update_mask")=cv::Mat())
      .def("background_image", &pybgm::BackgroundModelWrapper::GetBackgroundImage,
           "Returns the internal representation of the background (e.g. mean image).");
}
