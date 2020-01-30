#ifndef __VCP_BGM_BACKGROUND_MODEL_H__
#define __VCP_BGM_BACKGROUND_MODEL_H__

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
#else
    #include <opencv2/core.hpp>
#endif

namespace vcp
{
/** @brief Background models. */
namespace bgm
{

struct BgmParams
{
  std::string name;
  BgmParams(const std::string &name)
    : name(name)
  {}

  virtual ~BgmParams() = default;
};

/** @brief Base class of all background models. */
class BackgroundModel
{
public:
  /** @brief Destructor. */
  virtual ~BackgroundModel() {}


  /** @brief Init Initialize the background model with this image (assumed to be (mostly) static). */
  virtual bool Init(const cv::Mat &initial_bg_img) = 0;

  /**
    * @brief Reports the changes comparing the current frame to the background model.
    * @param[in] current_image current frame
    * @param[in] update_model boolean flag indicating whether to update the background model or not
    * @param[in] update_mask single channel mask to <b>update only the highlighted</b> regions.
    *            Watch out to really provide a proper mask (or don't use it at all).
    *            Use carefully, because +) (slow) objects won't merge into the BG but -) you may miss
    *            important updates.
    */
  virtual cv::Mat ReportChanges(const cv::Mat &current_image, bool update_model, const cv::Mat &update_mask=cv::Mat()) = 0;


  /** @brief Returns the background model (e.g. mean image). */
  virtual cv::Mat BackgroundImage() = 0;


  /**
   * @brief Change parametrization if needed.
   * @param params You have to use the correct parameter struct derived from BgmParams!
   * @return True upon success.
   */
  virtual bool UpdateParameters(const BgmParams &params) = 0;

protected:
  /** @brief Constructor. */
  BackgroundModel() {}
};
} // namespace bgm
} // namespace vcp

#endif // __VCP_BGM_BACKGROUND_MODEL_H__
