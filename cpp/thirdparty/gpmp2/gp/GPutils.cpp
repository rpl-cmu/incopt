/**
 *  @file  GPutils.cpp
 *  @brief GP utils, calculation of Qc, Q, Lamda matrices etc.
 *  @author Xinyan Yan, Jing Dong
 *  @date Qct 26, 2015
 **/

#include <cpp/thirdparty/gpmp2/gp/GPutils.h>

using namespace gtsam;


namespace incopt {

/* ************************************************************************** */
Matrix getQc(const SharedNoiseModel Qc_model) {
  noiseModel::Gaussian *Gassian_model =
      dynamic_cast<noiseModel::Gaussian*>(Qc_model.get());
  return (Gassian_model->R().transpose() * Gassian_model->R()).inverse();
}

} // namespace incopt

