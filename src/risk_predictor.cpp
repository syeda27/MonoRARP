/*
 * I do not think we actually need this to be in cpp, python works fine.
 * We should focus on putting the slower stuff to cpp first.
 * Tracker and scene.simulate are the slow ones.
 */

#include "risk_predictor.h"

namespace DRIVR {

RiskPredictor::RiskPredictor() {
  horizon_s = 10.0;
  step_s = 0.1;
  col_tol_x_m = 2.0;
  col_tol_y_m = 2.0;
  ttc_tol_s = 1.0;
}

RiskPredictor::RiskPredictor(
      float H,
      float step,
      float col_tol_x,
      float col_tol_y,
      float ttc_tol) {
    horizon_s = H;
    step_s = step;
    col_tol_x_m = col_tol_x;
    col_tol_y_m = col_tol_y;
    ttc_tol_s = ttc_tol;
}

float RiskPredictor::getRisk(State* state, std::string risk_type, int n_sims, bool verbose) {
    return 10;
}

void RiskPredictor::test() {
    std::cout << "Tested Risk." << std::endl;
}

} // end DRIVR
