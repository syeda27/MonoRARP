#include "scene.h"
#include "obj_det_state.h"
#include "risk_predictor.h"
#include "vehicle.h"

int main(int argc, char const *argv[]) {
  DRIVR::Vehicle vehicle;
  vehicle.test();
  DRIVR::Scene scene;
  scene.test();
  DRIVR::State state = DRIVR::State();
  state.test();
  DRIVR::RiskPredictor risk = DRIVR::RiskPredictor();
  risk.test();
  return 0;
}
