#include "vehicle_state.h"

namespace DRIVR {

  VehicleState::VehicleState() {
    quantity_to_value = std::unordered_map<std::string, float>();
    seconds = 0.0;
  }
  VehicleState::~VehicleState() {
    quantity_to_value.clear();
  }
}
