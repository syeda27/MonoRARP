#pragma once
#include <unordered_map>

namespace DRIVR {

class VehicleState {
  public:
  std::unordered_map<std::string, float> quantity_to_value;
  float seconds;
  VehicleState();
  ~VehicleState();
};


} // end DRIVR
