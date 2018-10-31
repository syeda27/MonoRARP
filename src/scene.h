#pragma once
#include <unordered_map>
#include <string>
#include <iostream>

#include "vehicle_state.h"
#include "types.h"

namespace DRIVR {

class Scene {
    public:

    std::unordered_map<std::string, VehicleState> id_to_vehiclestate;
    std::pair<float, float> ego_speed;
    std::pair<float, float> ego_accel;
    static const float lane_width;
    DriverModelArgs driver_model_means = DriverModelArgs({
      15.0, 1.5, 2.0, 0.5, 3.0, 0.2, 3.0, 0.2, 0.0
    });
    DriverModelArgs driver_model_vars = DriverModelArgs({
      10.0, 0.25, 0.2, 0.1, 0.1, 0.2, 0.1, 0.0
    });

    Scene();
    Scene(std::unordered_map<std::string, VehicleState> & vehicle_states,
          const std::pair<float, float> init_ego_speed=std::make_pair(0.0, 15.0),
          const std::pair<float, float> init_ego_accel=std::make_pair(0.0, 0.0));

    void test();
};

} // end DRIVR
