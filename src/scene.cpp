#include "scene.h"

namespace DRIVR {

Scene::Scene() {
  ego_speed = std::make_pair(0.0, 15.0);
  ego_accel = std::make_pair(0.0, 0.0);
  driver_model_means.des_v = ego_speed.second;
}

Scene::Scene(std::unordered_map<std::string, VehicleState> & vehicle_states,
          std::pair<float, float> init_ego_speed,
          std::pair<float, float> init_ego_accel) {
        id_to_vehiclestate = vehicle_states;
        ego_speed = init_ego_speed;
        ego_accel = init_ego_accel;
        driver_model_means.des_v = ego_speed.second;
}

void Scene::test() {
  std::cout << "Tested Scene." << std::endl;
}

} // end DRIVR
