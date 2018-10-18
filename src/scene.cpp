#include "scene.h"

namespace DRIVR {

Scene::Scene() {
  ego_speed = std::make_pair(0.0, 15.0);
  ego_accel = std::make_pair(0.0, 0.0);
}

Scene::Scene(std::unordered_map<std::string, Vehicle> * start_id_to_vehicle,
          std::pair<float, float> init_ego_speed,
          std::pair<float, float> init_ego_accel) {
        //id_to_vehicle = *start_id_to_vehicle;
        ego_speed = ego_speed;
        ego_accel = init_ego_accel;

        /*param_to_mean = std::unordered_map<std::string, float>({
            {"des_v", "15"},    // first IDM
            {"hdwy_t", 1.5},
            {"min_gap", 2.0},
            {"accel", 0.5},
            {"deccel", 3.0},            // below are mobil
            {"p", 0.2},
            {"b_safe", 3.0},
            {"a_thr", 0.2},
            {"delta_b", 0}
        });*/
}

void Scene::test() {
  std::cout << "Tested Scene." << std::endl;
}

} // end DRIVR
