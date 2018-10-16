#pragma once
#include <unordered_map>
#include <string>
#include "vehicle.h"

class Scene {
    public:

    static std::unordered_map<std::string, Vehicle> id_to_vehicle;
    std::pair<float, float> ego_speed;
    std::pair<float, float> ego_accel;
    static const float lane_width;
    static const std::unordered_map<std::string, float> param_to_mean;
    static const std::unordered_map<std::string, float> param_to_variance;


    Scene(std::unordered_map<std::string, Vehicle> * start_id_to_vehicle,
          const std::pair<float, float> init_ego_speed=std::make_pair(0.0, 15.0),
          const std::pair<float, float> init_ego_accel=std::make_pair(0.0, 0.0));

};
