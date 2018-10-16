#include <iostream>
#include <unordered_map>
#include <string>
#include "vehicle.h"

class Scene {
    public:
    std::unordered_map<std::string, Vehicle> id_to_vehicle;


    Scene(std::unordered_map<std::string, Vehicle> start_id_to_vehicle,
          std::pair<float, float> init_ego_speed=std::make_pair(0.0, 15.0),
          std::pair<float, float> init_ego_accel=std::make_pair(0.0, 0.0)) {
        id_to_vehicle = start_id_to_vehicle;
        ego_speed = ego_speed;
        ego_accel = init_ego_accel;
    }

    void clear_scene() {
        id_to_vehicle.clear();
    }

    private:
    std::pair<float, float> ego_speed;
    std::pair<float, float> ego_accel;
    float lane_width;
    std::unordered_map<std::string, float> param_to_mean;
    std::unordered_map<std::string, float> param_to_variance;

};

int main() {
    std::cout << "hello world." << std::endl;
    return 0;
}
