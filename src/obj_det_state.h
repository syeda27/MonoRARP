#pragma once
#include <unordered_map>
#include <string>
#include <vector>
#include <iostream>
#include "types.h"
#include "vehicle_state.h"

namespace DRIVR {

class VehicleStates {
    public:
    std::vector<VehicleState> states_over_time;
    void update_distance(DRIVR::Args args, DRIVR::Box box, int image_height, int image_width);
};

float triangle_similarity_distance(Box box, float focal, float carW);

float get_distance_far_box_edge(Box box, int im_w);

class State {

    public: // TODO(djp42) make some things private
    int max_history_frames;
    std::unordered_map<std::string, VehicleStates> obj_key_to_state;
    // object key : list of vehicle state over time
    //              vehicle state = map of quantity key to value, aka "distance_x" : 19.2
    float ego_speed_mps;

    State(int max_history=100, float ego_speed=0);

    void clear();

    VehicleStates get_state(std::string object_key);

    float get_ego_speed();
    void set_ego_speed(float speed_mps);

    void update_distance(Args args, Box box, int image_height, int image_width, std::string object_key);

    void test();
};

} // end DRIVR
