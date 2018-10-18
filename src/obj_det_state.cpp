#include "obj_det_state.h"

namespace DRIVR {

void VehicleStates::update_distance(
      Args args,
      Box box,
      int image_height,
      int image_width) {
    VehicleState newstate;
    if (get_distance_far_box_edge(box, image_width) < image_width / 10.0) {
        // otherwise, too off-center for this metric to work.
    }
}

float triangle_similarity_distance(Box box, float focal, float carW) {
    return 10;
}

float get_distance_far_box_edge(Box box, int im_w) {
    return 10;
}

State::State(int max_history, float ego_speed) {
    max_history_frames = max_history;
    ego_speed_mps = ego_speed;
}

void State::clear() {
    obj_key_to_state.clear();
}

VehicleStates State::get_state(std::string object_key) {
    return obj_key_to_state[object_key];
}

float State::get_ego_speed() {
    return ego_speed_mps;
}
void State::set_ego_speed(float speed_mps) {
    ego_speed_mps = speed_mps;
}

void State::update_distance(Args args, Box box, int image_height, int image_width, std::string object_key) {
    obj_key_to_state[object_key].update_distance(args, box, image_height, image_width);
}

} // end DRIVR
