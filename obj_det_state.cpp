#include <unordered_map>
#include <string>
#include <vector>


struct Args {
    float focal;
    float carW;
    float cameraH;
    float cameraMinAngle;
    float cameraMaxHorizAngle;
    float horizon;
};

struct Box {
    float left;
    float right;
    float top;
    float bottom;
};

class VehicleState {
    public:
    std::unordered_map<std::string, float> keys_to_quantities;
};

class VehicleStates {
    public:
    std::vector<VehicleState> states_over_time;

    void update_distance(Args args, Box box, int image_height, int image_width) {
        VehicleState newstate;
        if get_distance_far_box_edge(box, image_width) < image_width / 10.0 {
            // otherwise, too off-center for this metric to work.
            new_state.keys_to_quantities.insert(std::pair<std::string, float> triangle_d
                    (
                        "distance_y_t",
                        triangle_similarity_distance(box, args.focal, args.carW);
                    )
                )
        }
    }
};

float triangle_similarity_distance(Box box, float focal, float carW) {
    return 10;
}

float get_distance_far_box_edge(Box box, int im_w) {
    return 10;
}

class State {

    public: // TODO(djp42) make some things private
    int max_history_frames;
    std::unordered_map<std::string, VehicleStates> obj_key_to_state;
    // object key : list of vehicle state over time
    //              vehicle state = map of quantity key to value, aka "distance_x" : 19.2
    float ego_speed_mps;

    State(int max_history = 100, float ego_speed = 0) {
        max_history_frames = max_history;
        ego_speed_mps = ego_speed;
    }

    void clear() {
        states.clear();
    }

    std::vector<std::unordered_map<std::string, float>> get_state(std::string object_key) {
        return obj_key_to_state[object_key];
    }

    float get_ego_speed() {
        return ego_speed_mps;
    }
    void set_ego_speed(float speed_mps) {
        ego_speed_mps = speed_mps;
    }

    void update_distance(Args args, Box box, int image_height, int image_width, std::string object_key) {
        obj_key_to_state[object_key].update_distance(args, box, image_height, image_width);
    }


};
