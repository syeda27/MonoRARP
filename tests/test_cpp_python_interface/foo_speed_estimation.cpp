#include "../../speed_estimator_utils/speed_estimation.h"

extern "C" {
    Speed_estimator* Speed_estimator_new() { return new Speed_estimator(); }
    void Speed_estimator_hello_world(Speed_estimator* speed_estimator) {
        speed_estimator->hello_world();
    }
}
