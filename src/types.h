#pragma once

namespace DRIVR {

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

struct DriverModelArgs {
   float des_v;
   float hdwy_t;
   float min_gap;
   float accel;
   float deccel;
   float p;
   float b_safe;
   float a_thr;
   float delta_b;
};

} // end DRIVR
