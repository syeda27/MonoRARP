# TODO Use google unit tests
# Currently tests are run by running the python script

import time
import sys
sys.path.append("..")
sys.path.append("../driver_risk_utils/")
import defaults
import argument_utils

def test_default_args():
    start = time.time()
    args = argument_utils.parse_args()

    assert argument_utils.str2bool("False") == False
    assert argument_utils.str2bool("false") == False
    assert argument_utils.str2bool("true") == True
    assert argument_utils.str2bool("True") == True

    assert args.focal == defaults.FOCAL
    assert args.cameraH == defaults.CAMERA_HEIGHT_m
    assert args.cameraMinAngle == defaults.CAMERA_MIN_VERTICAL_ANGLE_deg
    assert args.cameraMaxHorizAngle == defaults.CAMERA_MAX_HORIZONTAL_ANGLE_deg
    assert args.horizon == defaults.HORIZON
    assert args.resolution_h == defaults.RESOLUTION_H
    assert args.resolution_w == defaults.RESOLUTION_W

    assert args.use_gps == argument_utils.str2bool(defaults.USE_GPS)
    assert args.gps_source == defaults.GPS_SOURCE
    assert args.accept_speed == argument_utils.str2bool(defaults.ACCEPT_SPEED)
    assert args.lane_based_speed == argument_utils.str2bool(defaults.LANE_BASED_SPEED)


    assert args.tracker_refresh == defaults.TRACKER_FORCE_REFRESH_EVERY_N_FRAMES
    assert args.track == argument_utils.str2bool(defaults.DO_TRACKING)
    assert args.tracker_type == defaults.TRACKER_TYPE
    assert args.tracker_hold == defaults.TRACKER_HOLD
    assert args.num_trackers == defaults.NUM_TRACKERS
    assert args.num_tracker_particles == defaults.NUM_TRACKER_PARTICLES

    assert str(args.source) == defaults.SOURCE
    assert args.save == argument_utils.str2bool(defaults.SAVE)
    assert args.save_path == defaults.SAVE_PATH

    assert args.queue == defaults.QUEUE
    assert args.det_thresh == defaults.DETECTION_THRESHOLD
    assert args.model == defaults.MODEL
    assert args.labels == defaults.LABELS
    assert args.carW == defaults.CAR_WIDTH_m
    assert args.calc_risk_n == defaults.CALC_RISK_EVERY_N_FRAMES
    assert args.device == defaults.DEVICE

    assert args.threaded_runner == defaults.THREADED_RUNNER
    assert args.thread_queue_size == defaults.THREAD_QUEUE_SIZE
    assert args.thread_wait_time == defaults.THREAD_WAIT_TIME
    assert args.thread_max_wait == defaults.THREAD_MAX_WAIT
    assert args.max_risk_threads == defaults.RISK_THREADS

    assert args.n_risk_sims == defaults.RISK_N_SIMS
    assert args.risk_H == defaults.RISK_HORIZON
    assert args.risk_step == defaults.RISK_STEP
    assert args.ttc_H == defaults.TTC_HORIZON
    assert args.ttc_step == defaults.TTC_STEP
    assert args.col_tol_x == defaults.COLLISION_TOL_X_m
    assert args.col_tol_y == defaults.COLLISION_TOL_Y_m
    assert args.embedded_risk == argument_utils.str2bool(defaults.EMBEDDED_RISK)

    assert args.detect_lanes == argument_utils.str2bool(defaults.DETECT_LANES)

    print("Test completed successfully in {:.2} seconds".format(time.time() - start))

if __name__ == "__main__":
    test_default_args()
