# TODO Use google unit tests
# Currently tests are run by running the python script

import time

import defaults
import argument_utils

def test_default_args():
    start = time.time()
    args = argument_utils.parse_args()
    assert args.focal == defaults.FOCAL
    assert args.cameraH == defaults.CAMERA_HEIGHT_m
    assert args.cameraMinAngle == defaults.CAMERA_MIN_VERTICAL_ANGLE_deg
    assert args.cameraMaxHorizAngle == defaults.CAMERA_MAX_HORIZONTAL_ANGLE_deg
    assert args.horizon == defaults.HORIZON

    assert args.use_gps == argument_utils.str2bool(defaults.USE_GPS)
    assert args.gps_source == defaults.GPS_SOURCE
    assert args.accept_speed == argument_utils.str2bool(defaults.ACCEPT_SPEED)

    assert args.tracker_refresh == defaults.TRACKER_FORCE_REFRESH_EVERY_N_FRAMES
    assert args.track == argument_utils.str2bool(defaults.DO_TRACKING)

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

    assert args.risk_H == defaults.RISK_HORIZON
    assert args.risk_step == defaults.RISK_STEP
    assert args.ttc_H == defaults.TTC_HORIZON
    assert args.ttc_step == defaults.TTC_STEP
    assert args.col_tol_x == defaults.COLLISION_TOL_X_m
    assert args.col_tol_y == defaults.COLLISION_TOL_Y_m
    assert args.embedded_risk == argument_utils.str2bool(defaults.EMBEDDED_RISK)
    assert args.max_risk_threads == defaults.RISK_THREADS

    print("Test completed successfully in {:.2} seconds".format(time.time() - start))

if __name__ == "__main__":
    test_default_args()
