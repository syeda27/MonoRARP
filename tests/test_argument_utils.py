# TODO Use google unit tests
# Currently tests are run by running the python script

import time
import sys
sys.path.append("..")
sys.path.append("../driver_risk_utils/")
import defaults
import argument_utils
import unittest

class TestArguments(unittest.TestCase):
    def test_str2bool(self):
        self.assertEqual(argument_utils.str2bool("False"), False)
        self.assertEqual(argument_utils.str2bool("false"), False)
        self.assertEqual(argument_utils.str2bool("true"), True)
        self.assertEqual(argument_utils.str2bool("True"), True)
        self.assertEqual(argument_utils.str2bool("Anything ELSE"), False)

    def test_default_camera(self):
        args = argument_utils.parse_args()
        self.assertEqual(args.focal, defaults.FOCAL)
        self.assertEqual(args.cameraH, defaults.CAMERA_HEIGHT_m)
        self.assertEqual(args.cameraMinAngle, defaults.CAMERA_MIN_VERTICAL_ANGLE_deg)
        self.assertEqual(args.cameraMaxHorizAngle, defaults.CAMERA_MAX_HORIZONTAL_ANGLE_deg)
        self.assertEqual(args.horizon, defaults.HORIZON)
        self.assertEqual(args.resolution_h, defaults.RESOLUTION_H)
        self.assertEqual(args.resolution_w, defaults.RESOLUTION_W)

    def test_default_speed(self):
        args = argument_utils.parse_args()
        self.assertEqual(args.use_gps, argument_utils.str2bool(defaults.USE_GPS))
        self.assertEqual(args.gps_source, defaults.GPS_SOURCE)
        self.assertEqual(args.accept_speed, argument_utils.str2bool(defaults.ACCEPT_SPEED))
        self.assertEqual(args.lane_based_speed, argument_utils.str2bool(defaults.LANE_BASED_SPEED))

    def test_default_tracker(self):
        args = argument_utils.parse_args()
        self.assertEqual(args.tracker_refresh, defaults.TRACKER_FORCE_REFRESH_EVERY_N_FRAMES)
        self.assertEqual(args.track, argument_utils.str2bool(defaults.DO_TRACKING))
        self.assertEqual(args.tracker_type, defaults.TRACKER_TYPE)
        self.assertEqual(args.tracker_hold, defaults.TRACKER_HOLD)
        self.assertEqual(args.num_trackers, defaults.NUM_TRACKERS)
        self.assertEqual(args.num_tracker_particles, defaults.NUM_TRACKER_PARTICLES)

    def test_default_io(self):
        args = argument_utils.parse_args()
        self.assertEqual(str(args.source), defaults.SOURCE)
        self.assertEqual(args.save, argument_utils.str2bool(defaults.SAVE))
        self.assertEqual(args.save_path, defaults.SAVE_PATH)

    def test_default_lane(self):
        args = argument_utils.parse_args()
        self.assertEqual(args.detect_lanes, argument_utils.str2bool(defaults.DETECT_LANES))

    def test_default_general(self):
        args = argument_utils.parse_args()
        self.assertEqual(args.queue, defaults.QUEUE)
        self.assertEqual(args.det_thresh, defaults.DETECTION_THRESHOLD)
        self.assertEqual(args.model, defaults.MODEL)
        self.assertEqual(args.labels, defaults.LABELS)
        self.assertEqual(args.carW, defaults.CAR_WIDTH_m)
        self.assertEqual(args.calc_risk_n, defaults.CALC_RISK_EVERY_N_FRAMES)
        self.assertEqual(args.device, defaults.DEVICE)

    def test_default_thread(self):
        args = argument_utils.parse_args()
        self.assertEqual(args.threaded_runner, defaults.THREADED_RUNNER)
        self.assertEqual(args.thread_queue_size, defaults.THREAD_QUEUE_SIZE)
        self.assertEqual(args.thread_wait_time, defaults.THREAD_WAIT_TIME)
        self.assertEqual(args.thread_max_wait, defaults.THREAD_MAX_WAIT)
        self.assertEqual(args.max_risk_threads, defaults.RISK_THREADS)

    def test_default_risk(self):
        args = argument_utils.parse_args()
        self.assertEqual(args.n_risk_sims, defaults.RISK_N_SIMS)
        self.assertEqual(args.risk_H, defaults.RISK_HORIZON)
        self.assertEqual(args.risk_step, defaults.RISK_STEP)
        self.assertEqual(args.ttc_H, defaults.TTC_HORIZON)
        self.assertEqual(args.ttc_step, defaults.TTC_STEP)
        self.assertEqual(args.col_tol_x, defaults.COLLISION_TOL_X_m)
        self.assertEqual(args.col_tol_y, defaults.COLLISION_TOL_Y_m)
        self.assertEqual(args.embedded_risk, argument_utils.str2bool(defaults.EMBEDDED_RISK))
        self.assertEqual(args.risk_type, defaults.RISK_TYPE)

    def test_default_offline(self):
        args = argument_utils.parse_args()
        self.assertEqual(args.offline, argument_utils.str2bool(defaults.OFFLINE))
        self.assertEqual(args.results_save_path, defaults.RESULTS_SAVE_PATH)
        self.assertEqual(args.overwrite_saves, argument_utils.str2bool(defaults.OVERWRITE_SAVES))

if __name__ == "__main__":
    unittest.main()
