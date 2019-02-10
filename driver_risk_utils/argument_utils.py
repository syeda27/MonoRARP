import argparse
try:
    from driver_risk_utils import defaults
except ImportError:  # For testing
    import defaults

import os

# for argparsing
def str2bool(v):
    return v.lower() == "true"

def add_camera_args(parser):
    parser.add_argument("--focal", type=int, default=defaults.FOCAL)
    parser.add_argument("--cameraH", type=float, default=defaults.CAMERA_HEIGHT_m)
    parser.add_argument("--cameraMinAngle", type=float,
            default=defaults.CAMERA_MIN_VERTICAL_ANGLE_deg)
    parser.add_argument("--cameraMaxHorizAngle", type=float,
            default=defaults.CAMERA_MAX_HORIZONTAL_ANGLE_deg)
    parser.add_argument("--horizon", type=float,
            default=defaults.HORIZON)
    parser.add_argument("--resolution_h", type=int, default=defaults.RESOLUTION_H)
    parser.add_argument("--resolution_w", type=int, default=defaults.RESOLUTION_W)

def add_speed_args(parser):
    parser.add_argument("--use_gps", type=str2bool,
            default=defaults.USE_GPS)
    parser.add_argument("--gps_source", type=str,
            default=defaults.GPS_SOURCE)
    parser.add_argument("--accept_speed", type=str2bool,
            default=defaults.ACCEPT_SPEED)
    parser.add_argument("--lane_based_speed", type=str2bool,
            default=defaults.LANE_BASED_SPEED)

def add_tracker_args(parser):
    parser.add_argument("--tracker_refresh", type=int,
            default=defaults.TRACKER_FORCE_REFRESH_EVERY_N_FRAMES)
    parser.add_argument("--track", type=str2bool,
            default=defaults.DO_TRACKING)
    parser.add_argument("--tracker_type", type=str, default=defaults.TRACKER_TYPE)
    parser.add_argument("--tracker_hold", type=int, default=defaults.TRACKER_HOLD)
    parser.add_argument("--num_trackers", type=int, default=defaults.NUM_TRACKERS)
    parser.add_argument("--num_tracker_particles", type=int, default=defaults.NUM_TRACKER_PARTICLES)

def add_input_output_args(parser):
    parser.add_argument("--source", type=str,
            default=defaults.SOURCE)
    parser.add_argument("--save", type=str2bool,
            default=defaults.SAVE)
    parser.add_argument("--save_path", type=str,
                    default=defaults.SAVE_PATH)

def add_lane_detection_args(parser):
    parser.add_argument("--detect_lanes", type=str2bool,
            default=defaults.DETECT_LANES)

def add_general_args(parser):
    parser.add_argument("--queue", type=int, default=defaults.QUEUE)
    parser.add_argument("--det_thresh", type=float,
            default=defaults.DETECTION_THRESHOLD)
    parser.add_argument("--model", type=str,
            default=defaults.MODEL)
    parser.add_argument("--labels", type=str,
            default=defaults.LABELS)

    parser.add_argument("--carW", type=float,
            default=defaults.CAR_WIDTH_m)
    parser.add_argument("--calc_risk_n", type=int,
            default=defaults.CALC_RISK_EVERY_N_FRAMES)
    # 1 to update every frame
    parser.add_argument("--device", type=str,
            default=defaults.DEVICE)
    # if we do not want to show the speed, set to false
    parser.add_argument("--show_speed", type=str2bool, default=defaults.SHOW_SPEED)

def add_thread_args(parser):
    parser.add_argument("--threaded_runner", type=str,
            default=defaults.THREADED_RUNNER)
    parser.add_argument("--thread_queue_size", type=int,
            default=defaults.THREAD_QUEUE_SIZE)
    parser.add_argument("--thread_wait_time", type=float,
            default=defaults.THREAD_WAIT_TIME)
    parser.add_argument("--thread_max_wait", type=float,
            default=defaults.THREAD_MAX_WAIT)
    parser.add_argument("--max_risk_threads", type=int,
            default=defaults.RISK_THREADS)

def add_risk_args(parser):
    parser.add_argument("--n_risk_sims", type=int, default=defaults.RISK_N_SIMS)
    parser.add_argument("--risk_H", type=float, default=defaults.RISK_HORIZON)
    parser.add_argument("--risk_step", type=float, default=defaults.RISK_STEP)
    parser.add_argument("--ttc_H", type=float, default=defaults.TTC_HORIZON)
    parser.add_argument("--ttc_step", type=float, default=defaults.TTC_STEP)
    parser.add_argument("--col_tol_x", type=float, default=defaults.COLLISION_TOL_X_m)
    parser.add_argument("--col_tol_y", type=float, default=defaults.COLLISION_TOL_Y_m)
    parser.add_argument("--embedded_risk", type=str2bool, default=defaults.EMBEDDED_RISK)
    parser.add_argument("--risk_type", type=str, default=defaults.RISK_TYPE)

def add_offline_args(parser):
    parser.add_argument("--offline", type=str2bool, default=defaults.OFFLINE)
    parser.add_argument("--results_save_path", type=str, default=defaults.RESULTS_SAVE_PATH)
    parser.add_argument("--overwrite_saves", type=str2bool, default=defaults.OVERWRITE_SAVES)

def add_eval_load_flags(parser):
    parser.add_argument("--prior_results_path", type=str, default=defaults.PRIOR_RESULTS_PATH)
    parser.add_argument("--L_EGO_SPEED", type=str2bool, default=defaults.MODULE_LOAD)
    parser.add_argument("--L_OBJ_DETECTOR", type=str2bool, default=defaults.MODULE_LOAD)
    parser.add_argument("--L_TRACKER", type=str2bool, default=defaults.MODULE_LOAD)
    parser.add_argument("--L_STATE", type=str2bool, default=defaults.MODULE_LOAD)
    parser.add_argument("--L_RISK", type=str2bool, default=defaults.MODULE_LOAD)
    # The reason we allow loading the risk is in case we want to just test speed
    # and other similar debugging purposes. Probably won't be useful to the average user.

"""
parse_args creates a parser object, adds the arguments and default arguments,
  and concludes by returning the parsed arugments.
  Additionally, it performs some simple initial checks

Returns:
  args: an object with all of the function arguments.
"""
def parse_args():
    parser = argparse.ArgumentParser()
    add_camera_args(parser)
    add_tracker_args(parser)
    add_speed_args(parser)
    add_input_output_args(parser)
    add_lane_detection_args(parser)
    add_general_args(parser)
    add_risk_args(parser)
    add_thread_args(parser)
    add_offline_args(parser)
    add_eval_load_flags(parser)
    args = parser.parse_args()

    return do_arg_checks(args)

"""
do_arg_checks:
    checks some of the args as desired.
Returns:
    args: If all checks succeed
Raises:
    ValueError:
        - if there is an invalid horizon argument
        - Invalid use of the KCF tracker (it must refresh)
        - Invalid path to prior results
        - Indicate we want to load prior results using threaded runner
            - (reason is just that it is currently UNSUPPORTED)
"""
def do_arg_checks(args):
    if args.source == "0" or args.source == "1":
        args.source = int(args.source)
    if args.horizon < 0 or args.horizon > 1:
        raise ValueError("ERROR: Must pass in a relative horizon position, between 0 and 1.")
    if args.track and args.tracker_type == "KCF" and args.tracker_refresh == 1:
        raise ValueError("ERROR: KCF tracker must refresh.")
    if args.track and args.tracker_type == "KCF" and args.threaded_runner == 'B':
        print("WARN: KCF tracker not 100% compatible with threaded runner B, and will drop frames.")
    if args.risk_type.lower() == "ttc" and args.offline == False:
        print("INFO: TTC risk selected, forcing no speed estimator.")
        args.use_gps = False
        args.accept_speed = False
        args.lane_based_speed = False
        args.embedded_risk = False
        args.max_risk_threads = 1
        args.show_speed = False
    if args.prior_results_path is not defaults.PRIOR_RESULTS_PATH:
        if not os.path.exists(args.prior_results_path):
            raise ValueError("ERROR: Prior results path does not exist.")
        if args.threaded_runner.lower() != "none":
            raise ValueError("ERROR: Threaded runner does not currently support loading prior results.")
        if not (args.L_EGO_SPEED or
                args.L_OBJ_DETECTOR or
                args.L_TRACKER or
                args.L_STATE or
                args.L_RISK):
            print("INFO: Indicated we wanted to load prior results, but no modules were flagged to load.")
    else:
        if (args.L_EGO_SPEED or
                args.L_OBJ_DETECTOR or
                args.L_TRACKER or
                args.L_STATE or
                args.L_RISK):
            raise ValueError("ERROR: Indicated we wanted to load a module, but no path was given.")
    return args
