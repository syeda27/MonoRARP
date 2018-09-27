import argparse
import os

# for argparsing
def str2bool(v):
    return v.lower() == "true"

def add_camera_args(parser):
    parser.add_argument("--focal", type=int, default=1000)
    parser.add_argument("--cameraH", type=float, default=1.0)
    parser.add_argument("--cameraMinAngle", type=float, default=55.0) #degrees
    parser.add_argument("--cameraMaxHorizAngle", type=float, default=90.0) #degrees
    parser.add_argument("--horizon", type=float, default=0.5) # 0 - 1

def add_general_args(parser):
    parser.add_argument("--queue", type=int, default=1)
    parser.add_argument("--det_thresh", type=float, default=0.5)
    parser.add_argument("--model", type=str, \
                    default='faster_rcnn_resnet101_kitti_2018_01_28')
    parser.add_argument("--labels", type=str, \
                    default=os.path.join('data', 'kitti_label_map.pbtxt'))

    parser.add_argument("--carW", type=float, default=1.8)
    parser.add_argument("--calc_risk_n", type=int, default=2)
    # 1 to update every frame

def add_gps_args(parser):
    parser.add_argument("--use_gps", type=str2bool, default="True")
    parser.add_argument("--gps_source", type=str, default="gps_logging.txt")
    parser.add_argument("--accept_speed", type=str2bool, default="False")

def add_tracker_args(parser):
    parser.add_argument("--tracker_refresh", type=int, default=25)
    parser.add_argument("--track", type=str2bool, default="True")

def add_input_output_args(parser):
    parser.add_argument("--source", type=str, default="0")
    parser.add_argument("--save", type=str2bool, default="True")
    parser.add_argument("--save_path", type=str, \
                    default='/home/derek/object_detection_mono_video/video.avi')
    parser.add_argument("--extra_import_path", type=str, \
                    default='/home/derek/object_detection_mono_video/')

'''
parse_args creates a parser object, adds the arguments and default arguments, 
  and concludes by returning the parsed arugments.
  Additionally, it performs some simple initial checks

Returns:
  args: an object with all of the function arguments.
'''
def parse_args():
    parser = argparse.ArgumentParser()
    add_camera_args(parser)
    add_tracker_args(parser)
    add_general_args(parser)
    add_gps_args(parser)
    add_input_output_args(parser)
    args = parser.parse_args()

    return do_arg_checks(args)

'''
do_arg_checks:
    checks some of the args as desired.
Returns:
    args: If all checks succeed
Raises:
    AssertionError: if there is an invalid horizon argument
'''
def do_arg_checks(args):
    if args.source == "0" or args.source == "1": 
        args.source = int(args.source)
        assert (args.horizon >= 0.0 and args.horizon <= 1.0), \
            'Must pass in a relative horizon position, between 0 and 1'

    return args

