import sys

sys.path.append("..")
from driver_risk_utils import argument_utils, general_utils
import speed_estimator
import cv2

time = 0.0
fps = 30.0

args = argument_utils.parse_args()
args.use_gps = False
args.lane_based_speed = True

# test with display
est = speed_estimator.SpeedEstimator(
    args,
    default_speed=25,
    verbose=True,
    display_speed_lane=True
)

for im_num in range(2700,2801):
    image = cv2.imread("GH_frames/"+str(im_num)+".jpg")
    #image = image[10:15,11:15]
    H, W, C = image.shape
    print(H,W,C)
    print(image[0,0,:])

    est.update_estimates(image, time)
    time += 1.0/fps

assert round(est.get_reading()) == 17 # mps
assert round(general_utils.mps_to_mph(est.get_reading())) == 37 #mph

cv2.destroyAllWindows()

# test without display
est = speed_estimator.SpeedEstimator(
    args,
    default_speed=25,
    verbose=True,
    display_speed_lane=False
)

for im_num in range(2700,2801):
    image = cv2.imread("GH_frames/"+str(im_num)+".jpg")
    #image = image[10:15,11:15]
    H, W, C = image.shape
    print(H,W,C)
    print(image[0,0,:])

    est.update_estimates(image, time)
    time += 1.0/fps

assert round(est.get_reading()) == 17 # mps
assert round(general_utils.mps_to_mph(est.get_reading())) == 37 #mph
