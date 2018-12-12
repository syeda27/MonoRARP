import sys

sys.path.append("..")
from driver_risk_utils import argument_utils, general_utils
import speed_estimator
import cv2

args = argument_utils.parse_args()
args.use_gps = False
args.lane_based_speed = True


def test_with_display():
    print("Testing on images with display!")
    time = 0.0
    fps = 30.0

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

def test_with_no_display():
    # test without display
    print("Testing no display!")
    time = 0.0
    fps = 30.0
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

def test_720p_video():
    print("Testing 720p video!")
    time = 0.0
    fps = 30.0
    est = speed_estimator.SpeedEstimator(
        args,
        default_speed=25,
        verbose=True,
        display_speed_lane=False
    )

    camera = cv2.VideoCapture('/scratch/derek/video_captures/FullFOVandHD/video11a.mp4')
    resolution = (1280, 720)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

    assert camera.isOpened(), \
            'Cannot capture source'
    done = False
    while not done:
        _, image = camera.read()
        if image is None:
            print('\nEnd of Video')
            done = True
            break

        #image = image[10:15,11:15]
        H, W, C = image.shape
        print(H,W,C)
        print(image[0,0,:])

        est.update_estimates(image, time)
        time += 1.0/fps

    assert round(est.get_reading()) == 17 # mps
    assert round(general_utils.mps_to_mph(est.get_reading())) == 37 #mph


test_with_display()
test_with_no_display()
#test_720p_video()
