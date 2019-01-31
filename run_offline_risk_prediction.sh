:'
 We want to create an offline version of the risk prediction model that
 is accurate enough to produce reasonable ground truth labels for a given video.
 The key is that we are no longer constrained by the real-time nature of the
 problem. We begin by using a more computationally intensive object detector,
 and more trackers/particles. Once that is working I will revisit if we should
 try something else.
'
# TODO save outputs to a csv (based on a flag).
# TODO bigger object detection model.
# TODO more tracker particles.


START_LOC=$(pwd)
# TODO PASS IN TF_LOC AS FIRST ARGUMENT (or change the path here)
TF_LOC='/home/derek/env_tf_models_research/object_detection'
if [ "$1" != "" ]; then
    TF_LOC=$1
fi

MODEL="/scratch/derek/obj_det_models/faster_rcnn_resnet101_kitti_2018_01_28"
DET_THRESH=0.01             # above 1 means nothing will get marked.
DEVICE='/gpu:0'

SAVE='false'
RUN='video_capture'
SOURCE='/scratch/derek/video_captures/video_GH010034.mp4'
SAVE_PATH='/scratch/derek/video_captures/video_GH010034_offline.mp4'

DO_TRACK='true'
TRACKER_TYPE="Particle"
TRACK_REFRESH=250             # 1 makes no refreshing
TRACKER_HOLD=10
USE_GPS='false'             # use speed readings from a GPS
LANE_BASED_SPEED='true'

# TODO simplify and make threaded runner either on or off, no options
THREADED_RUNNER='B'          # The runner-level threading method, or 'None'
THREAD_QUEUE_SIZE=3          # The size of the queue for threaded_runner
THREAD_WAIT_TIME=0.02        # The minimum amount of time to block on a queue, sec.
THREAD_MAX_WAIT=1.0          # maximum amount of time to block on a queue, sec.

FOCAL=1495                    # Genius at 1080p is ~850, at 720p is ~550
CAR_WIDTH=1.8                # meters
CAMERA_HEIGHT=1.08           # meters, for subaru forester, 1.2m
RELATIVE_HORIZON=0.545        # between 0 and 1, above 0.5 is above centerline
RESOLUTION_H=1080 # 0 means both not set
RESOLUTION_W=1920 # 0 means both not set


RISK_N_SIMS=50
RISK_H=10.0        # seconds
RISK_STEP=0.25   # seconds
COL_TOL_X=2.0    # meters
COL_TOL_Y=2.0    # meters
TTC_H=10.0        # seconds
TTC_STEP=0.25    # seconds
RISK_THREADS=10  # max number of threads (>1 --> threaded risk calcs)
EMBEDDED_RISK='true' # boolean, whether or not to calc risk while simulating.
RISK_TYPE="Online" # "TTC" for constant delta v, or "Online" for sims with models
CALC_RISK_EVERY_N_FRAMES=1

JOBS=`jobs -p`
if [ '$USE_GPS' = 'true' ]; then
    echo "Running gps command to $GPS_SOURCE"
    sudo gpsd -S 2949 -n -N -D 5 -b /dev/ttyUSB0 &> $GPS_SOURCE &
    JOBS=`jobs -p`
    sleep 1
    # TODO if you need to, change this command
fi
cd $TF_LOC

python3 $(echo $START_LOC)/launcher.py \
    --source $SOURCE --model $MODEL --device $DEVICE \
    --save $SAVE --save_path $SAVE_PATH \
    --focal $FOCAL --carW $CAR_WIDTH \
    --det_thresh $DET_THRESH --cameraH $CAMERA_HEIGHT \
    --horizon $RELATIVE_HORIZON \
    --resolution_h $RESOLUTION_H --resolution_w $RESOLUTION_W \
    --track $DO_TRACK --tracker_type $TRACKER_TYPE --tracker_refresh $TRACK_REFRESH \
    --tracker_hold $TRACKER_HOLD \
    --use_gps $USE_GPS --gps_source ${START_LOC}/$GPS_SOURCE \
    --lane_based_speed $LANE_BASED_SPEED --accept_speed $ACCEPT_SPEED \
    --risk_H $RISK_H --risk_step $RISK_STEP --n_risk_sims $RISK_N_SIMS \
    --risk_type $RISK_TYPE --ttc_H $TTC_H --ttc_step $TTC_STEP \
    --col_tol_x $COL_TOL_X --col_tol_y $COL_TOL_Y \
    --calc_risk_n $CALC_RISK_EVERY_N_FRAMES \
    --embedded_risk $EMBEDDED_RISK --max_risk_threads $RISK_THREADS \
    --threaded_runner $THREADED_RUNNER --thread_queue_size $THREAD_QUEUE_SIZE \
    --thread_max_wait $THREAD_MAX_WAIT --thread_wait_time $THREAD_WAIT_TIME
cd $START_LOC
for job in $JOBS
do
    echo "killing job $job"
    kill $job
done
