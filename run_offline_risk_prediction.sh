# We want to create an offline version of the risk prediction model that
# is accurate enough to produce reasonable ground truth labels for a given video.
# The key is that we are no longer constrained by the real-time nature of the
# problem. We begin by using a more computationally intensive object detector,
# and more trackers/particles. Once that is working I will revisit if we should
# try something else.

START_LOC=$(pwd)
# PASS IN TF_LOC AS FIRST ARGUMENT (or change the path here)
TF_LOC='/home/derek/env_tf_models_research/object_detection'
if [ "$1" != "" ]; then
    TF_LOC=$1
fi

OFFLINE='true'   # indicates we do not care about runtime and want to save all results.

MODEL="/scratch/derek/obj_det_models/faster_rcnn_resnet101_kitti_2018_01_28"
LABELS="data/kitti_label_map.pbtxt"

RUN_ID="KITTI"  # Inidcate the subdirectory of results.
RUN_NUM="2"   # Further specify the directory (concatenated to ID)

if [ "$RUN_ID" = "NAS" ]; then
    MODEL="/scratch/derek/obj_det_models/faster_rcnn_nas_coco_2018_01_28"
    LABELS="data/mscoco_label_map.pbtxt"
elif [ "$RUN_ID" = "INC_LOW" ]; then
    MODEL="/scratch/derek/obj_det_models/faster_rcnn_inception_resnet_v2_atrous_lowproposals_coco_2018_01_28"
    LABELS="data/mscoco_label_map.pbtxt"
fi

RUN_ID=${RUN_ID}$RUN_NUM

DET_THRESH=0.7             # above 1 means nothing will get marked.
DEVICE='/gpu:0'

RESULTS_SAVE_PATH='/scratch/derek/Allstate_data/results/'${RUN_ID}'/'
OVERWRITE_SAVES='true'
# TODO Double check your flag for overwriting!

# loading data:
PRIOR_RESULTS_PATH='/scratch/derek/Allstate_data/results/KITTI1/'
LOAD_EGO_SPEED='true'
LOAD_OBJ_DETECTOR='false'
LOAD_TRACKER='false'
LOAD_STATE='false'

mkdir -p $RESULTS_SAVE_PATH

SAVE_VIDEO='true'
SOURCE='/scratch/derek/Allstate_data/video3.mp4'
SAVE_VIDEO_PATH=${RESULTS_SAVE_PATH}'video_3_offline.mp4'

DO_TRACK='true'
TRACKER_TYPE="Particle"
TRACK_REFRESH=250             # 1 makes no refreshing
TRACKER_HOLD=10
LANE_BASED_SPEED='true'
MAX_TRACKERS=20
NUM_TRACKER_PARTICLES=100

THREADED_RUNNER='None'          # The runner-level threading method, or 'None'
THREAD_QUEUE_SIZE=3          # The size of the queue for threaded_runner
THREAD_WAIT_TIME=0.02        # The minimum amount of time to block on a queue, sec.
THREAD_MAX_WAIT=1.0          # maximum amount of time to block on a queue, sec.

FOCAL=1495                    # Genius at 1080p is ~850, at 720p is ~550
CAR_WIDTH=1.8                # meters
CAMERA_HEIGHT=1.08           # meters, for subaru forester, 1.2m
RELATIVE_HORIZON=0.485        # between 0 and 1, above 0.5 is above centerline
RESOLUTION_H=0 # 0 means both not set
RESOLUTION_W=0 # 0 means both not set

RISK_N_SIMS=200
RISK_H=10.0        # seconds
RISK_STEP=0.25   # seconds
COL_TOL_X=3.0    # meters
COL_TOL_Y=3.0    # meters
TTC_H=10.0        # seconds
TTC_STEP=0.25    # seconds
RISK_THREADS=10  # max number of threads (>1 --> threaded risk calcs)
EMBEDDED_RISK='true' # boolean, whether or not to calc risk while simulating.
RISK_TYPE="TTC" # "TTC" for constant delta v, or "Online" for sims with models
CALC_RISK_EVERY_N_FRAMES=1


cd $TF_LOC

python3 $(echo $START_LOC)/launcher.py \
    --source $SOURCE --model $MODEL --labels $LABELS --device $DEVICE \
    --save $SAVE_VIDEO --save_path $SAVE_VIDEO_PATH \
    --focal $FOCAL --carW $CAR_WIDTH \
    --det_thresh $DET_THRESH --cameraH $CAMERA_HEIGHT \
    --horizon $RELATIVE_HORIZON \
    --resolution_h $RESOLUTION_H --resolution_w $RESOLUTION_W \
    --track $DO_TRACK --tracker_type $TRACKER_TYPE --tracker_refresh $TRACK_REFRESH \
    --tracker_hold $TRACKER_HOLD --num_trackers $MAX_TRACKERS \
    --num_tracker_particles $NUM_TRACKER_PARTICLES \
    --lane_based_speed $LANE_BASED_SPEED \
    --risk_H $RISK_H --risk_step $RISK_STEP --n_risk_sims $RISK_N_SIMS \
    --risk_type $RISK_TYPE --ttc_H $TTC_H --ttc_step $TTC_STEP \
    --col_tol_x $COL_TOL_X --col_tol_y $COL_TOL_Y \
    --calc_risk_n $CALC_RISK_EVERY_N_FRAMES \
    --embedded_risk $EMBEDDED_RISK --max_risk_threads $RISK_THREADS \
    --threaded_runner $THREADED_RUNNER --thread_queue_size $THREAD_QUEUE_SIZE \
    --thread_max_wait $THREAD_MAX_WAIT --thread_wait_time $THREAD_WAIT_TIME \
    --offline $OFFLINE --results_save_path $RESULTS_SAVE_PATH \
    --overwrite_saves $OVERWRITE_SAVES --prior_results_path $PRIOR_RESULTS_PATH \
    --L_EGO_SPEED $LOAD_EGO_SPEED --L_OBJ_DETECTOR $LOAD_OBJ_DETECTOR \
    --L_TRACKER $LOAD_TRACKER --L_STATE $LOAD_STATE \
    | tee ${RESULTS_SAVE_PATH}'output.log'

cd $START_LOC
