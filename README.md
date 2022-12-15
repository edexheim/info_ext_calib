# Information-Theoretic Online Multi-Camera Extrinsic Calibration


## Dependencies

GTSAM, OpenCV, Eigen, TBB (prerequisite for GTSAM), Boost (prerequisite for GTSAM), Pangolin.

Last tested with: \
GTSAM commit f6ef1d6d2cdb84286befc3ce686e32ff4d653a6b (https://github.com/borglab/gtsam) \
OpenCV: 4.0.0 \
TBB: 2020.3 \
Boost: 1.75.0 \
Pangolin commit bcf41ec8b2853eb132dbe5cbe65ca8eade31846e: https://github.com/stevenlovegrove/Pangolin

## Examples

The important parameter files to modify in params/ are frontend.yaml, backend.yaml, sim.yaml, as well as platform-specific stereo configurations in params/stereo/.

### EuRoC

Download datasets [here](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).
For multi-session example, try V1_02 and V2_02 which have sufficient motion.

Note that each camera sensor.yaml shoulld be duplicated, renamed as sensor_opencv.yaml, and "%YAML:1.0" so that it is compatible with OpenCV FileStorage.

For a multi-session example, run commands: 

./calibrate euroc <euroc_path>/V1_02_medium/
./calibrate euroc <euroc_path>/V2_02_medium/

The session is saved after running through a dataset.  If you run on subsequent datasets, the extrinsics and segment database will be loaded and used.  This can be cleared using scripts/clear_database.py with the dataset type specified:

From the build folder, this is run as:
python ../scripts/clear_database.py euroc
