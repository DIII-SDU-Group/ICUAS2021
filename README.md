# cable_estimators
Estimates overhead power line cable position based on sensor data


<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Prerequisites

- ROS (Ubuntu 18.04.5 melodic tested)
- Python (2.7.17 tested)

### Installation

1. Clone the repo
   ```sh
	git clone https://gitlab.sdu.dk/nhma/cable_estimators.git
   ```

<!-- USAGE EXAMPLES -->
## Usage

0. Playback rosbag with data from relevant sensor (e.g. RealSense)
1. Execute execute cable estimator script for chosen sensor (e.g. RealSense)
   ```sh
   python realsense_cable_estimator.py
   ```
2. The script will project 3D points onto available 2D image (if one is available) and highlight estimated cable position.
	- Depth distance is indicated by 8-bit rgb value. E.g. rgb(0,0,255) = 9m, rgb(0,255,) = 4.5m, and rgb(255,0,0) = 0m. 
	- Purple circle indicates estimated cable position.
3. To visualize in rviz, add '/realsense_cable_estimate/point' topic
4. Additionally, raw x, y, and z float64 values are published.
5. All values are wrt. drone_center as origin.

## Modify
### scube_cable_estimator.py:
- Performs RANSAC line fitting (1000 its), plots inlier points on RGB image.
- Takes median of all inliers in x, y, z as respective estimated cable coordinate.
- Code includes two thresholds to filter points based on z-distance (depth):
   - low_threshold filters out any points below this value (in meters).
   - high_threshold filters put any points above this value (in meters).
   - these can be used if multiple objects are seen by sensor, but only one is relevant cable (fx in 1_1_3.bag).

### realsense_cable_estimator.py
- Code includes two thresholds to filter points based on z-distance (depth):
   - low_threshold filters out any points below this value (in meters).
   - high_threshold filters put any points above this value (in meters).
- Performs RANSAC line fitting (1000 its), plots inlier points on RGB image.
- Takes median of all inliers in x, y, z as respective estimated cable coordinate.
- Rosbag must be played at reduced rate, otherwise topics delayed.
   - at 1x rate, PointCloud2 ~3.5s delayed compared to Image.
- Can take several tries to get running.
- Computationally very heavy, maybe 2 Hz output frequency .

### mmwave_cable_estimator.py
- Finds closest point that is further away than 0.1m and within the image frame of realsense RGB camera (for sanity check). 
- Use low_threshold to set a minimum distance if irrelevant object is detected close to sensor.

### vu8_cable_estimator.py
- Finds average of all non-zero segment values.
- Use low_threshold to set a minimum distance if irrelevant object is detected close to sensor.
