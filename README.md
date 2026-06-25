# ROS 2 LED Tracking Package (`led_tracking_pkg`)

## Overview
This ROS 2 C++ package provides a robust computer vision node designed to extract and track underwater LED markers in real-time. The node subscribes to a camera image topic, applies **HSV color thresholding** to isolate the LEDs, calculates sub-pixel centroids using image moments, applies robust spatial sorting, and publishes the 2D image plane coordinates `(u, v)` of exactly 4 LEDs. 

It also includes built-in OpenCV visualization windows for live visual debugging of the camera feed and the binary threshold mask.

---

## Features
* **Real-Time Processing:** Built on `rclcpp` and `cv_bridge` utilizing modern ROS 2 Jazzy standards.
* **HSV Color Filtering:** Filters out ambient underwater noise by thresholding specific green brightness ranges.
* **Noise Reduction:** Filters small reflections using contour area thresholds.
* **Robust 4-Point Sorting:** Automatically sorts the 4 detected LEDs into a consistent spatial sequence (Top-Left, Top-Right, Bottom-Left, Bottom-Right) regardless of detection order.
* **Visual Debugging:** Renders detected centroids in real-time (`cv::imshow`) and displays the binary mask to help tune color parameters.
* **Safety Thresholds:** Suppresses bad tracking data, preventing unstable coordinates from polluting downstream control nodes.

---

## How to Run

1. **Launch your camera stream or rosbag** that publishes the raw images (default topic: `/camera/image_raw`). 

2. **Run the LED Tracker node:**
   ```bash
   ros2 run led_tracking_pkg led_tracker
