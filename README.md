# Group 3 – Baa Force One 

## Introduction

### Goal:
To provide the 'eye in the sky'. Develop the infrastructure for streaming the data from the provided drone, and automating the collection process to ensure the highest quality of data is available, surveying commercial solutions in the process. 

### Preparation in Advance:
Ensure your device has the following installed / available: 
- ROS2 Humble (Installed with Ubuntu or as a Docker image) 
- VS Code
- Android Studio

### Activities:
#### Primary Activity:
`Real-Time Data Streaming` – Utilising the DJI Mobile SDK v5, create an Android application to stream Video, GPS, Camera Information (EXIF/Pose), and Telemetry Data into the ROS2 system. 

#### Secondary Activity: 
`Remote Tele-Operation` – Developing the Android application further, integrate capabilities to send the drone to given coordinates, generating flight plans and risk assessments for manual approval. 

#### Stretch Activity:
`Autonomous Planning` – With the positions of detected targets received, develop an approach to autonomously manoeuvring the drone to keep the targets within the frame, making considerations to battery lifetime and hot-swapping drones. 

### Outcomes:
With the completion of these activities, the group will have the necessary pipelines in place for connecting data streams to the other groups in the project for consumption into ROS2 processes. 

### Future Engagement:
Further to the week, members will have the opportunity to engage with dynamic drone autonomy and advanced ariel planning with sheep monitoring. Members will also be encouraged to package and publish the developed application for use as a toolset for research beyond shepherding. # README

To integrate into other systems just copy and paste the src/uav_image folder into your ros2 package directory and build

Requires ffmpeg to be installed and accessable on the system. Along with the following ros2 packages

- cv_bridge
- image_transport
- sensor_msgs
- OpenCV

## Install

- Make sure you have pixi installed: https://pixi.sh/dev/installation/
- run `pixi install` in the project root
- install ffmpeg with your system package manager e.g: `sudo apt install ffmpeg`
- compile with `pixi run build`
- run with `pixi run run`

## ROS2 Topics

### uav_image package (C++)

#### uav_image node

- Topic: `/uav_image`
    - Message type: `sensor_msgs/msg/image`
    - CvImage in BGR8 format.
- Topic: `/uav_camera`
    - Message type: `sensor_msgs/msg/camera_info`
    - Data:
        - `height`: in pixels
        - `width`: in pixels 
        - `d`: distortion (list length 5), plumb bob model
        - `k`: intrinsic matrix, 3x3 row-major
        - `p`: projection matrix, 3x4
    - The matrices can be used to calculate FOV etc.


### TODO package

- Topic: `/uav_gps`
    - Message type: `sensor_msgs/msg/NatSatFix`
    - Data:
        - `latitude`
        - `longitude`
        - `altitude`
    - Altitude may be inaccurate. If this is the case we will spoof the number and set the drone altitude manually.
- Topic: `/uav_direction`
    - Message type: `std_msgs/msg/Float32`
    - Data:
        - `data`: Float32. A heading has a range of [-180, 180] degrees, where 0 represents True North. 

### map_publisher package

#### map_publisher node (Python)

- Topic: `/field_boundary`
    - Message type: `geometry_msgs/msg/Polygon` (array of `Point32`)
    - Data:
    - `x`: latitude
    - `y`: longitude
    - `z`: unused, default to 0


