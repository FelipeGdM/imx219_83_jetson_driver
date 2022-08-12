# IMX219-83 stereo camera Jetson driver for ROS 2

This package provides a driver to use IMX219-83 stereo camera with a Jetson platform with ROS 2. It uses OpenCV alongside a GStreamer pipeline to get the images from the camera connected through 2 CSI connections.

To launch the node, just run

```bash
ros2 run imx219_83_jetson_driver camera_node
```

To change some parameter, use `--ros-args -p arg1:=val1 -p arg2:=val2`

```bash
ros2 run imx219_83_jetson_driver camera_node --ros-args -p left_camera_name=:left -p right_camera_name:=right
```

Currently, the code is only implemented in a standalone node. A plugin version may ~or may not~ be implemented in the future.

## Published topics

|      Topic name      |           Topic type           | Description                               |
| :------------------: | :----------------------------: | :---------------------------------------- |
|  `/left/image_raw`   |    `sensor_msgs/msg/Image`     | Raw images provided from the left camera  |
| `/left/camera_info`  | `sensor_msgs::msg::CameraInfo` | Information about the left camera         |
|  `/right/image_raw`  |    `sensor_msgs/msg/Image`     | Raw images provided from the right camera |
| `/right/camera_info` | `sensor_msgs::msg::CameraInfo` | Information about the right camera        |

## Parameters

|     Parameter name      | Description                                                                                                                  |                      Default                      |
| :---------------------: | :--------------------------------------------------------------------------------------------------------------------------- | :-----------------------------------------------: |
|    `left_camera_id`     | Device identification of the left camera (i.e. `/dev/videoX`)                                                                |                        `1`                        |
|   `left_camera_name`    | Name of the left camera                                                                                                      |                   `left_camera`                   |
| `left_camera_info_url`  | URL of the YAML or INI file with camera intrisic parameters. It may be a `package://` or `file://` path                      | `file:///${ROS_HOME}/camera_info/stereo/left.ini` |
|    `right_camera_id`    | Device identification of the right camera (i.e. `/dev/videoX`)                                                               |                        `0`                        |
|   `right_camera_name`   | Name of the right camera                                                                                                     |                  `right_camera`                   |
| `right_camera_info_url` | URL of the YAML or INI file with camera intrisic parameters. It may be a `package://` or `file://` path                      | `file:///${ROS_HOME}/camera_info/stereo/left.ini` | `file:///${ROS_HOME}/camera_info/stereo/right.ini` |
|         `width`         | Width of the image, in pixels                                                                                                |                      `1280`                       |
|        `height`         | Height of the image, in pixel                                                                                                |                       `720`                       |
|       `framerate`       | Acquisition framerate, as a gstreamer string                                                                                 |                      `60/1`                       |
|      `flip_method`      | Rotation of the image, as used by gstreamer's **nvvidconv**. 0 corresponds to no rotation and 2 corresponds to 180° rotation |                        `2`                        |
|       `frame_id`        | Tf2 reference frame of the left camera (both images are published in the left reference frame)                               |                  `camera_frame`                   |

## ToDo

- Provide plugin interface for camera node
- Implement new fancy [NITROS transport](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros)

## Acknowledgments

Special thanks for [Jetson Hacks article](https://jetsonhacks.com/2019/04/02/jetson-nano-raspberry-pi-camera/) and [code](https://github.com/JetsonHacksNano/CSI-Camera) to shred some light on how interface CSI cameras with Jetson hardware.
