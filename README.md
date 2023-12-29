# face_recognition_ros

ROS2 package for using [face_recognition](https://github.com/ageitgey/face_recognition).

## Requirements

- ROS 2 (Tested on Humble, would work on any distributions with Python 3.3+ required by `face_recognition`)

Other dependencies are listed in `package.xml` and `requirements.txt`.

## Usage

### Setup

```
$ cd ~/ros2_ws/src
$ git clone https://github.com/mitukou1109/face_recognition_ros.git
$ cd face_recognition_ros
$ rosdep install -iyr --from-paths .
$ pip install -r requirements.txt
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source install/local_setup.bash
```

### Face identification

Prepare an image containing a face to identify. Assuming you have your selfie in home directory:

```
$ cd ~/ros2_ws/src/face_recognition_ros
$ python register_known_face.py ~/your_name.jpg "Your name"   # You can specify the name to register, otherwise the file's name will be used
Registered Your name
```

You can test with the following commands if you have a webcam connected to your PC:

```
$ sudo apt install ros-humble-v4l2-camera
$ ros2 launch face_recogntion_ros demo.launch.py
...
[INFO] [1703600037.356322923] [face_identifier]: Registered known faces: ['Your name']
...
```

![Demo screenshot](https://github.com/mitukou1109/face_recognition_ros/assets/50359861/3861b7af-4f6f-45f7-bd68-d8fd05b3112b)

### Face detection

TODO

## Topics

### `face_identifier`

Subscribed topics:

| Name         | Type                | Description |
| :----------- | :------------------ | :---------- |
| `/image_raw` | `sensor_msgs/Image` | Input image |

Published topics:

| Name             | Type                           | Description                                               |
| :--------------- | :----------------------------- | :-------------------------------------------------------- |
| `~/detections`   | `vision_msgs/Detection2DArray` | Sequence of bounding box and name for each detected faces |
| `~/result_image` | `sensor_msgs/Image`            | Input image with bounding boxes and names drawn           |

## Parameters

### `face_identifier`

| Name            | Type | Default value | Description                                                       |
| :-------------- | :--- | :------------ | :---------------------------------------------------------------- |
| `resize_height` | int  | `320`         | Height to resize input image before encoding for making it faster |
