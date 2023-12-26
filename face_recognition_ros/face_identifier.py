import glob
import os

import cv2
import cv_bridge
import face_recognition
import numpy as np
import rclpy
import rclpy.node
import rclpy.qos
import sensor_msgs.msg
import vision_msgs.msg
from ament_index_python.packages import get_package_share_directory


class FaceIdentifier(rclpy.node.Node):
    def __init__(self):
        super().__init__("face_identifier")

        self.known_face_encodings: list[np.ndarray] = []
        self.known_face_names: list[str] = []

        for known_face_image_file in glob.glob(
            os.path.join(
                get_package_share_directory("face_recognition_ros"),
                "resource",
                "known_faces",
                "**",
            ),
            recursive=True,
        ):
            try:
                known_face_image = face_recognition.load_image_file(
                    known_face_image_file
                )
            except:
                continue

            self.known_face_encodings.append(
                face_recognition.face_encodings(known_face_image)[0]
            )
            self.known_face_names.append(
                os.path.splitext(os.path.basename(known_face_image_file))[0]
            )

        assert self.known_face_names, "No known face registered"
        self.get_logger().info(f"Registered known faces: {self.known_face_names}")

        self.cv_bridge = cv_bridge.CvBridge()

        self.result_detection_pub = self.create_publisher(
            vision_msgs.msg.Detection2DArray, "~/result/detection", 5
        )
        self.result_image_pub = self.create_publisher(
            sensor_msgs.msg.Image, "~/result/image_raw", 5
        )
        image_sub_qos = rclpy.qos.qos_profile_sensor_data
        image_sub_qos.depth = 1
        self.image_sub = self.create_subscription(
            sensor_msgs.msg.Image,
            "/image_raw",
            self.image_callback,
            image_sub_qos,
        )

    def image_callback(self, msg: sensor_msgs.msg.Image):
        image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        resize_ratio = 320 / image.shape[0]
        image = cv2.resize(image, dsize=None, fx=resize_ratio, fy=resize_ratio)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        face_locations = face_recognition.face_locations(image)
        face_encodings = face_recognition.face_encodings(image, face_locations)

        detection_msg = vision_msgs.msg.Detection2DArray()
        for face_location, face_encoding in zip(face_locations, face_encodings):
            matches = face_recognition.compare_faces(
                self.known_face_encodings, face_encoding
            )
            distances = face_recognition.face_distance(
                self.known_face_encodings, face_encoding
            )
            best_match_index = np.argmin(distances)
            if matches[best_match_index]:
                face_name = self.known_face_names[best_match_index]
                face_center = vision_msgs.msg.Point2D(
                    x=(face_location[1] + face_location[3]) / (2 * resize_ratio),
                    y=(face_location[0] + face_location[2]) / (2 * resize_ratio),
                )
                bbox = vision_msgs.msg.BoundingBox2D(
                    center=vision_msgs.msg.Pose2D(position=face_center),
                    size_x=(face_location[1] - face_location[3]) / resize_ratio,
                    size_y=(face_location[2] - face_location[0]) / resize_ratio,
                )
                detection_msg.detections.append(
                    vision_msgs.msg.Detection2D(bbox=bbox, id=face_name)
                )
                cv2.rectangle(
                    image,
                    (face_location[1], face_location[0]),
                    (face_location[3], face_location[2]),
                    color=(255, 0, 0),
                    thickness=2,
                )
                cv2.rectangle(
                    image,
                    (face_location[3], face_location[2] - 35),
                    (face_location[1], face_location[2]),
                    color=(255, 0, 0),
                    thickness=cv2.FILLED,
                )
                cv2.putText(
                    image,
                    face_name,
                    (face_location[3], face_location[2] - 5),
                    fontFace=cv2.FONT_HERSHEY_DUPLEX,
                    fontScale=1.0,
                    color=(255, 255, 255),
                    thickness=1,
                )

        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = msg.header.frame_id
        self.result_detection_pub.publish(detection_msg)

        result_image_msg = self.cv_bridge.cv2_to_imgmsg(image, "rgb8")
        result_image_msg.header.stamp = self.get_clock().now().to_msg()
        result_image_msg.header.frame_id = msg.header.frame_id
        self.result_image_pub.publish(result_image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FaceIdentifier()
    rclpy.spin(node)
    rclpy.shutdown()