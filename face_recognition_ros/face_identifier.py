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
import std_srvs.srv
import vision_msgs.msg
from ament_index_python.packages import get_package_share_directory


class FaceIdentifier(rclpy.node.Node):
    PALETTE = [
        (0, 0, 255),
        (0, 255, 0),
        (255, 0, 0),
        (255, 255, 0),
        (255, 0, 255),
        (0, 255, 255),
        (255, 255, 255),
    ]

    def __init__(self) -> None:
        super().__init__("face_identifier")

        self.known_face_encodings: list[np.ndarray] = []
        self.known_face_names: list[str] = []

        self.load_known_faces()

        self.cv_bridge = cv_bridge.CvBridge()

        self.declare_parameter("resize_height", 320)

        self.detections_pub = self.create_publisher(
            vision_msgs.msg.Detection2DArray, "~/detections", 5
        )
        self.result_image_pub = self.create_publisher(
            sensor_msgs.msg.Image, "~/result_image", 5
        )
        source_image_sub_qos = rclpy.qos.qos_profile_sensor_data
        source_image_sub_qos.depth = 1
        self.source_image_sub = self.create_subscription(
            sensor_msgs.msg.Image,
            "/image_raw",
            self.source_image_callback,
            source_image_sub_qos,
        )

        self.reload_known_faces_srv = self.create_service(
            std_srvs.srv.Empty, "~/reload_known_faces", self.reload_known_faces_callback
        )

    def source_image_callback(self, msg: sensor_msgs.msg.Image) -> None:
        image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        result_image = image.copy()

        if self.known_face_encodings:
            resize_ratio = (
                self.get_parameter("resize_height").get_parameter_value().integer_value
                / image.shape[0]
            )
            image = cv2.resize(image, dsize=None, fx=resize_ratio, fy=resize_ratio)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            face_locations = face_recognition.face_locations(image)
            face_encodings = face_recognition.face_encodings(image, face_locations)

            detections_msg = vision_msgs.msg.Detection2DArray()
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
                    face_location = [round(x / resize_ratio) for x in face_location]
                    face_center = vision_msgs.msg.Point2D(
                        x=(face_location[1] + face_location[3]) / 2,
                        y=(face_location[0] + face_location[2]) / 2,
                    )
                    bbox = vision_msgs.msg.BoundingBox2D(
                        center=vision_msgs.msg.Pose2D(position=face_center),
                        size_x=float(face_location[1] - face_location[3]),
                        size_y=float(face_location[2] - face_location[0]),
                    )
                    detections_msg.detections.append(
                        vision_msgs.msg.Detection2D(bbox=bbox, id=face_name)
                    )
                    cv2.rectangle(
                        result_image,
                        (face_location[1], face_location[0]),
                        (face_location[3], face_location[2]),
                        color=self.PALETTE[best_match_index % len(self.PALETTE)],
                        thickness=2,
                    )
                    cv2.rectangle(
                        result_image,
                        (face_location[3], face_location[2] - 50),
                        (face_location[1], face_location[2]),
                        color=self.PALETTE[best_match_index % len(self.PALETTE)],
                        thickness=cv2.FILLED,
                    )
                    cv2.putText(
                        result_image,
                        face_name,
                        (face_location[3], face_location[2] - 5),
                        fontFace=cv2.FONT_HERSHEY_DUPLEX,
                        fontScale=1.5,
                        color=(0, 0, 0),
                        thickness=2,
                    )

            detections_msg.header.stamp = self.get_clock().now().to_msg()
            detections_msg.header.frame_id = msg.header.frame_id
            self.detections_pub.publish(detections_msg)

        result_image_msg = self.cv_bridge.cv2_to_imgmsg(result_image, "bgr8")
        result_image_msg.header.stamp = self.get_clock().now().to_msg()
        result_image_msg.header.frame_id = msg.header.frame_id
        self.result_image_pub.publish(result_image_msg)

    def reload_known_faces_callback(
        self, request: std_srvs.srv.Empty.Request, response: std_srvs.srv.Empty.Response
    ) -> std_srvs.srv.Empty.Response:
        self.load_known_faces()
        return response

    def load_known_faces(self) -> None:
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

        if self.known_face_names:
            self.get_logger().info(f"Loaded known faces: {self.known_face_names}")
        else:
            self.get_logger().warn("No known faces registered")


def main(args: list[str] = None):
    rclpy.init(args=args)
    node = FaceIdentifier()
    rclpy.spin(node)
    rclpy.shutdown()
