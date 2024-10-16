#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from lane_detection_msgs.msg import DetectionTarget, Lane, DetectionResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import cv2
import numpy as np


class Detector(Node):
    def __init__(self):
        super().__init__("detector")
        self.get_logger().info("Detector node has been started")

        self.bridge = CvBridge()

        self.result_pub = self.create_publisher(
            DetectionResult, "/detection_result", 10
        )
        self.dectected_img_pub = self.create_publisher(Image, "/detected_image", 10)
        self.target_sub = self.create_subscription(
            DetectionTarget, "/detection_target", self.target_callback, 10
        )

    def target_callback(self, msg):
        self.get_logger().info(
            f"Received detection target with frame_id {msg.frame_id}"
        )

        start_time = time.time()

        detection_result = DetectionResult()
        detection_result.header = msg.header
        detection_result.frame_id = msg.frame_id
        detection_result.gt_lanes = msg.gt_lanes

        img_cv = self.bridge.imgmsg_to_cv2(msg.image_raw, "bgr8")

        detected_lanes = self.lane_detection(img_cv)

        for lane in detected_lanes:
            lane_msg = Lane()
            lane_msg.coordinates = lane
            detection_result.lanes.append(lane_msg)

        end_time = time.time()
        detection_result.run_time = (end_time - start_time) * 1000

        self.get_logger().info(
            f"Detected with {len(detected_lanes)} lanes with run time {detection_result.run_time} ms"
        )

        gt_lanes = []
        for lane in msg.gt_lanes:
            gt_lanes.append(lane.coordinates)

        img_cv = self.draw_lanes(img_cv, detected_lanes, gt_lanes)

        self.result_pub.publish(detection_result)
        self.dectected_img_pub.publish(self.bridge.cv2_to_imgmsg(img_cv, "bgr8"))

    def canny_edge_detection(self, img_cv):
        gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 50, 150)
        return canny
    
    def region_of_interest(self, canny):
        height = canny.shape[0]
        polygons = np.array([[(200, height), (1100, height), (550, 250)]])
        mask = np.zeros_like(canny)
        cv2.fillPoly(mask, polygons, 255)
        masked_image = cv2.bitwise_and(canny, mask)
        return masked_image

    def lane_detection(self, img_cv):
        """
        TODO: Implement the lane detection function.
        """
        lane_image = np.copy(img_cv)
        canny = self.canny_edge_detection(lane_image)
        cropped_image = self.region_of_interest(canny)

        lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=5)
        
        return_lines = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                return_lines.append((x1, y1, x2, y2))
        return return_lines

    def draw_lanes(self, img_cv, lanes, gt_lanes):
        for lane in lanes:
            for i in range(0, len(lane), 2):
                if i + 1 < len(lane):
                    x = int(lane[i])
                    y = int(lane[i+1])
                    cv2.circle(img_cv, (x, y), 5, (0, 0, 255), -1)
        for lane in gt_lanes:
            for i in range(0, len(lane), 2):
                if i + 1 < len(lane):
                    x = int(lane[i])
                    y = int(lane[i+1])
                    cv2.circle(img_cv, (x, y), 5, (0, 255, 0), -1)
        return img_cv


def main():
    rclpy.init()
    node = Detector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
