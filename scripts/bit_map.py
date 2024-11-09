#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BitMapCreator(Node):
    def __init__(self):
        super().__init__('bit_map_creator')
        self.subscription = self.create_subscription(
            Image,
            'whycon/image_out',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.got_image = False

    def image_callback(self, msg):
        if self.got_image:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

        if len(corners) == 4:
            sorted_corners = self.sort_corners(corners)
            src_pts = np.float32([c[0] for c in sorted_corners])
            dst_pts = np.float32([[0, 0], [999, 0], [999, 999], [0, 999]])
            matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
            warped = cv2.warpPerspective(cv_image, matrix, (1000, 1000))
            gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            bitmap = np.ones((1000, 1000), dtype=np.uint8) * 255
            inflation_radius = 30

            for contour in contours:
                mask = np.zeros((1000, 1000), dtype=np.uint8)
                cv2.drawContours(mask, [contour], -1, 255, -1)
                kernel = np.ones((inflation_radius, inflation_radius), np.uint8)
                inflated_mask = cv2.dilate(mask, kernel, iterations=1)
                bitmap[inflated_mask > 0] = 0
            
            cv2.imwrite('2D_bit_map.png', bitmap)
            self.got_image = True
            self.get_logger().info('Bitmap created successfully')

    def sort_corners(self, corners):
        corners_list = [corner[0][0] for corner in corners]
        corners_list.sort(key=lambda x: x[1])
        top_two = corners_list[:2]
        bottom_two = corners_list[2:]
        top_two.sort(key=lambda x: x[0])
        bottom_two.sort(key=lambda x: x[0])
        return [np.array([top_two[0]]), np.array([top_two[1]]),
                np.array([bottom_two[1]]), np.array([bottom_two[0]])]

def main(args=None):
    rclpy.init(args=args)
    bit_map_creator = BitMapCreator()
    rclpy.spin(bit_map_creator)
    bit_map_creator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
