#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from swift_msgs.srv import Planning
import cv2
import numpy as np
from queue import PriorityQueue

class PathPlanningService(Node):
    def __init__(self):
        super().__init__('path_planning_service')
        self.service = self.create_service(Planning, 'plan_path', self.plan_path_callback)
        self.bitmap = cv2.imread('2D_bit_map.png', cv2.IMREAD_GRAYSCALE)
        self.get_logger().info('Path planning service started')

    def plan_path_callback(self, request, response):
        start_img = self.whycon_to_image(request.start)
        goal_img = self.whycon_to_image(request.goal)
        path_img = self.a_star(start_img, goal_img)
        path_whycon = []

        for point in path_img:
            whycon_point = self.image_to_whycon(point)
            path_point = Point()
            path_point.x = whycon_point[0]
            path_point.y = whycon_point[1]
            path_point.z = 27.0
            path_whycon.append(path_point)

        response.path = path_whycon
        return response

    def whycon_to_image(self, point):
        x_img = int((point.x + 5) * 100)
        y_img = int((point.y + 5) * 100)
        return (x_img, y_img)

    def image_to_whycon(self, point):
        x_whycon = (point[0] / 100.0) - 5
        y_whycon = (point[1] / 100.0) - 5
        return (x_whycon, y_whycon)

    def is_valid(self, x, y):
        return 0 <= x < 1000 and 0 <= y < 1000 and self.bitmap[y, x] == 255

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def a_star(self, start, goal):
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        directions = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]

        while not frontier.empty():
            current = frontier.get()[1]

            if current == goal:
                break

            for dx, dy in directions:
                next_pos = (current[0] + dx, current[1] + dy)

                if not self.is_valid(next_pos[0], next_pos[1]):
                    continue

                new_cost = cost_so_far[current] + self.heuristic(current, next_pos)

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal, next_pos)
                    frontier.put((priority, next_pos))
                    came_from[next_pos] = current

        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()

        return self.smooth_path(path)

    def smooth_path(self, path):
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            for look_ahead in range(len(path)-1, current_idx, -1):
                if self.has_line_of_sight(path[current_idx], path[look_ahead]):
                    smoothed.append(path[look_ahead])
                    current_idx = look_ahead
                    break
            else:
                current_idx += 1
                smoothed.append(path[current_idx])

        return smoothed

    def has_line_of_sight(self, start, end):
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        dx *= 2
        dy *= 2

        while n > 0:
            if not self.is_valid(x, y):
                return False
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
            n -= 1

        return True

def main():
    rclpy.init()
    path_planning_service = PathPlanningService()
    rclpy.spin(path_planning_service)
    path_planning_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
