import numpy as np
import os, sys
from ..geom.Vector3D import Vector3D
from ..geom.Point import Point
import math
import pygame

COLOR_SCARLET_RED_0 = pygame.Color(239, 41, 41)
COLOR_BUTTER_0 = pygame.Color(252, 233, 79)
COLOR_CHAMELEON_0 = pygame.Color(138, 226, 52)
COLOR_ALUMINIUM_0 = pygame.Color(238, 238, 236)
COLOR_ALUMINIUM_1 = pygame.Color(211, 215, 207)
COLOR_ALUMINIUM_2 = pygame.Color(186, 189, 182)
COLOR_ALUMINIUM_3 = pygame.Color(136, 138, 133)
COLOR_ALUMINIUM_4 = pygame.Color(85, 87, 83)
COLOR_ALUMINIUM_4_5 = pygame.Color(66, 62, 64)
COLOR_ALUMINIUM_5 = pygame.Color(46, 52, 54)
COLOR_ORANGE_0 = pygame.Color(252, 175, 62)
COLOR_WHITE = pygame.Color(255, 255, 255)

COLOR_GREEN_CAR = pygame.Color(139, 209, 81)
COLOR_YELLOW_CAR = pygame.Color(255, 243, 0)
COLOR_RED_CAR = pygame.Color(239, 41, 41)
car_color_set = [COLOR_RED_CAR, COLOR_YELLOW_CAR, COLOR_GREEN_CAR]

class WorldImage(object):

    def __init__(self, world, map_name, is_draw):

        # read world info
        self.world = world

        # Maximum size of a pygame subsurface
        self._width_in_pixels = (1 << 14) - 1  # 16383

        # Adapt Pixels per meter to make world fits in big_map_surface
        surface_pixel_per_meter = int(self._width_in_pixels / self.world.map._world_width)
        # if surface_pixel_per_meter > PIXELS_PER_METER:
        #     surface_pixel_per_meter = PIXELS_PER_METER
        self._pixels_per_meter = surface_pixel_per_meter

        self.precision = 0.05
        self.leftVector = Vector3D()
        self.driving_width = 4
        self.shoulder_width = 0.3
        self.sidewalk_width = 4

        _ME_PATH = os.path.abspath(os.path.join(os.getcwd(), ".."))

        DATA_PATH = os.path.normpath(os.path.join(_ME_PATH, './data/cache'))
        filename = map_name
        filename += ".tga"
        filepath = str(os.path.join(DATA_PATH, filename))
        self.big_map_surface = pygame.Surface((self._width_in_pixels, self._width_in_pixels)).convert()
        # load a copy for drawing clean map image on screen
        self.surface = pygame.Surface((self._width_in_pixels, self._width_in_pixels)).convert()
        if not is_draw and os.path.isfile(filepath):
            # Load image
            self.big_map_surface = pygame.image.load(filepath)
            self.surface = pygame.image.load(filepath)
        else:
            # Render map
            self.draw_topology(self.big_map_surface)
            self.draw_topology(self.surface)
            pygame.image.save(self.big_map_surface, filepath)

    def world_to_pixel(self, location, offset=(0, 0)):
        # it needs to be mirrored
        x = self._pixels_per_meter * location[0]
        y = self._pixels_per_meter * (self.world.map._world_width - location[1])
        return [int(x - offset[0]), int(y - offset[1])]

    def pixel_to_world(self, x_pixel, y_pixel):
        x = x_pixel / self._pixels_per_meter
        y = -(y_pixel / self._pixels_per_meter - self.world.map._world_width)
        return [x, y]

    def cursor_to_world(self, x_pixel, y_pixel):
        x = x_pixel / self._pixels_per_meter + self.world.map._world_offset[0]
        y = -(y_pixel / self._pixels_per_meter - self.world.map._world_width) + self.world.map._world_offset[1]
        return [x, y]

    def draw_topology(self, surface):
        surface.fill(COLOR_ALUMINIUM_4)

        def classify_lane_type(lanes):
            cnt_driving = 0
            driving_idx = []
            sidewalk = False
            shoulder = False
            for idx, lane in enumerate(lanes):
                if lane.type == "driving":
                    cnt_driving += 1
                    driving_idx.append(idx)
                elif lane.type == "sidewalk":
                    sidewalk = True
                elif lane.type == 'shoulder':
                    shoulder = True
                elif lane.type == 'parking':
                    continue
                elif lane.type == 'none':
                    cnt_driving += 1
                    driving_idx.append(idx)
            return (cnt_driving, driving_idx, sidewalk, shoulder)

        def draw_lane(surface, waypoints_np, directions_left_np, lane_width, color):
            lane_left_side = waypoints_np + directions_left_np * lane_width / 2
            lane_right_side = np.flipud(waypoints_np - directions_left_np * lane_width / 2)
            polygon = np.concatenate((lane_left_side, lane_right_side), axis=0)
            polygon.tolist()
            polygon = [self.world_to_pixel(x) for x in polygon]
            if len(polygon) > 2:
                pygame.draw.polygon(surface, color, polygon)

        def draw_solid_line(surface, color, closed, points, width):
            """Draws solid lines in a surface given a set of points, width and color"""
            if len(points) >= 2:
                pygame.draw.lines(surface, color, closed, points, width)

        def draw_broken_line(surface, color, closed, points, width):
            """Draws broken lines in a surface given a set of points, width and color"""
            # Select which lines are going to be rendered from the set of lines
            broken_lines = [x for n, x in enumerate(zip(*(iter(points),) * 40)) if n % 2 == 0]

            # Draw selected lines
            for line in broken_lines:
                pygame.draw.lines(surface, color, closed, line, width)

        def draw_dashed_line(surface, color, start_pos, end_pos, width=8, dash_length=10):
            """Draws dashed line for liane geometry given start and end points"""
            origin = Point(start_pos)
            target = Point(end_pos)
            displacement = target - origin
            length = len(displacement)
            if length == 0:
                return
            slope = displacement / length

            for index in range(0, int(length / dash_length), 2):
                start = origin + (slope * index * dash_length)
                end = origin + (slope * (index + 1) * dash_length)
                pygame.draw.line(surface, color, start.get(), end.get(), width)

        def draw_color(type):
            if type == 'driving':
                color = COLOR_ALUMINIUM_5
            elif type == 'shoulder':
                color = COLOR_ALUMINIUM_4_5
            elif type == 'parking':
                color = COLOR_ALUMINIUM_4_5
            elif type == 'sidewalk':
                color = COLOR_ALUMINIUM_3
            elif type == 'border':
                color = COLOR_WHITE
            else:
                color = COLOR_ALUMINIUM_5
            return color

        for road in self.world.map.roads.values():
            scf_whole_road = 0
            for lane_section_idx, lane_section in enumerate(road.lanes.laneSections):
                # lane information
                # draw lane of current lane section
                # count road lane and sidewalk

                cnt_driving_left, idx_left, sidewalk_left, shoulder_left = classify_lane_type(lane_section.leftLanes)
                left_type_list = []
                for lane in lane_section.leftLanes:
                    left_type_list.append(lane.type)
                cnt_driving_right, idx_right, sidewalk_right, shoulder_right = classify_lane_type(lane_section.rightLanes)
                right_type_list = []
                for lane in lane_section.rightLanes:
                    right_type_list.append(lane.type)

                offsets = road.lanes.laneOffsets[lane_section_idx]
                a_of, b_of, c_of, d_of = offsets.a, offsets.b, offsets.c, offsets.d

                scf = 0
                center_waypoints = []
                left_waypoints = []
                for i in range(len(lane_section.leftLanes) + 1):
                    left_waypoints.append([])

                right_waypoints = []
                for i in range(len(lane_section.rightLanes) + 1):
                    right_waypoints.append([])

                wayppoints_left_direction = []
                while (scf < lane_section.length and scf_whole_road < road.planView.getLength()):
                    # hdg is the forward vecot of the current point
                    pos, hdg = road.planView.calc(scf_whole_road)
                    pos[0], pos[1] = pos[0] - self.world.map._world_offset[0], pos[1] - self.world.map._world_offset[1]
                    # Returns the lateral vector based on forward vector
                    self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
                    self.leftVector.rotation2D(np.pi / 2)
                    pos[0] += self.leftVector.x * (a_of + scf * b_of + pow(scf, 2) * c_of + pow(scf, 3) * d_of)
                    pos[1] += self.leftVector.y * (a_of + scf * b_of + pow(scf, 2) * c_of + pow(scf, 3) * d_of)
                    center_waypoints.append(pos)
                    left_waypoints[0].append(pos)
                    right_waypoints[0].append(pos)
                    wayppoints_left_direction.append([self.leftVector.x, self.leftVector.y])
                    # left driving lanes
                    current_idx = 1
                    if len(lane_section.leftLanes) > 0:
                        for idx_lane in range(len(lane_section.leftLanes)):
                            x_prev_idx, y_prev_idx = left_waypoints[current_idx - 1][-1]
                            current_lane_width = lane_section.leftLanes[idx_lane].widths[0]
                            a_width, b_width = current_lane_width.a, current_lane_width.b
                            c_width, d_width = current_lane_width.c, current_lane_width.d
                            x = x_prev_idx + self.leftVector.x * (a_width + scf * b_width +
                                                                  pow(scf, 2) * c_width + pow(scf, 3) * d_width)
                            y = y_prev_idx + self.leftVector.y * (a_width + scf * b_width +
                                                                  pow(scf, 2) * c_width + pow(scf, 3) * d_width)
                            
                            left_waypoints[current_idx].append([x, y])
                            current_idx += 1

                    current_idx = 1
                    if len(lane_section.rightLanes) > 0:
                        for idx_lane in range(len(lane_section.rightLanes)):
                            x_prev_idx, y_prev_idx = right_waypoints[current_idx - 1][-1]
                            current_lane_width = lane_section.rightLanes[idx_lane].widths[0]
                            a_width, b_width = current_lane_width.a, current_lane_width.b
                            c_width, d_width = current_lane_width.c, current_lane_width.d
                            x = x_prev_idx - self.leftVector.x * (a_width + scf * b_width +
                                                                  pow(scf, 2) * c_width + pow(scf, 3) * d_width)
                            y = y_prev_idx - self.leftVector.y * (a_width + scf * b_width +
                                                                  pow(scf, 2) * c_width + pow(scf, 3) * d_width)

                            right_waypoints[current_idx].append([x, y])
                            current_idx += 1

                    scf += self.precision
                    scf_whole_road += self.precision


                waypoints_np = np.asarray(center_waypoints)
                wayppoints_directions_left_np = np.asarray(wayppoints_left_direction)


                # draw main road
                if len(lane_section.leftLanes):
                    for idx in range(len(lane_section.leftLanes)):
                        lane_left_side =  np.asarray(left_waypoints[idx])
                        lane_right_side = np.flipud(np.asarray(left_waypoints[idx + 1]))
                        polygon = np.concatenate((lane_left_side, lane_right_side), axis=0)
                        polygon.tolist()
                        polygon = [self.world_to_pixel(x) for x in polygon]
                        if len(polygon) > 2:
                            current_color = draw_color(left_type_list[idx])
                            pygame.draw.polygon(surface, current_color, polygon)

                if len(lane_section.rightLanes):
                    for idx in range(len(lane_section.rightLanes)):
                        lane_left_side = np.asarray(right_waypoints[idx])
                        lane_right_side = np.flipud(np.asarray(right_waypoints[idx + 1]))
                        polygon = np.concatenate((lane_left_side, lane_right_side), axis=0)
                        polygon.tolist()
                        polygon = [self.world_to_pixel(x) for x in polygon]
                        if len(polygon) > 2:
                            current_color = draw_color(right_type_list[idx])
                            pygame.draw.polygon(surface, current_color, polygon)


                if (road.junction == None and (cnt_driving_right > 0 and cnt_driving_left > 0)):
                    # draw center line
                    left_center_line = waypoints_np + wayppoints_directions_left_np * 0.25
                    right_center_line = waypoints_np - wayppoints_directions_left_np * 0.25
                    left_center_line.tolist()
                    right_center_line.tolist()
                    left_center_line = [self.world_to_pixel(x) for x in left_center_line]
                    right_center_line = [self.world_to_pixel(x) for x in right_center_line]
                    draw_solid_line(surface, COLOR_ORANGE_0, False, left_center_line, int(0.3 * self._pixels_per_meter))
                    draw_solid_line(surface, COLOR_ORANGE_0, False, right_center_line, int(0.3 * self._pixels_per_meter))

                # draw dashed line
                left_cnt = cnt_driving_left - 1
                right_cnt = cnt_driving_right - 1
                if (road.junction == None):
                    while(left_cnt > 0):
                        dashed_line_left = waypoints_np + wayppoints_directions_left_np * self.driving_width * left_cnt
                        dashed_line_left.tolist()
                        dashed_line_left = [self.world_to_pixel(x) for x in dashed_line_left]
                        draw_broken_line(surface, COLOR_ALUMINIUM_2, False, dashed_line_left, int(0.3 * self._pixels_per_meter))
                        left_cnt -= 1

                    while(right_cnt > 0):
                        dashed_line_right = waypoints_np - wayppoints_directions_left_np * self.driving_width * right_cnt
                        dashed_line_right.tolist()
                        dashed_line_right = [self.world_to_pixel(x) for x in dashed_line_right]
                        draw_broken_line(surface, COLOR_ALUMINIUM_2, False, dashed_line_right, int(0.3 * self._pixels_per_meter))
                        right_cnt -= 1


    def draw_vehicle(self, t):
        for vehicle_id in self.world.vehicle_record[t]:
            shape = self.world.vehicle_record[t][vehicle_id][0]
            plan_tra = self.world.vehicle_record[t][vehicle_id][1]
            shape_in_pixel = []
            for line in shape:
                line_in_pixel = (self.world_to_pixel(line[0]), self.world_to_pixel(line[1]))
                for point in line_in_pixel:
                    if point not in shape_in_pixel:
                        shape_in_pixel.append(point)
                # pygame.draw.line(self.surface, color, line_in_pixel[0], line_in_pixel[1], 25)

            color = car_color_set[2]
            pygame.draw.polygon(self.surface, color, shape_in_pixel, 0)

            for i in range(len(plan_tra)):
                vehicle_point = plan_tra[i]
                color = car_color_set[min(i // 5, 2)]
                pygame.draw.circle(self.surface, color, self.world_to_pixel(vehicle_point), 10, 10)

    def draw_chosen_vehicle(self, t, vehicle_id):
        shape = self.world.vehicle_record[t][vehicle_id][0]
        shape_in_pixel = []
        for line in shape:
            line_in_pixel = (self.world_to_pixel(line[0]), self.world_to_pixel(line[1]))
            for point in line_in_pixel:
                if point not in shape_in_pixel:
                    shape_in_pixel.append(point)

        color = car_color_set[0]
        pygame.draw.polygon(self.surface, color, shape_in_pixel, 0)

    def draw_traffic_light(self, t):
        for index in self.world.tl_record[t]:
            pos = []
            p1_to_world = self.world_to_pixel(self.world.tls[index].p1)
            p2_to_world = self.world_to_pixel(self.world.tls[index].p2)
            if self.world.tl_record[t][index] == 'r':
                color = COLOR_SCARLET_RED_0
            elif self.world.tl_record[t][index] == 'y':
                color = COLOR_BUTTER_0
            elif self.world.tl_record[t][index] == 'g':
                color = COLOR_CHAMELEON_0
            
            pygame.draw.line(self.surface,color,p1_to_world,p2_to_world,40)
