import numpy as np
import os, sys
from ..geom.Vector3D import Vector3D
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

COLOR_GREEN_CAR = pygame.Color(139, 209, 81)
COLOR_YELLOW_CAR = pygame.Color(255, 243, 0)
COLOR_RED_CAR = pygame.Color(239, 41, 41)
car_color_set = [COLOR_RED_CAR, COLOR_YELLOW_CAR, COLOR_GREEN_CAR]

class WorldImage(object):

    def __init__(self, world, display, map_name):

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
        if os.path.isfile(filepath):
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
            sidewalk = False
            shoulder = False
            for lane in lanes:
                if lane.type == "driving":
                    cnt_driving += 1
                elif lane.type == "sidewalk":
                    sidewalk = True
                else:
                    shoulder = True
            return (cnt_driving, sidewalk, shoulder)

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
            broken_lines = [x for n, x in enumerate(zip(*(iter(points),) * 20)) if n % 3 == 0]

            # Draw selected lines
            for line in broken_lines:
                pygame.draw.lines(surface, color, closed, line, width)

        for road in self.world.map.roads.values():
            geometries = road.planView.geometries
            # draw lane of current geometry
            # count road lane and sidewalk
            # write lateral shift of the road and draw lane to solve it
            # todo
            cnt_driving_left, sidewalk_left, shoulder_left = classify_lane_type(road.lanes.laneSections[0].leftLanes)
            cnt_driving_right, sidewalk_right, shoulder_right = classify_lane_type(
                road.lanes.laneSections[0].rightLanes)

            for geometry in geometries:
                scf = 0
                waypoints = []
                wayppoints_left_direction = []
                if (geometry.getGeoType() == 'Arc'):
                    while (geometry.getLength() > scf):
                        # hdg is the forward vecot of the current point
                        pos, hdg = geometry.calcPosition(scf)
                        pos[0], pos[1] = pos[0] - self.world.map._world_offset[0], pos[1] - self.world.map._world_offset[1]
                        # Returns the lateral vector based on forward vector
                        self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
                        self.leftVector.rotation2D(np.pi / 2)
                        waypoints.append(pos)
                        wayppoints_left_direction.append([self.leftVector.x, self.leftVector.y])
                        scf += self.precision
                else:
                    # Assume other type only include Line
                    # only start and end position are added
                    pos, hdg = geometry.calcPosition(0)
                    pos[0], pos[1] = pos[0] - self.world.map._world_offset[0], pos[1] - self.world.map._world_offset[1]
                    self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
                    self.leftVector.rotation2D(np.pi / 2)
                    waypoints.append(pos)
                    wayppoints_left_direction.append([self.leftVector.x, self.leftVector.y])

                    pos, hdg = geometry.calcPosition(geometry.getLength())
                    pos[0], pos[1] = pos[0] - self.world.map._world_offset[0], pos[1] - self.world.map._world_offset[1]
                    self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
                    self.leftVector.rotation2D(np.pi / 2)
                    waypoints.append(pos)
                    wayppoints_left_direction.append([self.leftVector.x, self.leftVector.y])


                waypoints_np = np.asarray(waypoints)
                wayppoints_directions_left_np = np.asarray(wayppoints_left_direction)

                # draw classified lane type
                if (sidewalk_left):
                    sidewalk_waypoints = waypoints_np + wayppoints_directions_left_np * \
                                         (
                                                 self.driving_width * cnt_driving_left + self.shoulder_width + self.sidewalk_width / 2)
                    draw_lane(surface, sidewalk_waypoints, wayppoints_directions_left_np, self.sidewalk_width,
                              COLOR_ALUMINIUM_3)
                if (sidewalk_right):
                    sidewalk_waypoints = waypoints_np - wayppoints_directions_left_np * \
                                         (
                                                 self.driving_width * cnt_driving_left + self.shoulder_width + self.sidewalk_width / 2)
                    draw_lane(surface, sidewalk_waypoints, wayppoints_directions_left_np, self.sidewalk_width,
                              COLOR_ALUMINIUM_3)
                if (shoulder_left):
                    sidewalk_waypoints = waypoints_np + wayppoints_directions_left_np * \
                                         (self.driving_width * cnt_driving_left + self.shoulder_width / 2)
                    draw_lane(surface, sidewalk_waypoints, wayppoints_directions_left_np, self.shoulder_width,
                              COLOR_ALUMINIUM_4_5)
                if (shoulder_right):
                    sidewalk_waypoints = waypoints_np - wayppoints_directions_left_np * \
                                         (self.driving_width * cnt_driving_left + self.shoulder_width / 2)
                    draw_lane(surface, sidewalk_waypoints, wayppoints_directions_left_np, self.shoulder_width,
                              COLOR_ALUMINIUM_4_5)

                # draw main road
                lane_left_side = waypoints_np + wayppoints_directions_left_np * self.driving_width * cnt_driving_left
                lane_right_side = np.flipud(
                    waypoints_np - wayppoints_directions_left_np * self.driving_width * cnt_driving_right)

                polygon = np.concatenate((lane_left_side, lane_right_side), axis=0)
                polygon.tolist()
                polygon = [self.world_to_pixel(x) for x in polygon]
                if len(polygon) > 2:
                    pygame.draw.polygon(surface, COLOR_ALUMINIUM_5, polygon)

                if (road.junction == None):
                    # draw center line
                    left_center_line = waypoints_np + wayppoints_directions_left_np * 0.25
                    right_center_line = waypoints_np - wayppoints_directions_left_np * 0.25
                    left_center_line.tolist()
                    right_center_line.tolist()
                    left_center_line = [self.world_to_pixel(x) for x in left_center_line]
                    right_center_line = [self.world_to_pixel(x) for x in right_center_line]
                    draw_solid_line(surface, COLOR_ORANGE_0, False, left_center_line, 8)
                    draw_solid_line(surface, COLOR_ORANGE_0, False, right_center_line, 8)

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
