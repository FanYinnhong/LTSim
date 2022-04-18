import threading
import math, os, sys, pdb

from pygame.locals import *
import numpy as np
import time, random
import heapq
import matplotlib.path as malPath
# in order to validate polygon shape
import matplotlib.pyplot as plt
from descartes import PolygonPatch

sys.path.append('..')
from lxml import etree
from lib.opendriveparser import parse_opendrive
from lib.geom.Vector3D import Vector3D
from visualization.palette import *
from carflow import generateCarFlow
import pandas as pd
from shapely.geometry import Polygon
from shapely.ops import cascaded_union
# from behavir_models import *
from frenet_planning import *

SAMPLE_TIME = 0.1


class Map(object):
    def __init__(self, network):
        self.roads = network.roads
        self.junctions = network.junctions
        self._road_id_to_junction_id = {}
        self.junction_road()
        self.connection_to_leaving()

        self._all_start_points = []
        self.get_all_start_points()
        self.get_junction_coor()
        # Road network's width or height.
        # left some space between road network and Pygame Window
        self.margin = 30
        min_x = min(self._all_start_points, key=lambda point: point[0])[0] - self.margin
        min_y = min(self._all_start_points, key=lambda point: point[1])[1] - self.margin
        max_x = max(self._all_start_points, key=lambda point: point[0])[0] + self.margin
        max_y = max(self._all_start_points, key=lambda point: point[1])[1] + self.margin
        self._world_width = max(max_x - min_x, max_y - min_y)
        self._world_offset = (min_x, min_y)
        self.leftVector = Vector3D()
        self.driving_width = 4
        # select a junciton id to get the shape
        self.shape = self.junction_polygon(60)
        self.shape_x, self.shape_y = self.shape.exterior.coords.xy
        del self._all_start_points


    def get_junction_coor(self):
        for junction in self.junctions.values():
            x_min = 100000
            y_max = -100000
            for connection in junction.connections:
                road = self.roads[connection.connectingRoad]
                x, y =road.planView.geometries[0].startPosition
                x_min = min(x_min, x)
                y_max = max(y_max, y)
            junction.pos_left_top = (x_min, y_max)

    def get_all_start_points(self):
        for road in self.roads.values():
            geometries = road.planView.geometries
            for geometry in geometries:
                self._all_start_points.append(geometry.getStartPosition())

    def junction_road(self):
        for road in self.roads.values():
            if road.junction != None:
                junction_id = road.junction
                # connecting road index for each junction
                for connection_idx, connection in enumerate(
                        self.junctions[junction_id].connections):
                    if connection.connectingRoad == road.id:
                        self._road_id_to_junction_id[road.id] = (junction_id, connection_idx)

    def connection_to_leaving(self):  # connectingRoad_to_leavingRoad
        for junction in self.junctions.values():
            for connection in junction.connections:
                # find leavingRoad id
                connectingRoad_id = connection.connectingRoad
                incomingRoad_id = connection.incomingRoad
                toId = connection.laneLinks[0].toId
                connectingRoad = self.roads[connectingRoad_id]
                if connectingRoad.link.predecessor.elementId != incomingRoad_id:
                    connection.leavingRoad = connectingRoad.link.predecessor.elementId
                    connection.contactPointOnLeavingRoad = connectingRoad.link.predecessor.contactPoint
                elif connectingRoad.link.successor.elementId != incomingRoad_id:
                    connection.leavingRoad = connectingRoad.link.successor.elementId
                    connection.contactPointOnLeavingRoad = connectingRoad.link.successor.contactPoint

    def frenet_to_cartesian(self, s, t, road_id):
        if (self.roads[road_id].length < s):
            raise OverflowError("Exceeds the limits of current road")

        pos, hdg = self.roads[road_id].planView.calc(s)
        self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
        x = pos[0] + self.leftVector.x * t
        y = pos[1] + self.leftVector.x * t
        return (x, y)

    def cartesian_to_frenet(self, x, y):
        nearest_road_id = self.nearest_start_points(x, y)
        road_forward = Vector3D()
        point_vector = Vector3D()
        for _, road_id in nearest_road_id:
            road = self.roads[road_id]
            s_current = 0
            t_current = 0
            # assuming tha each step is 0.3m
            while(s_current < road.length):
                pos. hdg = road.planView.calc(s_current)
                road_forward.x, road_forward.y = np.cos(hdg), np.sin(hdg)
                point_vector.x, point_vector.y = x - pos[0], y -pos[1]
                if (road_forward.dot(point_vector) < 0):
                    break
                s_current += 0.3
            t_current =  point_vector.length
            # check whether point is on the left side of the road
            road_forward.rotation2D(np.pi / 2)
            if road_forward.dot(point_vector) < 0:
                t_current = - t_current
            # we assume biggest width of the road is 10m.
            if t > 10 or t < -10 or s == 0:
                continue
            else:
                return (s_current, t_current, road_id)


    def collision_check(self, x, y):

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

        nearest_start_id = self.nearest_start_points(x, y)

        for _, start_id in nearest_start_id:
            geometries = self.roads[start_id].planView.geometries
            cnt_driving_left, sidewalk_left, shoulder_left = classify_lane_type(self.roads[start_id].lanes.laneSections[0].leftLanes)
            cnt_driving_right, sidewalk_right, shoulder_right = classify_lane_type(
                self.roads[start_id].lanes.laneSections[0].rightLanes)

            for geometry in geometries:
                pos_frenet = 0
                waypoints = []
                wayppoints_left_direction = []
                geo_type = geometry.getGeoType()

                # get the initial waypoints
                # it should gurantee that the waypoints in clockwise or counter clockwise
                pos, hdg = geometry.calcPosition(pos_frenet)
                # Returns the lateral vector based on forward vector
                self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
                self.leftVector.rotation2D(np.pi / 2)
                waypoints.append(pos)
                wayppoints_left_direction.append([self.leftVector.x, self.leftVector.y])

                if geo_type == "Line":
                    pos, hdg = geometry.calcPosition(geometry.getLength())
                    self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
                    self.leftVector.rotation2D(np.pi / 2)
                    waypoints.append(pos)
                    wayppoints_left_direction.append([self.leftVector.x, self.leftVector.y])

                elif geo_type == "Arc":
                    while(geometry.getLength() > pos_frenet):
                        pos, hdg = geometry.calcPosition(pos_frenet)
                        self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
                        self.leftVector.rotation2D(np.pi / 2)
                        waypoints.append(pos)
                        wayppoints_left_direction.append([self.leftVector.x, self.leftVector.y])
                        pos_frenet += 1

                waypoints_np = np.asarray(waypoints)
                wayppoints_directions_left_np = np.asarray(wayppoints_left_direction)
                # get main road
                # need some padding to get system more robust
                lane_left_side = waypoints_np + wayppoints_directions_left_np * self.driving_width * cnt_driving_left
                lane_right_side = np.flipud(
                    waypoints_np - wayppoints_directions_left_np * self.driving_width * cnt_driving_right)
                polygon = np.concatenate((lane_left_side, lane_right_side), axis=0)
                # print(polygon)
                # print(x,y)
                # polygon.tolist()
                bb_path = malPath.Path(polygon)
                is_in = bb_path.contains_point([x, y], radius=0.01) or bb_path.contains_point([x, y],radius=-0.01)

                # if is_inside(polygon, (x, y)):
                if is_in:
                    return start_id
        return -1

    def nearest_start_points(self, x, y):
        """ choose the five nearest road"""
        nearest_start_id = []
        for _, road in enumerate(self.roads.values()):
            start_x, start_y = road.planView.geometries[0].getStartPosition()
            temp = (x - start_x) ** 2 + (y - start_y) ** 2
            nearest_start_id.append([temp, road.id])
        heapq.heapify(nearest_start_id)
        res = []
        for _ in range(5):
            res.append(heapq.heappop(nearest_start_id))
        return res

    def junction_polygon(self, junction_id):
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

        def combineBorders(*geoms):
            return cascaded_union([
                geom if geom.is_valid else geom.buffer(0) for geom in geoms
            ])

        # creating polygon for each connecting road in the junction
        polygon_list = []
        for connection in self.junctions[junction_id].connections:
            road = self.roads[connection.connectingRoad]
            geometries = self.roads[connection.connectingRoad].planView.geometries
            cnt_driving_left, sidewalk_left, shoulder_left = classify_lane_type(road.lanes.laneSections[0].leftLanes)
            cnt_driving_right, sidewalk_right, shoulder_right = classify_lane_type(
                road.lanes.laneSections[0].rightLanes)

            for geometry in geometries:
                pos_frenet = 0
                waypoints = []
                wayppoints_left_direction = []
                geo_type = geometry.getGeoType()

                # get the initial waypoints
                # it should gurantee that the waypoints in clockwise or counter clockwise
                pos, hdg = geometry.calcPosition(pos_frenet)
                # Returns the lateral vector based on forward vector
                self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
                self.leftVector.rotation2D(np.pi / 2)
                waypoints.append(pos)
                wayppoints_left_direction.append([self.leftVector.x, self.leftVector.y])

                if geo_type == "Line":
                    pos, hdg = geometry.calcPosition(geometry.getLength())
                    self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
                    self.leftVector.rotation2D(np.pi / 2)
                    waypoints.append(pos)
                    wayppoints_left_direction.append([self.leftVector.x, self.leftVector.y])

                elif geo_type == "Arc":
                    while(geometry.getLength() > pos_frenet):
                        pos, hdg = geometry.calcPosition(pos_frenet)
                        self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
                        self.leftVector.rotation2D(np.pi / 2)
                        waypoints.append(pos)
                        wayppoints_left_direction.append([self.leftVector.x, self.leftVector.y])
                        pos_frenet += 1

                waypoints_np = np.asarray(waypoints)
                wayppoints_directions_left_np = np.asarray(wayppoints_left_direction)
                # get main road
                # need some padding to get system more robust
                lane_left_side = waypoints_np + wayppoints_directions_left_np * self.driving_width * cnt_driving_left
                lane_right_side = np.flipud(
                    waypoints_np - wayppoints_directions_left_np * self.driving_width * cnt_driving_right)
                temp = np.concatenate((lane_left_side, lane_right_side), axis=0)
                temp.tolist()
                # print(temp)
                # print("next")
                polygon_temp = Polygon(temp)
                print(polygon_temp.area)
                # print(polygon_temp)
                polygon_list.append(polygon_temp)
        u = polygon_list[0]
        for i in range(1, len(polygon_list)):
            u = combineBorders(u, polygon_list[i])

        # # plot the figure
        # for geom in u.geoms:
        #     plt.plot(*geom.exterior.xy)
        # plt.gca().axis("equal")
        # plt.show()
        BLUE = '#6699cc'
        GRAY = '#999999'
        fig = plt.figure()
        ax = fig.gca()
        ax.add_patch(PolygonPatch(u, fc=GRAY, ec=BLUE, alpha=0.5, zorder=2))
        ax.axis('scaled')
        plt.show()
        return u


class Vehicle(object):

    def __init__(self, id, world, map, road_id, direction, lane, scf, ST, info):
        # map info
        self.world = world
        self.map = map
        self.road_id = road_id
        self.lane = lane  # lane number: it represents the number of lane toward to centerline
        self.direction = direction  # 1 - the left side, 0 - the right side

        # attributes
        self.id = id
        self.vehicle_l, self.vehicle_w = info[4], info[5]
        self.shape = []

        # motion status
        self.scf = scf  # s-axis coordinate value of Frenet coordinate system
        self.scf_end = self.map.roads[self.road_id].length
        self.tcf = ((self.lane - 1) * 4 + 2)  # t-axis coordinate value  (absolute value)
        self.vel = info[3]
        self.hdg = 0
        self.ACC_MAX = 3
        self.DCC_MAX = 10
        self.EXPECT_VEL = info[3]
        # surrounding
        self.sur_vehicles = []
        self.plan_period = 20
        self.plan_gap = 5
        self.plan_road_id = 0
        self.plan_tra = []
        self.perceptron = Perceptron(self)
        self.predictor = Predictor(self)
        self.planner = Planner(self)

    def move(self, d_status, tl_record_t):
        # check whether vehicle is in road or junctions
        dp_s, dp_t, d_v, d_hdg = d_status
        # self.scf += dp_s
        self.scf += 2
        self.vel += d_v
        self.tcf += dp_t
        self.hdg += d_hdg
        pos_status = self.road_id, self.lane, self.scf, self.direction, self.scf_end

        def update_pos(pos_status, map):
            road_id, lane, scf, direction, scf_end = pos_status
            road = map.roads[road_id]
            is_junction = False if road.junction == None else True
            if (is_junction):
                # calculate pos_length
                # it needs to check the direction of the current road, it is determined by the contact point of the connecting road
                junction_id = map._road_id_to_junction_id[road_id]
                connection = map.junctions[junction_id[0]].connections[junction_id[1]]
                if scf > map.roads[connection.connectingRoad].length:
                    # the remain frenet in the new road
                    scf_temp = scf - map.roads[connection.connectingRoad].length
                    # determine the new road
                    item_leavingRoad = map.roads[connection.leavingRoad]
                    # update the road id of the new road
                    road_id = connection.leavingRoad

                    scf = scf_temp
                    scf_end = item_leavingRoad.length
                    if connection.contactPointOnLeavingRoad == "start":
                        direction = 0
                        # lane = 1
                    elif connection.contactPointOnLeavingRoad == "end":
                        direction = 1
                        # lane = 1
            else:  # drive on the road segment
                # drive along the positive position of the road
                if direction == 0:
                    road_next = road.link.successor
                else:
                    road_next = road.link.predecessor
                if (scf > road.length):
                    # if next is a junction
                    if road_next.elementType == "junction":
                        # Choose a road in junction's connections groups
                        junction = self.map.junctions[road_next.elementId]
                        connection_available = []
                        for connection in junction.connections:
                            if connection.incomingRoad == road.id:
                                connection_available.append(connection)
                        connection_chosen = random.choice(connection_available)
                        connectingRoad = connection_chosen.connectingRoad
                        if ((connectingRoad in tl_record_t) and tl_record_t[connectingRoad] == 'r'):
                            scf = road.length
                        else:
                            contactPoint = connection_chosen.contactPoint
                            # lane = connection_chosen.laneLinks[0].toId
                            scf_temp = scf - road.length
                            scf = scf_temp
                            scf_end = map.roads[connectingRoad].length
                            if contactPoint == "start":
                                direction = 0
                                # lane = 1
                            else:
                                direction = 1
                                # lane = 1
                            road_id = connectingRoad
                    # if next is a road
                    else:
                        scf_temp = scf - road.length
                        scf = scf_temp
                        scf_end = map.roads[road_next.elementId].length
                        if road_next.contactPoint == "start":
                            direction = 0
                            # lane = 1
                        else:
                            direction = 1
                            # lane = 1
                        road_id = road_next.elementId

            if scf > self.map.roads[road_id].length:
                pdb.set_trace()
            return road_id, lane, direction, scf, scf_end

        road_id, lane, direction, scf, scf_end = update_pos(pos_status, self.map)
        # pdb.set_trace()
        attitude = self.get_position(road_id, direction, scf, self.tcf, self.hdg)
        shape_points = get_shape_pos(self, attitude)
        if 0 and self.is_collide(attitude, shape_points) and self.vel > 0:
            self.vel = max(self.vel - 1, 0)
            self.move(d_status)
        else:
            self.road_id, self.lane, self.scf, self.direction, self.scf_end = road_id, lane, scf, direction, scf_end
            self.shape = shape_points

    def get_position(self, road_id, direction, scf, tcf, hdg):
        geometries = self.map.roads[road_id].planView.geometries
        if direction == 1:
            scf = self.map.roads[road_id].length - scf
        for idx, geometry in enumerate(geometries):
            if (scf > geometry.getLength()):
                scf -= geometry.getLength()
            else:
                pos, road_hdg = geometry.calcPosition(scf)
                break
        x, y = pos[0] - self.map._world_offset[0], pos[1] - self.map._world_offset[1]

        if direction > 0:
            rotation_radian = road_hdg + hdg
        else:
            rotation_radian = road_hdg - hdg
        # Returns the lateral vector based on forward vector
        leftVector = Vector3D()
        leftVector.x, leftVector.y = np.cos(rotation_radian), np.sin(rotation_radian)
        leftVector.rotation2D(-np.pi / 2)
        if direction > 0:
            x, y = x - leftVector.x * tcf, y - leftVector.y * tcf
            rotation_radian += np.pi
        else:
            x, y = x + leftVector.x * tcf, y + leftVector.y * tcf
            rotation_radian = rotation_radian
        return [x, y, rotation_radian]

    def is_collide(self, attitude, shape_points):
        def point2point(ego_ps, other_ps):
            ps_1 = [it[0] for it in ego_ps]
            ps_2 = [it[0] for it in other_ps]
            for p1 in ps_1:
                for p2 in ps_2:
                    if np.hypot(p1[0] - p2[0], p1[1] - p2[1]) < 1:
                        return True
            return False

        for car in self.world.vehicle_running:
            car = self.world.vehicle_running[car]
            if self != car:
                if np.hypot(attitude[0] - car.x, attitude[1] - car.y) < 6:
                    if point2point(shape_points, car.shape):
                        return True
        return False


class TrafficLight(object):
    def __init__(self, pos, init_phase, time_green, time_yellow, time_red, connect_id_list):
        self.connect_id_list = connect_id_list
        self.x = pos[0]
        self.y = pos[1]

        self.init_phase = init_phase
        self.time_green = time_green
        self.time_yellow = time_yellow
        self.time_red = time_red
        self.current_state = 'g'

        # self.time = 0
        # self.state_calculation()

    def state_calculation(self, current_time):
        time_temp = (current_time + self.init_phase) % (self.time_yellow + self.time_green + self.time_red)
        if time_temp < self.time_green:
            self.current_state = 'g'  # it means current state is green
        elif (time_temp > self.time_green and time_temp <= self.time_green + self.time_yellow):
            self.current_state = 'y'
        elif time_temp > self.time_green + self.time_yellow:
            self.current_state = 'r'


class Text(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=COLOR_WHITE):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.surface.fill(COLOR_BLACK)
        self.surface.blit(text_texture, (10, 11))


class WorldImage(object):

    def __init__(self, world, display):

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

        _ME_PATH = os.path.abspath(os.path.dirname(__file__))
        DATA_PATH = os.path.normpath(os.path.join(_ME_PATH, '../data/cache'))
        filename = "Town01_new.tga"
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
                # draw waypoints
                if (road.junction == None):
                    draw_lane(surface, waypoints_np, wayppoints_directions_left_np, 0.5, COLOR_ALUMINIUM_2)

    def draw_vehicle(self, t):
        for vehicle_id in self.world.vehicle_record[t]:
            shape = self.world.vehicle_record[t][vehicle_id]
            for line in shape:
                color = car_color_set[2]
                line_in_pixel = (self.world_to_pixel(line[0]), self.world_to_pixel(line[1]))
                pygame.draw.line(self.surface, color, line_in_pixel[0], line_in_pixel[1], 25)

    def draw_traffic_light(self, t):
        def make_surface(state):
            """Draws a traffic light, which is composed of a dark background surface with 3 circles that indicate its color depending on the state"""
            w = 100
            surface = pygame.Surface((w, 3 * w), pygame.SRCALPHA)
            surface.fill(COLOR_ALUMINIUM_5 if state != 'h' else COLOR_ORANGE_2)
            if state != 'h':
                hw = int(w / 2)
                off = COLOR_ALUMINIUM_4
                red = COLOR_SCARLET_RED_0
                yellow = COLOR_BUTTER_0
                green = COLOR_CHAMELEON_0

                # Draws the corresponding color if is on, otherwise it will be gray if its off
                pygame.draw.circle(surface, red if state == 'r' else off, (hw, hw), int(0.4 * w))
                pygame.draw.circle(surface, yellow if state == 'y' else off, (hw, w + hw), int(0.4 * w))
                pygame.draw.circle(surface, green if state == 'g' else off, (hw, 2 * w + hw), int(0.4 * w))

            return pygame.transform.scale(surface, (self._pixels_per_meter * 5, self._pixels_per_meter * 8))

        for index in self.world.tl_record[t]:
            pos = []
            pos.append(int(self.world.tls[index].x))
            pos.append(int(self.world.tls[index].y))

            tl_surface = make_surface(self.world.tl_record[t][index])
            x, y = self.world_to_pixel(pos)
            self.surface.blit(tl_surface, (x, y))


class World(object):
    """
    Class that contains all the information of
    simulation that is running on the server side
    """

    def __init__(self, run_time):
        self.run_time = run_time
        # World data
        self.map = None
        self.vehicle_running = {}
        self.vehicle_demand = {}
        self.vehicle_record = {}
        self.tls = {}
        self.tl_record = {}
        self.tl_control_record = {}

    def load_map(self):
        "load map files and get infos"

        def read_map_file():
            # read road network
            _ME_PATH = os.path.abspath(os.path.dirname(__file__))
            DATA_PATH = os.path.normpath(os.path.join(_ME_PATH, '../data/xml'))
            filename = "Town01_new.xodr"
            filepath = os.path.join(DATA_PATH, filename)
            # get road network info
            with open(filepath, 'r') as f:
                parser = etree.XMLParser()
                rootNode = etree.parse(f, parser).getroot()
                roadNetwork = parse_opendrive(rootNode)
            return roadNetwork

        self.map = Map(read_map_file())
        # Map info
        # self.surface_size = self.map.big_map_surface.get_width() # 16383

    def save_map(self):
        "save map to csv"
        _ME_PATH = os.path.abspath(os.path.dirname(__file__))
        DATA_PATH = os.path.normpath(os.path.join(_ME_PATH, '../data'))
        filename = "road.csv"
        filepath = os.path.join(DATA_PATH, filename)

        road_column = ['id', 'start point x', 'start point y', 'end point x','end point y','length',
                       'predecessor element id', 'predecessor element type',
                       'successor element id', 'successor element type']
        road_list = []
        for road in self.map.roads.values():
            start_point, _ = road.planView.calc(0)
            end_point, _ = road.planView.calc(road.length)
            list_temp = [road.id, start_point[0], start_point[1], end_point[0], end_point[1], road.length,
                         road.link.predecessor.elementId, road.link.predecessor.elementType,
                         road.link.successor.elementId, road.link.successor.elementType]
            road_list.append(list_temp)
        road_dataframe = pd.DataFrame(columns = road_column, data = road_list)
        road_dataframe.to_csv(filepath)

        junction_column = ['id', 'incoming road id', 'connecting road id', 'leaving road id']
        junction_list = []
        for junction in self.map.junctions.values():
            for connection in junction.connections:
                list_temp = [junction.id, connection.incomingRoad, connection.connectingRoad,
                             connection.leavingRoad]
                junction_list.append(list_temp)
        junction_dataframe = pd.DataFrame(columns = junction_column, data = junction_list)
        filename = "junction.csv"
        filepath = os.path.join(DATA_PATH, filename)
        junction_dataframe.to_csv(filepath)

    def generate_flow(self):
        flow = generateCarFlow(self.run_time, SAMPLE_TIME, 26)
        for t in range(self.run_time):
            for vehicle_info in flow:
                from_road_id = vehicle_info[0]
                to_road_id = vehicle_info[1]
                arrival_time = vehicle_info[2]
                temp_id = (from_road_id, to_road_id, arrival_time)
                if temp_id not in self.vehicle_demand and arrival_time > t:
                    scf = 0
                    lane_number = random.randint(1, 2)
                    direction = 0 if lane_number > 0 else 1
                    self.vehicle_demand[temp_id] = Vehicle(temp_id, self, self.map, from_road_id, direction,
                                                           abs(lane_number), scf, SAMPLE_TIME, vehicle_info)

    def action(self):
        for t in range(self.run_time):
            if t % 10 == 0:
                print('running time: ', t)

            # update the state of traffic light at current time
            self.tl_record[t] = {}
            self.tl_control_record[t] = {}
            for index in self.tls:
                self.tls[index].state_calculation(t)
                self.tl_record[t][index] = self.tls[index].current_state
                for connect_id in self.tls[index].connect_id_list:
                    self.tl_control_record[t][connect_id] = self.tls[index].current_state

            self.vehicle_record[t] = {}
            demand_pop = []
            for vehicle_id in self.vehicle_demand:
                if self.vehicle_demand[vehicle_id] != -1:
                    if t > vehicle_id[2]:
                        self.vehicle_running[vehicle_id] = self.vehicle_demand[vehicle_id]
                        demand_pop.append(vehicle_id)
            for vehicle_id in demand_pop:
                self.vehicle_demand.pop(vehicle_id)  # del vehicle

            running_pop = []
            for vehicle_id in self.vehicle_running:
                rt = 0.5  # reaction time
                # perception
                # ego status
                # other statuses
                if t + rt < self.run_time:
                    pass
                    # planning process, planning, expect_tra
                    # upate the state in car_status, real_tra
                vehicle = self.vehicle_running[vehicle_id]
                if len(
                        vehicle.plan_tra) < vehicle.plan_period - vehicle.plan_gap or vehicle.plan_road_id != vehicle.road_id:
                    vehicle.plan_road_id = vehicle.road_id
                    sur_traffic = vehicle.perceptron.get_sur_traffic(self.vehicle_running)
                    sur_traffic = vehicle.predictor.predict(sur_traffic, vehicle.plan_period)
                    motion_status = [vehicle.scf, vehicle.tcf, vehicle.vel, vehicle.hdg, vehicle.scf_end]
                    # pdb.set_trace()
                    vehicle.plan_tra = vehicle.planner.planning(motion_status, sur_traffic, vehicle.plan_period)
                    try:
                        cur_move = vehicle.plan_tra.pop(0)
                    except:
                        pdb.set_trace()
                        self.map.roads[vehicle.road_id].length
                else:
                    cur_move = vehicle.plan_tra.pop(0)
                vehicle.move(cur_move, self.tl_control_record[t])
                if vehicle.road_id == vehicle_id[1]:  # arrive at to_road
                    running_pop.append(vehicle_id)
                else:
                    self.vehicle_record[t][vehicle_id] = vehicle.shape[:]
            for vehicle_id in running_pop:
                self.vehicle_running.pop(vehicle_id)  # del vehicle

    def create_traffic_light(self):
        def get_init_pos(junction_id, connectingroad_id, right_vector):
            # connecting road is chosen to calculate road id
            junction = self.map.junctions[junction_id]
            for connection in junction.connections:
                if (connection.connectingRoad == connectingroad_id):
                    contact_point_temp = connection.contactPoint
            whether_right = 1
            whether_forward = 1
            if contact_point_temp == 'end':
                s_frenet = self.map.roads[connectingroad_id].length
                whether_right = -1
            else:  # contact_point_temp == 'start'
                s_frenet = 0
                whether_forward = -1
            geometries = self.map.roads[connectingroad_id].planView.geometries
            for idx, geometry in enumerate(geometries):
                if (s_frenet > geometry.getLength()):
                    s_frenet -= geometry.getLength()
                else:
                    pos, hdg = geometry.calcPosition(s_frenet)
                    break

            x, y = pos[0] - self.map._world_offset[0], pos[1] - self.map._world_offset[1]

            # set the tl position away from the cross sections
            right_vector.x, right_vector.y = np.cos(hdg), np.sin(hdg)
            x = x + right_vector.x * 10 * whether_forward
            y = y + right_vector.y * 10 * whether_forward

            right_vector.rotation2D(-np.pi / 2)
            x = x + right_vector.x * 20 * whether_right
            y = y + right_vector.y * 20 * whether_right
            return (x, y)

        right_vector = Vector3D()

        self.tls[0] = TrafficLight(get_init_pos(60, 62, right_vector), 30, 30, 0, 80, [61, 62])
        self.tls[1] = TrafficLight(get_init_pos(60, 68, right_vector), 0, 30, 0, 80, [67, 68])
        self.tls[2] = TrafficLight(get_init_pos(60, 73, right_vector), 0, 30, 0, 80, [73])

    def show_road_information(self):
        road_id = self.map.collision_check(self.x_world, self.y_world)
        if road_id >= 0:
            self.text.set_text("Road id: " + str(road_id))
        time.sleep(0.3)
        self.show_road_information()


    def visulizer(self, input_control):
        # initialize pygame
        pygame.init()
        screen_width = 640
        display = pygame.display.set_mode((screen_width, screen_width))
        pygame.display.set_caption("SIMULATION")
        world_image = WorldImage(self, display)
        self.text = Text(pygame.font.Font(pygame.font.get_default_font(), 20),
                         (screen_width, 40), (0, screen_width - 40))

        # world_image start
        # set the position and size of visualization window
        self.scaled_size = screen_width
        self.x_offset = 0
        self.y_offset = 0
        self.x_center_display = 0
        self.y_center_display = 0
        self.dragged = None
        self.mouse_x = None
        self.mouse_y = None
        self.x_world = 0
        self.y_world = 0
        thread1 = threading.Thread(name="show_road_information", target=self.show_road_information, args=())
        thread1.daemon = True
        thread1.start()

        # get the information through cursor
        self.cursor_x = 0
        self.cursor_y = 0

        # in order to get the time
        self.clock_tick = 0
        self.is_suspend = False
        def suspend():
            self.is_suspend = not self.is_suspend
        suspend_button = Button(50, 50, 70, 30, 'Suspend', suspend)
        def restart():
            self.clock_tick = 0
        restart_button = Button(50, 10, 70, 30, 'Restart', restart)
        double_click_timer = 0
        # double_click_boolean = False
        double_junction_id = -1

        while (True):
            if self.run_time <= self.clock_tick:
                sys.exit()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    # Handle mouse wheel for zooming in and out
                    if event.button == 1:
                        if suspend_button.button_rect.collidepoint(event.pos):
                            suspend_button.callback()
                            if self.is_suspend:
                                suspend_button.reset_text('Continue')
                            else:
                                suspend_button.reset_text('Suspend')
                        if restart_button.button_rect.collidepoint(event.pos):
                            restart_button.callback()

                        # using doble click to set the windows on the junction
                        if double_click_timer == 0:
                            double_click_timer = 0.001
                        elif double_click_timer < 0.5:
                            self.cursor_x, self.cursor_y = pygame.mouse.get_pos()
                            d_x, d_y = world_image.cursor_to_world(int((self.x_center_display + self.x_offset + self.cursor_x) * world_image._width_in_pixels / self.scaled_size),
                                                        int((self.y_center_display + self.y_offset + self.cursor_y) * world_image._width_in_pixels / self.scaled_size))
                            d_road_id = self.map.collision_check(d_x, d_y)
                            # print(d_road_id)
                            if d_road_id in self.map._road_id_to_junction_id.keys():
                                double_junction_id = self.map._road_id_to_junction_id[d_road_id][0]
                            double_click_timer = 0
                            # double_click_boolean = True
                    if event.button == 3:
                        self.dragged = True
                        self.mouse_x, self.mouse_y = event.pos
                    input_control._parse_zoom(event.button)
                elif event.type == pygame.MOUSEMOTION:
                    if self.dragged:
                        temp_x, temp_y = event.pos
                        self.x_offset -= temp_x - self.mouse_x
                        self.y_offset -= temp_y - self.mouse_y
                        self.mouse_y = temp_y
                        self.mouse_x = temp_x
                    else:
                        self.cursor_x, self.cursor_y = pygame.mouse.get_pos()
                        if suspend_button.button_rect.collidepoint(event.pos):
                            suspend_button.color = COLOR_ACTIVE
                        elif restart_button.button_rect.collidepoint(event.pos):
                            restart_button.color = COLOR_ACTIVE
                        else:
                            suspend_button.color = COLOR_INACTIVE
                            restart_button.color = COLOR_INACTIVE
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 3:
                        self.dragged = False

            display.fill(COLOR_ALUMINIUM_4)
            world_image.surface.blit(world_image.big_map_surface, (0, 0))
            world_image.draw_vehicle(self.clock_tick)
            world_image.draw_traffic_light(self.clock_tick)

            if double_junction_id >= 0:
                # we assume that all junctions sizes are similar and a constant 70 x 70m windows is adopted here
                input_control.wheel_offset = 1 / (self.map._world_width / 70)
                self.scaled_size = int(screen_width / input_control.wheel_offset)
                x_temp, y_temp = self.map.junctions[double_junction_id].pos_left_top
                x_temp -= (self.map._world_offset[0] + 35)
                y_temp -= (self.map._world_offset[1] - 35)
                x_temp, y_temp = world_image.world_to_pixel([x_temp, y_temp])
                self.x_offset = int(x_temp / world_image._width_in_pixels * self.scaled_size) - int(self.scaled_size / 2 - screen_width / 2)
                self.y_offset = int(y_temp / world_image._width_in_pixels * self.scaled_size) - int(self.scaled_size / 2 - screen_width / 2)
                double_junction_id = -1

            self.scaled_size = int(screen_width / input_control.wheel_offset)
            self.x_center_display = int(self.scaled_size / 2 - screen_width / 2)
            self.y_center_display = int(self.scaled_size / 2 - screen_width / 2)

            # transform the cursor position into the world coordinate and return information
            self.x_world, self.y_world = world_image.cursor_to_world(
                int((self.x_center_display + self.x_offset + self.cursor_x) * world_image._width_in_pixels / self.scaled_size),
                int((self.y_center_display + self.y_offset + self.cursor_y) * world_image._width_in_pixels / self.scaled_size))
            display.blit(pygame.transform.scale(world_image.surface,
                                                (self.scaled_size, self.scaled_size)),
                         (-self.x_center_display - self.x_offset, -self.y_center_display - self.y_offset))
            display.blit(self.text.surface, self.text.pos)

            # timer for implementing suspend function
            restart_button.draw(display)
            suspend_button.draw(display)
            if not self.is_suspend:
                self.clock_tick += 1

            # timer for implementing double click
            if double_click_timer != 0:
                double_click_timer += 0.05
                # Reset after 0.5 seconds
                if double_click_timer >= 0.5:
                    double_click_timer = 0

            pygame.display.update()
            time.sleep(0.05)


class Button(object):
    def __init__(self, x, y, w, h, text, callback):
        self.text_surf = pygame.font.Font.render(pygame.font.SysFont("calibri", 16),
                                                 text, True, COLOR_WHITE)
        self.button_rect = pygame.Rect(x, y, w, h)
        self.text_rect = self.text_surf.get_rect(center=self.button_rect.center)

        self.color = COLOR_INACTIVE
        self.callback = callback

    def draw(self, screen):
        pygame.draw.rect(screen, self.color, self.button_rect)
        screen.blit(self.text_surf, self.text_rect)

    def reset_text(self, text):
        self.text_surf = pygame.font.Font.render(pygame.font.SysFont("calibri", 16),
                                                 text, True, COLOR_WHITE)


class InputContrl(object):

    def __init__(self):
        self.mouse_pos = (0, 0)
        self.mouse_offset = [0.0, 0.0]
        self.wheel_offset = 1.0
        self.wheel_amount = 0.05

    def _parse_zoom(self, button):
        if button == 4:
            self.wheel_offset -= self.wheel_amount * 0.5
            if self.wheel_offset >= 1.0:
                self.wheel_offset = 1.0
        elif button == 5:
            self.wheel_offset += self.wheel_amount * 0.5
            if self.wheel_offset <= 0.1:
                self.wheel_offset = 0.1


def simulation():
    run_time = 300  # running time for simulation
    world = World(run_time)
    input_control = InputContrl()
    world.load_map()  # init network
    world.save_map()
    world.generate_flow()  # init flow
    world.create_traffic_light()
    world.action()  # vehicle control
    world.visulizer(input_control)  # visulization


def main():
    simulation()


if __name__ == '__main__':
    main()
