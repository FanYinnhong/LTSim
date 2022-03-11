import hashlib
import math
import os
from pickle import TRUE
import random
import sys, pdb
import copy
from pygame.locals import *

from sympy import false

sys.path.append('..')

from lxml import etree
from io import StringIO, BytesIO
from libs.opendriveparser import parse_opendrive
from libs.geom.Vector3D import Vector3D
import numpy as np
import time
import threading
import sys

import pygame

COLOR_BUTTER_0 = pygame.Color(252, 233, 79)
COLOR_BUTTER_1 = pygame.Color(237, 212, 0)
COLOR_BUTTER_2 = pygame.Color(196, 160, 0)

COLOR_ORANGE_0 = pygame.Color(252, 175, 62)
COLOR_ORANGE_1 = pygame.Color(245, 121, 0)
COLOR_ORANGE_2 = pygame.Color(209, 92, 0)

COLOR_CHOCOLATE_0 = pygame.Color(233, 185, 110)
COLOR_CHOCOLATE_1 = pygame.Color(193, 125, 17)
COLOR_CHOCOLATE_2 = pygame.Color(143, 89, 2)

COLOR_CHAMELEON_0 = pygame.Color(138, 226, 52)
COLOR_CHAMELEON_1 = pygame.Color(115, 210, 22)
COLOR_CHAMELEON_2 = pygame.Color(78, 154, 6)

COLOR_SKY_BLUE_0 = pygame.Color(114, 159, 207)
COLOR_SKY_BLUE_1 = pygame.Color(52, 101, 164)
COLOR_SKY_BLUE_2 = pygame.Color(32, 74, 135)

COLOR_PLUM_0 = pygame.Color(173, 127, 168)
COLOR_PLUM_1 = pygame.Color(117, 80, 123)
COLOR_PLUM_2 = pygame.Color(92, 53, 102)

COLOR_SCARLET_RED_0 = pygame.Color(239, 41, 41)
COLOR_SCARLET_RED_1 = pygame.Color(204, 0, 0)
COLOR_SCARLET_RED_2 = pygame.Color(164, 0, 0)

COLOR_ALUMINIUM_0 = pygame.Color(238, 238, 236)
COLOR_ALUMINIUM_1 = pygame.Color(211, 215, 207)
COLOR_ALUMINIUM_2 = pygame.Color(186, 189, 182)
COLOR_ALUMINIUM_3 = pygame.Color(136, 138, 133)
COLOR_ALUMINIUM_4 = pygame.Color(85, 87, 83)
COLOR_ALUMINIUM_4_5 = pygame.Color(66, 62, 64)
COLOR_ALUMINIUM_5 = pygame.Color(46, 52, 54)


COLOR_GREEN_CAR = pygame.Color(139, 209, 81)
COLOR_YELLOW_CAR = pygame.Color(255, 243, 0)
COLOR_RED_CAR = pygame.Color(239, 41, 41)

car_color_set = [COLOR_RED_CAR, COLOR_YELLOW_CAR, COLOR_GREEN_CAR]

COLOR_WHITE = pygame.Color(255, 255, 255)
COLOR_BLACK = pygame.Color(0, 0, 0)
#Module Defines
PIXELS_PER_METER = 12
MAP_DEFAULT_SCALE = 0.1

precision = 0.05

class MapImage(object):

    def __init__(self, world, display):

        # read road network
        self._roads = world.roads
        self._junctions = world.junctions

        # background = pygame.image.load('images/intersection.png')

        # Maximum size of a pygame subsurface
        self._width_in_pixels = (1 << 14) - 1 # 16383
        self._all_start_points = []
        self.get_all_start_points()

        self._road_id_to_index = {}
        self._junction_id_to_index = {}
        self._road_id_to_junction_index = {}
        self.id_to_index()
        self.connectingRoad_to_leavingRoad()

        # left some space between road network and Pygame Window
        margin = 30
        min_x = min(self._all_start_points, key=lambda point: point[0])[0] - margin
        min_y = min(self._all_start_points, key=lambda point: point[1])[1] - margin
        max_x = max(self._all_start_points, key=lambda point: point[0])[0] + margin
        max_y = max(self._all_start_points, key=lambda point: point[1])[1] + margin
        del self._all_start_points
        # Road network's width or height.
        self._world_width = max(max_x - min_x, max_y - min_y)
        self._world_offset = (min_x, min_y)

        # Adapt Pixels per meter to make world fits in big_map_surface
        surface_pixel_per_meter = int(self._width_in_pixels / self._world_width)
        # if surface_pixel_per_meter > PIXELS_PER_METER:
        #     surface_pixel_per_meter = PIXELS_PER_METER
        self._pixels_per_meter = surface_pixel_per_meter

        _ME_PATH = os.path.abspath(os.path.dirname(__file__))
        DATA_PATH = os.path.normpath(os.path.join(_ME_PATH, '../data/cache'))
        filename = "town01.tga"
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
            self.draw_road_map(self.big_map_surface, world)
            self.draw_road_map(self.surface, world)
            pygame.image.save(self.big_map_surface, filepath)

    def world_to_pixel(self, location, offset=(0, 0)):
        # it needs to be mirrored
        x = self._pixels_per_meter * (location[0] - self._world_offset[0])
        y = self._pixels_per_meter * (self._world_width - (location[1] - self._world_offset[1]))
        return [int(x - offset[0]), int(y - offset[1])]

    def draw_road_map(self, map_surface, world):
        map_surface.fill(COLOR_ALUMINIUM_4)
        precision = 0.05
        leftVector = Vector3D()
        driving_width = 4
        shoulder_width = 0.3
        sidewalk_width = 4

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

        def draw_topology(surface, roads):
            for road in roads:
                geometries = road.planView.geometries
                # draw lane of current geometry
                # count road lane and sidewalk
                # write lateral shift of the road and draw lane to solve it
                # todo
                cnt_driving_left ,sidewalk_left, shoulder_left = classify_lane_type(road.lanes.laneSections[0].leftLanes)
                cnt_driving_right, sidewalk_right, shoulder_right = classify_lane_type(road.lanes.laneSections[0].rightLanes)

                for geometry in geometries:
                    pos_frenet = 0
                    waypoints = []
                    wayppoints_left_direction = []

                    while (geometry.getLength() > pos_frenet):
                        # hdg is the forward vecot of the current point
                        pos, hdg = geometry.calcPosition(pos_frenet)

                        # Returns the lateral vector based on forward vector
                        leftVector.x, leftVector.y = np.cos(hdg), np.sin(hdg)
                        leftVector.rotation2D(np.pi / 2)
                        waypoints.append(pos)
                        wayppoints_left_direction.append([leftVector.x, leftVector.y])
                        pos_frenet += precision

                    waypoints_np = np.asarray(waypoints)
                    wayppoints_directions_left_np = np.asarray(wayppoints_left_direction)

                    # draw classified lane type
                    if (sidewalk_left):
                        sidewalk_waypoints = waypoints_np + wayppoints_directions_left_np * \
                                             (driving_width * cnt_driving_left + shoulder_width+ sidewalk_width / 2)
                        draw_lane(surface, sidewalk_waypoints, wayppoints_directions_left_np, sidewalk_width, COLOR_ALUMINIUM_3)
                    if (sidewalk_right):
                        sidewalk_waypoints = waypoints_np - wayppoints_directions_left_np * \
                                             (driving_width * cnt_driving_left + shoulder_width+ sidewalk_width / 2)
                        draw_lane(surface, sidewalk_waypoints, wayppoints_directions_left_np, sidewalk_width, COLOR_ALUMINIUM_3)
                    if (shoulder_left):
                        sidewalk_waypoints = waypoints_np + wayppoints_directions_left_np * \
                                             (driving_width * cnt_driving_left + shoulder_width / 2)
                        draw_lane(surface, sidewalk_waypoints, wayppoints_directions_left_np, shoulder_width, COLOR_ALUMINIUM_4_5)
                    if (shoulder_right):
                        sidewalk_waypoints = waypoints_np - wayppoints_directions_left_np * \
                                             (driving_width * cnt_driving_left + shoulder_width / 2)
                        draw_lane(surface, sidewalk_waypoints, wayppoints_directions_left_np, shoulder_width, COLOR_ALUMINIUM_4_5)

                    # draw main road
                    lane_left_side = waypoints_np + wayppoints_directions_left_np * driving_width * cnt_driving_left
                    lane_right_side = np.flipud(waypoints_np - wayppoints_directions_left_np * driving_width * cnt_driving_right)


                    polygon = np.concatenate((lane_left_side, lane_right_side), axis=0)
                    polygon.tolist()
                    polygon = [self.world_to_pixel(x) for x in polygon]
                    if len(polygon) > 2:
                        pygame.draw.polygon(surface, COLOR_ALUMINIUM_5, polygon)
                    # draw waypoints
                    if (road.junction == None):
                        draw_lane(surface, waypoints_np, wayppoints_directions_left_np, 0.5, COLOR_ALUMINIUM_2)

        draw_topology(map_surface, self._roads)
    def id_to_index(self):
        for idx, junction in enumerate(self._junctions):
            self._junction_id_to_index[junction.id] = idx
        for idx, road in enumerate(self._roads):
            self._road_id_to_index[road.id] = idx
            if road.junction != None:
                junction_id = road.junction
                for connection_idx, connection in enumerate(self._junctions[self._junction_id_to_index[junction_id]].connections):
                    if connection.connectingRoad == road.id:
                        self._road_id_to_junction_index[road.id] = \
                            (self._junction_id_to_index[junction_id], connection_idx)

    def connectingRoad_to_leavingRoad(self):
        for junction in self._junctions:
            for connection in junction.connections:
                # find leavingRoad id
                connectingRoad_id = connection.connectingRoad
                incomingRoad_id = connection.incomingRoad
                toId = connection.laneLinks[0].toId
                connectingRoad = self._roads[self._road_id_to_index[connectingRoad_id]]
                if connectingRoad.link.predecessor.elementId != incomingRoad_id:
                    connection.leavingRoad = connectingRoad.link.predecessor.elementId
                    connection.contactPointOnLeavingRoad = connectingRoad.link.predecessor.contactPoint
                elif connectingRoad.link.successor.elementId != incomingRoad_id:
                    connection.leavingRoad = connectingRoad.link.successor.elementId
                    connection.contactPointOnLeavingRoad = connectingRoad.link.successor.contactPoint

    def get_all_start_points(self):
        for road in self._roads:
            geometries = road.planView.geometries
            for geometry in geometries:
                self._all_start_points.append(geometry.getStartPosition())


class Map(object):
    def __init__(self,roads, junctions, index1, index2,index3, width):
        self.roads = roads
        self.junctions = junctions
        self._road_id_to_index = index1
        self._junction_id_to_index = index2
        self._road_id_to_junction_index = index3
        self._world_width = width
        

class Vehicle():

    def __init__(self, vehicle_class, road_id, direction, lane, pos_frenet, map, simulation, pixels_per_meter, world_offset):
        self.map = map
        self.leftVector = Vector3D()
        self.pixels_per_meter = pixels_per_meter
        self.world_offset = world_offset

        self.vehicle_l = int(self.pixels_per_meter * 3)
        self.vehicle_w = int(self.pixels_per_meter * 1)
        self.shape = []
        
        self.road_id = road_id
        # lane number: it represents the number of lane toward to centerline
        self.lane = lane
        # 1 means the left lane in current road, 0 means the right lane in current road
        self.direction = direction
        self.vehicle_class = vehicle_class
        self.speed = 2
        self.pos_frenet = pos_frenet
        self.hdg = 0
        self.x = 0
        self.y = 0
        self.rotation_degree = 0
        self.x_in_pixel, self.y_in_pixel = -1, -1
        self.last_in_pixel = [0,0]
        self.get_position()
        simulation.add(self)

    def move(self, speed):
        self.speed = speed
        # check whether vehicle is in road or junctions
        road = self.map.roads[self.map._road_id_to_index[self.road_id]]
        is_junction = False if road.junction == None else True
        if (is_junction):
            # calculate pos_length
            # it needs to check the direction of the current road, it is determined by the contact point of the connecting road
            junction_index = self.map._road_id_to_junction_index[self.road_id]
            connection = self.map.junctions[junction_index[0]].connections[junction_index[1]]
            # the direction of vehicle is the same with the junction curve
            if connection.contactPoint == "start":
                self.pos_frenet += self.speed
                # if vehicle has arrived at the end of the junction, then the vehicle runs into a new road
                if self.pos_frenet > self.map.roads[self.map._road_id_to_index[connection.connectingRoad]].length:
                    # the remain frenet in the new road
                    pos_frenet_temp = self.pos_frenet - self.map.roads[self.map._road_id_to_index[connection.connectingRoad]].length
                    # determine the new road
                    item_leavingRoad = self.map.roads[self.map._road_id_to_index[connection.leavingRoad]]
                    # update the road id of the new road
                    self.road_id = connection.leavingRoad
                    # if the direction of vehicle is the same with the junction curve, then determine the frenet, direction
                    if connection.contactPointOnLeavingRoad == "start":
                        self.pos_frenet = pos_frenet_temp
                        self.direction = 0
                        self.lane = 1
                    elif connection.contactPointOnLeavingRoad == "end":
                        self.pos_frenet = item_leavingRoad.length - pos_frenet_temp
                        self.direction = 1
                        self.lane = 1
            # the direction of vehicle is the different from the junction curve
            elif connection.contactPoint == "end":
                self.pos_frenet -= self.speed
                # if vehicle has arrived at the end of the junction, then the vehicle runs into a new road
                if self.pos_frenet < 0:
                    # the remain frenet in the new road
                    pos_frenet_temp = -self.pos_frenet
                    # determine the new road
                    item_leavingRoad = self.map.roads[self.map._road_id_to_index[connection.leavingRoad]]
                    self.road_id = connection.leavingRoad
                    # if the direction of vehicle is the same with the junction curve, then determine the frenet, direction 
                    if connection.contactPointOnLeavingRoad == "start":
                        self.pos_frenet = pos_frenet_temp
                        self.direction = 0
                        self.lane = 1
                    elif connection.contactPointOnLeavingRoad == "end":
                        self.pos_frenet = item_leavingRoad.length - pos_frenet_temp
                        self.direction = 1
                        self.lane = 1
        else: # drive on the road segment
            # drive along the positive position of the road
            if (self.direction == 0):
                self.pos_frenet += self.speed
                if (self.pos_frenet > road.length):
                    # if next is a junction
                    if road.link.successor.elementType == "junction":
                        junction_index = self.map._junction_id_to_index[road.link.successor.elementId]

                        # Choose a road in junction's connections groups
                        junction = self.map.junctions[junction_index]
                        connection_available = []
                        for connection in junction.connections:
                            if connection.incomingRoad == road.id:
                                connection_available.append(connection) 
                        connection_chosen = random.choice(connection_available)
                        connectingRoad = connection_chosen.connectingRoad
                        contactPoint = connection_chosen.contactPoint
                        self.lane = connection_chosen.laneLinks[0].toId
                        pos_frenet_temp = self.pos_frenet - road.length
                        if contactPoint == "start":
                            self.pos_frenet = pos_frenet_temp
                            self.direction = 0
                            self.lane = 1
                        else:
                            self.pos_frenet = self.map.roads[self.map._road_id_to_index[connectingRoad]].length\
                                              - pos_frenet_temp
                            self.direction = 1
                            self.lane = 1
                        self.road_id = connectingRoad
                    # if next is a road
                    else:
                        pos_frenet_temp = self.pos_frenet - road.length
                        if road.link.successor.contactPoint == "start":
                            self.pos_frenet = pos_frenet_temp
                            self.direction = 0
                            self.lane = 1
                        else:
                            self.pos_frenet = self.map.roads[self.map._road_id_to_index[road.link.successor.elementId]].length\
                                              - pos_frenet_temp
                            self.direction = 1
                            self.lane = 1
                        self.road_id = road.link.successor.elementId

            # drive along the opposite position of the road
            else:
                self.pos_frenet -= self.speed
                if (self.pos_frenet < 0):
                    # if next is a junction
                    if road.link.predecessor.elementType == "junction":

                        junction_index = self.map._junction_id_to_index[road.link.predecessor.elementId]

                        # Choose a road in junction's connections groups
                        junction = self.map.junctions[junction_index]
                        connection_available = []
                        for connection in junction.connections:
                            if connection.incomingRoad == road.id:
                                connection_available.append(connection)
                        connection_chosen = random.choice(connection_available)
                        connectingRoad = connection_chosen.connectingRoad
                        contactPoint = connection_chosen.contactPoint
                        self.lane = connection_chosen.laneLinks[0].toId
                        pos_frenet_temp = -self.pos_frenet
                        if contactPoint == "start":
                            self.pos_frenet = pos_frenet_temp
                            self.direction = 0
                            self.lane = 1
                        else:
                            self.pos_frenet = self.map.roads[self.map._road_id_to_index[connectingRoad]].length\
                                              - pos_frenet_temp
                            self.direction = 1
                            self.lane = 1
                        self.road_id = connectingRoad

                    # if next is a road
                    else:
                        # print(road.id)
                        # print(road.link.predecessor.elementId, road.link.predecessor.elementType)
                        pos_frenet_temp = -self.pos_frenet
                        if road.link.predecessor.contactPoint == "start":
                            self.pos_frenet = pos_frenet_temp
                            self.direction = 0
                            self.lane = 1
                        else:

                            self.pos_frenet = self.map.roads[self.map._road_id_to_index[road.link.predecessor.elementId]].length - pos_frenet_temp
                            self.direction = 1
                            self.lane = 1
                        self.road_id = road.link.predecessor.elementId
        self.get_position()

    def record(self):
        return [self.road_id,self.lane,self.pos_frenet,self.direction][:]

    def back2record(self, record_state):
        [self.road_id,self.lane,self.pos_frenet,self.direction] = record_state
        

    def get_position(self):
        s_pos = self.pos_frenet
        geometries = self.map.roads[self.map._road_id_to_index[self.road_id]].planView.geometries
        len_geometreis = len(geometries)
        pos = None
        for idx, geometry in enumerate(geometries):
            if (s_pos > geometry.getLength()):
                s_pos -= geometry.getLength()
            else:
                pos, hdg = geometry.calcPosition(s_pos)
                break
        self.hdg = hdg
        self.x, self.y = pos[0], pos[1]
        # Returns the lateral vector based on forward vector
        self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
        self.leftVector.rotation2D(np.pi / 2)


        if self.direction > 0:
            self.x, self.y = pos[0] - self.leftVector.x * self.lane * 2, pos[1] - self.leftVector.y * self.lane * 2
            self.rotation_degree = math.degrees(np.pi + hdg) % 360
        else:
            self.x, self.y = pos[0] + self.leftVector.x * self.lane * 2, pos[1] + self.leftVector.y * self.lane * 2
            self.rotation_degree = math.degrees(hdg) % 360

        # get position in pixel
        if self.last_in_pixel[0] != self.x_in_pixel or self.last_in_pixel[1] != self.y_in_pixel:
            self.last_in_pixel = [self.x_in_pixel, self.y_in_pixel]
        self.x_in_pixel, self.y_in_pixel = self.pos_in_pixel()
        

    def pos_in_pixel(self):
        x = self.pixels_per_meter * (self.x - self.world_offset[0])
        y = self.pixels_per_meter * (self.map._world_width - (self.y - self.world_offset[1]))
        return [int(x), int(y)]


    def get_shape_pos(self):
        def CooGet(para, margin = 0):
            hwm = self.vehicle_w 
            hlm = self.vehicle_l - self.vehicle_w 
            x, y, theta = para
            theta0 = math.degrees(math.atan(hwm/hlm))
            dx1 = round(np.sqrt(hwm**2+hlm**2) * math.cos(math.radians((theta0+theta))),3)
            dy1 = round(np.sqrt(hwm**2+hlm**2) * math.sin(math.radians((theta0+theta))),3)
            dx2 = round(np.sqrt(hwm**2+hlm**2) * math.cos(math.radians((theta0-theta))),3)
            dy2 = round(np.sqrt(hwm**2+hlm**2) * math.sin(math.radians((theta0-theta))),3)
            Pa = (round(x-dx2,3),round(y-dy2,3))
            Pb = (round(x-dx1,3),round(y+dy1,3))
            Pc = (round(x+dx2,3),round(y+dy2,3))
            Pd = (round(x+dx1,3),round(y-dy1,3))
            return [Pa, Pb, Pc, Pd]            
        [Pa, Pb, Pc, Pd] = CooGet([self.x_in_pixel, self.y_in_pixel, self.rotation_degree])
        shape_points = [(Pa,Pb),(Pb,Pc),(Pc,Pd),(Pd,Pa)]
        if self.last_in_pixel[0] < 0:
            self.shape = shape_points
        else:
            shape_line = {}
            for line in shape_points:
                shape_line[line] = np.hypot(line[0][0]-self.last_in_pixel[0], line[0][1]-self.last_in_pixel[1]) + \
                                   np.hypot(line[1][0]-self.last_in_pixel[0], line[1][1]-self.last_in_pixel[1])
            forward_shape_line = max(shape_line.items(), key = lambda x: x[1])[0]
            for i in range(len(shape_points)):
                if shape_points[i] == forward_shape_line:
                    # mid point in forward line
                    mid_point = ((forward_shape_line[0][0] + forward_shape_line[1][0])/2, 
                                (forward_shape_line[0][1] + forward_shape_line[1][1])/2)
                    forward_point = (2*mid_point[0] - self.x_in_pixel, 2*mid_point[1] - self.y_in_pixel)
                    break
            shape_points = shape_points[0:i] + [(forward_shape_line[0],forward_point),(forward_point,forward_shape_line[1])] + shape_points[i+1:]
            self.shape = shape_points
        return shape_points


class World(object):
    """Class that contains all the information of simulation that is running on the server side"""

    def __init__(self, display, vehicle_simulation):
        self.simulation_time = 0
        self._display = display
        # World data
        self.world = None
        self.vehicle_simulation = vehicle_simulation

        # Map info
        self.map_image = None
        self.map = None


    def start(self):
        """Build the map image, stores the needed modules and prepares rendering in Hero Mode"""
        self.world = self._get_data_from_simulation()
        self.map_image = MapImage(self.world, self._display)

        self.map = Map(self.world.roads, self.world.junctions, self.map_image._road_id_to_index,
                       self.map_image._junction_id_to_index, self.map_image._road_id_to_junction_index,
                       self.map_image._world_width)

        # Map info
        self.surface_size = self.map_image.big_map_surface.get_width() # 16383
        self._road_id_to_index = self.map_image._road_id_to_index
        self._junction_id_to_index = self.map_image._junction_id_to_index
        self._road_id_to_junction_index = self.map_image._road_id_to_junction_index

    def _get_data_from_simulation(self):
        """Retrieves the data from the server side"""
        # read road network
        return self.read_file()

    def read_file(self):
        _ME_PATH = os.path.abspath(os.path.dirname(__file__))
        DATA_PATH = os.path.normpath(os.path.join(_ME_PATH, '../data/xml'))
        filename = "Town01.xodr"
        filepath = os.path.join(DATA_PATH, filename)
        # get road network info
        with open(filepath, 'r') as f:
            parser = etree.XMLParser()
            rootNode = etree.parse(f, parser).getroot()
            roadNetwork = parse_opendrive(rootNode)
        return roadNetwork

    def generate_vehicles(self):
        while(True):
            vehicle_class = 0
            road_id = 0
            lane_number = 1
            if (lane_number < 0):
                direction = 0
            else:
                direction = 1
            pos_frenet = 0
            Vehicle(vehicle_class, road_id, direction, abs(lane_number), pos_frenet, self.map,
                    self.vehicle_simulation, self.map_image._pixels_per_meter, self.map_image._world_offset)  
            #    time.sleep(1)
            break
def point2point(ego_ps, other_ps):
    ps_1 = [it[0] for it in ego_ps]
    ps_2 = [it[0] for it in other_ps]
    for p1 in ps_1:
        for p2 in ps_2:
            if np.hypot(p1[0]-p2[0], p1[1]-p2[1]) < 50:
                return True
    return False


def is_collide(ego, all):
    for car in all:
        if ego != car:
            if np.hypot(ego.x_in_pixel-car.x_in_pixel, ego.y_in_pixel-car.y_in_pixel) < 200:
                if point2point(ego.get_shape_pos(), car.get_shape_pos()):
                    return True
    return False

def game_loop():
    """Initialized, Starts and runs all the needed modules for No Rendering Mode"""
    # initialize pygame
    pygame.init()
    vehicle_simulation = set()
    screen_width = 1220
    display = pygame.display.set_mode((screen_width, screen_width))
    pygame.display.set_caption("SIMULATION")

    # Init
    world = World(display, vehicle_simulation)

    # start each module
    world.start()

    # thread1 = threading.Thread(name="generate_vehicles", target = world.generate_vehicles(), args=())
    # thread1.daemon = True
    # thread1.start()
    world.generate_vehicles()    
    current_time = time.perf_counter()
    pause = True
    cnt = 0
    while (True):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            # pause or continue the loop by click the MOUSE
            if event.type == pygame.MOUSEBUTTONDOWN:
                pause = bool(1-pause)
        if (pause):
            if time.perf_counter() - current_time > 3 and cnt < 30:
                current_time = time.perf_counter()
                print(time.perf_counter() )
                world.generate_vehicles()    
                cnt += 1
            world.map_image.surface.blit(world.map_image.big_map_surface, (0,0))
            for vehicle in vehicle_simulation:
                record_state = vehicle.record()
                high_speed = random.randint(3,5)
                for speed in range(high_speed,-1,-1):                
                    vehicle.move(speed)
                    if is_collide(vehicle, vehicle_simulation):     
                        print("risk")
                        vehicle.back2record(record_state)
                    else:          
                        if is_collide(vehicle, vehicle_simulation):     
                            pdb.set_trace()
                        for line in vehicle.get_shape_pos():         
                            if speed == high_speed:
                                color = car_color_set[2]       
                            else:
                                color = car_color_set[0]       
                            pygame.draw.line(world.map_image.surface,color,line[0],line[1],30)     
                        break
            pop_list = []
            for vehicle in vehicle_simulation:
                if vehicle.road_id == 10:
                    pop_list.append(vehicle)   
            for vehicle in pop_list:    
                vehicle_simulation.remove(vehicle)
                cnt -= 1
                         

            display.blit(pygame.transform.scale(world.map_image.surface,
                                                display.get_rect().size), (0, 0))
            pygame.display.update()
            time.sleep(0.05)            

def main():
    game_loop()

if __name__ == '__main__':
    main()



