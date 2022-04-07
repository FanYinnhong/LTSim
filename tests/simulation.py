import threading
import math, os, sys, pdb

from pygame.locals import *
import numpy as np
import time, random

sys.path.append('..')
from lxml import etree
from libs.opendriveparser import parse_opendrive
from libs.geom.Vector3D import Vector3D
from visualization.palette import *
from carflow import generateCarFlow

from behavir_models import *
from frenet_planning import *

SAMPLE_TIME = 0.1


class Map(object):
    def __init__(self, network):
        self.roads = network.roads
        self.junctions = network.junctions
        self._road_id_to_index = {}
        self._junction_id_to_index = {}
        self._road_id_to_junction_index = {}
        self.id_to_index()
        self.connection_to_leaving()

        self._all_start_points = []
        self.get_all_start_points()
        # Road network's width or height.
        # left some space between road network and Pygame Window
        margin = 30
        min_x = min(self._all_start_points, key=lambda point: point[0])[0] - margin
        min_y = min(self._all_start_points, key=lambda point: point[1])[1] - margin
        max_x = max(self._all_start_points, key=lambda point: point[0])[0] + margin
        max_y = max(self._all_start_points, key=lambda point: point[1])[1] + margin       
        self._world_width = max(max_x - min_x, max_y - min_y)
        self._world_offset = (min_x, min_y)
        del self._all_start_points
                
    def get_all_start_points(self):
        for road in self.roads:
            geometries = road.planView.geometries
            for geometry in geometries:
                self._all_start_points.append(geometry.getStartPosition())

    def id_to_index(self):
        # junciton index
        for idx, junction in enumerate(self.junctions):
            self._junction_id_to_index[junction.id] = idx
        # road index (roads include junctions)
        for idx, road in enumerate(self.roads):
            self._road_id_to_index[road.id] = idx
            if road.junction != None:
                junction_id = road.junction
                # connecting road index for each junction
                for connection_idx, connection in enumerate(self.junctions[self._junction_id_to_index[junction_id]].connections):
                    if connection.connectingRoad == road.id:
                        self._road_id_to_junction_index[road.id] = \
                            (self._junction_id_to_index[junction_id], connection_idx)

    def connection_to_leaving(self): # connectingRoad_to_leavingRoad
        for junction in self.junctions:
            for connection in junction.connections:
                # find leavingRoad id
                connectingRoad_id = connection.connectingRoad
                incomingRoad_id = connection.incomingRoad
                toId = connection.laneLinks[0].toId
                connectingRoad = self.roads[self._road_id_to_index[connectingRoad_id]]
                if connectingRoad.link.predecessor.elementId != incomingRoad_id:
                    connection.leavingRoad = connectingRoad.link.predecessor.elementId
                    connection.contactPointOnLeavingRoad = connectingRoad.link.predecessor.contactPoint
                elif connectingRoad.link.successor.elementId != incomingRoad_id:
                    connection.leavingRoad = connectingRoad.link.successor.elementId
                    connection.contactPointOnLeavingRoad = connectingRoad.link.successor.contactPoint

        

class Vehicle(object):

    def __init__(self, id, world, map, road_id, direction, lane, scf, ST, info):
        # map info        
        self.world = world
        self.map = map
        self.road_id = road_id
        self.lane = lane # lane number: it represents the number of lane toward to centerline
        self.direction = direction #1 - the left side, 0 - the right side
        
        # attributes
        self.id = id
        self.vehicle_l, self.vehicle_w = info[4], info[5]
        self.shape = []

        # motion status   
        self.scf = scf                        # s-axis coordinate value of Frenet coordinate system
        self.scf_end = self.map.roads[self.map._road_id_to_index[self.road_id]].length
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
        self.scf += dp_s
        self.vel += d_v  
        self.tcf += dp_t
        self.hdg += d_hdg
        pos_status = self.road_id,self.lane,self.scf,self.direction,self.scf_end
        def update_pos(pos_status, map):
            road_id, lane, scf, direction, scf_end = pos_status
            road = map.roads[map._road_id_to_index[road_id]]    
            is_junction = False if road.junction == None else True
            if (is_junction):
                # calculate pos_length
                # it needs to check the direction of the current road, it is determined by the contact point of the connecting road
                junction_index = map._road_id_to_junction_index[road_id]
                connection = map.junctions[junction_index[0]].connections[junction_index[1]]
                # map.roads[map._road_id_to_index[connection.connectingRoad]].length == road.length
                # the direction of vehicle is the same with the junction curve
                if connection.contactPoint == "start":
                    # scf += dp_s
                    # if vehicle has arrived at the end of the junction, then the vehicle runs into a new road
                    # connection.connectingRoad == road.id
                    if scf > map.roads[map._road_id_to_index[connection.connectingRoad]].length:
                        # the remain frenet in the new road
                        scf_temp = scf - map.roads[map._road_id_to_index[connection.connectingRoad]].length
                        # determine the new road
                        item_leavingRoad = map.roads[map._road_id_to_index[connection.leavingRoad]]
                        # update the road id of the new road
                        road_id = connection.leavingRoad
                        # if the direction of vehicle is the same with the junction curve, then determine the frenet, direction
                        if connection.contactPointOnLeavingRoad == "start":
                            scf = scf_temp
                            direction = 0
                            scf_end = item_leavingRoad.length
                            # lane = 1
                        elif connection.contactPointOnLeavingRoad == "end":
                            scf = item_leavingRoad.length - scf_temp
                            direction = 1
                            scf_end = 0
                            # lane = 1
                # the direction of vehicle is the different from the junction curve
                elif connection.contactPoint == "end":
                    # scf -= dp_s
                    # if vehicle has arrived at the end of the junction, then the vehicle runs into a new road
                    if scf < 0:
                        # the remain frenet in the new road
                        scf_temp = -scf
                        # determine the new road
                        item_leavingRoad = map.roads[self.map._road_id_to_index[connection.leavingRoad]]
                        road_id = connection.leavingRoad
                        # if the direction of vehicle is the same with the junction curve, then determine the frenet, direction 
                        if connection.contactPointOnLeavingRoad == "start":
                            scf = scf_temp
                            direction = 0
                            scf_end = item_leavingRoad.length
                            # lane = 1
                        elif connection.contactPointOnLeavingRoad == "end":
                            scf = item_leavingRoad.length - scf_temp
                            direction = 1
                            scf_end = 0
                            # lane = 1
            else: # drive on the road segment
                # drive along the positive position of the road
                if (direction == 0):
                    # scf += dp_s
                    if (scf > road.length):
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
                            if ((connectingRoad in tl_record_t) and tl_record_t[connectingRoad] == 'r'):
                                scf = road.length

                            else:
                                contactPoint = connection_chosen.contactPoint
                                # lane = connection_chosen.laneLinks[0].toId
                                scf_temp = scf - road.length
                                if contactPoint == "start":
                                    scf = scf_temp
                                    direction = 0
                                    scf_end = map.roads[map._road_id_to_index[connectingRoad]].length
                                    # lane = 1
                                else:
                                    scf = map.roads[map._road_id_to_index[connectingRoad]].length\
                                                    - scf_temp
                                    direction = 1
                                    scf_end = 0
                                    # lane = 1
                                road_id = connectingRoad
                        # if next is a road
                        else:
                            scf_temp = scf - road.length
                            if road.link.successor.contactPoint == "start":
                                scf = scf_temp
                                direction = 0
                                scf_end = map.roads[self.map._road_id_to_index[road.link.successor.elementId]].length
                                # lane = 1
                            else:
                                scf = map.roads[self.map._road_id_to_index[road.link.successor.elementId]].length\
                                                - scf_temp
                                direction = 1
                                scf_end = 0
                                # lane = 1
                            road_id = road.link.successor.elementId

                # drive along the opposite position of the road
                else:
                    # scf -= dp_s
                    if (scf < 0):
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
                            if ((connectingRoad in tl_record_t) and tl_record_t[connectingRoad] == 'r'):
                                scf = 0
                            else:
                                contactPoint = connection_chosen.contactPoint
                                # lane = connection_chosen.laneLinks[0].toId
                                scf_temp = -scf
                                if contactPoint == "start":
                                    scf = scf_temp
                                    direction = 0
                                    scf_end = map.roads[map._road_id_to_index[connectingRoad]].length
                                    # lane = 1
                                else:
                                    scf = map.roads[map._road_id_to_index[connectingRoad]].length - scf_temp
                                    direction = 1
                                    scf_end = 0
                                    # lane = 1
                                road_id = connectingRoad

                        # if next is a road
                        else:
                            # print(road.id)
                            # print(road.link.predecessor.elementId, road.link.predecessor.elementType)
                            scf_temp = -scf
                            if road.link.predecessor.contactPoint == "start":
                                scf = scf_temp
                                direction = 0
                                scf_end = map.roads[map._road_id_to_index[road.link.predecessor.elementId]].length
                                # lane = 1
                            else:
                                scf = map.roads[map._road_id_to_index[road.link.predecessor.elementId]].length - scf_temp
                                direction = 1
                                scf_end = 0
                                # lane = 1
                            road_id = road.link.predecessor.elementId
            if scf > self.map.roads[map._road_id_to_index[road_id]].length:
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
            self.road_id,self.lane,self.scf,self.direction, self.scf_end = road_id, lane, scf, direction, scf_end
            self.shape = shape_points
        

    def get_position(self, road_id, direction, scf, tcf, hdg):          
        geometries = self.map.roads[self.map._road_id_to_index[road_id]].planView.geometries
        # len_geometreis = len(geometries)
        for idx, geometry in enumerate(geometries):
            if (scf > geometry.getLength()):
                scf -= geometry.getLength()
            else:
                pos, road_hdg = geometry.calcPosition(scf)
                break
        x, y = pos[0]-self.map._world_offset[0], pos[1]-self.map._world_offset[1]
        
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
                    if np.hypot(p1[0]-p2[0], p1[1]-p2[1]) < 1:
                        return True
            return False
        for car in self.world.vehicle_running:
            car = self.world.vehicle_running[car]
            if self != car:
                if np.hypot(attitude[0]-car.x, attitude[1]-car.y) < 6:
                    if point2point(shape_points, car.shape):
                        return True
        return False

class TrafficLight(object):
    def __init__(self, pos, init_phase,time_green, time_yellow, time_red, connect_id_list):
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
        time_temp = (current_time + self.init_phase)% (self.time_yellow + self.time_green + self.time_red)
        if time_temp < self.time_green:
            self.current_state = 'g' # it means current state is green
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
        self._width_in_pixels = (1 << 14) - 1 # 16383

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
        for road in self.world._roads:
            geometries = road.planView.geometries
            # draw lane of current geometry
            # count road lane and sidewalk
            # write lateral shift of the road and draw lane to solve it
            # todo
            cnt_driving_left ,sidewalk_left, shoulder_left = classify_lane_type(road.lanes.laneSections[0].leftLanes)
            cnt_driving_right, sidewalk_right, shoulder_right = classify_lane_type(road.lanes.laneSections[0].rightLanes)

            for geometry in geometries:
                scf = 0
                waypoints = []
                wayppoints_left_direction = []

                while (geometry.getLength() > scf):
                    # hdg is the forward vecot of the current point
                    pos, hdg = geometry.calcPosition(scf)
                    pos[0], pos[1] = pos[0]-self.map._world_offset[0], pos[1]-self.map._world_offset[1]
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
                                            (self.driving_width * cnt_driving_left + self.shoulder_width+ self.sidewalk_width / 2)
                    draw_lane(surface, sidewalk_waypoints, wayppoints_directions_left_np, self.sidewalk_width, COLOR_ALUMINIUM_3)
                if (sidewalk_right):
                    sidewalk_waypoints = waypoints_np - wayppoints_directions_left_np * \
                                            (self.driving_width * cnt_driving_left + self.shoulder_width+ self.sidewalk_width / 2)
                    draw_lane(surface, sidewalk_waypoints, wayppoints_directions_left_np, self.sidewalk_width, COLOR_ALUMINIUM_3)
                if (shoulder_left):
                    sidewalk_waypoints = waypoints_np + wayppoints_directions_left_np * \
                                            (self.driving_width * cnt_driving_left + self.shoulder_width / 2)
                    draw_lane(surface, sidewalk_waypoints, wayppoints_directions_left_np, self.shoulder_width, COLOR_ALUMINIUM_4_5)
                if (shoulder_right):
                    sidewalk_waypoints = waypoints_np - wayppoints_directions_left_np * \
                                            (self.driving_width * cnt_driving_left + self.shoulder_width / 2)
                    draw_lane(surface, sidewalk_waypoints, wayppoints_directions_left_np, self.shoulder_width, COLOR_ALUMINIUM_4_5)

                # draw main road
                lane_left_side = waypoints_np + wayppoints_directions_left_np * self.driving_width * cnt_driving_left
                lane_right_side = np.flipud(waypoints_np - wayppoints_directions_left_np * self.driving_width * cnt_driving_right)

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
                pygame.draw.line(self.surface,color,line_in_pixel[0],line_in_pixel[1],25)
    
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

    def generate_flow(self):
        flow =  generateCarFlow(self.run_time, SAMPLE_TIME, 26)
        for t in range(self.run_time):
            for vehicle_info in flow:
                from_road_id = vehicle_info[0]
                to_road_id = vehicle_info[1]
                arrival_time =vehicle_info[2]
                temp_id = (from_road_id, to_road_id, arrival_time)
                if temp_id not in self.vehicle_demand and arrival_time > t:
                    scf = 0
                    lane_number = random.randint(1,2)
                    direction = 0 if lane_number > 0 else 1              
                    self.vehicle_demand[temp_id] = Vehicle(temp_id, self, self.map, from_road_id, direction, abs(lane_number), scf, SAMPLE_TIME, vehicle_info)  

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
                self.vehicle_demand.pop(vehicle_id) # del vehicle
            
            running_pop = []
            for vehicle_id in self.vehicle_running:
                rt = 0.5  # reaction time
                # perception
                # ego status
                # other statuses      
                if t+rt < self.run_time:
                    pass
                    # planning process, planning, expect_tra
                    # upate the state in car_status, real_tra   
                vehicle = self.vehicle_running[vehicle_id]
                if len(vehicle.plan_tra) < vehicle.plan_period - vehicle.plan_gap or vehicle.plan_road_id != vehicle.road_id:
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
                        self.map.roads[self.map._road_id_to_index[vehicle.road_id]].length
                else:
                    cur_move = vehicle.plan_tra.pop(0)              
                vehicle.move(cur_move, self.tl_control_record[t])  
                if vehicle.road_id == vehicle_id[1]: # arrive at to_road
                    running_pop.append(vehicle_id)  
                else:    
                    self.vehicle_record[t][vehicle_id] = vehicle.shape[:]
            for vehicle_id in running_pop:
                self.vehicle_running.pop(vehicle_id) # del vehicle

    def create_traffic_light(self):
        def get_init_pos(junction_id, connectingroad_id, right_vector):
            # connecting road is chosen to calculate road id
            junction_index = self.map._junction_id_to_index[junction_id]
            junction = self.map.junctions[junction_index]
            for connection in junction.connections:
                if (connection.connectingRoad == connectingroad_id):
                    contact_point_temp = connection.contactPoint
            whether_right = 1
            whether_forward = 1
            if contact_point_temp == 'end':
                s_frenet = self.map.roads[self.map._road_id_to_index[connectingroad_id]].length
                whether_right = -1
            else: #  contact_point_temp == 'start'
                s_frenet = 0
                whether_forward = -1
            geometries = self.map.roads[self.map._road_id_to_index[connectingroad_id]].planView.geometries
            for idx, geometry in enumerate(geometries):
                if (s_frenet > geometry.getLength()):
                    s_frenet -= geometry.getLength()
                else:
                    pos, hdg = geometry.calcPosition(s_frenet)
                    break
           
            x, y = pos[0]-self.map._world_offset[0], pos[1]-self.map._world_offset[1]

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
        # road_id = self.map.collision_check(self.x_world, self.y_world)
        # if road_id >= 0:
        # self.text.set_text("Road id: " + str(road_id))
        self.text.set_text("Road id: xxx" )
        time.sleep(0.5)
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
        self.dragged = None
        self.mouse_x = None
        self.mouse_y = None
        self.x_world = 0
        self.y_world = 0
        thread1 = threading.Thread(name="show_road_information", target=self.show_road_information, args=())
        thread1.daemon = True
        thread1.start()

        # get the information through cursor
        self.cursor_x = None
        self.cursor_y = None

        for t in range(self.run_time):
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
                elif event.type == pygame.MOUSEBUTTONDOWN:
                # Handle mouse wheel for zooming in and out
                    if event.button == 1:
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
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        self.dragged = False
                # in order to get the information of the road
                if not self.dragged:
                    self.cursor_x, self.cursor_y = pygame.mouse.get_pos()

            display.fill(COLOR_ALUMINIUM_4)
            world_image.surface.blit(world_image.big_map_surface, (0, 0))
            world_image.draw_vehicle(t)
            world_image.draw_traffic_light(t)

            self.scaled_size = int(screen_width / input_control.wheel_offset)
            x_center_display = int(self.scaled_size / 2 - screen_width / 2)
            y_center_display = int(self.scaled_size / 2 - screen_width / 2)
            
            # transform the cursor position into the world coordinate and return information
            self.x_world, self.y_world = world_image.pixel_to_world(int((x_center_display + self.x_offset + self.cursor_x)
                                                              * world_image._width_in_pixels / self.scaled_size),
                                                          int((y_center_display + self.y_offset + self.cursor_y)
                                                              * world_image._width_in_pixels / self.scaled_size))

            display.blit(pygame.transform.scale(world_image.surface,
                                                (self.scaled_size, self.scaled_size)),
                         (-x_center_display - self.x_offset, -y_center_display - self.y_offset))
            display.blit(self.text.surface, self.text.pos)
            pygame.display.update()
            time.sleep(0.05)

class InputContrl(object):

    def __init__(self):
        self.mouse_pos = (0, 0)
        self.mouse_offset = [0.0, 0.0]
        self.wheel_offset = 1.0
        self.wheel_amount = 0.05
    def _parse_zoom(self, button):
        if button == 4:
            self.wheel_offset -= self.wheel_amount
            if self.wheel_offset >= 1.0:
                self.wheel_offset = 1.0
        elif button == 5:
            self.wheel_offset += self.wheel_amount
            if self.wheel_offset <= 0.1:
                self.wheel_offset = 0.1

def simulation():
    run_time = 500            # running time for simulation
    world = World(run_time)
    input_control = InputContrl()
    world.load_map()          # init network
    world.generate_flow()     # init flow
    world.create_traffic_light()
    world.action()            # vehicle control
    world.visulizer(input_control)         # visulization

def main():
    simulation()

if __name__ == '__main__':
    main()



