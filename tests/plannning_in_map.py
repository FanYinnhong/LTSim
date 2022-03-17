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
        for idx, junction in enumerate(self.junctions):
            self._junction_id_to_index[junction.id] = idx
        for idx, road in enumerate(self.roads):
            self._road_id_to_index[road.id] = idx
            if road.junction != None:
                junction_id = road.junction
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

    def __init__(self, map, road_id, direction, lane, pos_frenet, ST, info):
        self.leftVector = Vector3D()

        # map info        
        self.map = map
        self.road_id = road_id
        self.lane = lane # lane number: it represents the number of lane toward to centerline
        self.direction = direction #1 - the left side, 0 - the right side
        
        # attributes
        self.vehicle_l, self.vehicle_w = info[3], info[4]
        self.shape = []

        # motion status
        self.speed = info[2] * ST                
        self.pos_frenet = pos_frenet
        self.hdg = 0
        self.x = 0
        self.y = 0
        self.rotation_degree = 0
        self.last_state = [-1, -1]

    def move(self):
        # check whether vehicle is in road or junctions
        road = self.map.roads[self.map._road_id_to_index[self.road_id]]
        # pdb.set_trace()
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
        self.get_shape_pos()     

    def get_position(self):
        s_pos = self.pos_frenet
        # pdb.set_trace()
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
        
        if self.last_state[0] != self.x or self.last_state[1] != self.y:
            self.last_state = [self.x, self.y]
        
    def get_shape_pos(self):
        def CooGet(para, margin = 0):
            hwm = self.vehicle_w / 2
            hlm = self.vehicle_l / 2
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
        [Pa, Pb, Pc, Pd] = CooGet([self.x, self.y, self.rotation_degree])
        shape_points = [(Pa,Pb),(Pb,Pc),(Pc,Pd),(Pd,Pa)]
        if self.last_state[0] < 0:
            self.shape = shape_points
        else:
            shape_line = {}
            for line in shape_points:
                shape_line[line] = np.hypot(line[0][0]-self.last_state[0], line[0][1]-self.last_state[1]) + \
                                   np.hypot(line[1][0]-self.last_state[0], line[1][1]-self.last_state[1])
    
            forward_shape_line = max(shape_line.items(), key = lambda x: x[1])[0]
            for i in range(len(shape_points)):
                if shape_points[i] == forward_shape_line:
                    # mid point in forward line
                    mid_point = ((forward_shape_line[0][0] + forward_shape_line[1][0])/2, 
                                (forward_shape_line[0][1] + forward_shape_line[1][1])/2)
                    forward_point = (2*mid_point[0] - self.x, 2*mid_point[1] - self.y)
                    break
            shape_points = shape_points[0:i] + [(forward_shape_line[0],forward_point),(forward_point,forward_shape_line[1])] + shape_points[i+1:]
            self.shape = shape_points

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
        x = self._pixels_per_meter * (location[0] - self.world.map._world_offset[0])
        y = self._pixels_per_meter * (self.world.map._world_width - (location[1] - self.world.map._world_offset[1]))
        return [int(x - offset[0]), int(y - offset[1])]

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
                pos_frenet = 0
                waypoints = []
                wayppoints_left_direction = []

                while (geometry.getLength() > pos_frenet):
                    # hdg is the forward vecot of the current point
                    pos, hdg = geometry.calcPosition(pos_frenet)

                    # Returns the lateral vector based on forward vector
                    self.leftVector.x, self.leftVector.y = np.cos(hdg), np.sin(hdg)
                    self.leftVector.rotation2D(np.pi / 2)
                    waypoints.append(pos)
                    wayppoints_left_direction.append([self.leftVector.x, self.leftVector.y])
                    pos_frenet += self.precision

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
                line_in_pixel = (self.world_to_pixel(line[0]), self.world_to_pixel(line[1])) 
                pygame.draw.line(self.surface,car_color_set[2],line_in_pixel[0],line_in_pixel[1],30)     

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
        vmin, vmax, SampleTime = 20, 30, 0.1
        from_road_id = 0
        to_road_id = 10
        flow =  generateCarFlow(self.run_time, 1000, vmin, vmax, SampleTime)
        for t in range(self.run_time):
            for vehicle_info in flow:
                temp_id = (from_road_id, to_road_id, vehicle_info[1])
                if temp_id not in self.vehicle_demand and vehicle_info[1] > t:
                    pos_frenet = 0
                    road_id = 0
                    lane_number = 1
                    direction = 0 if lane_number < 0 else 1              
                    self.vehicle_demand[temp_id] = Vehicle(self.map, road_id, direction, \
                        abs(lane_number), pos_frenet, SampleTime, vehicle_info)  

    def action(self):
        for t in range(self.run_time): 
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
                vehicle = self.vehicle_running[vehicle_id]
                vehicle.move()  
                if vehicle.road_id == vehicle_id[1]: # arrive at to_road
                    running_pop.append(vehicle_id)  
                else:    
                    self.vehicle_record[t][vehicle_id] = vehicle.shape[:]
            for vehicle_id in running_pop:
                self.vehicle_running.pop(vehicle_id) # del vehicle

    def visulizer(self):
        # initialize pygame
        pygame.init()
        screen_width = 600  #1220
        display = pygame.display.set_mode((screen_width, screen_width))
        pygame.display.set_caption("SIMULATION")
        world_image = WorldImage(self, display)

        for t in range(self.run_time):
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
            world_image.surface.blit(world_image.big_map_surface, (0,0))
            world_image.draw_vehicle(t)

            display.blit(pygame.transform.scale(world_image.surface,display.get_rect().size), (0, 0))
            pygame.display.update()
            time.sleep(0.1)    

def simulation():
    run_time = 1000            # running time for simulation
    world = World(run_time)
    world.load_map()          # init network
    world.generate_flow()     # init flow
    world.action()
    world.visulizer()         # visulization

def main():
    simulation()

if __name__ == '__main__':
    main()



