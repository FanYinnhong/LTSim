import threading
import pandas as pd
import os, sys, time, random
sys.path.append('..')
sys.path.append('.')
import warnings 
warnings.filterwarnings("ignore") 
from pygame.locals import *
import pdb

from lxml import etree
from libs.opendriveparser import parse_opendrive
from libs.geom.Vector3D import Vector3D
from libs.utils.map import Map
from libs.utils.vehicle import Vehicle
from libs.utils.trafficLight import TrafficLight
from libs.utils.worldImage import WorldImage
from libs.utils.decoration import InputContrl, Button, Text
from libs.utils.carflow import generateCarFlow
from libs.utils.image_configure import *

SAMPLE_TIME = 0.1

class World(object):
    """
    Class that contains all the information of
    simulation that is running on the server side
    """

    def __init__(self, run_time, map_name):
        self.run_time = run_time
        # World data
        self.map = None
        self.vehicle_running = {}
        self.vehicle_demand = {}
        self.vehicle_record = {}
        self.tls = {}
        self.tl_record = {}
        self.tl_control_record = {}
        self.map_name = map_name
        self.input_control = InputContrl()

    def load_map(self):
        "load map files and get infos"

        def read_map_file():
            # read road network
            _ME_PATH = os.path.abspath(os.path.dirname(__file__))
            DATA_PATH = os.path.normpath(os.path.join(_ME_PATH, '../../data/xml'))
            filename = self.map_name
            filename += ".xodr"
            filepath = os.path.join(DATA_PATH, filename)
            # get road network infoplan_tra
            with open(filepath, 'r') as f:
                parser = etree.XMLParser()
                rootNode = etree.parse(f, parser).getroot()
                roadNetwork = parse_opendrive(rootNode)
            return roadNetwork

        self.map = Map(read_map_file())
    def save_map(self):
        "save map to csv"
        _ME_PATH = os.path.abspath(os.path.dirname(__file__))
        DATA_PATH = os.path.normpath(os.path.join(_ME_PATH, '../../data'))
        filename = "road.csv"
        filepath = os.path.join(DATA_PATH, filename)

        road_column = ['id', 'start point x', 'start point y', 'end point x','end point y','length',
                       'predecessor element id', 'predecessor element type',
                       'successor element id', 'successor element type']
        road_list = []
        for road in self.map.roads.values():
            start_point, _ = road.planView.calc(0)
            end_point, _ = road.planView.calc(road.length)
            list_temp = [road.id, start_point[0], start_point[1], end_point[0], end_point[1], road.length]
            if (road.link.predecessor != None):
                list_temp.append(road.link.predecessor.elementId)
                list_temp.append(road.link.predecessor.elementType)
            else:
                list_temp.append("None")
                list_temp.append("None")
            if (road.link.successor != None):
                list_temp.append(road.link.successor.elementId)
                list_temp.append(road.link.successor.elementType)
            else:
                list_temp.append("None")
                list_temp.append("None")
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

    def generate_flow(self, demands_url):
        flow = generateCarFlow(demands_url, self.run_time, SAMPLE_TIME)
        for t in range(self.run_time):
            for vehicle_info in flow:
                from_road_id = vehicle_info[0]
                to_road_id = vehicle_info[1]
                sgn = vehicle_info[2]
                arrival_time = vehicle_info[3]
                temp_id = (from_road_id, to_road_id, arrival_time, sgn)
                if temp_id not in self.vehicle_demand and arrival_time > t:
                    scf = 0
                    lane_number = random.randint(1, 2) * sgn
                    direction = 0 if lane_number > 0 else 1
                    self.vehicle_demand[temp_id] = Vehicle(temp_id, self, self.map, from_road_id, direction, abs(lane_number), scf, SAMPLE_TIME, vehicle_info)

    def action(self):
        vehicle_out = open('vehicle_out.csv', 'w+')
        for t in range(self.run_time):
            if t % 10 == 0:
                print('running time: ', t)

            # update the state of traffic light at current time
            self.tl_record[t] = {}
            # self.tl_control_record[t] = {}
            for index in self.tls:
                self.tls[index].state_calculation(t)
                self.tl_record[t][index] = self.tls[index].current_state
                #for connect_id in self.tls[index].connect_id_list:
                #    self.tl_control_record[t][connect_id] = self.tls[index].current_state

            self.vehicle_record[t] = {}
            # demand_pop = []
            for vehicle_id in self.vehicle_demand:
                if self.vehicle_demand[vehicle_id] != -1:
                    if t > vehicle_id[2]:
                        self.vehicle_running[vehicle_id] = self.vehicle_demand[vehicle_id]
            #            demand_pop.append(vehicle_id)
            # for vehicle_id in demand_pop:
            #      self.vehicle_demand.pop(vehicle_id)  # del vehicle

            running_pop = []
            cur_move = []
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
                scf_end = vehicle.scf_end
                vehicle.perceptron.get_sur_traffic(self.tl_record[t])
                vehicle.predictor.predict(vehicle.plan_period)
                if len(vehicle.plan_tra) <= vehicle.plan_period - vehicle.plan_gap or vehicle.plan_road_id != vehicle.road_id or vehicle.planner.tra_check(vehicle.plan_tra) or vehicle.scf_end != scf_end:
                    vehicle.plan_road_id = vehicle.road_id
                    vehicle.plan_direction = vehicle.direction
                    motion_status = [vehicle.scf, vehicle.tcf, vehicle.vel, vehicle.hdg, vehicle.scf_end]
                    # pdb.set_trace()
                    vehicle.plan_tra = vehicle.planner.planning(motion_status, vehicle.plan_period)
                    if vehicle_id in self.vehicle_demand:
                        if not vehicle.planner.tra_check(vehicle.plan_tra): 
                            self.vehicle_demand.pop(vehicle_id)  # del vehicle
                            cur_move = vehicle.plan_tra.pop(0)
                        else:
                            running_pop.append(vehicle_id)
                    else:
                        cur_move = vehicle.plan_tra.pop(0)
                else:
                    cur_move = vehicle.plan_tra.pop(0)
                if cur_move:
                    vehicle_out.write(str(vehicle.id) + '|' + str(vehicle.road_id) + '|' + str(cur_move) + '|' + str(vehicle.scf_end) +'\n')
                    vehicle.move(cur_move)
                    if vehicle.road_id == vehicle_id[1]:  # arrive at to_road
                        running_pop.append(vehicle_id)
                    else:
                        plan_tra_record = []
                        for point in vehicle.plan_tra:
                            _road_id, _direction, _scf, _tcf, _hdg, _attitude = vehicle.map.frenet_to_cartesian(vehicle.plan_road_id, vehicle.plan_direction, point[0], point[1], point[3])
                            if _road_id == -1:
                                break
                            plan_tra_record.append([_attitude[0], _attitude[1]])
                        motion_status = [vehicle.scf, vehicle.tcf, vehicle.vel, vehicle.hdg]
                        self.vehicle_record[t][vehicle_id] = [vehicle.shape[:], plan_tra_record[:], motion_status]
            for vehicle_id in running_pop:
                self.vehicle_running.pop(vehicle_id)  # del vehicle


    def create_traffic_light(self):
        def get_init_pos(incomingRoad, junction_id):
            if self.map.roads[incomingRoad].link.successor.elementId == junction_id:
                direction = 0
            elif self.map.roads[incomingRoad].link.predecessor.elementId == junction_id:
                direction = 1            
            scf = self.map.roads[incomingRoad].length - 0.1
            x_1, y_1, r_1 = self.map.get_position(incomingRoad, direction, scf, 0, 0)
            x_2, y_2, r_2 = self.map.get_position(incomingRoad, direction, scf, 8, 0)
            return [(x_1, y_1), (x_2, y_2)]

        right_vector = Vector3D()
        cycle_set = {}
        for junction_id in self.map.junctions.keys():   
            cycle_set[junction_id] = {}
            junction = self.map.junctions[junction_id]
            for connection in junction.connections:
                if connection.incomingRoad not in cycle_set[junction_id]:
                    cycle_set[junction_id][connection.incomingRoad] = []
                cycle_set[junction_id][connection.incomingRoad].append((
                    connection.connectingRoad,  
                    connection.leavingRoad))

        st, gt, yt, rt = 0, 80, 20, 200
        for junction_id in self.map.junctions.keys():   
            for incomingRoad in cycle_set[junction_id]:
                direction_group = []
                for item in cycle_set[junction_id][incomingRoad]:
                    direction_group.append(item[0])    
                self.tls[(junction_id, incomingRoad)] = TrafficLight(get_init_pos(incomingRoad, junction_id), st, gt, yt, rt, direction_group)
                st += (gt + yt)

    def nearest_car(self, d_x, d_y):
        vehicle_id_chosen = (-100, -100, -100)
        dis = None
        for vehicle_id in self.vehicle_record[self.clock_tick]:
            v_x = self.vehicle_record[self.clock_tick][vehicle_id][0][0][0][0] + self.map._world_offset[0]
            v_y = self.vehicle_record[self.clock_tick][vehicle_id][0][0][0][1] + self.map._world_offset[1]
            # print("car_x", v_x, "car_y", v_y)
            temp = (d_x - v_x) ** 2 + (d_y - v_y) ** 2
            if dis == None:
                if temp <= 1250:
                    dis = temp
                    vehicle_id_chosen = vehicle_id
            else:
                if temp < dis:
                    vehicle_id_chosen = vehicle_id
        return vehicle_id_chosen

    def update_info(self):
        sum_velocity = 0
        self.veh_count = 0
        self.average_speed = 0
        for vehicle_id in self.vehicle_record[self.clock_tick]:
            sum_velocity += self.vehicle_record[self.clock_tick][vehicle_id][2][2]
            self.veh_count +=1
        if self.veh_count >= 1:
            self.average_speed = round(sum_velocity/self.veh_count, 2)

    def show_road_information(self):
        road_id = self.map.collision_check(self.x_world, self.y_world)
        s = " Map: {map_name}\n Time: {time} s\n " \
            "Road count: {road_count}\n Junction count:{junc_count} \n " \
            "Vehicle count: {veh_count}\n Average speed: {av_speed} m/s\n ".format(map_name=self.map_name, time=round(self.clock_tick/10, 1),
                                                          road_count=len(self.map.roads),junc_count=len(self.map.junctions),
                                                          veh_count=self.veh_count, av_speed=self.average_speed)
        if road_id >= 0:
            s += "\n\n Road id: {road}.\n Predecessor: {pre}\n Successor: {suc}\n " \
                 "Length: {l}\n ".format(road=str(road_id),
                           pre=self.map.roads[road_id].link.predecessor.elementId,
                           suc=self.map.roads[road_id].link.successor.elementId,
                           l=round(self.map.roads[road_id].length, 2))

        if (self.focus_on and self.info_junction_id >= 0):
            x, y = self.map.junctions[self.info_junction_id].pos_left_top
            coor = "(" + str(round(x, 2)) + ", " + str(round(y, 2)) + ")"
            self.str_junction = "Junction id: {junc}.\n Left top coor:\n {lt_coor}.\n ".format(junc=self.info_junction_id,
                                                                                     lt_coor=coor)
        if (self.focus_on and len(self.str_junction) != 0):
            s += self.str_junction
        if (not self.focus_on):
            self.str_junction = ''
            self.info_junction_id = -1

        self.text.set_text(s)

        time.sleep(0.3)
        self.show_road_information()

    def draw_rect_alpha(self, surface, color, rect):
        shape_surf = pygame.Surface(pygame.Rect(rect).size, pygame.SRCALPHA)
        pygame.draw.rect(shape_surf, color, shape_surf.get_rect())
        surface.blit(shape_surf, rect)

    def visulizer(self, is_draw):
        # initialize pygame
        pygame.init()
        screen_width = 700
        display = pygame.display.set_mode((screen_width * 1.3, screen_width))
        pygame.display.set_caption("SIMULATION")
        world_image = WorldImage(self, self.map_name, is_draw) # 0 for re
        self.text = Text(pygame.font.Font(pygame.font.get_default_font(), 15),
                         (int(0.3*screen_width), int(0.95*screen_width)), (int(screen_width), int(0.05*screen_width)))
        self.str_junction = ''
        self.info_junction_id = -1

        # world_image start
        # set the position and size of visualization window
        self.scaled_size = screen_width
        self.prev_scaled_size = screen_width
        self.x_offset = 0
        self.y_offset = 0
        self.x_scale_offset = 0
        self.y_scale_offset = 0
        self.dragged = None
        self.mouse_x = None
        self.mouse_y = None
        self.x_world = 0
        self.y_world = 0

        # in order to get the time
        self.clock_tick = 0
        self.is_suspend = False
        self.veh_count = 0
        self.average_speed = 0

        # get focus mode
        self.focus_on = False
        right_click_junction_id = -1

        thread1 = threading.Thread(name="show_road_information", target=self.show_road_information, args=())
        thread1.daemon = True
        thread1.start()

        # get the information through cursor
        self.cursor_x = 0
        self.cursor_y = 0

        def suspend():
            self.is_suspend = not self.is_suspend
        suspend_button = Button(int(0.02 * screen_width), 0, int(0.1 * screen_width), int(0.05 * screen_width), 'Suspend', suspend)

        def restart():
            self.clock_tick = 0
            self.is_suspend = False
            suspend_button.reset_text('Suspend')
            self.x_offset = 0
            self.y_offset = 0
            self.x_scale_offset = 0
            self.y_scale_offset = 0
            self.prev_scaled_size = screen_width
            self.vehicle_id_chosen = (-100, -100, -100)
            self.hero_mode = False
            self.hero_mode_choose = False
            self.input_control.wheel_offset = 1
        restart_button = Button(int(0.14 * screen_width), 0,
                                int(0.1 * screen_width), int(0.05 * screen_width), 'Restart', restart)
        def restore():
            self.x_offset = 0
            self.y_offset = 0
            self.x_scale_offset = 0
            self.y_scale_offset = 0
            self.prev_scaled_size = screen_width
            self.input_control.wheel_offset = 1
            self.vehicle_id_chosen = (-100, -100, -100)
            self.hero_mode = False
            self.hero_mode_choose = False
        restore_button = Button(int(0.26 * screen_width), 0,
                                int(0.1 * screen_width), int(0.05 * screen_width), 'Restore', restore)
        def focus():
            focus_button.button_rect = pygame.Rect(-100, -100, int(0.2 * screen_width), int(0.05 * screen_width))
        focus_button = Button(-100, -100, int(0.2 * screen_width), int(0.05 * screen_width),
                              'Focus junction', focus)

        self.movable = False
        def whether_movable():
            self.movable = not self.movable
        movable_button = Button(int(0.38 * screen_width), 0,
                                int(0.1 * screen_width), int(0.05 * screen_width), 'Movable', whether_movable)

        self.vehicle_id_chosen = (-100, -100, -100)
        self.hero_mode_choose = False
        def hero_choose():
            if self.hero_mode:
                self.hero_mode_choose = False
            else:
                self.hero_mode_choose = not self.hero_mode_choose
            self.hero_mode = False
            if not self.hero_mode_choose:
                self.vehicle_id_chosen = (-100, -100, -100)
        hero_button = Button(int(0.5 * screen_width), 0,
                             int(0.15 * screen_width), int(0.05 * screen_width), 'Hero Mode', hero_choose)
        self.hero_mode = False
        def hero_car():
            hero_car_button.button_rect = pygame.Rect(-200, -100, int(0.2 * screen_width),
                                                      int(0.05 * screen_width))
            self.hero_mode = True
            self.hero_mode_choose = False
        hero_car_button = Button(-200, -100, int(0.2 * screen_width), int(0.05 * screen_width),
                                 'Choose car', hero_car)

        while (True):
            if self.run_time == self.clock_tick + 1:
                if not self.is_suspend:
                    suspend_button.callback()
                    suspend_button.reset_text('Exit')
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
                        elif restart_button.button_rect.collidepoint(event.pos):
                            restart_button.callback()
                        elif restore_button.button_rect.collidepoint(event.pos):
                            restore_button.callback()
                            self.focus_on = False
                        elif focus_button.button_rect.collidepoint(event.pos):
                            focus_button.callback()
                            self.focus_on = True
                        elif movable_button.button_rect.collidepoint(event.pos):
                            movable_button.callback()
                        elif hero_button.button_rect.collidepoint(event.pos):
                            hero_button.callback()
                        elif hero_car_button.button_rect.collidepoint(event.pos):
                            hero_car_button.callback()
                        else:
                            # make sure the menu list is not shown on the screen
                            focus_button.button_rect = pygame.Rect(-100, -100,
                                                                   int(0.2 * screen_width), int(0.05 * screen_width))
                            hero_car_button.button_rect = pygame.Rect(-200, -100,
                                                                   int(0.2 * screen_width), int(0.05 * screen_width))
                            if (not self.hero_mode):
                                self.vehicle_id_chosen = (-100, -100, -100)
                            if self.movable:
                                self.dragged = True
                                self.mouse_x, self.mouse_y = event.pos
                    if event.button == 3:
                        self.cursor_x, self.cursor_y = pygame.mouse.get_pos()
                        d_x, d_y = world_image.cursor_to_world(
                            int((self.x_scale_offset + self.x_offset + self.cursor_x)
                                * world_image._width_in_pixels / self.scaled_size),
                            int((self.y_scale_offset + self.y_offset + self.cursor_y)
                                * world_image._width_in_pixels / self.scaled_size))
                        if not self.hero_mode_choose:
                            d_road_id = self.map.collision_check(d_x, d_y)
                            if d_road_id in self.map._road_id_to_junction_id.keys():
                                right_click_junction_id = self.map._road_id_to_junction_id[d_road_id][0]
                                focus_button.button_rect = pygame.Rect(self.cursor_x, self.cursor_y,
                                                                       int(0.2 * screen_width), int(0.05 * screen_width))
                        else:
                            self.vehicle_id_chosen = self.nearest_car(d_x, d_y)
                            # print(d_x,d_y)
                            if self.vehicle_id_chosen != (-100, -100, -100):
                                hero_car_button.button_rect = pygame.Rect(self.cursor_x, self.cursor_y,
                                                              int(0.2 * screen_width), int(0.05 * screen_width))

                    self.input_control._parse_zoom(event.button)

                elif event.type == pygame.MOUSEMOTION:
                    if self.dragged and self.movable:
                        temp_x, temp_y = event.pos
                        self.x_offset -= temp_x - self.mouse_x
                        self.y_offset -= temp_y - self.mouse_y
                        self.mouse_y = temp_y
                        self.mouse_x = temp_x
                    elif not self.dragged:
                        self.cursor_x, self.cursor_y = pygame.mouse.get_pos()
                        if suspend_button.button_rect.collidepoint(event.pos):
                            suspend_button.color = COLOR_ACTIVE
                        elif restart_button.button_rect.collidepoint(event.pos):
                            restart_button.color = COLOR_ACTIVE
                        elif restore_button.button_rect.collidepoint(event.pos):
                            restore_button.color = COLOR_ACTIVE
                        elif focus_button.button_rect.collidepoint(event.pos):
                            focus_button.color = COLOR_ACTIVE
                        else:
                            suspend_button.color = COLOR_INACTIVE
                            restart_button.color = COLOR_INACTIVE
                            restore_button.color = COLOR_INACTIVE
                            focus_button.color = COLOR_INACTIVE
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        self.dragged = False

            display.fill(COLOR_ALUMINIUM_4)
            world_image.surface.blit(world_image.big_map_surface, (0, 0))
            world_image.draw_traffic_light(self.clock_tick)
            world_image.draw_vehicle(self.clock_tick)
            if (self.hero_mode_choose or self.hero_mode) and self.vehicle_id_chosen != (-100, -100, -100):
                world_image.draw_chosen_vehicle(self.clock_tick, self.vehicle_id_chosen)
            self.update_info()

            if right_click_junction_id >= 0 and self.focus_on:
                # we assume that all junctions sizes are similar and a constant 120 x 120m windows is adopted here
                self.input_control.wheel_offset = 1 / (self.map._world_width / 120)
                self.scaled_size = int(screen_width / self.input_control.wheel_offset)
                self.prev_scaled_size = self.scaled_size
                self.y_scale_offset = 0
                self.x_scale_offset = 0
                x_temp, y_temp = self.map.junctions[right_click_junction_id].pos_left_top
                x_temp -= (self.map._world_offset[0] + 35)
                y_temp -= (self.map._world_offset[1] - 35)
                x_temp, y_temp = world_image.world_to_pixel([x_temp, y_temp])
                self.x_offset = int(x_temp / world_image._width_in_pixels * self.scaled_size)
                self.y_offset = int(y_temp / world_image._width_in_pixels * self.scaled_size)
                self.info_junction_id = right_click_junction_id
                right_click_junction_id = -1

            if self.hero_mode and self.vehicle_id_chosen != (-100, -100, -100):
                # the horizon of the hero mode is 100 x 100m
                self.input_control.wheel_offset = 1 / (self.map._world_width / 100)
                self.scaled_size = int(screen_width / self.input_control.wheel_offset)
                self.prev_scaled_size = self.scaled_size
                self.y_scale_offset = 0
                self.x_scale_offset = 0
                x_car = self.vehicle_record[self.clock_tick][self.vehicle_id_chosen][0][0][0][0]
                y_car = self.vehicle_record[self.clock_tick][self.vehicle_id_chosen][0][0][0][1]
                x_car -= 50
                y_car += 50
                x_car, y_car = world_image.world_to_pixel([x_car, y_car])
                self.x_offset = int(x_car / world_image._width_in_pixels * self.scaled_size)
                self.y_offset = int(y_car / world_image._width_in_pixels * self.scaled_size)

            self.scaled_size = int(screen_width / self.input_control.wheel_offset)
            px = (screen_width/2 + self.x_scale_offset + self.x_offset) / float(self.prev_scaled_size)
            py = (screen_width/2 + self.y_scale_offset + self.y_offset) / float(self.prev_scaled_size)
            diff_between_scales = (-(float(self.prev_scaled_size) * px) + (float(self.scaled_size) * px),
                                   -(float(self.prev_scaled_size) * py) + (float(self.scaled_size) * py))
            self.x_scale_offset = self.x_scale_offset + diff_between_scales[0]
            self.y_scale_offset = self.y_scale_offset + diff_between_scales[1]
            self.prev_scaled_size = self.scaled_size

            # transform the cursor position into the world coordinate and return information
            self.x_world, self.y_world = world_image.cursor_to_world(
                int((self.x_scale_offset + self.x_offset + self.cursor_x) * world_image._width_in_pixels / self.scaled_size),
                int((self.y_scale_offset + self.y_offset + self.cursor_y) * world_image._width_in_pixels / self.scaled_size))
            display.blit(pygame.transform.scale(world_image.surface,
                                                (self.scaled_size, self.scaled_size)),
                         (- self.x_offset - self.x_scale_offset,
                          - self.y_offset - self.y_scale_offset))
            # adding mask
            if self.focus_on:
                self.draw_rect_alpha(display, (160, 160, 160, 127),
                                     (0, 0, screen_width, int(0.15 * screen_width)))
                self.draw_rect_alpha(display, (160, 160, 160, 127),
                                     (0, int(0.15 * screen_width), int(0.15 * screen_width), int(0.7 * screen_width)))
                self.draw_rect_alpha(display, (160, 160, 160, 127),
                                     (int(0.85 * screen_width), int(0.15 * screen_width),
                                      int(0.15 * screen_width), int(0.7 * screen_width)))
                self.draw_rect_alpha(display, (160, 160, 160, 127),
                                     (0, int(0.85 * screen_width), screen_width, int(0.15 * screen_width)))

            display.blit(self.text.surface, self.text.pos)
            pygame.draw.rect(display, COLOR_INACTIVE, pygame.Rect(0, 0, 1.3 * screen_width, int(0.05 * screen_width)))
            # timer for implementing suspend function
            restart_button.draw(display)
            suspend_button.draw(display)
            restore_button.draw(display)
            focus_button.draw(display)
            hero_car_button.draw(display)
            if self.movable:
                movable_button.color = COLOR_ACTIVE
            else:
                movable_button.color = COLOR_INACTIVE
            if self.hero_mode_choose or self.hero_mode:
                hero_button.color = COLOR_ACTIVE
            else:
                hero_button.color = COLOR_INACTIVE
            movable_button.draw(display)
            hero_button.draw(display)

            if not self.is_suspend and not self.hero_mode_choose:
                self.clock_tick += 1

            pygame.display.update()
            time.sleep(0.001)
