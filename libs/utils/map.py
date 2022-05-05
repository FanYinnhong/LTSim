import pandas as pd
from shapely.geometry import Polygon
from shapely.ops import unary_union
import math
import numpy as np
import heapq
import matplotlib.path as malPath
# in order to validate polygon shape
import matplotlib.pyplot as plt
from descartes import PolygonPatch
from ..geom.Vector3D import Vector3D
import random

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
        # select a junction id to get the shape
        # self.shape = self.junction_polygon(60)
        # xy = np.array(self.shape.exterior)
        # self.shape_x, self.shape_y = xy[:,0], xy[:,1]
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

    def frenet_to_cartesian(self, road_id, direction, scf, tcf, hdg):
        # if (self.roads[road_id].length < s):
        #     raise OverflowError("Exceeds the limits of current road")    
        while(scf > self.roads[road_id].length):
            road = self.roads[road_id]
            scf -= road.length
            is_junction = False if road.junction == None else True
            if (is_junction):
                # calculate pos_length
                # it needs to check the direction of the current road, it is determined by the contact point of the connecting road
                junction_id = self._road_id_to_junction_id[road_id]
                connection = self.junctions[junction_id[0]].connections[junction_id[1]]
                # the remain frenet in the new road
                # scf -= self.roads[connection.connectingRoad].length
                # update the road id of the new road
                road_id = connection.leavingRoad
                if connection.contactPointOnLeavingRoad == "start":
                    direction = 0
                else:
                    direction = 1
            else:  # drive on the road segment
                # drive along the positive position of the road
                if direction == 0:
                    road_next = road.link.successor
                else:
                    road_next = road.link.predecessor
                # if next is a junction
                # scf -= road.length
                if road_next.elementType == "junction":
                    # Choose a road in junction's connections groups
                    junction = self.junctions[road_next.elementId]
                    connection_available = []
                    for connection in junction.connections:
                        if connection.incomingRoad == road.id:
                            connection_available.append(connection)
                            break
                    connection_chosen = random.choice(connection_available)
                    connectingRoad = connection_chosen.connectingRoad
                    contactPoint = connection_chosen.contactPoint
                    if contactPoint == "start":
                        direction = 0
                    else:
                        direction = 1
                    road_id = connectingRoad
                # if next is a road
                else:
                    if road_next.contactPoint == "start":
                        direction = 0
                    else:
                        direction = 1
                    road_id = road_next.elementId
            if road_id == -1:
                return -1, -1, -1, -1, -1, -1
        return road_id, direction, scf, tcf, hdg, self.get_position(road_id, direction, scf, tcf, hdg)

    def get_position(self, road_id, direction, scf, tcf, hdg):
        geometries = self.roads[road_id].planView.geometries
        if direction == 1:
            scf = self.roads[road_id].length - scf
        for idx, geometry in enumerate(geometries):
            if (scf > geometry.getLength()):
                scf -= geometry.getLength()
            else:
                pos, road_hdg = geometry.calcPosition(scf)
                break
        x, y = pos[0] - self._world_offset[0], pos[1] - self._world_offset[1]
        # if direction > 0:
        #     rotation_radian = road_hdg + hdg
        # else:
        #     rotation_radian = road_hdg - hdg
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
                pos, hdg = road.planView.calc(s_current)
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
            if t_current > 10 or t_current < -10 or s_current == 0:
                continue
            else:
                return (s_current, t_current, road_id)
        return None


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
            return unary_union([
                geom if geom.is_valid else geom.buffer(0) for geom in geoms])

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
                # print(polygon_temp.area)
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
        # plt.show()
        return u