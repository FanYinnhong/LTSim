from turtle import pd
import numpy as np
import sys
import pdb

# allcars are the status before reaction time
def getSurCar(ego, radius = 100):
    surCar = {}
    allcar = ego.world.vehicle_running
    # get other cars by Euclidean distance
    # status: [pos_x, pos_y, vel, theta, phi, acc, omega]
    for other_id in allcar:
        other = allcar[other_id]
        if ego != other:            
            if ego.x >= 0 and other.road_id == ego.road_id and other.direction == ego.direction and np.hypot(ego.x - other.x, ego.y - other.y) < radius:
                surCar[other_id] = other
    return surCar

def get_speed_sign(ego):
    road = ego.map.roads[ego.map._road_id_to_index[ego.road_id]]
    is_junction = False if road.junction == None else True
    if (is_junction):
        junction_index = ego.map._road_id_to_junction_index[ego.road_id]
        connection = ego.map.junctions[junction_index[0]].connections[junction_index[1]]
        if connection.contactPoint == "start":            
            speed_sign = 1
        elif connection.contactPoint == "end":
            speed_sign = -1
    else: # drive on the road segment
        # drive along the positive position of the road
        if (ego.direction == 0):
            speed_sign = 1
        else:
            speed_sign = -1
    return speed_sign

def planning(ego, SampleTime):
    surcar = getSurCar(ego)
    speed_sign = get_speed_sign(ego)
    # determine the lead vehicle 
    lead_vehicle = None
    for vehicle_id in surcar:
        vehicle = surcar[vehicle_id]
        if vehicle.pos_frenet * speed_sign > ego.pos_frenet * speed_sign:
            if not lead_vehicle:
                lead_vehicle = vehicle
            elif vehicle.pos_frenet * speed_sign < lead_vehicle.pos_frenet * speed_sign:
                lead_vehicle = vehicle
    if lead_vehicle:
        acc = IDM(ego.speed, lead_vehicle.speed, abs(lead_vehicle.pos_frenet - ego.pos_frenet))
    else:
        acc = IDM(ego.speed, -1, -1)

    dp_s = max(ego.speed * SampleTime + 0.5 * acc * SampleTime**2, 0)
    dp_t = 0
    d_v = acc * SampleTime
    d_hdg = 0
    d_status = [dp_s, dp_t, d_v, d_hdg]
    return d_status


def IDM(v_cur, v_lead, s_cur, v_des = 30):
    '''
    :reference: https://traffic-simulation.de/info/info_IDM.html
    :return: acceleration
    '''
    a_max = 10
    T = 1.0
    s_0 = 2.0
    acc = 3.0
    dec = 6.0
    if v_lead == -1:
        v_lead = v_des
        s_cur = sys.maxsize

    d_v = v_cur - v_lead
    if s_cur == 0:
        s_cur = 0.00001
    s_star = s_0 + max(0, (v_cur * T + (v_cur * d_v) / (2 * np.sqrt(acc * dec))))
    a = a_max * (1 - pow(v_cur / v_des, 4) - pow(s_star / s_cur, 2))
    if a > 0:
        a = min(acc, a)
    if a < 0:
        a = max(-dec, a)
    return a