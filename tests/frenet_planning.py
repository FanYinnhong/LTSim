from re import S
import numpy as np
import pdb, math, random
from copy import deepcopy

def get_shape_pos(vehicle, attitude):
    x, y, rotation_radian = attitude
    def CooGet(para, margin = 0):
        half_width = vehicle.vehicle_w / 2 + margin
        half_length = vehicle.vehicle_l / 2 + margin
        half_diagonal = np.sqrt(half_width**2+half_length**2) 
        x, y, theta = para
        theta0 = np.arctan(half_width/half_length)
        dx1 = round(half_diagonal * np.cos(theta0+theta),3)
        dy1 = round(half_diagonal * np.sin(theta0+theta),3)
        dx2 = round(half_diagonal * np.cos(theta0-theta),3)
        dy2 = round(half_diagonal * np.sin(theta0-theta),3)
        Pa = (round(x-dx2,3),round(y+dy2,3))
        Pb = (round(x-dx1,3),round(y-dy1,3))
        Pc = (round(x+dx2,3),round(y-dy2,3))
        Pd = (round(x+dx1,3),round(y+dy1,3))
        return [Pa, Pb, Pc, Pd]    
    [Pa, Pb, Pc, Pd] = CooGet([x, y, rotation_radian])
    shape_points = [(Pa,Pb),(Pb,Pc),(Pc,Pd),(Pd,Pa)]
    hdg_vec = (np.cos(rotation_radian), np.sin(rotation_radian))
    shape_line = {}
    for line in shape_points:
        shape_line[line] = np.sign((line[0][0] - x) * hdg_vec[0] + (line[0][1] - y) * hdg_vec[1]) + np.sign((line[1][0] - x) * hdg_vec[0] + (line[1][1] - y) * hdg_vec[1])
    forward_shape_line = max(shape_line.items(), key = lambda x: x[1])[0]
    for i in range(len(shape_points)):
        if shape_points[i] == forward_shape_line:
            # mid point in forward line
            mid_point = ((forward_shape_line[0][0] + forward_shape_line[1][0])/2, 
                        (forward_shape_line[0][1] + forward_shape_line[1][1])/2)
            forward_point = ((3*mid_point[0] - x)/2, (3*mid_point[1] - y)/2)
            break
    shape_points = shape_points[0:i] + [(forward_shape_line[0],forward_point),(forward_point,forward_shape_line[1])] + shape_points[i+1:]
    return shape_points

def poly_fitting(status_0, status_t, period): # fit by cubic polynomial function    
    tra = []
    px_0,py_0,v0,theta_0 = status_0
    px_t,py_t,vt,theta_t = status_t
    t = period
    vx_0, vy_0 = v0 * np.cos(theta_0), v0 * np.sin(theta_0)
    vx_t, vy_t = vt * np.cos(theta_t), vt * np.sin(theta_t)
    # P*A = B, A = P^-1 * B
    P = np.matrix([[t**2   , t       , 0       , 0       ],
                  [0       , 0       , t**2    , t       ],
                  [1/3*t**3, 1/2*t**2, 0       , 0       ],
                  [0       , 0       , 1/3*t**3, 1/2*t**2]])
    B = np.matrix([[vx_t-vx_0],[vy_t-vy_0],[px_t-(px_0+vx_0*t)],[py_t-(py_0+vy_0*t)]]) 
    A = np.dot(P.I,B).tolist()
    a1, b1, a2, b2 = A[0][0],A[1][0],A[2][0],A[3][0]
    t_list = np.arange(1, round(t*10)+1)/10
    vx = a1*(t_list)**2 + b1*(t_list) + vx_0
    vy = a2*(t_list)**2 + b2*(t_list) + vy_0
    px = 1/3*a1*(t_list)**3 + 1/2*b1*(t_list)**2 + vx_0*(t_list) + px_0
    py = 1/3*a2*(t_list)**3 + 1/2*b2*(t_list)**2 + vy_0*(t_list) + py_0
    if min(vx) < -0.01:
        pdb.set_trace()
    theta = theta_0
    for i in range(len(t_list)):
        if vx[i] > 0.1:
            theta = np.arctan(vy[i]/(vx[i]))  # keep the same with last point when vel is very low
        tra.append([round(px[i],3), round(py[i],3), round(np.hypot(vx[i],vy[i]),3), round(theta,3)])        
    return tra

def Bezier_fitting(status_0, status_t, period):
    def B(n,i,t):    
        return math.factorial(n)/(math.factorial(i)*math.factorial(n-i)) * t**i * (1-t)**(n-i)
    def Bezier(p,n,c,T):
        res = {}
        k = 0
        for t in np.arange(0, T+c, c):
            x, y = 0, 0
            for i in range(n+1):
                x += B(n,i,t/T) * p[i][0] 
                y += B(n,i,t/T) * p[i][1] 
            if t>0 :
                k = np.arctan((y - y_p)/(x - x_p+0.0001))
            t = round(t, 2)
            x, y, k = round(x, 3), round(y, 3), round(k, 3)
            res[t] = [x,y,k]
            x_p, y_p = x, y
        c2 = c+c
        res[0][2] = round(res[c][2] * 3 - res[c2][2],3)
        return res
    steps =  round(period * 10)
    px_0,py_0,v0,theta_0 = status_0
    px_t,py_t,vt,theta_t = status_t
    Bp = [(px_0,py_0), (1/2*(px_0 + px_t), py_0), (1/2*(px_0 + px_t), py_t), (px_t, py_t)]
    B_Tra = Bezier(Bp, len(Bp)-1, 1, steps)
    tra = []
    for i in range(1, steps + 1):
        v = v0 * (1 - i/steps) + vt * i/steps
        # theta = theta_0 * (1 - i/steps) + theta_t * i/steps
        tra.append(B_Tra[i][0:2] + [v] + [B_Tra[i][2]])
    return tra

def tra_fitting(status_0, status_t_arr, period): # fit by cubic polynomial function    
    tra = []
    for status_t in status_t_arr:
        tra += Bezier_fitting(status_0, status_t, period)
        status_0 = status_t[:]
    return tra

def get_obj_point(vehicle, scf, tcf, vel, PLAN_TIME, scf_end, sampling_times = 1):
    # sampling in acceleration
    def sampling_process(sample, permutation):
        if permutation == []:
            return [[item] for item in sample]
        else:
            ret = []
            for item in sample:
                for sample in permutation:
                    ret.append(sample + [item])
            return ret
    acc_sample = [vehicle.ACC_MAX, 0, -vehicle.DCC_MAX]
    acc_permutation = []
    tcf_list = [2, 6]
    tcf_permutation = []
    for i in range(sampling_times):
        acc_permutation = sampling_process(acc_sample, acc_permutation)
        tcf_permutation = sampling_process(tcf_list, tcf_permutation)

    obj_points_temp = []
    for action in acc_permutation:
        sub_obj_points = []
        s0, v0 = scf, vel
        for acc in action:
            vt = min(max(v0 + acc * PLAN_TIME, 0), vehicle.EXPECT_VEL)
            st = s0 + (v0 + vt)/2 * PLAN_TIME    
            if st > scf_end or (vt + 0)/2 * PLAN_TIME > (scf_end - 5 - st):
                st = max(scf_end - 5, s0)
                vt = 0
            sub_obj_points.append([st, tcf, vt, 0])
            s0, v0 = st, vt        
        if sub_obj_points not in obj_points_temp and len(sub_obj_points) == len(action):
            obj_points_temp.append(sub_obj_points) 

    last_action = acc_permutation[-1]
    last_choice = []
    s0, v0 = scf, vel
    for acc in last_action:
        tcf_min = tcf_list[0] if np.abs(tcf_list[0]-tcf) < np.abs(tcf_list[1]-tcf) else tcf_list[1]
        vt = min(max(v0 + acc * PLAN_TIME, 0), vehicle.EXPECT_VEL)
        st = s0 + (v0 + vt)/2 * PLAN_TIME      
        last_choice.append([st, tcf_min, vt, 0])
        s0, v0 = st, vt   
    if last_choice not in obj_points_temp:
        obj_points_temp.append(last_choice)
        
    obj_points = []
    for obj_point in obj_points_temp:
        for i in range(len(tcf_permutation)):
            obj_points.append(deepcopy(obj_point)) 
            for j in range(sampling_times):
                obj_points[-1][j][1] = tcf_permutation[i][j]
    
    def diff_arr(arr):
        ret = 0
        for i in range(1, len(arr)):
            ret = ret + 1 if arr[i] != arr[i-1] else ret
        return ret
    obj_points.sort(key = lambda arr: diff_arr([tcf] + [item[1] for item in arr]))
    obj_points.sort(key = lambda arr: arr[-1][0], reverse = True)  
    for i in range(len(obj_points)):
        for j in range(len(obj_points[i])):
            scf_diff = abs(obj_points[i][j][0] - scf)
            tcf_diff = abs(obj_points[i][j][1] - tcf)
            if scf_diff < tcf_diff*2:
                obj_points[i] = -1
                break
    obj_points = [item for item in obj_points if item != -1]
    if last_choice not in obj_points:
        obj_points.append(last_choice)
    return obj_points, last_choice

def is_collide_point(ego, pos_1, other, pos_2):  
    def point2point(ego_ps, other_ps):
        ps_1 = [it[0] for it in ego_ps]
        ps_2 = [it[0] for it in other_ps]
        for p1 in ps_1:
            for p2 in ps_2:
                if np.hypot(p1[0]-p2[0], p1[1]-p2[1]) < 2:
                    return True
        return False
    if  np.hypot(pos_1[0]-pos_2[0], pos_1[1]-pos_2[1]) < 6:
        attitude1 = [pos_1[0], pos_1[1], pos_1[3]]
        attitude2 = [pos_2[0], pos_2[1], pos_2[3]]
        if point2point(get_shape_pos(ego, attitude1), get_shape_pos(other, attitude2)):
            return True
    return False    

def is_cross_tra(ego, tra_1, other, tra_2, threshold):
    for i in range(len(tra_1)):
        p0 = tra_1[i]
        p1 = tra_2[i]
        s_0, t_0, v_0, theta_0 = p0 
        s_1, t_1, v_1, theta_1 = p1
        rt = 0
        v_0_m = v_0 * np.cos(theta_0)
        v_1_m = v_1 * np.cos(theta_1)
        dis_acc = s_1 - s_0
        if abs(t_0 - t_1) < 2:
            if tra_1[0][0] < tra_2[0][0]:
                risk = (2 * rt * v_0_m + v_0_m ** 2 / (2 * ego.DCC_MAX) - v_1_m**2/(2 * other.DCC_MAX) + ego.vehicle_l) / (dis_acc + 0.0001)
                if risk > threshold or is_collide_point(ego, p0, other, p1):
                    return True
        else:
            if is_collide_point(ego, p0, other, p1):
                return True
    return False

def tra_check(ego, cur_tra, sur_traffic, threshold = 0.8):    
    for sur_car_id in sur_traffic:
        if sur_car_id in ego.world.vehicle_running:
            sur_car = ego.world.vehicle_running[sur_car_id]
            if is_cross_tra(ego, cur_tra, sur_car, sur_traffic[sur_car_id], threshold):
                return True
    return False

class Perceptron():
    def __init__(self, ego):
        self.ego = ego
        self.PER_RADIUS = 100

    def get_sur_traffic(self, tls):   # get surrounding vehicles
        all_vehicles = self.ego.world.vehicle_running
        cur_road = self.ego.map.roads[self.ego.road_id]
        cur_direction = self.ego.direction
        vehicles_in_range = {}
        per_range = [self.ego.scf + self.PER_RADIUS, self.ego.scf - self.PER_RADIUS]
        remain = []

        def road_chain_forward(cur_road, cur_direction, scf_pos):
            for vehicle_id in all_vehicles:
                if vehicle_id != self.ego.id:
                    vehicle = all_vehicles[vehicle_id]
                    if (vehicle.road_id, vehicle.direction) == (cur_road.id, cur_direction):
                        scf = vehicle.scf + scf_pos
                        if scf <= per_range[0] and scf >= per_range[1]:
                            vehicles_in_range[vehicle_id] = [scf, vehicle.tcf, vehicle.vel, vehicle.hdg]
            if cur_direction == 0:
                road_next = cur_road.link.successor
            else:
                road_next = cur_road.link.predecessor
            road_next_id = road_next.elementId
            if road_next_id == -1:
                return 
            if road_next.elementType == "junction":
                junction_index = road_next_id
                # Choose a road in junction's connections groups
                if (((junction_index, cur_road.id) in tls) and tls[(junction_index, cur_road.id)] != 'g'):
                    remain.append(scf_pos + cur_road.length)
                return
            else:
                road_next_length = self.ego.map.roads[road_next_id].length
                if road_next.contactPoint == "start":
                    road_next_direction = 0
                else:
                    road_next_direction = 1
                if scf_pos + road_next_length > per_range[0]:
                    return
                road_chain_forward(self.ego.map.roads[road_next_id], road_next_direction, scf_pos + road_next_length)
        road_chain_forward(cur_road, cur_direction, 0)

        if remain != []:
            self.ego.scf_end = min(remain)
        else:
            self.ego.scf_end = 1000

        def road_chain_backward(cur_road, cur_direction, scf_pos):
            if scf_pos < per_range[1]:
                return
            if cur_direction == 0:
                road_last = cur_road.link.predecessor
            else:
                road_last = cur_road.link.successor
            if road_last.elementType == "junction":
                return
            else:
                road_last_id = road_last.elementId
                if road_last_id == -1:
                    return
                if road_last.contactPoint == "start":
                    road_last_direction = 1
                else:
                    road_last_direction = 0
                road_last_length = self.ego.map.roads[road_last_id].length
                for vehicle_id in all_vehicles:
                    vehicle = all_vehicles[vehicle_id]
                    if (vehicle.road_id, vehicle.direction) == (road_last_id, road_last_direction):
                        scf = vehicle.scf - road_last_length + scf_pos
                        if scf <= per_range[0] and scf >= per_range[1]:
                            vehicles_in_range[vehicle_id] = [scf, vehicle.tcf, vehicle.vel, vehicle.hdg]
                road_chain_backward(self.ego.map.roads[road_last_id], road_last_direction, scf_pos - road_last_length)
        road_chain_backward(cur_road, cur_direction, 0)

        self.ego.sur_traffic = vehicles_in_range

class Predictor():
    def __init__(self, ego):
        self.ego = ego
    
    def predict(self, period, sampling_times = 1):
        def predict_tra(cur_state, period, rt):
            num = period + rt
            tra = []
            px, py, vel, hdg = cur_state
            px_list = px + vel * np.cos(hdg) / 10 * np.arange(1, num+1)
            py_list = py + vel * np.sin(hdg) / 10 * np.arange(1, num+1)
            for t in range(num):
                tra.append([px_list[t],py_list[t],vel,hdg])
            return tra
        for vehicle_id in self.ego.sur_traffic:
            self.ego.sur_traffic[vehicle_id] = predict_tra(self.ego.sur_traffic[vehicle_id], period * sampling_times, 0)

class Planner():
    def __init__(self, ego):
        self.ego = ego
    def planning(self, motion_status, period):
        # pdb.set_trace()
        scf, tcf, vel, hdg, scf_end = motion_status
        obj_points, last_choice_p = get_obj_point(self.ego, scf, tcf, vel, period/10, scf_end)
        if not last_choice_p:
            pdb.set_trace()
        for obj_point in obj_points:
            cur_tra = tra_fitting([scf, tcf, vel, hdg], obj_point, period/10)  # polynomial function
            if obj_point == last_choice_p:
                best_tra = cur_tra
            if not tra_check(self.ego, cur_tra, self.ego.sur_traffic) and len(cur_tra) > 0:            # check trajectories
                best_tra = cur_tra
                break
        try:
            return best_tra
        except:
            pdb.set_trace()