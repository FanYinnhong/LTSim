import numpy as np
import pdb

def d_tranform(direction):
    if direction == 0:
        return 1
    else:
        return -1

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
    px_0,py_0,v0,theta_0, px_end = status_0
    px_t,py_t,vt,theta_t, px_end = status_t
    if px_end == 0:
        v0, vt = -v0, -vt
    t = period
    vx_0, vy_0 = v0 * np.cos(theta_0), v0 * np.sin(theta_0)
    vx_t, vy_t = vt * np.cos(theta_t), vt * np.sin(theta_t)
    # P*A = B, A = P^-1 * B
    P = np.matrix([[t**2    , t       , 0       , 0       ],
                   [0       , 0       , t**2    , t       ],
                   [1/3*t**3, 1/2*t**2, 0       , 0       ],
                   [0       , 0       , 1/3*t**3, 1/2*t**2]])
    B = np.matrix([[vx_t-vx_0],[vy_t-vy_0],[px_t-(px_0+vx_0*t)],[py_t-(py_0+vy_0*t)]]) 
    A = np.dot(P.I,B).tolist()
    a1, b1, a2, b2 = A[0][0],A[1][0],A[2][0],A[3][0]
    tra = []
    t_list = np.arange(1, round(t*10)+1)/10
    vx = a1*(t_list)**2 + b1*(t_list) + vx_0
    vy = a2*(t_list)**2 + b2*(t_list) + vy_0
    px = 1/3*a1*(t_list)**3 + 1/2*b1*(t_list)**2 + vx_0*(t_list) + px_0
    py = 1/3*a2*(t_list)**3 + 1/2*b2*(t_list)**2 + vy_0*(t_list) + py_0
    theta = theta_0
    for i in range(len(t_list)):
        if vx[i] > 0.1:
            theta = np.arctan(vy[i]/(vx[i]))  # keep the same with last point when vel is very low
        tra.append([px[i],py[i],np.hypot(vx[i],vy[i]),theta])       # get the best choice  
        if (px_end == 0 and px[i] < 0) or (px_end > 0 and px[i] > px_end):
            break
    # print(tra[-1], px_end)
    return tra

def get_obj_point(vehicle, scf, tcf, PLAN_TIME, scf_end):
    t_list = [[2, abs(2-tcf)], [6, abs(6-tcf)]]
    t_list.sort(key = lambda arr: arr[1])
    num = 3 # number of subsection
    v_max = min(vehicle.vel + vehicle.ACC_MAX * PLAN_TIME, vehicle.EXPECT_VEL)
    v_min = max(vehicle.vel - vehicle.DCC_MAX * PLAN_TIME, 0)
    dv1, dv2 = (v_max - vehicle.vel) / num, (vehicle.vel - v_min) / num
    v_list = [v_max - dv1 * i for i in range(num)] + [vehicle.vel] + [vehicle.vel - dv2 * i for i in range(1, num+1)]
    s_list = [[scf + d_tranform(vehicle.direction) * (vehicle.vel + obj_v) / 2 * PLAN_TIME, obj_v] for obj_v in v_list]
    obj_points = []
    for s in s_list:
        for t in t_list:
            obj_points.append((s[0], t[0], s[1], 0, scf_end))
            if s[0] == s_list[-1][0] and t[0] == t_list[0][0]:
                last_choice = (s[0], t[0], s[1], 0, scf_end)
    return obj_points, last_choice

def is_collide_point(ego, pos_1, other, pos_2):  
    def point2point(ego_ps, other_ps):
        ps_1 = [it[0] for it in ego_ps]
        ps_2 = [it[0] for it in other_ps]
        for p1 in ps_1:
            for p2 in ps_2:
                if np.hypot(p1[0]-p2[0], p1[1]-p2[1]) < 1:
                    return True
        return False
    if  np.hypot(pos_1[0]-pos_2[0], pos_1[1]-pos_2[1]) < 6:
        attitude1 = [pos_1[0], pos_1[1], pos_1[3]]
        attitude2 = [pos_2[0], pos_2[1], pos_2[3]]
        if point2point(get_shape_pos(ego, attitude1), get_shape_pos(other, attitude2)):
            return True
    return False    

def is_cross_tra(ego, tra_1, other, tra_2):
    for i in range(len(tra_1)):
        p0 = tra_1[i]
        p1 = tra_2[i]
        if is_collide_point(ego, p0, other, p1):
            return True
    return False

def tra_check(ego, cur_tra, sur_traffic):     
    for sur_car in sur_traffic:
        if is_cross_tra(ego, cur_tra, sur_car, sur_traffic[sur_car]):
            return 1
    return 0

class Perceptron():
    def __init__(self, ego):     
        self.ego = ego
        self.PER_RADIUS = 100
        self.sur_traffic = {}                  # traffic info
    def get_sur_traffic(self, all_vehicles):   # get surrounding vehicles
        self.sur_traffic = {}
        for vehicle_id in all_vehicles:
            if vehicle_id != self.ego.id:
                vehicle = all_vehicles[vehicle_id]
                if vehicle.road_id == self.ego.road_id :  # two vehicle in the same road
                    if self.ego.lane == vehicle.lane and d_tranform(vehicle.direction) * vehicle.scf > d_tranform(self.ego.direction) * self.ego.scf and np.hypot(vehicle.scf-self.ego.scf, vehicle.tcf-self.ego.tcf) < self.PER_RADIUS:  # distance is close
                        self.sur_traffic[vehicle] = []
        return self.sur_traffic

class Predictor():   
    def __init__(self, ego):     
        self.ego = ego
        self.PER_RADIUS = 50
    def predict(self, sur_traffic, period):
        def predict_tra(car, period, rt):
            num = period + rt
            tra = []
            px, py, vel, hdg = car.scf, car.tcf, car.vel, car.hdg
            px_list = px + vel * np.cos(hdg) / 10 * np.arange(0, num)
            py_list = py + vel * np.sin(hdg) / 10 * np.arange(0, num)
            for t in range(num):
                tra.append([px_list[t],py_list[t],vel,hdg])
            return tra
        for vehicle in sur_traffic:
            sur_traffic[vehicle] = predict_tra(vehicle, period, 0)
        return sur_traffic

class Planner():
    def __init__(self, ego):     
        self.ego = ego
    def planning(self, motion_status, sur_traffic, period):
        # pdb.set_trace()
        obj_points, last_choice_p = get_obj_point(self.ego, motion_status[0], motion_status[1], period/10, motion_status[4]) 
        for obj_point in obj_points:
            cur_tra = poly_fitting(motion_status, obj_point, period/10)  # polynomial function
            if obj_point == last_choice_p:
                best_tra = cur_tra
            if not tra_check(self.ego, cur_tra, sur_traffic) and len(cur_tra) > 0:            # check trajectories
                best_tra = cur_tra
                break     

        delta_tra = [[0 for j in range(len(best_tra[i]))] for i in range(len(best_tra))]
        for i in range(len(best_tra)):
            for j in range(len(motion_status)-1):
                if i == 0:
                    delta_tra[i][j] = best_tra[i][j] - motion_status[j]   
                else:
                    delta_tra[i][j] = best_tra[i][j] - best_tra[i-1][j]  
        return delta_tra