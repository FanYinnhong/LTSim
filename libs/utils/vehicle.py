from libs.utils.frenet_planning import *

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
        self.vehicle_l, self.vehicle_w = info[5], info[6]
        self.shape = []

        # motion status
        self.scf = scf  # s-axis coordinate value of Frenet coordinate system
        self.scf_end = 1000
        self.tcf = ((self.lane - 1) * 4 + 2)  # t-axis coordinate value  (absolute value)
        self.vel = 0
        self.hdg = 0
        self.ACC_MAX = 2
        self.DCC_MAX = 10
        self.EXPECT_VEL = info[4]
        # surrounding
        self.sur_traffic = {}
        self.plan_period = 20
        self.plan_gap = 20
        self.plan_road_id = 0
        self.plan_tra = []
        self.perceptron = Perceptron(self)
        self.predictor = Predictor(self)
        self.planner = Planner(self)

    def move(self, new_status):
        # check whether vehicle is in road or junctions
        # dp_s, dp_t, d_v, d_hdg = d_status
        _s, _t, _v, _hdg = new_status
        # self.scf += dp_s
        self.scf = _s
        self.vel = _v
        self.tcf = _t
        self.hdg = _hdg
        self.road_id, self.direction, self.scf, self.tcf, self.hdg, attitude = self.map.frenet_to_cartesian(self.road_id, self.direction, self.scf, self.tcf, self.hdg)
        if attitude != -1:
            self.shape = get_shape_pos(self, attitude)

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