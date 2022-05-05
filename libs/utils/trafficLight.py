class TrafficLight(object):
    def __init__(self, pos, init_phase, time_green, time_yellow, time_red, connect_id_list):
        self.connect_id_list = connect_id_list
        self.p1 = pos[0]
        self.p2 = pos[1]

        self.init_phase = init_phase
        self.time_green = time_green
        self.time_yellow = time_yellow
        self.time_red = time_red
        self.current_state = ''

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