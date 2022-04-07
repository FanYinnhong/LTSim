import random, math
V_MIN = 15
V_MAX = 30 
def generateCarFlow(run_time, ST, lane_num):
    random.seed(0)
    CarInfo = []
    for from_id in range(10):
        for to_id in range(9,10):
            qr = random.randint(500,500) 
            minTimeHeadWay = 1/ST
            lampda = qr/3600
            hwp = 0
            Length, Width = 4, 2
            while(1):
                # generate arrival time
                temp = int(-1/lampda*math.log(random.random())/ST)
                if temp < minTimeHeadWay and len(CarInfo) > 0:
                    temp = minTimeHeadWay
                hw = hwp + temp
                if hw > run_time:
                    break
                v = round(random.random()*(V_MAX-V_MIN),2)+V_MIN
                CarInfo.append([from_id, to_id, int(hw), v, Length, Width])
                hwp = hw 
    return CarInfo

