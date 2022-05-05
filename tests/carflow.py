import random, math
V_MIN = 10
V_MAX = 10
def generateCarFlow(run_time, ST, lane_num):
    random.seed(0)
    CarInfo = []
    for from_id in [0,3,7,5]:
        for to_id in [-1]:
            qr = random.randint(300,300) 
            minTimeHeadWay = 3 / ST
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


