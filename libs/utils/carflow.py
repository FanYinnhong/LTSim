import random, math
V_MIN = 10 
V_MAX = 15
def generateCarFlow(demands_url, run_time, ST):
    random.seed(0)
    CarInfo = []    
    infilie = open(demands_url)
    demand = infilie.readline()
    while(len(demand) > 1):
        sub = demand.strip('\n').split(',')
        try:
            from_id, to_id, direction, qr = int(sub[0]), int(sub[1]), int(sub[2]), int(sub[3])
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
                CarInfo.append([from_id, to_id, direction, int(hw), v, Length, Width])
                hwp = hw 
        except:
            pass        
        demand = infilie.readline()
    return CarInfo


