import random, math

def generateCarFlow(run_time, qr, vmin = 10, vmax = 20, SampleTime = 0.1):
    minTimeHeadWay = 3/SampleTime
    lampda = qr/3600
    hwp = 0
    CarInfo = []
    cnt = 0
    Length, Width = 3, 1
    while(1): 
        # generate arrival time
        temp = int(-1/lampda*math.log(random.random())/SampleTime)
        if temp < minTimeHeadWay and len(CarInfo) > 0:
            temp = minTimeHeadWay
        hw = hwp + temp
        if hw > run_time:
            break
        v = round(random.random()*(vmax-vmin),2)+vmin
        CarInfo.append([cnt, hw, v, Length, Width])
        hwp = hw 
        cnt += 1
    return CarInfo

