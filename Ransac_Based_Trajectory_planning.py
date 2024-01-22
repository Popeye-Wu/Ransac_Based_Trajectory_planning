# -*- coding: utf-8 -*-
"""
Created on Thu Jan 11 20:45:17 2024

@author: Cooper
"""
import numpy as np
import matplotlib.pyplot as plt
import random
import math

'''
MultipleFireSpotsLocation = np.array( [[471, 420, 23],
                               [168, 420, 67],
                               [471, 420, 23],
                               [471, 420, 23],
                               [471, 420, 23]])
FireNumber = len(MultipleFireSpots)
HomeLocation = np.zeros(shape=(1,3))
'''

class MultipleFirePathPlanning():
    "how to get the optimal trajectory to extinguish the forest fire"
    
def __init__(self):
    print("我是大雁")
        

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6378137.0  # 地球半径，单位米
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) * math.sin(delta_phi / 2) + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) * math.sin(delta_lambda / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c
    return distance

def distance_components(current_point, target_point):
    distance = haversine_distance(current_point[0], current_point[1], target_point[0], target_point[1])
    x_distance = (target_point[0] - current_point[0]) * math.cos(math.radians(target_point[0])) * 111320  # 1度纬度约等于111320米
    y_distance = (target_point[1] - current_point[1]) * math.cos(math.radians(target_point[0])) * 111320  # 1度经度在赤道处约等于111320米
    z_distance = target_point[2] - current_point[2]
    
    return x_distance, y_distance, z_distance

def BatchConversion(home_point,fire_spots):
    MultipleNumber = len(fire_spots)
    XYZ_Distance = np.zeros(shape=(MultipleNumber,3))
    
    for i in range(MultipleNumber):
       XYZ_Distance[i] = distance_components (home_point,fire_spots[i])
    # raw fire spots date with relative distance X, Y
    return XYZ_Distance

def InterPoint(k1, b1, k2,b2):
    '''
    Parameters
    ----------
    line1 : TYPE
        DESCRIPTION.
    line2 : TYPE
        DESCRIPTION.

    Returns
    -------
    None.
    '''
    k2 = -1 / k1
    inter_x = (b2 - b1)  / (k1 - k2)
    inter_y = inter_x * k1 + b1
    return inter_x, inter_y

def RANSAC2(Mutiple_Fire_spots, L, V, ValveDelay, MotionSpeed):
    # convert to xy array
    MultipleFireSpotsLen = len(Mutiple_Fire_spots)
    RANDOM_X = np.array(Mutiple_Fire_spots[:,0]) 
    RANDOM_Y = np.array(Mutiple_Fire_spots[:,0])

    '''
    # 数据量。
    SIZE = 50
    SIZE_N = 10 # the numbe of noise
    # 产生数据。np.linspace 返回一个一维数组，SIZE指定数组长度。
    # 数组最小值是0，最大值是10。所有元素间隔相等。
    X = np.linspace(0, 10, SIZE)
    Y = -2 * X + 5

    fig = plt.figure()
    # 画图区域分成1行1列。选择第一块区域。
    ax1 = fig.add_subplot(111)
    # 标题
    ax1.set_title("Top-View of Fireland ")




    # 让散点图的数据更加随机并且添加一些噪声。
    random_x = []
    random_y = []

    random_x2 = []
    random_y2 = []

    random_x2b = []
    random_y2b = []

    random_x22 = []
    random_y22 = []

    random_x22b = []
    random_y22b = []
    # 添加直线随机噪声
    for i in range(SIZE):
        random_x.append(X[i] + random.uniform(-2, 2)) 
        random_y.append(Y[i] + random.uniform(-2, 2)) 
    # 添加随机噪声
    for i in range(SIZE_N):
        random_x.append(random.uniform(-SIZE,SIZE))
        random_y.append(random.uniform(-SIZE,SIZE))
    RANDOM_X = np.array(random_x) # 散点图的横轴。
    RANDOM_Y = np.array(random_y) # 散点图的纵轴。
'''
    random_x = []
    random_y = []
  
    random_x2 = []
    random_y2 = []
  
    random_x2b = []
    random_y2b = []
  
    random_x22 = []
    random_y22 = []
  
    random_x22b = []
    random_y22b = []
    # 使用RANSAC算法估算模型
    # 迭代最大次数，每次得到更好的估计会优化iters的数值
    iters = 2000
    iters2 = int(iters/2)
    # 数据和模型之间可接受的差值
    sigma = 1.5 # related with altitude of flying
    f_t = 5
    sigma2 = MotionSpeed * f_t # the time span of water flow from the tank
    #sigma2 = 8
    # 最好模型的参数估计和内点数目
    best_a = 0
    best_b = 0
    best_a2 = 0
    best_b2 = 0
    pretotal = 0
    pretotal2 = 0
    # 希望的得到正确模型的概率
    P = 0.99

    for i in range(iters):
        # update the record position for seconde RANSAC 
        random_x2 = []
        random_y2 = []
        # 随机在数据中红选出两个点去求解模型
        sample_index = random.sample(range(MultipleFireSpotsLen),2)
        x_1 = RANDOM_X[sample_index[0]]
        x_2 = RANDOM_X[sample_index[1]]
        y_1 = RANDOM_Y[sample_index[0]]
        y_2 = RANDOM_Y[sample_index[1]]

        # y = ax + b 求解出a，b
        a = (y_2 - y_1) / (x_2 - x_1)
        b = y_1 - a * x_1

        # 算出内点数目
        total_inlier = 0
        for index in range(MultipleFireSpotsLen): # SIZE * 2 is because add 2 times noise of SIZE
            y_estimate = a * RANDOM_X[index] + b
            if abs(y_estimate - RANDOM_Y[index]) < sigma:
                total_inlier = total_inlier + 1
                # record these points that between +-sigma
                random_x2.append(RANDOM_X[index])
                random_y2.append(RANDOM_Y[index])

        # 判断当前的模型是否比之前估算的模型好
        if total_inlier > pretotal:
            iters = math.log(1 - P) / math.log(1 - pow(total_inlier / (MultipleFireSpotsLen), 2))
            pretotal = total_inlier
            best_a = a
            best_b = b
            # update the latest better points
            random_x2b = np.array(pretotal) # 散点图的横轴。
            random_y2b = np.array(pretotal) # 散点图的纵轴。
            random_x2b = random_x2
            random_y2b = random_y2
            SIZE2 = pretotal
     
        # 判断是否当前模型已经超过八成的点
        if total_inlier > 0.97 * MultipleFireSpotsLen:
            break
    '''
    # 用我们得到的最佳估计画图
    # 横轴名称。
    ax1.set_xlabel("top view x-axis")
    # 纵轴名称。
    ax1.set_ylabel("top view y-axis")
    ax1.scatter(RANDOM_X, RANDOM_Y, c='r', marker='^')

    # show the ransac2 points:
    #ax1.scatter(random_x2b, random_y2b, c='b', marker='v')

    # 直线图

    Y = best_a * RANDOM_X + best_b

    ax1.plot(RANDOM_X, Y, linewidth=1.0, linestyle='-.', c='black',)
    ax1.plot(RANDOM_X, Y + sigma, linewidth=3.0, linestyle='-', c='b',)
    ax1.plot(RANDOM_X, Y - sigma, linewidth=3.0, linestyle='-', c='b',)
    text = "best_a = " + str(best_a) + "\nbest_b = " + str(best_b)
    plt.text(5,50, text,
             fontdict={'size': 12, 'color': 'b'})
    '''



    # the seconde ransac call the point that cover the largest area
    RANDOM_XX = np.array(random_x2b) # 散点图的横轴。
    RANDOM_YY = np.array(random_y2b) # 散点图的纵轴。

    for i in range(iters2):
        random_x22 = []
        random_y22 = []
        # 随机在数据中红选出一个点去求解模型
        sample_index2 = random.sample(range(SIZE2),1)
        x_12 = RANDOM_XX[sample_index2[0]]
        y_12 = RANDOM_YY[sample_index2[0]]


        # y = ax + b 求解出a，b
        a2 = -1 / a
        b2 = y_12 - (a2 * x_12)

        # 算出内点数目
        total_inlier2 = 0
        for index in range(SIZE2):    # SIZE * 2 is because add 2 times noise of SIZE
            y_estimate2 = a2 * RANDOM_XX[index] + b2
            if abs(y_estimate2 - RANDOM_YY[index]) < sigma2:
                total_inlier2 = total_inlier2 + 1
                # record these points that between +-sigma
                random_x22.append(RANDOM_XX[index])
                random_y22.append(RANDOM_YY[index])
                

        # 判断当前的模型是否比之前估算的模型好
        if total_inlier2 > pretotal2:
            print("total_inlier2:", total_inlier2)
            print("SIZE2:", SIZE2)
            iters = math.log(1 - P) / math.log(1 - pow(total_inlier2 / SIZE2, 2))
            pretotal2 = total_inlier2
            best_a2 = a2
            best_b2 = b2
            
            # update the latest better points
            random_x22b = np.array(pretotal2) # 散点图的横轴。
            random_y22b = np.array(pretotal2) # 散点图的纵轴。
            random_x22b = random_x22
            random_y22b = random_y22
     
        # 判断是否当前模型已经超过八成的点
        if total_inlier2 > 0.9 * SIZE2:
            break
        
    '''  
    # 用我们得到的最佳估计画图
    YY = best_a2 * RANDOM_XX + best_b2
    
    # show the ransac2 points:
    ax1.scatter(random_x22b, random_y22b, c='g', marker='o')

    ax1.set_aspect('equal', adjustable='box')
    # 直线图
    ax1.plot(RANDOM_XX, YY, linewidth=1.0, linestyle='-.', c='g' )
    ax1.plot(RANDOM_XX, YY + sigma2, linewidth=3.0, linestyle='-', c='g',)
    ax1.plot(RANDOM_XX, YY - sigma2, linewidth=3.0, linestyle='-', c='g',)
    text = "best_a2 = " + str(best_a2) + "\nbest_b2 = " + str(best_b2)
    plt.text(1,30, text,
             fontdict={'size': 12, 'color': 'g'})
    plt.show()
    '''
    # Get the interpoint
    inter_x, inter_y = InterPoint(a, b, a2, b2)
    Middle_point = np.array([0, 0])
    Middle_point[0] = inter_x
    Middle_point[1] = inter_y

    # According the sigma2, Delay and MotionSpeed to get the 
    WatCovLen = MotionSpeed * ValveDelay + sigma2 / 2 + 3
    
    # chose the best start_point (屌炸了！！)
    PreStaPoint1 = a * (inter_x + WatCovLen) + b
    PreStaPoint2 = a * (inter_x - WatCovLen) + b
    error1 = math.sqrt((WatCovLen)*(WatCovLen) + (PreStaPoint1 - inter_y) * (PreStaPoint1 - inter_y))
    error2 = math.sqrt((WatCovLen)*(WatCovLen) + (PreStaPoint2 - inter_y) * (PreStaPoint1 - inter_y))
    
    start_point = np.array([0, 0])
    end_point = np.array([0, 0])
    if (error1) > (error2):
        start_point[0] = inter_x - WatCovLen
        start_point[1] = PreStaPoint2
        end_point[0] = inter_x - WatCovLen
        end_point[1] = PreStaPoint1
        
    start_point[0] = inter_x + WatCovLen
    start_point[1] = PreStaPoint1
    end_point[0] = inter_x - WatCovLen
    end_point[1] = PreStaPoint2
    
    return start_point, Middle_point, end_point



