"""
粒子群算法求解TSP问题
随机在（0,101）二维平面生成20个点
距离最小化
"""
import math
import random
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.pylab import mpl

mpl.rcParams['font.sans-serif'] = ['SimHei']  # 添加这条可以让图形显示中文


def calDistance(CityCoordinates):
    '''
    计算城市间距离
    输入：CityCoordinates-城市坐标；
    输出：城市间距离矩阵-dis_matrix
    '''
    dis_matrix = pd.DataFrame(data=None, columns=range(len(CityCoordinates)), index=range(len(CityCoordinates)))
    for i in range(len(CityCoordinates)):
        xi, yi = CityCoordinates[i][0], CityCoordinates[i][1]
        for j in range(len(CityCoordinates)):
            xj, yj = CityCoordinates[j][0], CityCoordinates[j][1]
            if (xi == xj) & (yi == yj):
                dis_matrix.iloc[i, j] = round(math.pow(10, 10))
            else:
                dis_matrix.iloc[i, j] = round(math.sqrt((xi - xj) ** 2 + (yi - yj) ** 2), 2)
    return dis_matrix


def calFitness(line, dis_matrix):
    '''
    计算路径距离，即评价函数
    输入：line-路径，dis_matrix-城市间距离矩阵；
    输出：路径距离-dis_sum
    '''
    dis_sum = 0
    dis = 0
    for i in range(len(line) - 1):
        dis = dis_matrix.loc[line[i], line[i + 1]]  # 计算距离
        dis_sum = dis_sum + dis
    dis = dis_matrix.loc[line[-1], line[0]]
    dis_sum = dis_sum + dis
    return round(dis_sum, 1)


def draw_path(line, CityCoordinates):
    '''
    #画路径图
    输入：line-路径，CityCoordinates-城市坐标；
    输出：路径图
    '''
    x, y = [], []
    for i in line:
        Coordinate = CityCoordinates[i]
        x.append(Coordinate[0])
        y.append(Coordinate[1])
    x.append(x[0])
    y.append(y[0])
    plt.plot(x, y, 'r-', color='#4169E1', alpha=0.8, linewidth=0.8)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()


def crossover(bird, pLine, gLine, w, c1, c2):
    '''
    采用顺序交叉方式；交叉的parent1为粒子本身，分别以w/(w+c1+c2),c1/(w+c1+c2),c2/(w+c1+c2)
    的概率接受粒子本身逆序、当前最优解、全局最优解作为parent2,只选择其中一个作为parent2；
    输入：bird-粒子,pLine-当前最优解,gLine-全局最优解,w-惯性因子,c1-自我认知因子,c2-社会认知因子；
    输出：交叉后的粒子-croBird；
    '''
    croBird = [None] * len(bird)  # 初始化
    parent1 = bird  # 选择parent1
    # 选择parent2（轮盘赌操作）
    randNum = random.uniform(0, sum([w, c1, c2]))
    if randNum <= w:
        parent2 = [bird[i] for i in range(len(bird) - 1, -1, -1)]  # bird的逆序
    elif randNum <= w + c1:
        parent2 = pLine
    else:
        parent2 = gLine

    # parent1-> croBird
    start_pos = random.randint(0, len(parent1) - 1)
    end_pos = random.randint(0, len(parent1) - 1)
    if start_pos > end_pos:
        start_pos, end_pos = end_pos, start_pos
    croBird[start_pos:end_pos + 1] = parent1[start_pos:end_pos + 1].copy()

    # parent2 -> croBird
    # 至于其中自身惯性的逆序操作，据说是因为如果是正序，那就相当于没交叉，相当于没探索。而逆序事实上他的顺序依然没变，只是其中一段调换了。也是有一定道理
    list1 = list(range(0, start_pos))
    list2 = list(range(end_pos + 1, len(parent2)))
    list_index = list1 + list2  # croBird从后往前填充
    for i in list_index:
        for j in range(0, len(parent2)):
            if parent2[j] not in croBird:
                croBird[i] = parent2[j]
                break

    return croBird


if __name__ == '__main__':
    # 参数
    CityNum = 20  # 城市数量
    MinCoordinate = 0  # 二维坐标最小值
    MaxCoordinate = 101  # 二维坐标最大值
    iterMax = 200  # 迭代次数
    iterI = 1  # 当前迭代次数

    # PSO参数
    birdNum = 50  # 粒子数量
    w = 0.2  # 惯性因子
    c1 = 0.4  # 自我认知因子
    c2 = 0.4  # 社会认知因子
    pBest, pLine = 0, []  # 当前最优值、当前最优解，（自我认知部分）
    gBest, gLine = 0, []  # 全局最优值、全局最优解，（社会认知部分）

    # 随机生成城市数据,城市序号为0,1,2,3...
    # CityCoordinates = [(random.randint(MinCoordinate,MaxCoordinate),random.randint(MinCoordinate,MaxCoordinate)) for i in range(CityNum)]
    CityCoordinates = [(88, 16), (42, 76), (5, 76), (69, 13), (73, 56), (100, 100), (22, 92), (48, 74), (73, 46),
                       (39, 1), (51, 75), (92, 2), (101, 44), (55, 26), (71, 27), (42, 81), (51, 91), (89, 54),
                       (33, 18), (40, 78)]
    dis_matrix = calDistance(CityCoordinates)  # 计算城市间距离,生成矩阵

    birdPop = [random.sample(range(len(CityCoordinates)), len(CityCoordinates)) for i in range(birdNum)]  # 初始化种群，随机生成
    fits = [calFitness(birdPop[i], dis_matrix) for i in range(birdNum)]  # 计算种群适应度，适应度先简单的设为距离
    gBest = pBest = min(fits)  # 全局最优值、当前最优值
    gLine = pLine = birdPop[fits.index(min(fits))]  # 全局最优解、当前最优解

    while iterI <= iterMax:  # 迭代开始
        for i in range(len(birdPop)):
            birdPop[i] = crossover(birdPop[i], pLine, gLine, w, c1, c2)
            fits[i] = calFitness(birdPop[i], dis_matrix)

        pBest, pLine = min(fits), birdPop[fits.index(min(fits))]
        if min(fits) <= gBest:
            gBest, gLine = min(fits), birdPop[fits.index(min(fits))]

        print(iterI, gBest)  # 打印当前代数和最佳适应度值
        iterI += 1  # 迭代计数加一

    print(gLine)  # 路径顺序
    draw_path(gLine, CityCoordinates)  # 画路径图