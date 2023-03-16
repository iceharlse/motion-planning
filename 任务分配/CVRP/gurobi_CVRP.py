import numpy as np
import matplotlib.pyplot as plt
from gurobipy import *

if __name__ == '__main__':
    ################参数################
    # 车辆参数
    CAPACITY = 120  # 车辆最大容量
    DISTABCE = 250  # 车辆最大行驶距离
    C0 = 30  # 车辆启动成本
    C1 = 1  # 车辆单位距离行驶成本

    # DistributionCenter = #配送中心
    n = 31  # 客户数量
    Customer = [(3, 3), (96, 24), (40, 5), (49, 8), (13, 7), (29, 89), (48, 30), (84, 39), (14, 47), (2, 24), (3, 82),
                (65, 10), (98, 52), (84, 25), (41, 69), (1, 65),
                (51, 71), (75, 83), (29, 32), (83, 3), (50, 93), (80, 94), (5, 42), (62, 70), (31, 62), (19, 97),
                (91, 75), (27, 49), (23, 15), (20, 70), (85, 60), (98, 85)]  # 有32个，包括了起始点
    xc = [x for x, y in Customer]  # 横坐标
    yc = [y for x, y in Customer]  # 纵坐标
    Demand = [16, 11, 6, 10, 7, 12, 16, 6, 16, 8, 14, 7, 16, 3, 22, 18, 19, 1, 14, 8, 12, 4, 8, 24, 24, 2, 10, 15, 2,
              14, 9]  # 客户的需求 31个

    N = list(range(1, n + 1))  # 客户
    V = list(range(0, n + 1))  # 配送点，包括了起始点
    A = [(i, j) for i in V for j in V if i != j]  # 配送点之间的边
    Cost = {(i, j): np.hypot(xc[i] - xc[j], yc[i] - yc[j]) * C1 for i, j in A}  # 配送点之间的距离
    q = {i: Demand[i - 1] for i in N}

    #############变量##############
    mdl = Model('CVRP')  # 初始化求解器
    x = mdl.addVars(A, vtype=GRB.BINARY)
    u = mdl.addVars(N, vtype=GRB.CONTINUOUS)
    K = mdl.addVar(vtype=GRB.INTEGER)

    # 目标函数
    mdl.modelSense = GRB.MAXIMIZE
    mdl.setObjective(C0 * K + (quicksum(x[i, j] * Cost[i, j] for i, j in A)))

    # 约束
    mdl.addConstrs(quicksum(x[i, j] for j in V if i != j) == 1 for i in N)
    mdl.addConstrs(quicksum(x[i, j] for i in V if i != j) == 1 for j in N)
    mdl.addConstrs((x[i, j] == 1) >> (u[i] + q[j] == u[j]) for i, j in A if i != 0 and j != 0)
    mdl.addConstrs(u[i] >= q[i] for i in N)
    mdl.addConstrs(u[i] <= CAPACITY for i in N)

    mdl.optimize()

    active_arts = [a for a in A if x[a].x > 0.9]
    print(active_arts)

    ##############画图############
    for index, [i, j] in enumerate(active_arts):
        plt.plot([xc[i], xc[j]], [yc[i], yc[j]], c='r')
    plt.plot(xc[0], yc[0], c='g', marker='s')
    plt.scatter(xc, yc, c='b')
    plt.show()
