"""
函数功能：读取地图，设置起点坐标和终点坐标，规划出一条全局路径
函数输入：地图，起点终点
函数输出：一条waypoint
"""

import numpy as np
import random
import matplotlib.pyplot as plt
import scipy.spatial
# import matplotlib.image as mpimg
import cv2
import math
import bisect

# # parameter
# MAX_T = 100.0  # maximum time to the goal [s]
# MIN_T = 5.0  # minimum time to the goal[s]

N_SAMPLE = 2000  # number of sample_points
N_KNN = 20  # number of edge from one sampled point
MAX_EDGE_LEN = 30.0  # [m] Maximum edge length

show_animation = False

class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        """
        Calc first derivative

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B


class Spline2D:
    """
    2D Cubic Spline class

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw

# class quinic_polynomial:
#
#     def __init__(self, xs, vxs, axs, xe, vxe, axe, T):
#
#         # calc coefficient of quinic polynomial
#         self.xs = xs
#         self.vxs = vxs
#         self.axs = axs
#         self.xe = xe
#         self.vxe = vxe
#         self.axe = axe
#
#         self.a0 = xs
#         self.a1 = vxs
#         self.a2 = axs / 2.0
#
#         A = np.array([[T**3, T**4, T**5],
#                       [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
#                       [6 * T, 12 * T ** 2, 20 * T ** 3]])
#         b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2,
#                       vxe - self.a1 - 2 * self.a2 * T,
#                       axe - 2 * self.a2])
#         x = np.linalg.solve(A, b)
#
#         self.a3 = x[0]
#         self.a4 = x[1]
#         self.a5 = x[2]
#
#     def calc_point(self, t):
#         xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
#             self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5
#
#         return xt
#
#     def calc_first_derivative(self, t):
#         xt = self.a1 + 2 * self.a2 * t + \
#             3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4
#
#         return xt
#
#     def calc_second_derivative(self, t):
#         xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3
#
#         return xt
#
#     def calc_third_derivative(self, t):
#         xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2
#
#         return xt


class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        u"""
        Search NN

        inp: input data, single frame or multi frame

        """

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist
        else:
            dist, index = self.tree.query(inp, k=k)
            return index, dist

    def search_in_distance(self, inp, r):
        u"""
        find points with in a distance r
        """

        index = self.tree.query_ball_point(inp, r)
        return index


def PRM_planning(sx, sy, gx, gy, ox, oy, rr):
    obkdtree = KDTree(np.vstack((ox, oy)).T)

    sample_x, sample_y = sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree)
    if show_animation:
        plt.plot(sample_x, sample_y, ".b")

    road_map = generate_roadmap(sample_x, sample_y, rr, obkdtree)

    rx, ry = dijkstra_planning(
        sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y)

    return rx, ry


def is_collision(sx, sy, gx, gy, rr, okdtree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.sqrt(dx ** 2 + dy ** 2)

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    nstep = round(d / D)

    for i in range(nstep):
        idxs, dist = okdtree.search(np.matrix([x, y]).T)
        if dist[0] <= rr:
            return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    idxs, dist = okdtree.search(np.matrix([gx, gy]).T)
    if dist[0] <= rr:
        return True  # collision

    return False  # OK


def generate_roadmap(sample_x, sample_y, rr, obkdtree):
    """
    Road map generation

    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    rr: Robot Radius[m]
    obkdtree: KDTree object of obstacles
    """

    road_map = []
    nsample = len(sample_x)
    skdtree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(nsample), sample_x, sample_y):

        index, dists = skdtree.search(
            np.matrix([ix, iy]).T, k=nsample)
        inds = index[0][0]
        edge_id = []
        #  print(index)

        for ii in range(1, len(inds)):
            nx = sample_x[inds[ii]]
            ny = sample_y[inds[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obkdtree):
                edge_id.append(inds[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)

    return road_map


def dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(sx, sy, 0.0, -1)
    ngoal = Node(gx, gy, 0.0, -1)

    openset, closedset = dict(), dict()
    openset[len(road_map) - 2] = nstart

    while True:
        if len(openset) == 0:
            print("Cannot find path")
            break

        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]

        # show graph
        if show_animation and len(closedset.keys()) % 2 == 0:
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            print("goal is found!")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.sqrt(dx ** 2 + dy ** 2)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    # generate final course
    rx, ry = [ngoal.x], [ngoal.y]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x)
        ry.append(n.y)
        pind = n.pind

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):
    for i in range(len(road_map)):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


def sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree):
    maxx = max(ox)
    maxy = max(oy)
    minx = min(ox)
    miny = min(oy)

    sample_x, sample_y = [], []

    while len(sample_x) <= N_SAMPLE:
        tx = (random.random() - minx) * (maxx - minx)
        ty = (random.random() - miny) * (maxy - miny)

        index, dist = obkdtree.search(np.matrix([tx, ty]).T)

        if dist[0] >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y


# def quinic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt):
#     """
#     quinic polynomial planner
#
#     input
#         sx: start x position [m]
#         sy: start y position [m]
#         syaw: start yaw angle [rad]
#         sa: start accel [m/ss]
#         gx: goal x position [m]
#         gy: goal y position [m]
#         gyaw: goal yaw angle [rad]
#         ga: goal accel [m/ss]
#         max_accel: maximum accel [m/ss]
#         max_jerk: maximum jerk [m/sss]
#         dt: time tick [s]
#
#     return
#         time: time result
#         rx: x position result list
#         ry: y position result list
#         ryaw: yaw angle result list
#         rv: velocity result list
#         ra: accel result list
#
#     """
#
#     vxs = sv * math.cos(syaw)
#     vys = sv * math.sin(syaw)
#     vxg = gv * math.cos(gyaw)
#     vyg = gv * math.sin(gyaw)
#
#     axs = sa * math.cos(syaw)
#     ays = sa * math.sin(syaw)
#     axg = ga * math.cos(gyaw)
#     ayg = ga * math.sin(gyaw)
#
#     for T in np.arange(MIN_T, MAX_T, MIN_T):
#         xqp = quinic_polynomial(sx, vxs, axs, gx, vxg, axg, T)
#         yqp = quinic_polynomial(sy, vys, ays, gy, vyg, ayg, T)
#
#         time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []
#
#         for t in np.arange(0.0, T + dt, dt):
#             time.append(t)
#             rx.append(xqp.calc_point(t))
#             ry.append(yqp.calc_point(t))
#
#             vx = xqp.calc_first_derivative(t)
#             vy = yqp.calc_first_derivative(t)
#             v = np.hypot(vx, vy)
#             yaw = math.atan2(vy, vx)
#             rv.append(v)
#             ryaw.append(yaw)
#
#             ax = xqp.calc_second_derivative(t)
#             ay = yqp.calc_second_derivative(t)
#             a = np.hypot(ax, ay)
#             if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
#                 a *= -1
#             ra.append(a)
#
#             jx = xqp.calc_third_derivative(t)
#             jy = yqp.calc_third_derivative(t)
#             j = np.hypot(jx, jy)
#             if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
#                 j *= -1
#             rj.append(j)
#
#         if max([abs(i) for i in ra]) <= max_accel and max([abs(i) for i in rj]) <= max_jerk:
#             print("find path!!")
#             break
#
#     if show_animation:
#         for i in range(len(rx)):
#             plt.cla()
#             plt.grid(True)
#             plt.axis("equal")
#             plot_arrow(sx, sy, syaw)
#             plot_arrow(gx, gy, gyaw)
#             plot_arrow(rx[i], ry[i], ryaw[i])
#             plt.title("Time[s]:" + str(time[i])[0:4] +
#                       " v[m/s]:" + str(rv[i])[0:4] +
#                       " a[m/ss]:" + str(ra[i])[0:4] +
#                       " jerk[m/sss]:" + str(rj[i])[0:4],
#                       )
#             plt.pause(0.001)
#
#     return time, rx, ry, ryaw, rv, ra, rj


# def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
#     """
#     Plot arrow
#     """
#
#     if not isinstance(x, float):
#         for (ix, iy, iyaw) in zip(x, y, yaw):
#             plot_arrow(ix, iy, iyaw)
#     else:
#         plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
#                   fc=fc, ec=ec, head_width=width, head_length=width)
#         plt.plot(x, y)


def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s

# 将 RGB 转为灰度图
def rgb2gray(rgb):
    return np.dot(rgb[..., :3], [0.299, 0.587, 0.114])


def grey2bin(grey):
    return np.where(grey > 0.5, 1.0, 0)


if __name__ == '__main__':
    im = np.array(plt.imread('2_3_label.png'))

    im_grey = rgb2gray(im)

    im_bin = grey2bin(im_grey)

    im_small = cv2.resize(im_bin, (200, 200))
    # ,interpolation=cv2.INTER_AREA)
    im_small = grey2bin(im_small)

    sx = 2.0 * 2  # [m]
    sy = 29.0 * 2  # [m]
    gx = 90.0 * 2  # [m]
    gy = 81.0 * 2  # [m]
    # grid_size = 1.0  # [m]
    robot_size = 1.0  # [m]
    obs = np.where(im_small == 0)
    ox = obs[1]  # [m]
    oy = obs[0]  # [m]
    # oy = oy[::-1]  # 将向量倒序输出
    # print(oy)

    # plt.imshow(im_small)
    # plt.plot(sx, sy, "rx")
    # plt.plot(gx, gy, "bx")
    rx, ry = PRM_planning(sx, sy, gx, gy, ox, oy, robot_size)
    rx = [x * 25 for x in rx]
    ry = [y * 25 for y in ry]
    rx_cubic_spline, ry_cubic_spline, ryaw, rk, s = calc_spline_course(rx, ry)
    plt.plot(rx_cubic_spline,ry_cubic_spline,"-m")
    # rx = np.array(rx)*25
    # ry = np.array(ry)*25
    # plt.plot(rx, ry, "-r")
    x = rx
    y = ry
    x[0] = rx[0]
    x[1] = (rx[0] + rx[1] + rx[2]) / 3
    for i in range(len(rx)):
        if i > 1 and i < len(rx) - 2:
            x[i] = (rx[i - 2] + rx[i - 1] + rx[i] + rx[i + 1] + rx[i + 2]) / 5
    x[len(rx) - 1:len(rx)] = rx[len(rx) - 1:len(rx)]
    y[0] = ry[0]
    y[1] = (ry[0] + ry[1] + ry[2]) / 3
    for i in range(len(ry)):
        if i > 1 and i < len(ry) - 2:
            y[i] = (ry[i - 2] + ry[i - 1] + ry[i] + ry[i + 1] + ry[i + 2]) / 5
    y[len(rx) - 1:len(rx)] = ry[len(rx) - 1:len(rx)]

    plt.imshow(im_bin)

    # plt.plot(x, y, "-b")
    plt.show()
    print("process end")
