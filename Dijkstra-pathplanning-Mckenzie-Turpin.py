import matplotlib.pyplot as plt
import math
import numpy as np

show_animation = True


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def dijkstra_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]
    one = 1
    figure, axes = plt.subplots()

    obmap, minx, miny, maxx, maxy, xw, y, ox, oy = calc_obstacle_map(one)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]
        #  print("current", current)

        # show graph
        if show_animation:
            plt.plot(current.x * reso, current.y * reso, "xc")
            #plt.plot(ox, oy, "-b")
            plt.imshow(obmap)
            plt.xlim(0, 400)
            plt.ylim(0, 250)
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(motion)):
            node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry


def calc_final_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def verify_node(node, obmap, minx, miny, maxx, maxy):
    if obmap[node.x][node.y]:
        return False

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x > maxx:
        return False
    elif node.y > maxy:
        return False

    return True


def calc_obstacle_map(one):
    minx = 0
    miny = 0
    maxx = 400
    maxy = 250
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    # print("xwidth:", xwidth)
    # print("ywidth:", ywidth)

    obmap = np.zeros(100000).reshape((250, 400))
    obmap[:, 0] = one
    obmap[:, 399] = one
    obmap[:1] = one
    obmap[249:] = one

    j = 184
    m = 184
    p = 184
    s = 184
    v = 120
    y = 140
    b = 80
    e = 59
    dia = 184
    dia2 = 184

    for i in range(35, 114):
        while j <= 209:
            j = round(0.316 * i + 173.62)
            k = 249 - j
            i = i + 1
            obmap[k, i] = one

    for l in range(35, 104):
        while m >= 99:
            m = round(-1.2319 * l + 229.348)
            n = 249 - m
            l = l + 1
            obmap[n, l] = one

    for o in range(79, 114):
        while p <= 209:
            p = round(0.8571 * o + 111.4286)
            q = 249 - p
            o = o + 1
            obmap[q, o] = one

    for r in range(79, 104):
        while s >= 99:
            s = round(-3.2 * r + 436)
            t = 249 - s
            r = r + 1
            obmap[t, r] = one

    for u in range(164, 199):
        while v <= 140:
            v = round(0.5771 * u + 24.971)
            w = 249 - v
            u = u + 1
            obmap[w, u] = one

    for x in range(199, 234):
        while y >= 120:
            y = round(-0.5771 * x + 255.82)
            z = 249 - y
            x = x + 1
            obmap[z, x] = one

    for a in range(164, 199):
        while b >= 59:
            b = round(-0.5771 * a + 175.02)
            c = 249 - b
            a = a + 1
            obmap[c, a] = one

    for d in range(199, 234):
        while e <= 79:
            e = round(0.5771 * d - 55.82)
            f = 249 - e
            d = d + 1
            obmap[f, d] = one

    for g in range(130, 167):
        g = g + 1
        obmap[g, 164] = one

    for h in range(130, 167):
        h = h + 1
        obmap[h, 236] = one

    for cir in range(259, 339):
        while dia > 184:
            dia = math.sqrt(1600 - (cir - 300) ** 2) + 185
            pnt = 249 - dia
            cir = cir + 1
            obmap[pnt, cir] = one

    for cir2 in range(259, 339):
        while dia2 < 184:
            dia2 = round(math.sqrt(1600 - (cir2 - 300) ** 2) + 185)
            pnt2 = 249 - dia2
            cir2 = cir2 + 1
            obmap[pnt2, cir2] = one

    oy, ox = np.where(obmap == 1)

    #print(oy)

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth, ox, oy


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 300.0  # [m]
    gy = 100.0  # [m]
    grid_size = 10.0  # [m]
    robot_size = 1.0  # [m]
    one = 1

    figure, axes = plt.subplots()

    obmap, minx, miny, maxx, maxy, xwidth, ywidth, ox, oy = calc_obstacle_map(one)

    if show_animation:
        plt.plot(gx, gy, "xb")
        #plt.plot(ox, oy, "-b")
        plt.imshow(obmap)
        plt.grid(True)
        plt.xlim(0, 400)
        plt.ylim(0, 250)
        plt.axis("equal")

    rx, ry = dijkstra_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)

    if show_animation:
        plt.plot(rx, ry, "-r")
        #plt.plot(ox, oy, "-b")
        plt.xlim(0, 400)
        plt.ylim(0, 250)
        #axes.add_artist(obmap)
        plt.show()


if __name__ == '__main__':
    main()
