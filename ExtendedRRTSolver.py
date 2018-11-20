import matplotlib.pyplot as plt
plt.ion()

from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import numpy.linalg as linalg
import numpy.random as nr
import scipy.spatial as spatial
from functools import reduce
import argparse
from IPython import embed

debug = False

class Obj():

    def __init__(self, vertices):
        self.vertices = np.array(vertices)
        self.N = len(vertices)

    def nearestpoints(self, pos):
        """
        Return the nearest point on each edge toward target pos.
        """
        points = []
        for i in range(self.N):
            p1 = self.vertices[i]
            p2 = self.vertices[int((i + 1) % self.N)]

            assert linalg.norm(p2 - p1) > 0.0, "\nvertices = {}\np1 = {}, p2 = {}".format(self.vertices, p1, p2)
            alpha = np.clip((np.dot(pos - p1, p2 - p1) /
                             sum((p2 - p1) ** 2)), 0.0, 1.0)
            points.append(p1 + alpha * (p2 - p1))
        return points

    def intersect(self, p1, p2):
        for i in range(self.N):
            q1 = self.vertices[i]
            q2 = self.vertices[(i + 1) % self.N]
            if Obj.segmentintersect(p1, p2, q1, q2):
                return True
        return False

    @staticmethod
    def segmentintersect(p1, p2, q1, q2):
        def crossprod2d(v1, v2):
            return v1[0] * v2[1] - v1[1] * v2[0]
        p, q, r, s = p1, q1, p2 - p1, q2 - q1

        if crossprod2d(r, s) == 0:
            if crossprod2d((q - p), r) == 0:
                return False
            else:
                return False

        u1 = crossprod2d((q - p), s) / crossprod2d(r, s)
        u2 = crossprod2d((p - q), r) / crossprod2d(s, r)
        if 0. < u1 < 1. and 0. < u2 < 1.:
            return True
        else:
            return False

    @staticmethod
    def triangleoverlap(p1, p2, p3, q1, q2, q3):
        if min(p1[0], p2[0], p3[0]) > max(q1[0], q2[0], q3[0]):
            return False
        if min(p1[1], p2[1], p3[1]) > max(q1[1], q2[1], q3[1]):
            return False
        if max(p1[0], p2[0], p3[0]) < min(q1[0], q2[0], q3[0]):
            return False
        if max(p1[1], p2[1], p3[1]) < min(q1[1], q2[1], q3[1]):
            return False
        pv = [p1, p2, p3]
        qv = [q1, q2, q3]
        ppair = [[p1, p2], [p2, p3], [p3, p1]]
        qpair = [[q1, q2], [q2, q3], [q3, q1]]
        for i in range(3):
            for j in range(3):
                if Obj.segmentintersect(ppair[i][0], ppair[i][1], qpair[j][0], qpair[j][1]):
                    return True
        for p in pv:
            if Obj.pintriangle(p, q1, q2, q3):
                return True
        for q in qv:
            if Obj.pintriangle(q, p1, p2, p3):
                return True
        return False

    @staticmethod
    def pintriangle(p, p0, p1, p2):
        u = p1 - p0
        v = p2 - p0
        w = p - p0

        vcw = np.cross(v, w)
        vcu = np.cross(v, u)
        if np.dot(vcw, vcu) < 0.: return False
        ucw = np.cross(u, w)
        ucv = np.cross(u, v)
        if np.dot(ucw, ucv) < 0.: return False

        area = linalg.norm(ucv)
        r, t = linalg.norm(vcw) / area, linalg.norm(ucw) / area
        return r + t <= 1.

    @staticmethod
    def triangulate(points):
        """
        Ear clipping
        """
        assert len(points) >= 3, "Len(points) < 3, seriously?"
        tri = []
        remaining_idx = [i for i in range(len(points))]
        while len(remaining_idx) >= 3:
            for i in range(len(remaining_idx)):
                idx1, idx2, idx3 = remaining_idx[(i - 1) % len(remaining_idx)], remaining_idx[i], remaining_idx[(i + 1) % len(remaining_idx)]
                p1, p2, p3 = np.array(points[idx1]), np.array(points[idx2]), np.array(points[idx3])
                print("Cross prod = {}".format(np.cross(p2 - p1, p3 - p1)))
                # if len(points) == 20:
                #     embed()
                if np.cross(p2 - p1, p3 - p1) > 0.0:   # Counterclockwise and not colinear
                    flag = False
                    for j in range(len(remaining_idx)):
                        idx = remaining_idx[j]
                        if idx in (idx1, idx2, idx3):
                            continue
                        p = np.array(points[idx])
                        # embed()
                        if Obj.pintriangle(p, p1, p2, p3):
                            print("P = {} in this (p1, p2, p3)".format(p))
                            flag = True
                            break
                    if not flag:                        # Ear found!
                        tri.append([idx1, idx2, idx3])
                        remaining_idx.remove(idx2)
                        break
                if i == len(remaining_idx) - 1:
                    print("Remaining points = {}".format(points[np.array(remaining_idx)]))
                    embed()
        return np.array(tri)

    def visualize(self, ax):
        vertices = self.vertices.tolist() + [[0, 0]]
        vertices = np.array(vertices)
        codes = [Path.MOVETO] + [Path.LINETO] * (self.N - 1) + [Path.CLOSEPOLY]

        path = Path(vertices, codes)
        pathpatch = patches.PathPatch(path, facecolor='None', edgecolor='xkcd:violet')

        ax.add_patch(pathpatch)

        ax.dataLim.update_from_data_xy(vertices)
        # ax.autoscale_view()

        return path


class RRTNode():

    _xsize = -1
    _ysize = -1
    _r = -1
    _theta = -1

    def __init__(self, position, orientation, parent=-1):
        self.pos = np.array(position)
        self.ori = orientation
        self.parent = parent

    @staticmethod
    def setsize(xsize, ysize):
        RRTNode._xsize = xsize
        RRTNode._ysize = ysize
        RRTNode._r = np.sqrt(xsize ** 2 + ysize ** 2)
        RRTNode._theta = np.rad2deg(np.arctan2(xsize, ysize))

    def getvertices(self, dori=0.0):
        xsize, ysize = RRTNode._xsize, RRTNode._ysize
        oldpos = np.array([
            [-xsize, -ysize],
            [-xsize, ysize],
            [xsize, ysize],
            [xsize, -ysize]
        ])
        trans = np.array([
            [np.cos(np.deg2rad(self.ori + dori)), -np.sin(np.deg2rad(self.ori + dori))],
            [np.sin(np.deg2rad(self.ori + dori)), np.cos(np.deg2rad(self.ori + dori))]
        ])
        return np.matmul(trans, oldpos.T).T + np.vstack([self.pos for _ in range(4)])

    def cangoto(self, node2, obs):
        # 1. Rotate to target orientation. Note that ori and ori + 180 are both acceptable targets
        rotationflag = -1
        try:
            nearestpoints = reduce(
                lambda x, y: x + y, [x.nearestpoints(self.pos) for x in obs])
        except AssertionError as e:
            print(e)
            embed()
        nearestpoints = list(filter(lambda x: linalg.norm(
            x - self.pos) <= RRTNode._r, nearestpoints))
        nporis = [self.vec2angle(x - self.pos) for x in nearestpoints]

        oldoris = [self.ori - RRTNode._theta, self.ori + RRTNode._theta, self.ori + 180 - RRTNode._theta, self.ori + 180 + RRTNode._theta]

        doris = []
        target = node2.ori
        now = self.ori
        if target < now:
            target += 180.
        dori1 = target - now
        target -= 180.
        dori2 = now - target

        oldposs = self.getvertices(dori=dori1)
        newposs = node2.getvertices()

        collide11 = any([any([(x - y) % 360 < dori1 for y in oldoris]) for x in nporis])
        collide21 = any([any([(y - x) % 360 < dori2 for y in oldoris]) for x in nporis])
        if debug:
            embed()
        if collide11 and collide21:
            print("\tCollide 1")
            return False

        """
        collide2 = any([Obj.pintriangle(x, self.pos, oldposs[0], oldposs[1]) for x in nearestpoints])
        if collide2:
            print("\tCollode 2")
            return False

        collide3 = any([Obj.pintriangle(x, self.pos, oldposs[2], oldposs[3]) for x in nearestpoints])
        if collide3:
            print("\tCollide 3")
            return False
        """
        if not collide11:
            doris.append(dori1)
        if not collide21:
            doris.append(dori2)

        # 2. Translation
        oldposs = self.getvertices(dori=dori1)
        # oldposs = oldposs if rotationflag == 1 else [
            # oldposs[2], oldposs[3], oldposs[0], oldposs[1]]    # Counterclockwise. Swap vertice idx
        points = np.vstack([oldposs, newposs])
        hull = spatial.ConvexHull(points)
        points = points[hull.vertices]
        tris = Obj.triangulate(points)
        for ob in obs:
            for obtri in ob.tri:
                for tri in tris:
                    p1, p2, p3, q1, q2, q3 = ob.vertices[obtri[0]], ob.vertices[obtri[1]], ob.vertices[obtri[2]], points[tri[0]], points[tri[1]], points[tri[2]]
                    if debug:
                        embed()
                    if Obj.triangleoverlap(p1, p2, p3, q1, q2, q3):
                        print("\tCollide 4")
                        return False

        print("\tNo collision detected.")
        return True

    def vec2angle(self, v):
        # [0, 360)
        angle = np.rad2deg(np.arctan2(v[0], v[1]))
        if v[0] < 0.0:
            angle += 180.
        return angle

    def visualize(self, ax, color='cyan', linkto=None):
        vertices = self.getvertices().tolist() + [[0, 0]]
        vertices = np.array(vertices)
        codes = [Path.MOVETO] + [Path.LINETO] * 3 + [Path.CLOSEPOLY]

        path = Path(vertices, codes)
        pathpatch = patches.PathPatch(path, facecolor='None', edgecolor='xkcd:{color}'.format(color=color))

        ax.add_patch(pathpatch)

        if linkto is not None:
            xlist = [self.pos[0], linkto.pos[0]]
            ylist = [self.pos[1], linkto.pos[1]]
            ax.plot(xlist, ylist, color='r', linewidth=1, alpha=0.6)

    def __sub__(self, node2):
        return self.pos - node2.pos


def loadData(obstacle_path, config_path):
    obs = []
    rrttree = []
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        lastvnum = -1
        tmp = []
        for linenum, line in enumerate(f):
            coordinates = tuple(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                if lastvnum != -1:
                    assert lastvnum == len(
                        tmp), "Input format error at Line {}".format(linenum)
                    obs.append(Obj(tmp))
                    tmp = []
                lastvnum = coordinates[0]
                quantity -= 1
            else:
                tmp.append(coordinates)
        assert quantity == 0
        obs.append(Obj(tmp))

    with open(config_path) as f:
        botsize = tuple(map(int, f.readline().strip().split(' ')))
        start = tuple(map(int, f.readline().strip().split(' ')))
        goal = tuple(map(int, f.readline().strip().split(' ')))

    RRTNode.setsize(botsize[0], botsize[1])
    goalnode = RRTNode(position=np.array(
        [goal[0], goal[1]]), orientation=goal[2])
    rrttree.append(RRTNode(position=np.array(
        [start[0], start[1]]), orientation=start[2]))

    return obs, goalnode, rrttree


def stepForward(rrttree, goalnode, obs, stepsize, eps, ax):
    def nearestneighbor(_rrttree, _pos):
        return np.argmin([linalg.norm(x.pos - _pos) for x in _rrttree])
    while True:
        if nr.rand() >= eps:
            randp = nr.random((2, )) * 600.
            rando = nr.randint(low=0, high=36) * 5
        else:
            randp = goalnode.pos
            if nr.rand() >= 0.5:
                rando = goalnode.ori
            else:
                rando = nr.randint(low=0, high=36) * 5


        if debug:
            print("Set randp and rando here")
            embed()
        nnidx = nearestneighbor(rrttree, randp)
        if linalg.norm(rrttree[nnidx].pos - randp) != 0.0:
            break

    randdir = (randp - rrttree[nnidx].pos) / \
        linalg.norm(randp - rrttree[nnidx].pos)
    newp = stepsize * randdir + rrttree[nnidx].pos
    newo = rando

    # Assume: rotation, then translation
    node1 = rrttree[nnidx]
    node2 = RRTNode(newp, newo, parent=nnidx)
    if debug:
        print("Test cangoto here")
        embed()
    if node1.cangoto(node2, obs):
        rrttree.append(node2)
        node2.visualize(ax, linkto=rrttree[nnidx])

        if linalg.norm(goalnode - node2) < stepsize and node2.cangoto(goalnode, obs):
            goalnode.parent = len(rrttree) - 1
            rrttree.append(goalnode)
            goalnode.visualize(ax, linkto=node2)
            return (True, True)
        else:
            return (False, True)
    else:
        return (False, False)


def work(args):
    global debug
    stepsize = 10                   # Global Stepsize
    eps = 0.05                       # Epsilon-greedy

    fig, ax = plt.subplots()
    plt.xlim(0, 600)
    plt.ylim(0, 600)
    ax.set_aspect('equal')

    obs, goalnode, rrttree = loadData(args.obstacle_path, args.config_path)


    rrttree[0].visualize(ax, color='bright green')
    goalnode.visualize(ax, color='fuchsia')
    ax.set_title('Rapidly-exploring Random Tree')
    ax.invert_yaxis()
    plt.draw()
    fig.canvas.draw_idle()

    for ob in obs:
        ob.visualize(ax)
        plt.pause(1e-7)

    for ob in obs:
        ob.tri = Obj.triangulate(ob.vertices)

    flag = False
    show_steps = False
    # action = plt.waitforbuttonpress()       # Key: True, Mouse: Falsee
    while not flag:
        print("Stepping forward")
        flag, node_added = stepForward(rrttree, goalnode, obs, stepsize, eps, ax)
        if node_added:
            # fig.canvas.draw_idle()
            plt.pause(1e-7)

            if show_steps:
                action = plt.waitforbuttonpress()       # Key: True, Mouse: False
                if not action:
                    debug = True
                    embed()
                    debug = False

    idx = len(rrttree) - 1
    while idx != -1:
        rrttree[idx].visualize(ax, color='bright orange')
        idx = rrttree[idx].parent
        plt.pause(1e-7)

    print("Path visualized!")
    print("Press any key to quit.")
    
    action = plt.waitforbuttonpress()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path', nargs='*', default="obst.txt", help="File path for obstacle set")
    parser.add_argument('config_path', nargs='*', default="extfinalgoal.txt", help="File path for robot size and start/goal position/orientation")
    work(parser.parse_args())
