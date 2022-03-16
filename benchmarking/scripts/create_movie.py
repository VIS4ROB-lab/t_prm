#!/usr/bin/env python3

"""
 *********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022,
 *  ETH Zurich - V4RL, Department of Mechanical and Process Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Matthias Busenhart
 *********************************************************************
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import os
from matplotlib.collections import PatchCollection

FRAMES_PER_SECOND = 30
DT = 1./FRAMES_PER_SECOND
MAX_TIME = 22.
ROBOT_SIZE = 0.25


class Obstacle:
    def __init__(self, x, y, r, vx, vy):
        self.x = x
        self.y = y
        self.r = r
        self.vx = vx
        self.vy = vy


class BM:
    def __init__(self):
        self.name = ""
        self.start = []
        self.goal = []
        self.obstacles = []
        self.number_of_runs = 0


def parse_info():
    info_file = "info_about_bm.txt"
    # parse the file and create a vector of bms
    bms = []
    load_obstacles = False
    load_queries = False
    with open(info_file) as f:
        bm = None
        for line in f:
            if len(line) == 0:
                continue
            if line[0] == '=':
                if bm is not None:
                    bms.append(bm)
                bm = BM()
            elif line.startswith("Benchmark:"):
                bm.name = line.split(":")[1].strip()
            elif line.startswith("Num. of runs:"):
                bm.number_of_runs = int(line.split(":")[1].strip())
            elif line.startswith("OBSTACLES"):
                load_obstacles = True
                load_queries = False
            elif line.startswith("QUERIES"):
                load_obstacles = False
                load_queries = True
            elif line.startswith("Num."):
                pass
            else:
                # should be either obstacles or queries
                if load_obstacles:
                    x, y, z, r, vx, vy, vz = line.split(",")
                    bm.obstacles.append(Obstacle(float(x), float(
                        y), float(r) - ROBOT_SIZE, float(vx), float(vy)))
                elif load_queries:
                    x, y, z, x2, y2, z2 = line.split(",")
                    bm.start.append([float(x), float(y)])
                    bm.goal.append([float(x2), float(y2)])
        bms.append(bm)
    return bms[0]  # only parse first bm


def get_path_for_time(path, time):
    """
    the path is like [[x1,y1,z1, t1], [x2,y2,z2, t2], ...]
    we have to search for the time which is highger, then
    interpolate the last segment linearly and return the new path (new start + end of input path)
    """
    if len(path) == 0:
        return [], []
    for i, segment in enumerate(path):
        if len(segment) < 3:
            return [], []
        if segment[3] > time:
            break

    # the segment is i-1 in which we currently are
    if i == 0:
        print("time is smaller than first segment")
        return path[0], path

    if i == len(path):
        print("time is bigger than last segment")
        return path, path[-1]

    # interpolate
    x1, y1, z1, t1 = path[i-1]
    x2, y2, z2, t2 = path[i]
    t = (time - t1) / (t2 - t1)
    x = x1 + t * (x2 - x1)
    y = y1 + t * (y2 - y1)
    z = z1 + t * (z2 - z1)
    ret_a = []
    ret_b = []
    ret_a.extend(path[:i])
    ret_a.append([x, y, z, time])

    ret_b.append([x, y, z, time])
    ret_b.extend(path[i:])

    return ret_a, ret_b


def compute_time(x1, y1, z1, x2, y2, z2, speed=0.75):
    # compute the norm of the vector
    length = np.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
    return length / speed


def read_path(filename):
    data = open(filename).read()
    data = data.split("\n")
    data = [line.split(" ") for line in data]
    ret = []
    total_time = 0.
    floats = []
    for line in data:
        if len(line) == 0 or len(line) == 1:
            continue
        if len(line) < 6:
            print(f"Error parsing {filename} at line {line}")
        floats = [float(x) for x in line]
        time = compute_time(*floats[:6])
        ret.append([*floats[:3], total_time])

        total_time += time
    # add last three points
    ret.append([*floats[3:6], total_time])
    return ret


def find_all_intermediate_paths_for(planner):
    # get all files in the current directory
    files = os.listdir()
    # find all files with the format:
    # INTERMEDIATE_PATH_{planner}_AT_{time}.txt
    # and return the time
    time = [float(x.split("_")[-1].split(".txt")[0])
            for x in files if x.startswith("INTERMEDIATE_PATH_" + planner)]
    time.sort()
    ret = []
    for t in time:
        path = read_path(f"INTERMEDIATE_PATH_{planner}_AT_{t:.6f}.txt")
        ret.append([t, path])

    ret.append([1000, []])
    return ret


full_tprm = read_path("path_T-PRM_0_0_0.path")
full_ompl_rrt = read_path("path_OMPL RRTstar_0_0_0.path")
full_ompl_prm = read_path("path_OMPL PRM_0_0_0.path")

intermediate_tprm = find_all_intermediate_paths_for("T-PRM A-star")
intermediate_ompl_rrt = find_all_intermediate_paths_for("OMPL RRTstar")
intermediate_ompl_prm = find_all_intermediate_paths_for("OMPL PRM")

bm = parse_info()

fig, ax = plt.subplots(figsize=(8, 8))
(ln,) = plt.plot([], [], "r")

colors = {
    "tprm": "g",
    "ompl_rrt": "y",
    "ompl_prm": "b"
}


def draw_path(path, ax, color, label, alpha=1., addLabel=True):
    if len(path) == 0 or len(path[0]) < 3:
        return
    # convert to a np array
    path = np.array(path)
    # draw the path
    if addLabel:
        ax.plot(path[:, 0], path[:, 1], color, label=label, alpha=alpha)
    else:
        ax.plot(path[:, 0], path[:, 1], color, alpha=alpha)


def draw_robot(path, ax, color, zorder=1):
    if len(path) == 0 or len(path[0]) < 3:
        return
    ax.add_collection(PatchCollection([patches.Circle(
        (path[0][0], path[0][1]), ROBOT_SIZE, color=color)], match_original=True, zorder=zorder))


def draw_intermediate_paths(i_paths, time, ax, color):
    return # TODO: remove this line if you want to draw the paths yet to be driven
    if time == 0:
        draw_path(i_paths[0][1], ax, color, "", 0.5, False)
        return
    # search the entry in i_paths which is lower than time
    for i, (t, path) in enumerate(i_paths):
        if t >= time:
            break
    # draw the path
    _, path = get_path_for_time(i_paths[i-1][1], time - i_paths[i-1][0])
    # draw_path(_, ax, color, f"intermediate path {i}", 1.0, False)
    draw_path(path, ax, color, f"intermediate path {i}", 0.5, False)


def init_anim():
    return (ln,)


def update(frame):
    ax.clear()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])

    # draw obstacles
    for obstacle in bm.obstacles:
        pos_x = obstacle.x + frame * DT * obstacle.vx
        pos_y = obstacle.y + frame * DT * obstacle.vy
        ax.add_patch(plt.Circle((pos_x, pos_y), obstacle.r, color='gray'))

    # T-PRM
    tprm_path_a, tprm_path_b = get_path_for_time(full_tprm, frame * DT)
    draw_path(tprm_path_a, ax, colors["tprm"], "T-PRM")
    #draw_path(tprm_path_b, ax, colors["tprm"], "T-PRM", 0.5, False)

    # OMPL PRM
    omplprm_path_a, omplprm_path_b = get_path_for_time(
        full_ompl_prm, frame * DT)
    draw_path(omplprm_path_a, ax, colors["ompl_prm"], "OMPL PRM")
    draw_intermediate_paths(intermediate_ompl_prm,
                            frame * DT, ax, colors["ompl_prm"])

    # OMPL RRT
    omplrrt_path_a, omplrrt_path_b = get_path_for_time(
        full_ompl_rrt, frame * DT)
    draw_path(omplrrt_path_a, ax, colors["ompl_rrt"], "OMPL RRT*")
    draw_intermediate_paths(intermediate_ompl_rrt,
                            frame * DT, ax, colors["ompl_rrt"])

    draw_robot(omplrrt_path_b, ax, colors["ompl_rrt"],zorder=5)
    draw_robot(omplprm_path_b, ax, colors["ompl_prm"],zorder=6)
    draw_robot(tprm_path_b, ax, colors["tprm"],zorder=7)

    # show legend
    ax.legend(loc='upper left')

    ax.set_title(f"{frame * DT:.2f}s")

    plt.tight_layout()
    return (ln,)


ani = FuncAnimation(fig, update, frames=range(
    int(MAX_TIME / DT)), init_func=init_anim, blit=False)
    
plt.show()
#ani.save("movie.mp4", writer="ffmpeg", fps=FRAMES_PER_SECOND)
