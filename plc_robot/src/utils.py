import numpy as np
import lc_planner_py
import math
from scipy.spatial import ConvexHull
import time
import matplotlib.pyplot as plt

Z_OFFSET = 0.30
heuristic_planner = lc_planner_py.PlannerHeuristicGreedy(0.4)


def ranges_to_design_points(ranges, thetas):
    x = ranges * np.sin(np.deg2rad(thetas))
    z = ranges * np.cos(np.deg2rad(thetas))
    return np.hstack([x.reshape(-1, 1), z.reshape(-1, 1)])

def ranges_to_design_filter(ranges, thetas):
    design = []
    for idx, range in enumerate(ranges):
        if range <= 8:
           x = range * np.sin(np.deg2rad(thetas[idx]))
           y = range * np.cos(np.deg2rad(thetas[idx]))
           design.append([x, y])
    return np.array(design)

def transform_joints(joint_positions, lc_orient, lc_trans):
    transformed_joints = []
    for position in joint_positions:
        position = lc_orient.T @ (position - lc_trans)
        transformed_joints.append(position[2])
    return transformed_joints

def get_plane_pts(min_z, thetas):
    x = np.linspace(-8, 8, len(thetas), dtype=np.float32)
    z = min_z * np.ones_like(x) - Z_OFFSET
    #design_pts = np.hstack([x.reshape(-1, 1), z.reshape(-1, 1)])
    design_pts = ranges_to_design_points(z, thetas)
    return design_pts

def get_ranges():
    unit_spacing = np.linspace(0, 1, 1000, dtype=np.float32)  # (R,)
    ranges = 3 + (7 - 3) * unit_spacing  # (R,)
    return ranges

def get_closest_pts(joint_positions, lc_archi_orient, lc_archi_trans,
        lc_devel_orient, lc_devel_trans, lc_thetas):
    design = []
    positions_list = []
    hulls = []
    positions = []

    for robot in joint_positions:
        positions = []
        for position in joint_positions[robot]:
            if robot == "p1":
                position = lc_archi_orient.T @ (position - lc_archi_trans)
                positions.append((position[0], position[2]))
            elif robot == "p2":
                position = lc_devel_orient.T @ (position - lc_devel_trans)
                positions.append((position[0], position[2]))

        positions = np.array(positions)
        hulls.append(ConvexHull(positions))
        positions_list.append(positions)

    for ray in lc_thetas:
        closest = 100000
        closestPoint = None
        for idx, hull in enumerate(hulls):
            positions = positions_list[idx]
            for i in range(len(hull.vertices)):
                intersectPoint = ray_trace(
                    (positions[hull.vertices[i]],
                    positions[hull.vertices[(i+1)%len(hull.vertices)]]),
                    ray)
                if intersectPoint is not None:
                    # Get distance between ray source and intersect point
                    ray_dx = intersectPoint[0]
                    ray_dy = intersectPoint[1]
                    # If the intersect point is closer than the previous closest intersect point, it becomes the closest intersect point
                    distance = math.sqrt(ray_dx**2 + ray_dy**2)
                    if (distance < closest):
                        closest = distance
                        closestPoint = np.array([intersectPoint[0], intersectPoint[1]])

                    #x = 10 * np.sin(np.deg2rad(ray))
                    #z = 10 * np.cos(np.deg2rad(ray))
                    #closestPoint = np.array([x.item(), z.item()])

        if closestPoint is not None:
            design.append(closestPoint)
        else:
            design.append(ray*5.5)

    design = np.array(design)

    # plot hull vertices
    # for idx, hull in enumerate(hulls):
    #     positions = positions_list[idx]
    #     plt.scatter(positions[:,0], positions[:,1])
    #     plt.scatter(design[:,0], design[:,1])
    #     for i in range(len(hull.vertices)):
    #         plt.plot((positions[hull.vertices[i]][0], positions[hull.vertices[(i+1)%len(hull.vertices)]][0]),
    #                  (positions[hull.vertices[i]][1], positions[hull.vertices[(i+1)%len(hull.vertices)]][1]))
    # plt.show()

    design -= np.array([0, Z_OFFSET])
    return design

def ray_trace(face, ray_point):
    x1 = face[0][0]
    y1 = face[0][1]
    x2 = face[1][0]
    y2 = face[1][1]
    x3 = 0
    y3 = 0
    x4 = ray_point[0]
    y4 = ray_point[1]

    denominator = (x1 - x2) * (y4 - y3) - (y1 - y2) * (x4 - x3)
    numerator = (x1 - x3) * (y4 - y3) - (y1 - y3) * (x4 - x3)
    if denominator == 0:
        return None
    
    t = numerator / denominator
    u = ((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator

    if 1 > t > 0 and u > 0:
        x = u * x4
        y = u * y4
        collidePos = [x, y]
        return collidePos
