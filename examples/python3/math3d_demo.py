#!/usr/bin/env python
# coding=utf-8
"""A demo application showing some math capabilities."""

import numpy as np
import os
import sys
import matplotlib.pyplot as plt

# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vcp import math3d


if __name__ == "__main__":
    # Plane from 3 points
    plane = math3d.plane_from_three_points((10,234,0), (13,-23,-16), (234.2,37,999))
    print('Plane from 3 points:', plane)

    # Collinear - should be all 0, and a warning issued
    plane = math3d.plane_from_three_points((-7,3,0), (3,3,10), (5,3,12))
    print('Invalid plane from collinear points:', plane)

    # Compute distances:
    plane = math3d.plane_from_three_points((-1,-2,2), (-1,2,2), (1,0,1))
    print('Plane:', plane)
    pt1, pt2, pt3 = (0,15,2), np.array([1.40425069,  0.,  4.30850138]), (3,0,0)

    print('Distance plane to point ', pt1, ' is ', math3d.distance_point_plane(pt1, plane), ' should be "something"') # Something
    print('Distance plane to point ', pt2, ' is ', math3d.distance_point_plane(pt2, plane), ' should be 3.14') # Exactly 3.14 away from the plane's z-intercept
    print('Distance plane to point ', pt3, ' is ', math3d.distance_point_plane(pt3, plane), ' should be 0 (pt is x-intercept)') # On the plane (x-intercept)

    # Point on plane
    pt = -plane[3]*np.array(plane)[:3]
    print('Point ({:.1f},{:.1f},{:.1f}) is {:.1f} from plane (should be on plane)'.format(pt[0], pt[1], pt[2], math3d.distance_point_plane(pt, plane)))
    pt = -plane[3]*np.array(plane)[:3]  + np.array(plane)[:3]
    print('Point ({:.1f},{:.1f},{:.1f}) is {:.1f} from plane (should be 1.0)'.format(pt[0], pt[1], pt[2], math3d.distance_point_plane(pt, plane)))
    pt = -plane[3]*np.array(plane)[:3]  - 23.0*np.array(plane)[:3]
    print('Point ({:.1f},{:.1f},{:.1f}) is {:.1f} from plane (should be -23.0)'.format(pt[0], pt[1], pt[2], math3d.distance_point_plane(pt, plane)))
    print


    groundplane = [0.0, 0, 1, 0]
    print('Angle between line and plane: {} degrees (90?)'.format(np.rad2deg(math3d.angle_line_plane(
        ((0,0,0),(0,0,1)), groundplane))))
    print('Angle between line and plane: {} degrees (45?)'.format(np.rad2deg(math3d.angle_line_plane(
        ((0,0,0),(0,1,1)), groundplane))))
    print('Angle between line and plane: {} degrees '.format(np.rad2deg(math3d.angle_line_plane(
        ((0,0,0),(1,1,1)), groundplane))))
    print('Angle between line and plane: {} degrees (0?), parallel (on plane)!!! intersection point is: {}'.format(np.rad2deg(math3d.angle_line_plane(
        ((0,0,0),(1,1,0)), groundplane)), math3d.intersection_line_plane(((0,0,0),(1,1,0)), groundplane)))
    print('Angle between line and plane: {} degrees (0?), parallel (not on plane)!!! intersection point is: {}'.format(np.rad2deg(math3d.angle_line_plane(
        ((0,0,1),(1,1,1)), groundplane)), math3d.intersection_line_plane(((0,0,1),(1,1,1)), groundplane))) # What to do with parallel? TODO
    print('Angle between line and plane: {} degrees (close to 0, but still larger than?)'.format(np.rad2deg(math3d.angle_line_plane(
        ((0,0,1000),(0,10000,1000.1)), groundplane))))
    print

    # Plane and line/segment intersections
    lines = [((0,0,0), (0,0,1)),
        ((0,1,0), (3,-1,1)),
        ((0,1.9,0), (0,-1,1.5)),
        ((-1,-1,1.9),(-1,1,1.9))]
    expected_segment = [False, True, True, False]
    expected_line = [True, True, True, False]
    for i, line in enumerate(lines):
        # Segment intersection
        pt = math3d.intersection_line_segment_plane(line, plane)
        intercepts = pt is not None
        if intercepts:
            print('Segment {} intersects plane {} at {} (should be {})'.format(line, plane, pt, expected_segment[i]))
        else:
            print('Segment {} does not intersect plane (should be {})'.format(line, expected_segment[i]))

        if intercepts is not expected_segment[i]:
            raise ValueError("Wrong result for line segment[{}]".format(i))

        # Line intersection
        pt = math3d.intersection_line_plane(line, plane)
        intercepts = pt is not None
        if intercepts:
            print('Line {} intersects plane {} at {} (should be {})'.format(line, plane, pt, expected_line[i]))
        else:
            print('Line {} does not intersect plane (should be {})'.format(line, expected_line[i]))

        if intercepts is not expected_line[i]:
            raise ValueError("Wrong result for line[{}]".format(i))
        print('\n')

    # Point to line/line segment distance
    line = ((1,1,1), (1,1,3))
    points =        [(2,-1,0.5), (0,0,0), (1,1,1), (1,1,2.9), (1,1,3.25), (1,-1,2)]
    expected_line = [2.2361,     1.4142,        0,      0,        0,        2]
    expected_segment=[ 2.2913,   1.7321,        0,      0,        0.25,     2]
    for i, pt in enumerate(points):
        dist = math3d.distance3d_point_line_segment(pt, line)
        print('Point {} is {} away from segment {}, should be {}'.format(pt, dist, line, expected_segment[i]))
        dist = math3d.distance3d_point_line(pt, line)
        print('Point {} is {} away from line {}, should be {}\n'.format(pt, dist, line, expected_line[i]))

