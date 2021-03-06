#!/usr/bin/env python
# coding=utf-8
"""A demo application showing some math capabilities."""

import pytest
import numpy as np
import os
import sys
# import matplotlib.pyplot as plt

# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vcp import math2d
from vcp import math3d
from vcp import imutils
from vcp import imvis

def _tangent_helper(img, center1, radius1, center2, radius2):
    num_transverse, tt1, tt2 = math2d.transverse_common_tangents_circles(center1, radius1, center2, radius2)
    num_direct, td1, td2 = math2d.direct_common_tangents_circles(center1, radius1, center2, radius2)
    
    tt = list()
    if num_transverse > 0:
        tt.append(tt1)
        if num_transverse == 2:
            tt.append(tt2)
    if tt:
        img = imvis.draw_lines(img, tt, default_color=(255, 0, 0), line_width=2)

    td = list()
    if num_direct > 0:
        td.append(td1)
        if num_direct == 2:
            td.append(td2)
    if td:
        img = imvis.draw_lines(img, td, default_color=(0, 255, 0), line_width=2)

    img = imvis.draw_points(img, [center1], color=(0, 0, 255), radius=radius1, line_width=1)
    img = imvis.draw_points(img, [center2], color=(0, 0, 255), radius=radius2, line_width=1)

    bboxes = [([center1[0]-radius1, center1[1]-radius1, 2*radius1, 2*radius1], (0, 0, 255), '{:d} | {:d}'.format(num_transverse, num_direct))]
    img = imvis.draw_bboxes2d(img, bboxes, text_anchor='center', line_width=0, fill_opacity=0.0)
    return img


def test_circle_creation():
    cres = math2d.circle_from_three_points((0,0), (0,0), (10, 20))
    assert cres[0] == False
    x, y, r = 3, 4, 5
    cres = math2d.circle_from_three_points((x, y+r), (x, y-r), (x+r, y))
    assert cres[0]
    assert cres[1][0] == x and cres[1][1] == y
    assert cres[2] == r


def tuple_approx(p, expected):
    assert len(p) == len(expected)
    for i in range(len(expected)):
        assert p[i] == pytest.approx(expected[i])


def test_plane_creation():
    plane = math3d.plane_from_three_points((10,234,0), (13,-23,-16), (234.2,37,999))
    tuple_approx(plane, (-0.976462, -0.02473778, 0.21426381, 15.553265))
    # Collinear - should be all 0, and a warning issued
    plane = math3d.plane_from_three_points((-7,3,0), (3,3,10), (5,3,12))
    tuple_approx(plane, (0, 0, 0, 0))


def test_plane_distances():
    # Compute distances:
    plane = math3d.plane_from_three_points((-1,-2,2), (-1,2,2), (1,0,1))
    pt1, pt2, pt3 = (0,15,2), np.array([1.40425069,  0.,  4.30850138]), (3,0,0)
    assert math3d.distance_point_plane(pt1, plane) == pytest.approx(plane[0])
     # Exactly 3.14 away from the plane's z-intercept
    assert math3d.distance_point_plane(pt2, plane) == pytest.approx(-3.14)
    assert math3d.distance_point_plane(pt3, plane) == pytest.approx(0)


def test_point_on_plane():
    plane = math3d.plane_from_three_points((-1,-2,2), (-1,2,2), (1,0,1))
    pt = -plane[3]*np.array(plane)[:3]
    # print('Point ({:.1f},{:.1f},{:.1f}) is {:.1f} from plane'.format(pt[0], pt[1], pt[2], math3d.distance_point_plane(pt, plane)))
    assert math3d.distance_point_plane(pt, plane) == pytest.approx(0)
    pt = -plane[3]*np.array(plane)[:3]  + np.array(plane)[:3]
    # print('Point ({:.1f},{:.1f},{:.1f}) is {:.1f} from plane (should be 1.0)'.format(pt[0], pt[1], pt[2], math3d.distance_point_plane(pt, plane)))
    assert math3d.distance_point_plane(pt, plane) == pytest.approx(1.0)
    pt = -plane[3]*np.array(plane)[:3]  - 23.0*np.array(plane)[:3]
    assert math3d.distance_point_plane(pt, plane) == pytest.approx(-23)


def test_plane_angles():
    groundplane = [0.0, 0, 1, 0]
    assert np.rad2deg(math3d.angle_line_plane(
        ((0,0,0),(0,0,1)), groundplane)) == pytest.approx(90)
    assert np.rad2deg(math3d.angle_line_plane(
        ((0,0,0),(0,1,1)), groundplane)) == pytest.approx(45)
    assert np.rad2deg(math3d.angle_line_plane(
        ((0,0,0),(1,1,1)), groundplane)) == pytest.approx(35.26438968)
    # Line parallel to plane (ON plane):
    assert np.rad2deg(math3d.angle_line_plane(
        ((0,0,0),(1,1,0)), groundplane)) == pytest.approx(0)
    tuple_approx(math3d.intersection_line_plane(((0,0,0),(1,1,0)), groundplane), (0, 0, 0))
    # Line parallel to plane (OFF plane):
    assert np.rad2deg(math3d.angle_line_plane(
        ((0,0,1),(1,1,1)), groundplane)) == pytest.approx(0)
    assert math3d.intersection_line_plane(((0,0,1),(1,1,1)), groundplane) is None
    assert np.rad2deg(math3d.angle_line_plane(
        ((0,0,1000),(0,10000,1000.1)), groundplane)) == pytest.approx(0.000572957795111855)


def test_plane_line_intersection():
    plane = math3d.plane_from_three_points((-1,-2,2), (-1,2,2), (1,0,1))
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
        assert intercepts == expected_segment[i]

        # Line intersection
        pt = math3d.intersection_line_plane(line, plane)
        intercepts = pt is not None
        assert intercepts == expected_line[i]


def test_point_line_distance3d():
    # Point to line/line segment distance
    line = ((1,1,1), (1,1,3))
    points =        [(2,-1,0.5), (0,0,0), (1,1,1), (1,1,2.9), (1,1,3.25), (1,-1,2)]
    expected_line = [2.2361,     1.4142,        0,      0,        0,        2]
    expected_segment=[ 2.2913,   1.7321,        0,      0,        0.25,     2]
    for i, pt in enumerate(points):
        dist = math3d.distance3d_point_line_segment(pt, line)
        assert dist == pytest.approx(expected_segment[i], 1e-4)
        dist = math3d.distance3d_point_line(pt, line)
        assert dist == pytest.approx(expected_line[i], 1e-4)

#TODO test is_point_in_circle intersection_circle_circle, etc.

def test_rdp():
    # RDP polyline simplification:
    def to_xy(a):
        N = len(a)
        x = np.array([e[0] for e in a])
        y = np.array([e[1] for e in a])
        return (x,y)

    x = np.arange(0,5,0.02)
    y = np.exp(-x) * np.cos(2.0*np.pi*x)
    polyline = [(x[i], y[i]) for i in range(len(x))]
    polyline_simple = math2d.simplify_trajectory_rdp(polyline, 0.5)
    assert len(polyline) == 250
    assert len(polyline_simple) == 5
    # Check that start/end stays the same
    assert polyline[0] == pytest.approx(polyline_simple[0])
    assert polyline[-1] == pytest.approx(polyline_simple[-1])
    # xs, ys = to_xy(polyline_simple)
    # plt.plot(xs, ys)
    # plt.show()

    polyline_simple = math2d.simplify_trajectory_rdp(polyline, 0.1)
    assert len(polyline_simple) == 8
    assert polyline[0] == pytest.approx(polyline_simple[0])
    assert polyline[-1] == pytest.approx(polyline_simple[-1])
    # xs, ys = to_xy(polyline_simple)
    # plt.plot(xs, ys)
    # plt.show()


def test_point_in_poly():
    # Point in polygon tests:
    polygon = [(1,1), (2,4), (4,4), (2,2), (4,1)] # open polygon
    pt = (0,0)
    assert math2d.is_point_in_closed_polygon(pt, polygon) == False
    pt = (1,1)
    assert math2d.is_point_in_closed_polygon(pt, polygon) == True
    pt = (2,1.5)
    assert math2d.is_point_in_closed_polygon(pt, polygon) == True
    pt = (2,1)
    assert math2d.is_point_in_closed_polygon(pt, polygon) == True
    pt = (3,2)
    assert math2d.is_point_in_closed_polygon(pt, polygon) == False
    pt = (4.5,4)
    assert math2d.is_point_in_closed_polygon(pt, polygon) == False


def test_point_poly_distance():
    # Point to polygon distance:
    polygon = [(1,1), (2,4), (4,4), (2,2), (4,1)] # open polygon
    pt = (0,0)
    assert math2d.distance_point_closed_polygon(pt, polygon) == pytest.approx(np.sqrt(2))
    pt = (1,1)
    assert math2d.distance_point_closed_polygon(pt, polygon) == pytest.approx(0)
    pt = (2,1.5)
    expected = 0.4472135954999579
    assert math2d.distance_point_closed_polygon(pt, polygon) == pytest.approx(-expected)
    pt = (2,1)
    assert math2d.distance_point_closed_polygon(pt, polygon) == pytest.approx(0)
    pt = (3,2)
    assert math2d.distance_point_closed_polygon(pt, polygon) == pytest.approx(expected)
    pt = (4.5,4)
    assert math2d.distance_point_closed_polygon(pt, polygon) == pytest.approx(0.5)
    
    polygon = [(1,1), (1,-1), (-1,-1), (-1,1)]
    pt = (-1, -0.781278)
    assert math2d.distance_point_closed_polygon(pt, polygon) == pytest.approx(0)
    assert math2d.is_point_in_closed_polygon(pt, polygon) == True


def test_closest_point2d():
    line = ((1, 1), (3, 3))
    pts = [(-1, 0), (0.5, 1.5), (2.5, 1.5), (3, 3), (6, 2)]
    exp_line = [(-0.5, -0.5), (1, 1), (2, 2), (3, 3), (4, 4)]
    exp_segment = [(1, 1), (1, 1), (2, 2), (3, 3), (3, 3)]
    for i in range(len(pts)):
        res = math2d.closest_point_on_line(pts[i], line)
        tuple_approx(res, exp_line[i])
        res = math2d.closest_point_on_line_segment(pts[i], line)
        tuple_approx(res, exp_segment[i])


if __name__ == "__main__":
    # Rotate rect: #TODO not math related - move to imvis demo
    rgb = imutils.imread('../../examples/data/flamingo.jpg', mode='RGB', flip_channels=False)
    im_height, im_width = rgb.shape[0], rgb.shape[1]
    center = (im_width/2.0, im_height/2.0)
    rect = (im_width*3/4, im_height/2, 50, 80)
    vis_img = imvis.draw_rects(rgb, [rect], fill_opacity=0.4, line_width=2, dash_length=10, color=(220, 0, 255))

    rotrect = imutils.rotate_rect(rect, center, np.deg2rad(5))
    vis_img = imvis.draw_rotated_rects(vis_img, [rotrect], fill_opacity=0.4, line_width=2, dash_length=10, color=(220, 0, 0))

    imvis.imshow(vis_img, title="rotated rect", wait_ms=200)

    #TODO make testable
    ## Find the convex hull of this polyline
    # First, scale it to the target image size
    target_sz = (1024,768)
    xmin,xmax = np.min(x), np.max(x)
    ymin,ymax = np.min(y), np.max(y)
    padding=20
    scale_x = (target_sz[0]-2*padding)/(xmax-xmin)
    scale_y = (target_sz[1]-2*padding)/(ymax-ymin)
    # Subsample the polyline to get some points for convex hull computation
    polygon = [((p[0]-xmin)*scale_x+padding, target_sz[1]-((p[1]-ymin)*scale_y+padding)) for p in [polyline[sub] for sub in range(0,len(polyline),5)]]


    for i in range(5):
        if i == 0:
            poly = polygon
        else:
            print('Using random permutation of the polygon...')
            perm = np.random.permutation(len(polygon))
            # print('Permutation of polygon: {}'.format(perm))
            poly = [polygon[idx] for idx in perm]
        hull = math2d.convex_hull(poly)

        vis_img = np.zeros((target_sz[1], target_sz[0], 3), dtype=np.uint8)
        vis_img = imvis.draw_polygon(vis_img, hull, (0,255,255), fill_opacity=0.4, line_width=-1)
        vis_img = imvis.draw_points(vis_img, poly, radius=5, opacity=1.0) # color=(255,0,0)

        # Compute bounding rectangles
        arect = math2d.axis_aligned_bounding_rect(poly)
        rrect = math2d.rotated_bounding_rect(poly)
        vis_img = imvis.draw_rects(vis_img, [arect], line_width=2, color=(255,0,255))
        vis_img = imvis.draw_rotated_rects(vis_img, [rrect], line_width=2, color=(0,0,255))
        imvis.imshow(vis_img, title="polygon", wait_ms=-1)
    

    #################
    # Line - Circle
    line = ((1,1.5), (2,1.7))
    print('No intersection? {}'.format(math2d.intersection_line_circle(line, (2.5, 0.5), 1.0)))

    line = ((1,1.5), (2,1.5))
    print('Tangent? {}'.format(math2d.intersection_line_circle(line, (2.5, 0.5), 1.0)))

    line = ((1,1.3), (7,0.5))
    print('Intersections? {}'.format(math2d.intersection_line_circle(line, (2.5, 0.5), 1.0)))


    ###################
    # Circle stuff
    img = np.zeros((512, 512, 3), dtype=np.uint8)
    # common case
    center1, radius1 = (100, 100), 90
    center2, radius2 = (300, 60), 50
    img = _tangent_helper(img, center1, radius1, center2, radius2)
    # same radii
    center1, radius1 = (420, 30), 40
    center2, radius2 = (420, 120), 40
    img = _tangent_helper(img, center1, radius1, center2, radius2)
    # intersecting (outer)
    center1, radius1 = (100, 230), 30
    center2, radius2 = (160, 230), 30
    img = _tangent_helper(img, center1, radius1, center2, radius2)
    # intersecting (outer) different radii
    center1, radius1 = (300, 450), 50
    center2, radius2 = (390, 450), 40
    img = _tangent_helper(img, center1, radius1, center2, radius2)
    # overlapping
    center1, radius1 = (300, 230), 80
    center2, radius2 = (350, 230), 70
    img = _tangent_helper(img, center1, radius1, center2, radius2)
    # intersecting (inner)
    center1, radius1 = (100, 400), 80
    center2, radius2 = (140, 400), 40
    img = _tangent_helper(img, center1, radius1, center2, radius2)
    # inside
    center1, radius1 = (425, 350), 40
    center2, radius2 = (420, 350), 50
    img = _tangent_helper(img, center1, radius1, center2, radius2)

    imvis.imshow(img, title="Tangents", wait_ms=-1)
