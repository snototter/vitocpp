#!/usr/bin/env python
# coding=utf-8
"""A demo application showing some math capabilities."""

import numpy as np
import os
import sys
import matplotlib.pyplot as plt

# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vcp import math2d
from vcp import imutils
from vcp import imvis


def demo_intersect(img):
    #TODO move vertical line to the left!
    l1 = ((76, 24), (76, 170)) # vertical line
    l2 = ((32, 64), (160, 32)) # top slanted line
    l3 = ((108, 96), (172, 80)) # bottom slanted line
    lines = [l1, l2, l3]
    c1 = ((88, 160), 32) # Center, radius 
    c2 = ((150, 180), 42)
    circles = [c1, c2]

    intersection_points = list() # "Real" intersection points
    wannabe_intersection_points = list() # Points which would be intersection points if the segments were infinite lines

    ##### Line-Line/Segment intersections
    ipl_12 = math2d.intersection_line_line_segment(l1, l2)
    intersection_points.append(ipl_12)
    # Line SEGMENT l3 doesn't intersect the vertical line
    ipl_13 = math2d.intersection_line_line_segment(l1, l3)
    assert ipl_13 is None
    # ... but they intersect, if we assume l3 was a line
    ipl_13 = math2d.intersection_line_line(l1, l3)
    wannabe_intersection_points.append(ipl_13)
    ipl_23 = math2d.intersection_line_line(l2, l3)
    assert ipl_23 is None

    ##### Line-Circle intersections
    def _app(lst, cint):
        if cint[0] > 0:
            lst.append(cint[1])
        if cint[0] > 1:
            lst.append(cint[2])
    intersect = math2d.intersection_line_segment_circle(l1, *c1)
    _app(intersection_points, intersect)
    # To get the second point, we need to interpret the segment as an infinite line
    intersect = math2d.intersection_line_circle(l1, *c1)
    for idx in range(intersect[0]):
        p = intersect[1 + idx]
        if not p in intersection_points:
            wannabe_intersection_points.append(p)
    # The other lines will never intersect the circle
    intersect = math2d.intersection_line_segment_circle(l2, *c1)
    assert intersect[0] == 0

    ##### Circle-Circle intersections
    intersect = math2d.intersection_circle_circle(*c1, *c2)
    _app(intersection_points, intersect)
    
    ##### Draw
    img = imvis.draw_circles(img, [c[0] for c in circles], [c[1] for c in circles],
        default_color=(0, 255, 255), thickness=2)
    img = imvis.draw_lines(img, lines, default_color=(0, 0, 200), line_width=2)
    # img = imvis.draw_points(img, intersection_points, color=(60, 180, 0), radius=10, line_width=2, opacity=1)
    # img = imvis.draw_points(img, wannabe_intersection_points, color=(200, 0, 0), radius=10, line_width=2, opacity=1)
    img = imvis.draw_crosses(img, intersection_points, color=(60, 180, 0), diagonal=20, line_width=2, vertical=False, opacity=1)
    img = imvis.draw_crosses(img, wannabe_intersection_points, color=(200, 0, 0), diagonal=20, line_width=2, vertical=False, dash_length=9, opacity=1)
    # Add label
    return imvis.draw_text_box(img, '2D Intersection',
            (110, img.shape[0]-10),
            text_anchor='south', bg_color=(0, 0, 0),
            font_color=(-1, -1, -1), font_scale=1.0,
            font_thickness=1, padding=5, fill_opacity=0.8)


def _tangent_helper(img, center1, radius1, center2, radius2):
    num_transverse, tt1, tt2 = math2d.transverse_common_tangents_circles(center1, radius1, center2, radius2)
    num_direct, td1, td2 = math2d.direct_common_tangents_circles(center1, radius1, center2, radius2)
    
    tt = list()
    if num_transverse > 0:
        tt.append(tt1)
        if num_transverse == 2:
            tt.append(tt2)
    if tt:
        img = imvis.draw_lines(img, tt, default_color=(200, 0, 0), line_width=2)

    td = list()
    if num_direct > 0:
        td.append(td1)
        if num_direct == 2:
            td.append(td2)
    if td:
        img = imvis.draw_lines(img, td, default_color=(60, 180, 0), line_width=2)

    img = imvis.draw_circles(img, [center1, center2], [radius1, radius2],
        default_color=(0, 255, 255), thickness=2)

    # bboxes = [([center1[0]-radius1, center1[1]-radius1, 2*radius1, 2*radius1], (0, 0, 255), '{:d} | {:d}'.format(num_transverse, num_direct))]
    # img = imvis.draw_bboxes2d(img, bboxes, text_anchor='center', line_width=0, fill_opacity=0.0)
    return img


def demo_tangents(img):
    c1 = ((260, 40), 32) # Center, radius 
    c2 = ((300, 130), 45)
    c3 = ((300, 195), 20)
    img = _tangent_helper(img, *c1, *c2)
    img = _tangent_helper(img, *c2, *c3)
    # Add label
    return imvis.draw_text_box(img, 'Tangents',
            (296, img.shape[0]-10),
            text_anchor='south', bg_color=(0, 0, 0),
            font_color=(-1, -1, -1), font_scale=1.0,
            font_thickness=1, padding=5, fill_opacity=0.8)


def demo_convhull(img):
    pts = [
        (512, 32),
        (480, 77),
        (520, 200),
        (570, 96)
    ]
    #TODO random sample?
    chull = math2d.convex_hull(pts)
    # Draw convex hull as closed polygon (need to append first hull point)
    img = imvis.draw_polygon(img, [*chull, chull[0]], color=(0, 255, 255), line_width=3, dash_length=-1, fill_opacity=0.3)
    img = imvis.draw_points(img, pts, color=(0, 0, 255), radius=5, line_width=-1, opacity=1)
    # Add label
    return imvis.draw_text_box(img, 'Polygons/Hull',
            (530, img.shape[0]-10),
            text_anchor='south', bg_color=(0, 0, 0),
            font_color=(-1, -1, -1), font_scale=1.0,
            font_thickness=1, padding=5, fill_opacity=0.8)


if __name__ == "__main__":
    # RGBA, transparent image
    vis_img = np.zeros((256, 1024, 4))
    vis_img = demo_intersect(vis_img)

    vis_img = demo_tangents(vis_img)

    vis_img = demo_convhull(vis_img)

    imvis.imshow(vis_img, title='Geometry Utilities', wait_ms=-1)
    imutils.imsave('../../doc/example-geometry.png', vis_img)
    # # #TODO remove this
    # # print(math2d.circle_from_three_points((0,0), (0,0), (10, 20)))

    # # # Plane from 3 points
    # # plane = cmath.plane_from_three_points((10,234,0), (13,-23,-16), (234.2,37,999))
    # # print(plane)

    # # # Collinear - should be all 0, and a warning issued
    # # plane = cmath.plane_from_three_points((-7,3,0), (3,3,10), (5,3,12))
    # # print(plane)

    # # # Compute distances:
    # # plane = cmath.plane_from_three_points((-1,-2,2), (-1,2,2), (1,0,1))
    # # print(plane)
    # # pt1, pt2, pt3 = (0,15,2), np.array([1.40425069,  0.,  4.30850138]), (3,0,0)

    # # print(cmath.distance3d_point_plane(pt1, plane)) # Something
    # # print(cmath.distance3d_point_plane(pt2, plane)) # Exactly 3.14 away from the plane's z-intercept
    # # print(cmath.distance3d_point_plane(pt3, plane)) # On the plane (x-intercept)
    # # # Point on plane
    # # pt = -plane[3]*np.array(plane)[:3]
    # # print('Point ({:.1f},{:.1f},{:.1f}) is {:.1f} from plane'.format(pt[0], pt[1], pt[2], cmath.distance3d_point_plane(pt, plane)))
    # # pt = -plane[3]*np.array(plane)[:3]  + np.array(plane)[:3]
    # # print('Point ({:.1f},{:.1f},{:.1f}) is {:.1f} from plane (should be 1.0)'.format(pt[0], pt[1], pt[2], cmath.distance3d_point_plane(pt, plane)))
    # # pt = -plane[3]*np.array(plane)[:3]  - 23.0*np.array(plane)[:3]
    # # print('Point ({:.1f},{:.1f},{:.1f}) is {:.1f} from plane (should be -23.0)'.format(pt[0], pt[1], pt[2], cmath.distance3d_point_plane(pt, plane)))
    # # print


    # # groundplane = [0.0, 0, 1, 0]
    # # print('Angle between line and plane: {} degrees (90?)'.format(np.rad2deg(cmath.angle_line_plane(
    # #     ((0,0,0),(0,0,1)), groundplane))))
    # # print('Angle between line and plane: {} degrees (45?)'.format(np.rad2deg(cmath.angle_line_plane(
    # #     ((0,0,0),(0,1,1)), groundplane))))
    # # print('Angle between line and plane: {} degrees '.format(np.rad2deg(cmath.angle_line_plane(
    # #     ((0,0,0),(1,1,1)), groundplane))))
    # # print('Angle between line and plane: {} degrees (0?)'.format(np.rad2deg(cmath.angle_line_plane(
    # #     ((0,0,0),(1,1,0)), groundplane))))
    # # print('Angle between line and plane: {} degrees (0?)'.format(np.rad2deg(cmath.angle_line_plane(
    # #     ((0,0,1),(1,1,1)), groundplane)))) # What to do with parallel? TODO
    # # print('Angle between line and plane: {} degrees (close to 0, but still larger than?)'.format(np.rad2deg(cmath.angle_line_plane(
    # #     ((0,0,1000),(0,10000,1000.1)), groundplane))))
    # # print


    # # # Plane and line/segment intersections
    # # lines = [((0,0,0), (0,0,1)),
    # #     ((0,1,0), (3,-1,1)),
    # #     ((0,1.9,0), (0,-1,1.5)),
    # #     ((-1,-1,1.9),(-1,1,1.9))]
    # # expected_segment = [False, True, True, False]
    # # expected_line = [True, True, True, False]
    # # for i, line in enumerate(lines):
    # #     # Segment intersection
    # #     pt = cmath.intersection3d_line_segment_plane(line, plane)
    # #     intercepts = pt is not None
    # #     if intercepts:
    # #         print('Segment {} intersects plane {} at {} (should be {})'.format(line, plane, pt, expected_segment[i]))
    # #     else:
    # #         print('Segment {} does not intersect plane (should be {})'.format(line, expected_segment[i]))

    # #     if intercepts is not expected_segment[i]:
    # #         raise ValueError("Wrong result for line segment[{}]".format(i))

    # #     # Line intersection
    # #     pt = cmath.intersection3d_line_plane(line, plane)
    # #     intercepts = pt is not None
    # #     if intercepts:
    # #         print('Line {} intersects plane {} at {} (should be {})'.format(line, plane, pt, expected_line[i]))
    # #     else:
    # #         print('Line {} does not intersect plane (should be {})'.format(line, expected_line[i]))

    # #     if intercepts is not expected_line[i]:
    # #         raise ValueError("Wrong result for line[{}]".format(i))
    # #     print('\n')

    # # # Point to line/line segment distance
    # # line = ((1,1,1), (1,1,3))
    # # points =        [(2,-1,0.5), (0,0,0), (1,1,1), (1,1,2.9), (1,1,3.25), (1,-1,2)]
    # # expected_line = [2.2361,     1.4142,        0,      0,        0,        2]
    # # expected_segment=[ 2.2913,   1.7321,        0,      0,        0.25,     2]
    # # for i, pt in enumerate(points):
    # #     dist = cmath.distance3d_point_line_segment(pt, line)
    # #     print('Point {} is {} away from segment {}, should be {}'.format(pt, dist, line, expected_segment[i]))
    # #     dist = cmath.distance3d_point_line(pt, line)
    # #     print('Point {} is {} away from line {}, should be {}\n'.format(pt, dist, line, expected_line[i]))

    
    # # Rotate rect: #TODO not math related
    # rgb = imutils.imread('../../examples/data/flamingo.jpg', mode='RGB', flip_channels=False)
    # im_height, im_width = rgb.shape[0], rgb.shape[1]
    # center = (im_width/2.0, im_height/2.0)
    # rect = (im_width*3/4, im_height/2, 50, 80)
    # vis_img = imvis.draw_rects(rgb, [rect], fill_opacity=0.4, line_width=2, dash_length=10, color=(220, 0, 255))

    # rotrect = imutils.rotate_rect(rect, center, np.deg2rad(5))
    # vis_img = imvis.draw_rotated_rects(vis_img, [rotrect], fill_opacity=0.4, line_width=2, dash_length=10, color=(220, 0, 0))

    # imvis.imshow(vis_img, title="rotated rect", wait_ms=200)


    # ########################################################################
    # ## Polygon
    # # RDP polyline simplification:
    # def to_xy(a):
    #     N = len(a)
    #     x = np.array([e[0] for e in a])
    #     y = np.array([e[1] for e in a])
    #     return (x,y)

    # x = np.arange(0,5,0.02)
    # y = np.exp(-x) * np.cos(2.0*np.pi*x)
    # polyline = [(x[i], y[i]) for i in range(len(x))]
    # plt.plot(x, y)
    # # plt.show()

    # polyline_simple = math2d.simplify_trajectory_rdp(polyline, 0.5)
    # print('RDP from {} points down to {}'.format(len(polyline), len(polyline_simple)))
    # print('Original start|end:', polyline[0], polyline[-1])
    # print('Simplified:        ', polyline_simple[0], polyline_simple[-1])
    # xs, ys = to_xy(polyline_simple)
    # plt.plot(xs, ys)
    # # plt.show()

    # polyline_simple = math2d.simplify_trajectory_rdp(polyline, 0.1)
    # print('RDP from {} points down to {}'.format(len(polyline), len(polyline_simple)))
    # print('Original start|end:', polyline[0], polyline[-1])
    # print('Simplified:        ', polyline_simple[0], polyline_simple[-1])
    # print
    # xs, ys = to_xy(polyline_simple)
    # plt.plot(xs, ys)
    # print('Waiting for you to close plot')
    # plt.show()

    # ## Find the convex hull of this polyline
    # # First, scale it to the target image size
    # target_sz = (1024,768)
    # xmin,xmax = np.min(x), np.max(x)
    # ymin,ymax = np.min(y), np.max(y)
    # padding=20
    # scale_x = (target_sz[0]-2*padding)/(xmax-xmin)
    # scale_y = (target_sz[1]-2*padding)/(ymax-ymin)
    # # Subsample the polyline to get some points for convex hull computation
    # polygon = [((p[0]-xmin)*scale_x+padding, target_sz[1]-((p[1]-ymin)*scale_y+padding)) for p in [polyline[sub] for sub in range(0,len(polyline),5)]]


    # for i in range(5):
    #     if i == 0:
    #         poly = polygon
    #     else:
    #         print('Using random permutation of the polygon...')
    #         perm = np.random.permutation(len(polygon))
    #         # print('Permutation of polygon: {}'.format(perm))
    #         poly = [polygon[idx] for idx in perm]
    #     hull = math2d.convex_hull(poly)

    #     vis_img = np.zeros((target_sz[1], target_sz[0], 3), dtype=np.uint8)
    #     vis_img = imvis.draw_polygon(vis_img, hull, (0,255,255), fill_opacity=0.4, line_width=-1)
    #     vis_img = imvis.draw_points(vis_img, poly, radius=5, opacity=1.0) # color=(255,0,0)

    #     # Compute bounding rectangles
    #     arect = math2d.axis_aligned_bounding_rect(poly)
    #     rrect = math2d.rotated_bounding_rect(poly)
    #     vis_img = imvis.draw_rects(vis_img, [arect], line_width=2, color=(255,0,255))
    #     vis_img = imvis.draw_rotated_rects(vis_img, [rrect], line_width=2, color=(0,0,255))
    #     imvis.imshow(vis_img, title="polygon", wait_ms=-1)
    

    # # Point in polygon tests:
    # polygon = [(1,1), (2,4), (4,4), (2,2), (4,1)] # open polygon
    # pt = (0,0)
    # print('Is {} in polygon? {} should be False'.format(pt, math2d.is_point_in_closed_polygon(pt, polygon)))
    # pt = (1,1)
    # print('Is {} in polygon? {} should be True'.format(pt, math2d.is_point_in_closed_polygon(pt, polygon)))
    # pt = (2,1.5)
    # print('Is {} in polygon? {} should be True'.format(pt, math2d.is_point_in_closed_polygon(pt, polygon)))
    # pt = (2,1)
    # print('Is {} in polygon? {} should be True'.format(pt, math2d.is_point_in_closed_polygon(pt, polygon)))
    # pt = (3,2)
    # print('Is {} in polygon? {} should be False'.format(pt, math2d.is_point_in_closed_polygon(pt, polygon)))
    # pt = (4.5,4)
    # print('Is {} in polygon? {} should be False'.format(pt, math2d.is_point_in_closed_polygon(pt, polygon)))
    # print

    # # Point to polygon distance:
    # polygon = [(1,1), (2,4), (4,4), (2,2), (4,1)] # open polygon
    # pt = (0,0)
    # print('Distance {} to poly: {}, should be {}'.format(pt, math2d.distance_point_closed_polygon(pt, polygon), np.sqrt(2)))
    # pt = (1,1)
    # print('Distance {} to poly: {}, should be 0'.format(pt, math2d.distance_point_closed_polygon(pt, polygon)))
    # pt = (2,1.5)
    # print('Distance {} to poly: {}, should be less than 0.5'.format(pt, math2d.distance_point_closed_polygon(pt, polygon)))
    # pt = (2,1)
    # print('Distance {} to poly: {}, should be 0'.format(pt, math2d.distance_point_closed_polygon(pt, polygon)))
    # pt = (3,2)
    # print('Distance {} to poly: {}, should be less than 0.5'.format(pt, math2d.distance_point_closed_polygon(pt, polygon)))
    # pt = (4.5,4)
    # print('Distance {} to poly: {}, should be 0.5'.format(pt, math2d.distance_point_closed_polygon(pt, polygon)))
    # print

    # polygon = [(1,1), (1,-1), (-1,-1), (-1,1)]
    # pt = (-1, -0.781278)
    # print('Distance {} to poly: {}'.format(pt, math2d.distance_point_closed_polygon(pt, polygon)))
    # print('         {} inside:  {}'.format(pt, math2d.is_point_in_closed_polygon(pt, polygon)))


    # #################
    # # Line - Circle
    # line = ((1,1.5), (2,1.7))
    # print('No intersection? {}'.format(math2d.intersection_line_circle(line, (2.5, 0.5), 1.0)))

    # line = ((1,1.5), (2,1.5))
    # print('Tangent? {}'.format(math2d.intersection_line_circle(line, (2.5, 0.5), 1.0)))

    # line = ((1,1.3), (7,0.5))
    # print('Intersections? {}'.format(math2d.intersection_line_circle(line, (2.5, 0.5), 1.0)))


    # ###################
    # # Circle stuff
    # img = np.zeros((512, 512, 3), dtype=np.uint8)
    # # common case
    # center1, radius1 = (100, 100), 90
    # center2, radius2 = (300, 60), 50
    # img = _tangent_helper(img, center1, radius1, center2, radius2)
    # # same radii
    # center1, radius1 = (420, 30), 40
    # center2, radius2 = (420, 120), 40
    # img = _tangent_helper(img, center1, radius1, center2, radius2)
    # # intersecting (outer)
    # center1, radius1 = (100, 230), 30
    # center2, radius2 = (160, 230), 30
    # img = _tangent_helper(img, center1, radius1, center2, radius2)
    # # intersecting (outer) different radii
    # center1, radius1 = (300, 450), 50
    # center2, radius2 = (390, 450), 40
    # img = _tangent_helper(img, center1, radius1, center2, radius2)
    # # overlapping
    # center1, radius1 = (300, 230), 80
    # center2, radius2 = (350, 230), 70
    # img = _tangent_helper(img, center1, radius1, center2, radius2)
    # # intersecting (inner)
    # center1, radius1 = (100, 400), 80
    # center2, radius2 = (140, 400), 40
    # img = _tangent_helper(img, center1, radius1, center2, radius2)
    # # inside
    # center1, radius1 = (425, 350), 40
    # center2, radius2 = (420, 350), 50
    # img = _tangent_helper(img, center1, radius1, center2, radius2)

    # imvis.imshow(img, title="Tangents", wait_ms=-1)
