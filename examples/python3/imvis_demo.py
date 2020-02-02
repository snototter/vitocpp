#!/usr/bin/env python
# coding=utf-8
"""Showcasing image visualizations"""

import os
import sys
import cv2

import numpy as np

# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vcp import colormaps
from vcp import imutils
from vcp import imvis
from vito import cam_projections as prjutils


##############################################
# Utility functions for this demo


def rotx3d(theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, ct, -st],
        [0.0, st, ct]], dtype=np.float64)


def roty3d(theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    return np.array([
        [ct, 0.0, st],
        [0.0, 1.0, 0.0],
        [-st, 0.0, ct]], dtype=np.float64)


def rotz3d(theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    return np.array([
        [ct, -st, 0.0],
        [st, ct, 0.0],
        [0.0, 0.0, 1.0]], dtype=np.float64)


def pt2tuple(pt):
    return (pt[0,0], pt[1,0], pt[2,0])


def rotate_bbox3d(box, deg_x, deg_y, deg_z):
    Rx = rotx3d(np.deg2rad(deg_x))
    Ry = roty3d(np.deg2rad(deg_y))
    Rz = rotz3d(np.deg2rad(deg_z))
    R = prjutils.matmul(Rx, prjutils.matmul(Ry, Rz))
    # print('Euler angles are: ', np.rad2deg(cmath.euler_angles_from_rotation_matrix(R)).reshape((1,3)))
    coords = box[0]
    color = box[1]
    coords = [pt2tuple(prjutils.apply_transformation(R, np.array([pt[0], pt[1], pt[2]]).reshape((3,1)))) for pt in coords]
    if len(box) == 2:
        return (coords, color)
    return (coords, color, box[2])


def rotate_bbox3d_center(box, deg_x, deg_y, deg_z):
    cb = center_bbox3d(box)
    box = shift_bbox3d(box, -cb[0], -cb[1], -cb[2])
    box = rotate_bbox3d(box, deg_x, deg_y, deg_z)
    return shift_bbox3d(box, cb[0], cb[1], cb[2])


def shift_bbox3d(box, dx, dy, dz):
    coords = box[0]
    color = box[1]
    coords = [(pt[0]+dx, pt[1]+dy, pt[2]+dz) for pt in coords]
    if len(box) == 2:
        return (coords, color)
    return (coords, color, box[2])


def center_bbox3d(box):
    coords = box[0]
    sx = sum([pt[0] for pt in coords])
    sy = sum([pt[1] for pt in coords])
    sz = sum([pt[2] for pt in coords])
    return (sx/8.0, sy/8.0, sz/8.0)


##############################################
# Actual demo functions


def demo_pseudocolor():
    # Pseudocoloring
    peaks = imutils.imread('../data/peaks.png', mode='L')
    pc1 = imvis.pseudocolor(
        peaks, limits=[0, 255],  # uint8 data
        color_map=colormaps.colormap_temperature_rgb)
    pc2 = imvis.pseudocolor(
        peaks.astype(np.float64)/255.0,  # default limits are [0, 1]
        color_map=colormaps.colormap_viridis_rgb)
    pc3 = imvis.pseudocolor(
        ((peaks / 25) - 5).astype(np.int16),  # reduce input to few categories
        limits=None,   # Will be computed from data
        color_map=colormaps.colormap_viridis_rgb)
    # Display as a single collage
    collage = imvis.make_collage([pc1, pc2, pc3], padding=10, bg_color=(0, 0, 0))
    imvis.imshow(collage, title='Pseudocoloring', wait_ms=-1)


def demo_overlay():
    # Overlay
    rgb = imutils.imread('../data/flamingo.jpg', mode='L')
    # Generate some data to overlay
    im_height, im_width = rgb.shape[0], rgb.shape[1]
    peak_pos = (im_width*0.75, im_height*0.15)
    xv, yv = np.meshgrid(np.arange(0, im_width), np.arange(0, im_height))
    overlay = np.exp(-(np.power(xv-peak_pos[0], 2) + np.power(yv-peak_pos[1], 2)) / (3e4))
    overlay_vis = imvis.overlay(imvis.pseudocolor(overlay), rgb, 0.7)

    # Highlight regions
    rgb = imutils.imread('../data/flamingo.jpg', mode='RGB')
    rgb_mask = np.zeros((rgb.shape[0], rgb.shape[1]), dtype=np.uint8)
    rgb_mask[160:334, 120:290] = 1
    highlight = imvis.highlight(rgb, rgb_mask)
    # highlight another region, this time in blue
    rgb_mask[:] = 0
    rgb_mask[200:374, 250:420] = 1
    highlight = imvis.highlight(highlight, rgb_mask, color=(0,0,255))

    # Combine all "colored" images:
    collage = imvis.make_collage([overlay_vis, highlight], padding=5, bg_color=(255, 255, 255))
    imvis.imshow(collage, title="Overlay & Highlights", wait_ms=-1)


if __name__ == "__main__":
    demo_pseudocolor()

    demo_overlay()

    # TODO maybe add a draw-primitives demo?
    #collage = imvis.draw_circles(collage, [(30, 20), (100, 200)], [10, 27], thickness=-1, line_type=16)


    # ############################################################################
    # ## Stereoscopic images
    # # Load rectified stereo pair as NumPy array, RGB ordering
    # rect_left = imutils.imread('../../examples/data/stereo-rect-left.jpg')
    # rect_right = imutils.imread('../../examples/data/stereo-rect-right.jpg')

    # # # Show pair
    # # collage = imvis.make_collage([rect_left, rect_right], fixed_size_per_image=(640, 480), bg_color=(255,255,255), padding=5)
    # # imvis.imshow(collage, is_rgb=True, title="A collage")

    # # Stereo anaglyph
    # anaglyph = imvis.make_anaglyph(rect_left, rect_right, shift=(40,0))
    # imvis.imshow(anaglyph, title='Anaglyph RGB from Stereo pair', flip_channels=False)

    
    ############################################################################
    ## Drawing
    # Multiple bounding boxes
    rgb = imutils.imread('../data/flamingo.jpg', mode='RGB')
    im_height, im_width = rgb.shape[0], rgb.shape[1]
    bboxes2d = [
        (np.array([10,5,30,40])),
        (np.array([10,50,30,40]),),
        ([90.23, 50.2, 75, 100], None, None),
        ([300, 50, 75, 100], (255,0,255), 'Hat'),
        ((10, 200, 150, 75), None, "Hat, too"),
        ([220, 230, 160, 70], (0,200,250), 'Eyes', True)
        ]

    vis_img = imvis.draw_bboxes2d(rgb, bboxes2d, text_anchor='northeast', font_scale=1.0, font_thickness=1, text_box_opacity=0.6, line_width=2)


    # Draw a trajectory
    trajectory = [(53.3967,  51.8996), ( 86.3324,  58.8860), (133.2407,  58.8860), (184.1413,  62.8782), (226.0595,  62.8782),
        (274.9639,  66.8704), (303.9074,  69.8645), (326.8626,  78.8470), (336.8431,  91.8216), (332.8509, 100.8041), (317.8801, 112.7807),
        (288.9366, 127.7515), (256.0010, 136.7339), (201.1082, 150.7066), (158.1920, 176.6559), (141.2251, 188.6326), (120.2661, 210.5897),
        (128.2505, 229.5526), (174.1608, 237.5370), (208.0945, 239.5331), (267.9776, 242.5273), (306.9016, 244.5234), (354.8080, 244.5234),
         (386.7456, 252.5078), (424.6715, 282.4493), (444.6326, 325.3655), (445.6306, 345.3265), (406.7066, 384.2505), (363.7904, 409.2018),
         (306.9016, 434.1530), (237.0380, 456.1101), (158.1920, 471.0809), ( 62.3791, 479.0653)]
    vis_img = imvis.draw_fading_trajectory(vis_img, trajectory, newest_position_first=False,
        smoothing_window=7, trajectory_length=-1, obj_color=(0,0,255), fade_color=(180,180,180),
        max_line_width=4, dash_length=-1)

    # Flip the trajectory ;-)
    trajectory = imutils.flip_points(trajectory, image_size=(im_width, im_height), flip_horizontally=True, flip_vertically=False)
    # Draw without fading
    vis_img = imvis.draw_trajectory(vis_img, trajectory, smoothing_window=-1, color=(255,0,0), dash_length=10, line_width=3)

    # Rotate the trajectory ;-)
    trajectory = imutils.rotate_points(trajectory, (im_width/2.0, im_height/2.0), np.deg2rad(-20))
    vis_img = imvis.draw_fading_trajectory(vis_img, trajectory, smoothing_window=-1, obj_color=(200,0,200), dash_length=10, max_line_width=4)
    

    # # Rotate the trajectory back again and simplify using RDP
    # #FIXME FIXME FIXME move to mat?
    # trajectory = imutils.rotate_points(trajectory, (im_width/2.0, im_height/2.0), np.deg2rad(+20))
    # print('Trajectory length before RDP {}'.format(len(trajectory)))
    # trajectory = geoutils.simplify_trajectory_rdp2d(trajectory, 50)
    # print('Trajectory length after RDP {}'.format(len(trajectory)))
    # vis_img = trajvis.draw_trajectory(vis_img, trajectory, smoothing_window=-1, obj_color=(0,0,0), dash_length=10, line_width=3)
    vis_boxes2 = vis_img.copy()


    # Draw rounded boxes
    vis_img = imvis.draw_rounded_rects(rgb, [[100, 70, 242, 180], (10, 200, 150, 75),
        (-20, -30, 100, 70), (im_width-30, im_height*3/4, 100, 70)], # special ones touching the border
        corner_percentage=0.2, fill_opacity=0.4, line_width=0, color=(0, 200, 200), non_overlapping=True)


    # Draw filled,dashed rect
    vis_img = imvis.draw_rects(vis_img, [[220, 230, 160, 70]], fill_opacity=0.4, line_width=3, dash_length=10, color=(220, 0, 255))


    # Draw lines - a line is a list or tuple: ((start-point), (end-point), is_dashed, color)
    lines = [(np.array([10,20]).astype(np.int64), (im_width,im_height)),
            [(20,im_height/2), (im_width-40,im_height*3/4), False],
            ((im_width-5,10), (50,im_height-70), True, (255, 0, 255))]
    vis_img = imvis.draw_lines(vis_img, lines, line_width=5, default_color=(200,255,0), dash_length=25)


    # Draw arrows - same format as lines (see above)
    arrows = [((10,100), (im_width-100,im_height/2)),
             ((im_width/2,10), (im_width/2,im_height*3/4), True)]
    vis_img = imvis.draw_arrows(vis_img, arrows, line_width=2, default_color=(0,200,255), dash_length=25, arrow_head_factor=0.1)
    

    # Draw a fancy text box
    vis_img = imvis.draw_text_box(vis_img, "Small centered text", (414, 181), 
                    font_scale=0.75, font_thickness=1, text_anchor='center')
    vis_img = imvis.draw_text_box(vis_img, "Nasty text outside image", (im_width*3/4, 20), 
                    font_scale=2, font_thickness=2, text_anchor='south', bg_color=(255,255,255))
    vis_img = imvis.draw_text_box(vis_img, "Nasty text outside image", (im_width/2, im_height-30), 
                    font_scale=2, font_thickness=3, text_anchor='northeast', bg_color=(255,0,0))

    vis_fancy = vis_img.copy()
    
    
    # Rectangles (pt coords of rotated rects are their centers!)
    # Touching left/top border
    vis_img = imvis.draw_rotated_rects(rgb, [[20, 30, 120, 40, 45]], fill_opacity=0.0, line_width=4, 
            dash_length=-1, color=(200, 20, 0))
    vis_img = imvis.draw_rotated_rects(vis_img, [[20, 30, 120, 40, 45]], fill_opacity=0.2, line_width=2, 
            dash_length=10, color=(0, 200, 0))
    # Touching right/top border
    vis_img = imvis.draw_rotated_rects(vis_img, [[im_width-20, 35, 120, 40, 45]], fill_opacity=0.0, 
            line_width=4, dash_length=-1, color=(200, 20, 200))
    vis_img = imvis.draw_rotated_rects(vis_img, [[im_width-60, 35, 120, 40, 45]], fill_opacity=0.2, 
            line_width=2, dash_length=10, color=(0, 200, 0))

    # Bottom/right
    vis_img = imvis.draw_rotated_rects(vis_img, [[300, 500, 120, 40, 95]], fill_opacity=0.4, 
            line_width=2, dash_length=-1, color=(0, 20, 200))
    vis_img = imvis.draw_rotated_rects(vis_img, [[480, 400, 120, 40, -60]], fill_opacity=0.7, 
            line_width=2, dash_length=10, color=(200, 0, 200))
    vis_img = imvis.draw_rotated_rects(vis_img, [[480, 500, 120, 40, -120]], fill_opacity=0.79,
            line_width=2, dash_length=10, color=(0, 200, 200))

    # Axis-aligned boxes, touching borders
    vis_img = imvis.draw_rects(vis_img, [[-10, 150, 20, 30]], fill_opacity=0.0, 
            line_width=2, dash_length=-1, color=(200, 20, 0))
    vis_img = imvis.draw_rects(vis_img, [[im_width/2, -10, 20, 30]], fill_opacity=0.0, 
            line_width=2, dash_length=-1, color=(200, 20, 0))
    vis_img = imvis.draw_rects(vis_img, [[im_width-10, im_height/2, 20, 30]], fill_opacity=0.0, 
            line_width=2, dash_length=-1, color=(200, 20, 0))
    vis_img = imvis.draw_rects(vis_img, [[im_width/2, im_height-10, 20, 30]], fill_opacity=0.0, 
            line_width=2, dash_length=-1, color=(200, 20, 0))
    

    collage = imvis.make_collage([vis_boxes2, vis_fancy, vis_img], padding=5, bg_color=(255,255,255))
    imvis.imshow(collage, title="Drawing primitives (highly cluttered)")

    
    ## TODO For data augmentation, see augmentation_demo.py (rotated bounding boxes, rotating and clipping, etc.)
    # # Rotate image and rect (data augmentation sort of):
    # # rotcenter = (0,0)
    # rotcenter = (im_width/2.0,im_height/2.0)
    # for angle in range(0, 400, 10):
    #     M = cv2.getRotationMatrix2D(rotcenter, angle, 1)
    #     vis_img = cv2.warpAffine(rgb, M, (im_width, im_height))
    #     # rect around the eyes
    #     rect = [220, 230, 160, 70]
    #     rotrect = imutils.rotate_rect(rect, rotcenter, np.deg2rad(angle))
    #     vis_img = imvis.draw_rotated_rects(vis_img, [rotrect], fill_opacity=0.4, line_width=2, dash_length=-1, color=(0, 200, 200), flip_color_channels=False)
    #     # another rect
    #     rect = [225, 435, 20, 15]
    #     rotrect = imutils.rotate_rect(rect, rotcenter, np.deg2rad(angle))
    #     vis_img = imvis.draw_rotated_rects(vis_img, [rotrect], fill_opacity=0.4, line_width=2, dash_length=10, color=(0, 200, 200), flip_color_channels=False)
    #
    #     k = imvis.imshow(vis_img, title="rotated stuff", wait_ms=100) & 0xFF
    #     if k == ord('q') or k == 27:
    #         break


    ############################################################################
    ## 3D Bounding Box magic
    # Dummy boxes:
    box3da = ([(-3,5,2.5), (-1.5,5,2.5), (-1.5,3,2.5), np.array((-3,3,2.5)),
               (-3,5,0), (-1.5,5,0), (-1.5,3,0), (-3,3,0)], (255,0,0), "A box") # Must be a tuple!!!


    box3db = ([(-2,-2,-0.5), (-2,2,-0.5), (0.1,2,-0.5), (0.1,-2,-0.5),
               (-2,-2,-1.5), (-2,2,-1.5), (0.1,2,-1.5), (0.1,-2,-1.5)], (0,255,0), "A better box") # Must be a tuple!!!

    box3dc = ([[-0.5, -0.5, 1], [-0.5, 0.5, 1], [0.5, 0.5, 1], [0.5, -0.5, 1],
               [-0.5, -0.5, 0], [-0.5, 0.5, 0], np.array([0.5, 0.5, 0]), np.array([0.5, -0.5, 0])],
              (0, 255, 255), "Tilted occluder")
    box3dc = shift_bbox3d(rotate_bbox3d_center(box3dc, 45, 0, 10), 0, 0.5, 0)

    # Dummy camera parameters:
    R = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0]], dtype=np.float64)
    C = np.array([0,-1, 1], dtype=np.float64).reshape((3,1))
    t = -np.dot(R, C)
    K = np.array([[300, 0, 512], [0, 300, 384], [0,0,1]], dtype=np.float64)

    # Show off (1)
    for angle in range(0,90,2):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [box3da, rotate_bbox3d(box3db, 0, 0, angle), box3dc], 
                    K, R, t, line_width=3, text_anchor='center', non_overlapping=True)
    
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=100) & 0xFF
        if k == 27:
            break


    # Show off (2)
    for angle in range(0,360,2):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [rotate_bbox3d_center(box3da, angle, 0, 0)], K, R, t, line_width=3, dash_length=-1, text_anchor='southeast')
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=10) & 0xFF
        if k == 27:
            break

    # Show off (3)
    for angle in range(0,360,2):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [rotate_bbox3d_center(box3da, 0, angle, 0)], K, R, t, line_width=3)
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=10) & 0xFF
        if k == 27:
            break

    # Show off (4)
    for angle in range(0,360,2):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [rotate_bbox3d_center(box3da, 0, 0, angle)], K, R, t, line_width=3)
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=10) & 0xFF
        if k == 27:
            break

    # Show off (5)
    for shift in range(0,-48,-1):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [shift_bbox3d(rotate_bbox3d(box3da, 0, 0, 10), 1.9, shift/10.0, 0)], K, R, t, line_width=3)
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=50) & 0xFF
        if k == 27:
            break

    # Show off (6)
    box = rotate_bbox3d_center(box3da, 90, 0, 0)
    for shift in range(0,-54,-1):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [shift_bbox3d(box, 1.5, shift/10.0, 0)], K, R, t, line_width=3)
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=50) & 0xFF
        if k == 27:
            break

    # Show off (7)
    box = rotate_bbox3d_center(box3da, 40, 0, 20)
    for shift in range(0,-60,-1):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [shift_bbox3d(box, 1.5, shift/10.0, 0)], K, R, t, line_width=3)
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=50) & 0xFF
        if k == 27:
            break

    # Show off (8)
    for shift in range(0,-63,-1):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [shift_bbox3d(rotate_bbox3d_center(box3db, 10, 10, -20), 1, 4+shift/10.0, 1.5)], K, R, t, line_width=3)
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=50) & 0xFF
        if k == 27:
            break


    ####################################################################
    ## Putting visualizations together for a more "real-world" example:
    bbox = rotate_bbox3d_center(([[-0.5, -0.5, 2], [-0.5, 0.5, 2.5], [0.5, 0.5, 2.5], [0.5, -0.5, 2],
                                 [-0.5, -0.5, 0], [-0.5, 0.5, 0], [0.5, 0.5, 0], [0.5, -0.5, 0]],
                                 (255, 0, 255), "Cotton Eye Joe"), 5, 0, 20)
    bbox = shift_bbox3d(bbox, -0.5, 0.7, 0)
    trajectory = [(444,606), (508,644), (555,666),(605,672),(650,666),(685,656),(713,629),(750,600),
                  (790,590),(835,602),(874,611),(911,612),(944,595),(962,577)]


    # # Cotton Eye Joe:
    # img = imutils.imread('./data/pvtvis.jpg')
    # # Where did he come from
    # vis_img = imvis.draw_fading_trajectory(img, trajectory, newest_position_first=True,
    #     smoothing_window=5, trajectory_length=-1, obj_color=(255,0,0), fade_color=(180,180,180),
    #     max_line_width=5, dash_length=-1)

    # vis_img = imvis.draw_text_box(vis_img, "Where did he come from?", (720, 590), font_scale=1.5, font_thickness=2,
    #     text_anchor='center', font_color=(0,0,0), bg_color=(255,0,0), fill_opacity=0.5, padding=10)

    # # Where will he go
    # vis_img = imvis.draw_arrows(vis_img, [((330,520),(110,400))], line_width=3,
    #     default_color=(255,0,0), dash_length=25, arrow_head_factor=0.1)

    # vis_img = imvis.draw_text_box(vis_img, "Where will he go?", (140,395), font_scale=1.5, font_thickness=2,
    #     text_anchor='south', font_color=(0,0,0), bg_color=(255,0,0), fill_opacity=0.5, padding=10)

    # vis_img = imvis.draw_bboxes3d(vis_img, [bbox], K, R, t, line_width=3, fill_opacity_top=0.2,
    #     fill_opacity_bottom=0.3, font_scale=1.5, font_thickness=2, font_color=(0,0,0), text_anchor='north', text_box_padding=10)

    # imutils.imsave("pvtvis.jpg", vis_img, flip_channels=True)
    # imvis.imshow(vis_img, title="what is it good for?", wait_ms=-1)

