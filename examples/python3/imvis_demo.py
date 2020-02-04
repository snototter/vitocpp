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
    images = list()
    pc = imvis.pseudocolor(
        peaks, limits=[0, 255],  # uint8 data
        color_map=colormaps.colormap_temperature_rgb)
    images.append(pc)
    pc = imvis.pseudocolor(
        peaks.astype(np.int32), limits=[0, 255],  # int32 data
        color_map=colormaps.colormap_turbo_rgb)
    images.append(pc)
    pc = imvis.pseudocolor(
        peaks.astype(np.float64)/255.0,  # default limits are [0, 1]
        color_map=colormaps.colormap_viridis_rgb)
    images.append(pc)
    pc = imvis.pseudocolor(
        ((peaks / 25) - 5).astype(np.int16),  # reduce input to few categories
        limits=None,   # Will be computed from data
        color_map=colormaps.colormap_viridis_rgb)
    images.append(pc)
    # Display as a single collage
    collage = imvis.make_collage(images, padding=10, bg_color=(0, 0, 0), num_images_per_row=len(images))
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


def demo_primitives():
    img = imutils.imread('../data/ninja.jpg', mode='RGB')  # Load grayscale as 3-channel image
    # Draw rounded box(es)
    vis_img = imvis.draw_rounded_rects(img, [(9, 23, 149, 106)],
        corner_percentage=0.25, fill_opacity=0.75, line_width=0, color=(0, 200, 200), non_overlapping=True)
    # Draw filled & dashed rect
    # vis_img = imvis.draw_rects(vis_img, [(178, 164, 43, 29)], fill_opacity=0.4, line_width=2, dash_length=10, color=(220, 0, 255))

    # Draw lines - a line is a list or tuple: ((start-point), (end-point), is_dashed, color)
    lines = [
        [(7, 184), (329, 211), True, (255, 0, 255)],  # Dashed
        [(42, 147), (337, 168), False, (255, 0, 255)]]  # Solid
    vis_img = imvis.draw_lines(vis_img, lines, line_width=3, default_color=(200, 255, 0), dash_length=10)

    # Draw arrows - same format as lines (see above)
    arrows = [[(314, 20), (175, 38), False, (0, 255, 255)]]
        #[(320, 33), (316, 87), True, (0, 255, 255)]]
    vis_img = imvis.draw_arrows(vis_img, arrows, line_width=2, default_color=(0,200,255), dash_length=10, arrow_head_factor=0.1)
    
    # Draw text box
    vis_img = imvis.draw_text_box(vis_img, 'Angry',
        (283, 68), text_anchor='west',
        bg_color=(255, 0, 255), font_color=(-1, -1, -1),
        font_scale=1.0, font_thickness=1,
        padding=5, fill_opacity=0.8)
    #TODO arrows, etc

    return vis_img


def demo_bbox2d():
    # Bounding boxes
    img = imutils.imread('../data/ninja.jpg', mode='RGB')  # Load grayscale as 3-channel image
    bboxes2d = [
        ((178, 164, 43, 29), (255, 0, 255), '', True),  # Dashed violett rect (at the target's dot)
        (np.array([177, 76, 147, 53]), (0, 255, 255), 'Sword'),  # Also accepts a 4-element np.array
        ]
    vis_img = imvis.draw_bboxes2d(
        img, bboxes2d, text_anchor='south',
        font_scale=1.0, font_thickness=1, text_box_opacity=0.75,
        line_width=2)

    # Rotated bounding box
    vis_img = imvis.draw_rotated_bboxes2d(vis_img,
        [((252, 148, 60, 30, 15), (0, 200, 0), 'Shoes'),
        ((80, 68, 150, 90, -10), (0, 255, 255), 'Lens')],
        line_width=2,
        fill_opacity=0.4,
        font_color=(0, 0, 0),
        text_anchor="north",
        font_scale=1.0, font_thickness=1,
        text_padding=5,
        text_box_opacity=0.8)

    # Trajectory
    traj_sword = [(323, 96), (321, 83), (317, 68), (310, 54), (305, 44), (294, 35),
        (283, 29), (273, 27), (261, 26), (246, 28), (231, 33), (217, 40), (207, 49),
        (201, 62), (196, 74), (192, 87), (183, 100), (175, 112), (159, 120), (144, 123),
        (128, 123), (115, 119)]
    # traj_ninja = [(278, 148), (290, 147), (302, 150), (307, 162),
    #     (307, 177), (287, 184), (270, 187), (256, 195), (239, 199), (222, 211), (211, 222), (204, 230), (197, 239)]

    vis_img = imvis.draw_fading_trajectory(vis_img, traj_sword, newest_position_first=True,
        smoothing_window=7, trajectory_length=-1,
        obj_color=(0, 0, 255), fade_color=(180, 180, 180),  # Fade towards gray
        max_line_width=4, dash_length=-1)
    from vcp import ui_basics
    print(ui_basics.select_points(vis_img))
    
    # # Flip the trajectory
    # trajectory = imutils.flip_points(trajectory, image_size=(im_width, im_height), flip_horizontally=True, flip_vertically=False)
    # # Rotate the trajectory
    # trajectory = imutils.rotate_points(trajectory, (im_width/2.0, im_height/2.0), np.deg2rad(-20))
    # # Draw without fading
    # vis_img = imvis.draw_trajectory(vis_img, trajectory, smoothing_window=-1, color=(255,0,0), dash_length=10, line_width=3)
    # Simplify with RDP    
    # trajectory = math2d.simplify_trajectory_rdp(trajectory, 50)
    # vis_img = trajvis.draw_trajectory(vis_img, trajectory, smoothing_window=-1, obj_color=(0,0,0), dash_length=10, line_width=3)
    return vis_img

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
    images = list()
    names = list()
    
    images.append(demo_bbox2d())
    names.append('2D BBox & Trajectory')

    

    images.append(demo_primitives())
    names.append('Primitives')

    # Add alpha channel to render the README visualization
    images[0] = np.dstack((images[0], 255*np.ones(images[0].shape[:2], dtype=np.uint8)))
    padding=10
    collage = imvis.make_collage(images, padding=padding, bg_color=(0, 0, 0, 0), num_images_per_row=len(images))
    # TODO add labels
    height, width = collage.shape[:2]
    mask_width = (width - (len(names)-1)*padding) / len(names)
    for i in range(len(names)):
        pos = (i * (mask_width + padding) + mask_width/2, height - 10)
        collage = imvis.draw_text_box(collage, names[i],
            pos, text_anchor='south', bg_color=(0, 0, 0),
            font_color=(-1, -1, -1), font_scale=1.0,
            font_thickness=1, padding=5, fill_opacity=0.8)

    imvis.imshow(collage, title='Basic Drawing', wait_ms=-1)
    imutils.imsave('example-imvis.png', collage)
    raise RuntimeError()
    
    
    # # Rectangles (pt coords of rotated rects are their centers!)
    # # Touching left/top border
    # vis_img = imvis.draw_rotated_rects(rgb, [[20, 30, 120, 40, 45]], fill_opacity=0.0, line_width=4, 
    #         dash_length=-1, color=(200, 20, 0))
    # vis_img = imvis.draw_rotated_rects(vis_img, [[20, 30, 120, 40, 45]], fill_opacity=0.2, line_width=2, 
    #         dash_length=10, color=(0, 200, 0))
    # # Touching right/top border
    # vis_img = imvis.draw_rotated_rects(vis_img, [[im_width-20, 35, 120, 40, 45]], fill_opacity=0.0, 
    #         line_width=4, dash_length=-1, color=(200, 20, 200))
    # vis_img = imvis.draw_rotated_rects(vis_img, [[im_width-60, 35, 120, 40, 45]], fill_opacity=0.2, 
    #         line_width=2, dash_length=10, color=(0, 200, 0))

    # # Bottom/right
    # vis_img = imvis.draw_rotated_rects(vis_img, [[300, 500, 120, 40, 95]], fill_opacity=0.4, 
    #         line_width=2, dash_length=-1, color=(0, 20, 200))
    # vis_img = imvis.draw_rotated_rects(vis_img, [[480, 400, 120, 40, -60]], fill_opacity=0.7, 
    #         line_width=2, dash_length=10, color=(200, 0, 200))
    # vis_img = imvis.draw_rotated_rects(vis_img, [[480, 500, 120, 40, -120]], fill_opacity=0.79,
    #         line_width=2, dash_length=10, color=(0, 200, 200))

    # # Axis-aligned boxes, touching borders
    # vis_img = imvis.draw_rects(vis_img, [[-10, 150, 20, 30]], fill_opacity=0.0, 
    #         line_width=2, dash_length=-1, color=(200, 20, 0))
    # vis_img = imvis.draw_rects(vis_img, [[im_width/2, -10, 20, 30]], fill_opacity=0.0, 
    #         line_width=2, dash_length=-1, color=(200, 20, 0))
    # vis_img = imvis.draw_rects(vis_img, [[im_width-10, im_height/2, 20, 30]], fill_opacity=0.0, 
    #         line_width=2, dash_length=-1, color=(200, 20, 0))
    # vis_img = imvis.draw_rects(vis_img, [[im_width/2, im_height-10, 20, 30]], fill_opacity=0.0, 
    #         line_width=2, dash_length=-1, color=(200, 20, 0))
    

    # collage = imvis.make_collage([vis_boxes2, vis_fancy, vis_img], padding=5, bg_color=(255,255,255))
    # imvis.imshow(collage, title="Drawing primitives (highly cluttered)")


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
