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
    """3D rotation matrix, x-axis."""
    ct = np.cos(theta)
    st = np.sin(theta)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, ct, -st],
        [0.0, st, ct]], dtype=np.float64)


def roty3d(theta):
    """3D rotation matrix, y-axis."""
    ct = np.cos(theta)
    st = np.sin(theta)
    return np.array([
        [ct, 0.0, st],
        [0.0, 1.0, 0.0],
        [-st, 0.0, ct]], dtype=np.float64)


def rotz3d(theta):
    """3D rotation matrix, z-axis."""
    ct = np.cos(theta)
    st = np.sin(theta)
    return np.array([
        [ct, -st, 0.0],
        [st, ct, 0.0],
        [0.0, 0.0, 1.0]], dtype=np.float64)


def rot3d(deg_x, deg_y, deg_z):
    """Returns the 3D rotation matrix in ZYX (i.e. yaw-pitch-roll) order."""
    Rx = rotx3d(np.deg2rad(deg_x))
    Ry = roty3d(np.deg2rad(deg_y))
    Rz = rotz3d(np.deg2rad(deg_z))
    R = prjutils.matmul(Rx, prjutils.matmul(Ry, Rz))
    return R


def pt2tuple(pt):
    """Convert a 3D point to a tuple."""
    return (pt[0,0], pt[1,0], pt[2,0])


def rotate_bbox3d(box, deg_x, deg_y, deg_z):
    """Rotates the bounding box around the origin (of the coordinate system)."""
    R = rot3d(deg_x, deg_y, deg_z)
    # print('Euler angles are: ', np.rad2deg(cmath.euler_angles_from_rotation_matrix(R)).reshape((1,3)))
    coords = box[0]
    color = box[1]
    coords = [pt2tuple(prjutils.apply_transformation(R, np.array([pt[0], pt[1], pt[2]]).reshape((3,1)))) for pt in coords]
    if len(box) == 2:
        return (coords, color)
    return (coords, color, box[2])


def rotate_bbox3d_center(box, deg_x, deg_y, deg_z):
    """Rotates the 3D bounding box by [deg_x, deg_y, deg_z] around its center."""
    cb = center_bbox3d(box)
    box = shift_bbox3d(box, -cb[0], -cb[1], -cb[2])
    box = rotate_bbox3d(box, deg_x, deg_y, deg_z)
    return shift_bbox3d(box, cb[0], cb[1], cb[2])


def shift_bbox3d(box, dx, dy, dz):
    """Shifts the 3D bounding box by [dx, dy, dz]."""
    coords = box[0]
    color = box[1]
    coords = [(pt[0]+dx, pt[1]+dy, pt[2]+dz) for pt in coords]
    if len(box) == 2:
        return (coords, color)
    return (coords, color, box[2])


def center_bbox3d(box):
    """Computes the center of the 3D bounding box."""
    coords = box[0]
    sx = sum([pt[0] for pt in coords])
    sy = sum([pt[1] for pt in coords])
    sz = sum([pt[2] for pt in coords])
    return (sx/8.0, sy/8.0, sz/8.0)


##############################################
# Actual demo functions


def demo_pseudocolor():
    """Pseudocoloring."""
    peaks = imutils.imread('../data/peaks.png', mode='L')
    # For visualization purposes only, reduce input to a few
    # distinct categories/labels:
    data = ((peaks / 25) - 5).astype(np.int16)
    names = ['Bone', 'Magma', 'Viridis']
    images = list()
    for name in names:
        pc = imvis.pseudocolor(
            data, limits=None,  # Compute min/max from data
            color_map=colormaps.by_name(name, return_rgb=True))
        images.append(pc)

    # Display as a single collage
    padding = 10
    # Add alpha channel to render the collage nicely for the repo's README
    images[0] = np.dstack((images[0], 255*np.ones(images[0].shape[:2], dtype=np.uint8)))
    collage = imvis.make_collage(images, padding=padding, fixed_size_per_image=(200, 200), bg_color=(0, 0, 0, 0), num_images_per_row=len(images))

    # Add labels
    height, width = collage.shape[:2]
    mask_width = (width - (len(names)-1)*padding) / len(names)
    for i in range(len(names)):
        pos = (i * (mask_width + padding) + mask_width/2, height - 10)
        collage = imvis.draw_text_box(collage, names[i],
            pos, text_anchor='south', bg_color=(0, 0, 0),
            font_color=(-1, -1, -1), font_scale=1.0,
            font_thickness=1, padding=5, fill_opacity=0.8)

    imvis.imshow(collage, title='Pseudocoloring', wait_ms=-1)
    imutils.imsave('../../doc/example-pseudocolor.png', collage)


def demo_overlay():
    """How to overlay images and highlight regions."""
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
    """How to draw basic shapes, text boxes, etc."""
    # * rotated rect around dot, maybe
    img = imutils.imread('../data/ninja.jpg', mode='RGB')  # Load grayscale as 3-channel image
    # Draw rounded box(es)
    vis_img = imvis.draw_rounded_rects(img, [(9, 23, 149, 106)],
        corner_percentage=0.2, fill_opacity=0.75, line_width=0, color=(0, 200, 200), non_overlapping=True)
    # Draw filled & dashed rect
    # vis_img = imvis.draw_rects(vis_img, [(178, 164, 43, 29)], fill_opacity=0.4, line_width=2, dash_length=10, color=(220, 0, 255))

    # Draw rotated rectangle
    vis_img = imvis.draw_rotated_rects(vis_img, 
        [(60, 220, 70, 45, -55)], 
        fill_opacity=0.5, line_width=2, 
        dash_length=15, color=(0, 200, 0))

    # Draw lines - a line is a list or tuple: ((start-point), (end-point), is_dashed, color)
    lines = [
        [(7, 184), (329, 211), True, (255, 0, 255)],  # Dashed
        [(42, 147), (337, 168), False, (255, 0, 255)]]  # Solid
    vis_img = imvis.draw_lines(vis_img, lines, line_width=3, default_color=(200, 255, 0), dash_length=15)

    # Draw arrows - same format as lines (see above)
    arrows = [[(314, 20), (175, 38), False, (0, 255, 255)],
        ((314+20, 20+30), (175+20, 38+30), True, (255, 0, 255))]
        #[(320, 33), (316, 87), True, (0, 255, 255)]]
    vis_img = imvis.draw_arrows(vis_img, arrows, line_width=2, default_color=(0,200,255), dash_length=15, arrow_head_factor=0.1)
    
    # Draw text box
    vis_img = imvis.draw_text_box(vis_img, 'Angry',
        (283, 68), text_anchor='west',
        bg_color=(255, 0, 255), font_color=(-1, -1, -1),
        font_scale=1.0, font_thickness=1,
        padding=5, fill_opacity=0.8)

    imvis.imshow(vis_img, title='Drawing Primitives', wait_ms=10)


def demo_bbox2d():
    """Renders 2D bounding boxes (with annotations) & fading trajectories."""
    # Bounding boxes
    img = imutils.imread('../data/ninja.jpg', mode='RGB')  # Load grayscale as 3-channel image
    bboxes2d = [
        # ((178, 164, 43, 29), (255, 0, 255), '', True),  # Dashed violett rect (at the target's dot)
        (np.array([177, 76, 147, 53]), (0, 255, 255), 'Katana'),  # Also accepts a 4-element np.array
        ]
    vis_img = imvis.draw_bboxes2d(
        img, bboxes2d, text_anchor='south',
        font_scale=1.0, font_thickness=1, text_box_opacity=0.75,
        line_width=2)

    # Rotated bounding box
    vis_img = imvis.draw_rotated_bboxes2d(vis_img,
        [((252, 148, 60, 30, 15), (0, 200, 0), 'Tabi'),
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
    # from vcp import ui_basics
    # print(ui_basics.select_points(vis_img))
    
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


def calibrated_example():
    """Returns an exemplary image with corresponding camera calibration and pose."""
    img = imutils.imread('../data/ninja.jpg', mode='RGB')  # Load grayscale as 3-channel image
    img_height, img_width = img.shape[:2]
    # Get camera extrinsics via PnP
    img_points = np.array([(4.4, 183.4), (76.6, 190.6), (155.5, 197.5), (240.2, 204.8),
        (328.4, 210.6), (44, 147.1), (103.2, 151.1), (167.7, 156.2), (235, 160), (305.9, 164.7),
        (122, 125), (175.9, 128.1), (289, 134.4), (327.1, 115.5), (33.4, 253.4)], dtype=np.float32)
    # Corresponding grid points
    grid_points = [
        (0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (0, 1), (1, 1), (2, 1),
        (3, 1), (4, 1), (1, 2), (2, 2), (4, 2), (5, 3), (1, -1)]
    # Convert to mm
    grid_width_mm = 24.0
    obj_points = np.array([[s[0]*grid_width_mm, s[1]*grid_width_mm, 0.0] for s in grid_points], dtype=np.float32)
    # Compute focal length in pixels
    focal_length_mm = 5.6
    sensor_width_mm = 7.3 # sensor_height_mm = 5.47, sensor resolution 7296x5472 (40 MP)
    fl = focal_length_mm / sensor_width_mm * img_width
    # Good enough guess for principal point and distortion :-p
    cx = img_width / 2.0
    cy = img_height / 2.0
    dist_coeff = np.zeros(4)
    K = np.float64([[fl, 0, cx], [0, fl, cy], [0, 0, 1]]).reshape((3, 3, 1))
    # print(K)
    # print(obj_points, obj_points.shape, np.transpose(obj_points))
    # print(img_points.shape)
    # objectPoints = np.random.random((10,3,1))
    # imagePoints = np.random.random((10,2,1))
    # cameraMatrix = np.eye(3)
    # distCoeffs = np.zeros((5,1))
    # ret, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)
    # Solve for the extrinsics
    ret, rvec, t = cv2.solvePnP(obj_points, img_points, K, dist_coeff)
    if not ret:
        raise RuntimeError('Could not recover camera pose!')
    # print(ret, rvec, t)
    # verts = cv2.projectPoints(obj_points, rvec, t, K, dist_coeff)[0].reshape(-1, 2)
    # print(verts)
    R, _ = cv2.Rodrigues(rvec)
    # vis_img = imvis.draw_points(img, np.transpose(img_points), color=(255, 0, 0))
    # vis_img = imvis.draw_points(vis_img, np.transpose(verts), color=(0, 255, 255))
    return img, K, R, t


def demo_bbox3d():
    """Renders 3D bounding boxes (with annotations)."""
    img, K, R, t = calibrated_example()

    def _mkbb(footpoint, size, angles, color, label):
        xh = size[0] / 2.0
        yh = size[1] / 2.0
        zh = size[2] / 2.0
        # Centered 3d box
        box = [
            (-xh, -yh, +zh), (+xh, -yh, +zh), (+xh, +yh, +zh), (-xh, +yh, +zh), # First, the 4 top corners
            (-xh, -yh, -zh), (+xh, -yh, -zh), (+xh, +yh, -zh), (-xh, +yh, -zh), # Then, the 4 bottom corners
        ]
        # A 3d bounding box is a box with RGB color and an optional TODO??? label
        box3d = (box, color, label)  # Must be a tuple!!!
        return shift_bbox3d(rotate_bbox3d(box3d, angles[0], angles[1], angles[2]),
            footpoint[0], footpoint[1], footpoint[2] + zh)

    bboxes = [
        _mkbb((78, 34, 0), (26, 20, 45), (0, 0, -30), (255, 0, 255), 'Ninja'),  # The ninja
        _mkbb((36, -12, 0), (24, 24, 15), (0, 0, 0), (0, 200, 0), None),  # Small box in front
        _mkbb((6, 48, 10), (60, 35, 35), (6, -2, 30), None, 'Lens')
    ]
    vis_img = imvis.draw_bboxes3d(img, bboxes, K, R, t,
        scale_image_points=1.0, default_box_color=(255, 0, 0),
        line_width=2, dash_length=10,
        fill_opacity_top=0.2, fill_opacity_bottom=0.5,
        font_color=(0, 0, 0), text_anchor='center',
        font_scale=1.0, font_thickness=1,
        text_box_padding=5, text_box_opacity=0.7,
        non_overlapping=True)
    return vis_img


def demo_vis_extrinsics():
    """Visualizes the ground plane grid, coordinate system axes, etc."""
    img, K, R, t = calibrated_example()
    # Draw the horizon (or print a warning if it lies outside the fov)
    vis_img = imvis.draw_horizon(img, K, R, t,
        color=(0, 255, 255),
        scale_image_points=1.0,
        text_opacity=0.7,
        line_width=3,
        dash_length=-1,
        warn_if_not_visible=True)
    # Overlay the groundplane grid
    vis_img = imvis.draw_groundplane_grid(vis_img, K, R, t,
        grid_spacing=24,
        grid_limits=(0, -24, 96, 196),
        grid_origin=(0, 0),
        scale_image_points=1.0,
        point_radius=4,
        point_thickness=-1,
        line_thickness=2,
        opacity=0.9,
        output_rgb=True)
    # Overlay the axis (actually not starting at 0,0,0 so we can see it fully inside the image...)
    vis_img = imvis.draw_xyz_axes(vis_img, K, R, t,
        origin=(12, 12, 0),
        scale_axes=48,
        scale_image_points=1.0,
        line_width=4,
        dash_length=-1,
        image_is_rgb=True)
    return vis_img


def show_off_boxes3d():
    # 3D bounding box animations (to show clipping, visibility tests, etc.)
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
        vis_img = imvis.draw_text_box(vis_img, 'Press ESC to cancel', (512, 5), text_anchor='north')
        k = imvis.imshow(vis_img, title="3d bbox (non-overlapping)", wait_ms=100) & 0xFF
        if k == 27:
            return


    # Show off (2)
    for angle in range(0,360,2):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [rotate_bbox3d_center(box3da, angle, 0, 0)], K, R, t, line_width=3, dash_length=-1, text_anchor='southeast')
        vis_img = imvis.draw_text_box(vis_img, 'Press ESC to cancel', (512, 5), text_anchor='north')
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=10) & 0xFF
        if k == 27:
            return

    # Show off (3)
    for angle in range(0,360,2):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [rotate_bbox3d_center(box3da, 0, angle, 0)], K, R, t, line_width=3)
        vis_img = imvis.draw_text_box(vis_img, 'Press ESC to cancel', (512, 5), text_anchor='north')
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=10) & 0xFF
        if k == 27:
            return

    # Show off (4)
    for angle in range(0,360,2):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [rotate_bbox3d_center(box3da, 0, 0, angle)], K, R, t, line_width=3)
        vis_img = imvis.draw_text_box(vis_img, 'Press ESC to cancel', (512, 5), text_anchor='north')
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=10) & 0xFF
        if k == 27:
            return

    # Show off (5)
    for shift in range(0,-48,-1):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [shift_bbox3d(rotate_bbox3d(box3da, 0, 0, 10), 1.9, shift/10.0, 0)], K, R, t, line_width=3)
        vis_img = imvis.draw_text_box(vis_img, 'Press ESC to cancel', (512, 5), text_anchor='north')
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=50) & 0xFF
        if k == 27:
            return

    # Show off (6)
    box = rotate_bbox3d_center(box3da, 90, 0, 0)
    for shift in range(0,-54,-1):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [shift_bbox3d(box, 1.5, shift/10.0, 0)], K, R, t, line_width=3)
        vis_img = imvis.draw_text_box(vis_img, 'Press ESC to cancel', (512, 5), text_anchor='north')
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=50) & 0xFF
        if k == 27:
            return

    # Show off (7)
    box = rotate_bbox3d_center(box3da, 40, 0, 20)
    for shift in range(0,-60,-1):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [shift_bbox3d(box, 1.5, shift/10.0, 0)], K, R, t, line_width=3)
        vis_img = imvis.draw_text_box(vis_img, 'Press ESC to cancel', (512, 5), text_anchor='north')
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=50) & 0xFF
        if k == 27:
            return

    # Show off (8)
    for shift in range(0,-63,-1):
        vis_img = np.zeros((768, 1024, 3), dtype=np.uint8)
        vis_img = imvis.draw_bboxes3d(vis_img, [shift_bbox3d(rotate_bbox3d_center(box3db, 10, 10, -20), 1, 4+shift/10.0, 1.5)], K, R, t, line_width=3)
        vis_img = imvis.draw_text_box(vis_img, 'Press ESC to cancel', (512, 5), text_anchor='north')
        k = imvis.imshow(vis_img, title="3d bbox", wait_ms=50) & 0xFF
        if k == 27:
            return


if __name__ == "__main__":
    # Render a collage of 2D & 3D bounding boxes + coordinate system visuals
    images = list()
    names = list()
    
    images.append(demo_bbox2d())
    names.append('2D Bounding Boxes & Trajectory')

    images.append(demo_bbox3d())
    names.append('3D Bounding Boxes')

    images.append(demo_vis_extrinsics())
    names.append('World Coodinate System')

    # Add alpha channel to render the README visualization nicely for web display
    images[0] = np.dstack((images[0], 255*np.ones(images[0].shape[:2], dtype=np.uint8)))
    padding=10
    collage = imvis.make_collage(images, padding=padding, bg_color=(0, 0, 0, 0), fixed_size_per_image=(300, 225), num_images_per_row=2)
    
    # Add labels
    height, width = collage.shape[:2]
    mask_width = (width - padding) / 2
    for i in range(len(names)):
        pos = ((i % 2) * (mask_width + padding) + mask_width/2,
            (i // 2) * (225 + padding) + (225 - 10))
        collage = imvis.draw_text_box(collage, names[i],
            pos, text_anchor='south', bg_color=(0, 0, 0),
            font_color=(-1, -1, -1), font_scale=1.0,
            font_thickness=1, padding=5, fill_opacity=0.8)

    imvis.imshow(collage, title='Basic Drawing', wait_ms=-1)
    imutils.imsave('../../doc/example-imvis.png', collage)

    # ############################################################################
    # Drawing basic shapes/primitives
    demo_primitives()

    # ############################################################################
    # Pseudocolorization
    demo_pseudocolor()

    # ############################################################################
    # Overlay images and highlight regions
    demo_overlay()

    # ############################################################################
    # ## Stereoscopic images
    # TODO capture example stereo data + calibration
    # # Load rectified stereo pair as NumPy array, RGB ordering
    # rect_left = imutils.imread('../../examples/data/stereo-rect-left.jpg')
    # rect_right = imutils.imread('../../examples/data/stereo-rect-right.jpg')

    # # # Show pair
    # # collage = imvis.make_collage([rect_left, rect_right], fixed_size_per_image=(640, 480), bg_color=(255,255,255), padding=5)
    # # imvis.imshow(collage, is_rgb=True, title="A collage")

    # # Stereo anaglyph
    # anaglyph = imvis.make_anaglyph(rect_left, rect_right, shift=(40,0))
    # imvis.imshow(anaglyph, title='Anaglyph RGB from Stereo pair', flip_channels=False)
    
    # ############################################################################
    # 3D bounding box animations (to show clipping, visibility tests, etc.)
    show_off_boxes3d()
