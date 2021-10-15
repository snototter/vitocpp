"""
Manually align depth to RGB stream (using a Kinect Azure)
"""

import os
import sys
import numpy as np
import cv2

# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vito import pyutils
from vito import cam_projections as prjutils
from vcp import colormaps
from vcp import imutils
from vcp import imvis
from vcp import best


def vis_depth(f, max_depth=2000):
    return imvis.pseudocolor(f, limits=[0, max_depth], color_map=colormaps.colormap_turbo_rgb)


def vis_infrared(f, max_ir=65535):
    # Normalize/stretch
    f = ((f.astype(np.float32) / max_ir) * 255).astype(np.uint8)
    return imutils.transform(f, 'histeq', 'gray2rgb')


def overlay_labels(frames, labels):
    assert(len(frames) == len(labels))
    for idx in range(len(frames)):
        frames[idx] = imvis.draw_text_box(frames[idx], 
                    labels[idx],
                    (frames[idx].shape[1]//2, 10), 'north',
                    bg_color=(0, 0, 0), font_color=(-1, -1, -1),
                    font_scale=1.0, font_thickness=1,
                    padding=5, fill_opacity=0.5)
    return frames


def align_depth_to_color(depth, K_color, K_depth, Rt_stereo, width_color, height_color):
    target_width = 400
    target_height = 400
    sx = target_width / width_color
    sy = target_height / height_color
    width_color = target_width
    height_color = target_height
    print(depth.shape)
    
    K_c = K_color.copy()
    K_c[0, 0] = K_c[0, 0] * sx
    K_c[0, 1] = K_c[0, 1] * sx
    K_c[0, 2] = K_c[0, 2] * sx

    K_c[1, 1] = K_c[1, 1] * sy
    K_c[1, 2] = K_c[1, 2] * sy
    
    #### Depth image to 3D (reference frame: stereo camera, not necessarily the actual common world frame of the multicam setup)
    height_d, width_d = depth.shape[:2]
    xx_d, yy_d = np.meshgrid(np.arange(0, width_d), np.arange(0, height_d))
    pixels_d = np.row_stack((
        xx_d[:].reshape((1, -1)),
        yy_d[:].reshape((1, -1)),
        np.ones((1, height_d*width_d), dtype=xx_d.dtype)))
    valid = depth > 0
    pixels_d = pixels_d[:, valid.flatten()]
    depth_valid = depth[valid]
    pts_norm_d = prjutils.apply_transformation(np.linalg.inv(K_depth), pixels_d)
    pts_cam_d = prjutils.shift_points_along_viewing_rays(pts_norm_d, depth_valid.reshape((1, -1)))
    pts_3d = prjutils.matmul(Rt_stereo[0], pts_cam_d) + Rt_stereo[1]
    
    #### Project into color image
    pixels_c = prjutils.apply_projection(K_c, pts_3d)
    # print('Depth2color pixels (shape, min, max)', pixels_c.shape, np.min(pixels_c, axis=1), np.max(pixels_c, axis=1))
    x_c = pixels_c[0, :].astype(np.int64)
    y_c = pixels_c[1, :].astype(np.int64)
    valid_x = np.logical_and(x_c >= 0, x_c < width_color)
    valid_y = np.logical_and(y_c >= 0, y_c < height_color)
    valid = np.logical_and(valid_x, valid_y)
    x_c = x_c[valid]
    y_c = y_c[valid]
    values = pts_3d[2, valid]

    aligned = np.zeros((height_color, width_color), dtype=depth.dtype)
    aligned[y_c, x_c] = values

    return aligned


def img_resolution(npimg):
    return (npimg.shape[1], npimg.shape[0])


def demo_align():
    cfg_base_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'data-best')
    cfg_file = os.path.join(cfg_base_path, 'k4a-manual-alignment.cfg')

    streamer = best.MulticamStepper(cfg_file, True, cfg_file_rel_path_base_dir=cfg_base_path, verbose=True)
    capture = streamer.start()
    
    _, frameset = streamer.next_frameset()
    
    K_color = capture.intrinsics(0)
    K_depth = capture.intrinsics(1)
    Rt_stereo = capture.stereo_transformation(1)
    D_color = capture.distortion_coefficients(0)
    D_depth = capture.distortion_coefficients(1)


    alignment = None
    num_frames_processed = 1
    while streamer.is_available():
        capture, frameset = streamer.next_frameset()
        if frameset is None:
            print('Invalid frameset received, terminating now!')
            break
        color, depth, infrared = frameset

        if alignment is None:
            alignment = best.RgbdAlignment(K_color, K_depth, Rt_stereo[0], Rt_stereo[1], img_resolution(color), img_resolution(depth), D_color, D_depth)
        # pyutils.tic('cpp-align')
        # aligned_depth_cpp = alignment.align_d2c(depth)
        # pyutils.toc('cpp-align')

        pyutils.tic('cpp-align-di')
        aligned_depth_cpp, aligned_ir_cpp = alignment.align_di2c(depth, infrared)
        pyutils.toc('cpp-align-di')

        

        vis_frames = [color, vis_depth(depth), vis_infrared(infrared)]
        vis_labels = ['RGB', 'Depth (original)', 'IR (original)']
        
        ## The cpp version takes care of properly interpolating depth values during alignment
        # takes about 3-5ms per 640x480 depth
        # Additionally aligning the intensity image adds another 0.8-1ms to the cpp processing time
        ## The python version is a naive reprojection + binning (without depth ordering, filtering, interpolation, etc.)
        # takes about 20-30 ms (~4-5 times slower than cpp)
        # pyutils.tic('py-align')
        # aligned_depth_py = align_depth_to_color(depth, K_color, K_depth, Rt_stereo, color.shape[1], color.shape[0])
        # pyutils.toc('py-align')

        aligned_depth = aligned_depth_cpp
        vis_aligned_depth = vis_depth(aligned_depth)
        vis_frames.append(vis_aligned_depth)
        vis_labels.append('Aligned Depth')

        aligned_ir = aligned_ir_cpp
        vis_aligned_ir = vis_infrared(aligned_ir)
        # vis_frames.append(vis_aligned_ir)
        # vis_labels.append('Aligned IR')
        
        # color_resized = cv2.resize(color, (aligned_depth.shape[1], aligned_depth.shape[0]))
        # vis_frames.append(color_resized)
        # vis_labels.append('color resized')

        # vis_frames.append(imvis.overlay(vis_aligned_depth, color_resized, 0.7))#, aligned_depth > 0))
        vis_frames.append(imvis.overlay(vis_aligned_depth, color, 0.7))
        vis_labels.append('RGB+D')

        vis_frames.append(imvis.overlay(vis_aligned_ir, color, 0.7))
        vis_labels.append('RGB+IR')

        # Visualization (rescale, overlay a label, show a single collage)
        bg_color = (180, 180, 180)
        vis_frames = [imutils.aspect_aware_resize(f, (640, 480), padding_value=bg_color)[0] for f in vis_frames]
        vis_frames = overlay_labels(vis_frames, vis_labels)

        img_per_row = len(vis_frames) // 2
        img_per_row = img_per_row if len(vis_frames) % 2 == 0 else img_per_row + 1
        collage = imvis.make_collage(vis_frames, bg_color=bg_color,
                                     num_images_per_row=img_per_row)
        k = imvis.imshow(collage, title='Stream', wait_ms=20)
        if k == ord('q') or k == 27:
            break

        num_frames_processed += 1


if __name__ == '__main__':
    demo_align()
