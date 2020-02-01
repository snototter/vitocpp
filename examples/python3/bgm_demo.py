#!/usr/bin/env python
# coding=utf-8
"""Showcasing background models."""

import os
import sys
import cv2

import numpy as np
from vito import pyutils

# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
from vcp import bgm
from vcp import colormaps
from vcp import imutils
from vcp import imvis


def demo(pseudocolor=True, inspect=False):
    imgs = [imutils.imread('../data/ninja-01.jpg'), imutils.imread('../data/ninja-02.jpg')]

    bgmodels = list()
    bgmodel = bgm.BackgroundModel()
    bgmodel.approximate_median_bgm(
        adaptation_step=5,
        fg_report_threshold=50,
        median_on_grayscale=True)
    bgmodels.append(bgmodel)

    bgmodel = bgm.BackgroundModel()
    bgmodel.block_mean_bgm(
        block_size=(16, 16),
        block_overlap=0.5,
        update_rate=0.01,
        fg_report_threshold=50,
        channel='grayscale')
    bgmodels.append(bgmodel)

    bgmodel = bgm.BackgroundModel()
    bgmodel.gaussian_mixture_bgm(
        history=500,
        detect_shadows=True,
        var_thresh=100,
        comp_thresh=0.05)
    bgmodels.append(bgmodel)

    bgmodel = bgm.BackgroundModel()
    bgmodel.normalized_rgb_bgm(
        report_as_binary=False,
        # binary_reporting_threshold=20,
        update_rate=0.1,
        alpha=0.1,
        beta=1.0)
    bgmodels.append(bgmodel)

    print('Note that initialization time for the first BGM includes')
    print('OpenCV initialization. On my i7 this took ~120ms.')
    times = list()
    fg_masks = list()
    for bgmodel in bgmodels:
        pyutils.tic('init')
        bgmodel.init(imgs[0])
        t_init = pyutils.ttoc('init')
        pyutils.tic('apply')
        mask = bgmodel.report_changes(imgs[1])
        t_apply = pyutils.ttoc('apply')

        if pseudocolor:
            fg_masks.append(imvis.pseudocolor(mask, limits=None, color_map=colormaps.colormap_viridis_rgb))
        else:
            fg_masks.append(mask)
        print('init/apply: {:17s} {:7.2f} ms, {:7.2f} ms'.format(bgmodel.name(), t_init, t_apply))

        if inspect:
            import iminspect
            iminspect.show(mask)

    padding = 10
    collage = imvis.make_collage([imutils.imread('../data/ninja-seq.png'), *fg_masks],
        padding=padding,
        fixed_size_per_image=(240, 320),
        bg_color=(0, 0, 0, 0),
        num_images_per_row=5)

    # Overlay names
    names = ['Input'] + [bg.name() for bg in bgmodels]
    height, width = collage.shape[:2]
    mask_width = (width - (len(names)-1)*padding) / len(names)
    for i in range(len(names)):
        pos = (i * (mask_width + padding) + mask_width/2, height - 10 if i > 0 else height/5)
        collage = imvis.draw_text_box(collage, names[i],
            pos, text_anchor='south', bg_color=(255, 255, 255),
            font_color=(-1, -1, -1), font_scale=1.0,
            font_thickness=1, padding=5, fill_opacity=0.8)
    
    imvis.imshow(collage, title="Background Subtraction", wait_ms=-1)


if __name__ == "__main__":
    demo()
