import os
import sys
import cv2
import numpy as np
from vito import imutils
from vito import cam_projections as prj

from iminspect import inputs, imgview

from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QVBoxLayout
from PyQt5.QtCore import Qt, QSize


sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'gen'))
import vcp.imvis as imvis

# def rotx3d(theta):
#     """3D x-axis rotation."""
#     ct = np.cos(theta)
#     st = np.sin(theta)
#     return np.array([
#         [1.0, 0.0, 0.0],
#         [0.0, ct, -st],
#         [0.0, st, ct]], dtype=np.float64)


# def roty3d(theta):
#     """3D y-axis rotation."""
#     ct = np.cos(theta)
#     st = np.sin(theta)
#     return np.array([
#         [ct, 0.0, st],
#         [0.0, 1.0, 0.0],
#         [-st, 0.0, ct]], dtype=np.float64)


# def rotz3d(theta):
#     """3D z-axis rotation."""
#     ct = np.cos(theta)
#     st = np.sin(theta)
#     return np.array([
#         [ct, -st, 0.0],
#         [st, ct, 0.0],
#         [0.0, 0.0, 1.0]], dtype=np.float64)


# def perspectiveView(img, angle_x, angle_y, angle_z, tx, ty, tz, bgcolor=None):
#     """Changes the camera extrinsics (angles in degrees) and return the corresponding view."""
#     im_height, im_width = img.shape[0:2]
#     Rx = rotx3d(np.deg2rad(angle_x))
#     print(Rx)
#     Ry = roty3d(np.deg2rad(angle_y))
#     print(Ry)
#     Rz = rotz3d(np.deg2rad(angle_z))
#     R = prj.matmul(Rx, prj.matmul(Ry, Rz))
#     t = np.array([tx, ty, tz], dtype=np.float64).reshape((3, 1))

#     # Corners of the input image, at z=1
#     corners3d_src = np.array([
#             [-1, 1, 1, -1],
#             [-1, -1, 1, 1],
#             [1, 1, 1, 1]
#         ], dtype=np.float64)
#     corners2d_src = np.array([
#             [0, im_width-1, im_width-1, 0],
#             [0, 0, im_height-1, im_height-1]
#         ], dtype=np.float64)

#     # Check where the image would be warped to
#     px = im_width / 2.0
#     py = im_height / 2.0
#     fx = im_width / 2.0
#     fy = im_height / 2.0
#     K = np.array([[fx, 0.0, px], [0.0, fy, py], [0.0, 0.0, 1.0]], dtype=np.float64)
#     P = prj.P_from_K_R_t(K, R, t)
#     print('Check initial', K, R, t)

#     corners2d_dst = prj.apply_projection(P, corners3d_src)
#     min_ = np.min(corners2d_dst, axis=1)
#     max_ = np.max(corners2d_dst, axis=1)

#     # Clip the maximum output size
#     width_out = min(2*im_width, int(np.ceil(max_[0] - min_[0])))
#     height_out = min(2*im_height, int(np.ceil(max_[1] - min_[1])))

#     # Adjust principal point offset so that warped image is visible
#     px -= min_[0]
#     py -= min_[1]
#     # Recompute projected corner points
#     K = np.array([[fx, 0.0, px], [0.0, fy, py], [0.0, 0.0, 1.0]], dtype=np.float64)
#     P = prj.P_from_K_R_t(K, R, t)
#     corners2d_dst = prj.apply_projection(P, corners3d_src)
#     #TODO FIXME Check why 0,0,0,0,0,0 still introduces a perspective!
#     print('Check final', K, R, t)
#     # Warp the image
#     M = cv2.getPerspectiveTransform(
#         corners2d_src.astype(np.float32).transpose(), 
#         corners2d_dst.astype(np.float32).transpose())
#     warped = cv2.warpPerspective(img, M, (width_out, height_out), borderValue=bgcolor)
#     if bgcolor is not None:
#         return warped
#     else:
#         # Warp an alpha mask
#         mask = np.zeros((im_height, im_width), dtype=np.uint8)
#         mask[:] = 255
#         warped_mask = cv2.warpPerspective(mask, M, (width_out, height_out))#, flags=cv2.INTER_NEAREST)
#         return np.dstack((warped, warped_mask))


class DemoApplication(QMainWindow):
    def __init__(self):
        super(DemoApplication, self).__init__()
        self._img_np = None
        self.__prepare_layout()

    def __prepare_layout(self):
        self._main_widget = QWidget()
        main_layout = QVBoxLayout()

        self._angle_x = inputs.SliderSelectionWidget('Angle x:', -180, 180, 361, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=150)
        self._angle_x.value_changed.connect(self.__changed)
        main_layout.addWidget(self._angle_x)
        self._angle_y = inputs.SliderSelectionWidget('Angle y:', -180, 180, 361, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=150)
        self._angle_y.value_changed.connect(self.__changed)
        main_layout.addWidget(self._angle_y)
        self._angle_z = inputs.SliderSelectionWidget('Angle z:', -180, 180, 361, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=150)
        self._angle_z.value_changed.connect(self.__changed)
        main_layout.addWidget(self._angle_z)


        self._tx = inputs.SliderSelectionWidget('tx:', -5, 5, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._tx.value_changed.connect(self.__changed)
        main_layout.addWidget(self._tx)
        self._ty = inputs.SliderSelectionWidget('ty:', -5, 5, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._ty.value_changed.connect(self.__changed)
        main_layout.addWidget(self._ty)
        self._tz = inputs.SliderSelectionWidget('tz:', -5, 5, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._tz.value_changed.connect(self.__changed)
        main_layout.addWidget(self._tz)

        self._bgcb = inputs.CheckBoxWidget('Transparent background:', is_checked=False)
        self._bgcb.value_changed.connect(self.__changed)
        main_layout.addWidget(self._bgcb)

        main_layout.addWidget(inputs.HLine())

        self._viewer = imgview.ImageViewer()
        main_layout.addWidget(self._viewer)

        self._main_widget.setLayout(main_layout)
        self.setCentralWidget(self._main_widget)
        self.resize(QSize(640, 480))


    def __changed(self, value):
        ax = self._angle_x.value()
        ay = self._angle_y.value()
        az = self._angle_z.value()
        tx = self._tx.value()
        ty = self._ty.value()
        tz = self._tz.value()
        warped = imvis.render_perspective(
            self._img_np, rx=ax, ry=ay, rz=az, angles_in_deg=True,
            tx=tx, ty=ty, tz=tz, bg_color=(-1, -1, -1) if self._bgcb.get_input() else (255, 255, 255))
        # warped = perspectiveView(self._img_np, ax, ay, az, tx, ty, tz, None if self._bgcb.get_input() else (255, 255, 255))
        self._viewer.showImage(warped)

    def displayImage(self, img_np):
        self._img_np = img_np
        self._viewer.showImage(img_np)


def gui():
    img = imutils.imread(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..' , '..', 'data' , 'flamingo.jpg'))
    app = QApplication(['Vis'])
    main_widget = DemoApplication()
    main_widget.displayImage(img)
    main_widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    #demo()
    gui()
