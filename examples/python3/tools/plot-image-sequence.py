import os
import sys
import numpy as np

from iminspect import inputs, imgview, inspection_widgets

from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QVBoxLayout, \
    QHBoxLayout, QFileDialog, QShortcut
from PyQt5.QtCore import QSize
from PyQt5.QtGui import QKeySequence

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'gen'))
from vcp import imvis, imutils


class DemoApplication(QMainWindow):
    def __init__(self):
        super(DemoApplication, self).__init__()
        self._images = None
        self._vis_np = None
        self.__prepare_layout()
        self.__prepare_shortcuts()

    def __prepare_shortcuts(self):
        # Open file
        sc = QShortcut(QKeySequence('Ctrl+O'), self)
        sc.activated.connect(self.__load_request)
        # Save file
        sc = QShortcut(QKeySequence('Ctrl+S'), self)
        sc.activated.connect(self.__save_request)

    def __prepare_layout(self):
        self._main_widget = QWidget()
        input_layout = QVBoxLayout()
        min_lbl_width = 160

        self._angle_x = inputs.SliderSelectionWidget('Camera rotation x:', -180, 180, 180, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=min_lbl_width)
        self._angle_x.value_changed.connect(self.__changed)
        input_layout.addWidget(self._angle_x)
        self._angle_y = inputs.SliderSelectionWidget('Camera rotation y:', -180, 180, 180, 4,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=min_lbl_width)
        self._angle_y.value_changed.connect(self.__changed)
        input_layout.addWidget(self._angle_y)
        self._angle_z = inputs.SliderSelectionWidget('Camera rotation z:', -180, 180, 180, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=min_lbl_width)
        self._angle_z.value_changed.connect(self.__changed)
        input_layout.addWidget(self._angle_z)


        self._tx = inputs.SliderSelectionWidget('Camera translation x:', -20, 20, 100, -3,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=min_lbl_width)
        self._tx.value_changed.connect(self.__changed)
        input_layout.addWidget(self._tx)
        self._ty = inputs.SliderSelectionWidget('Camera translation y:', -20, 20, 100, 2.8,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=min_lbl_width)
        self._ty.value_changed.connect(self.__changed)
        input_layout.addWidget(self._ty)
        self._tz = inputs.SliderSelectionWidget('Camera translation z:', -5, 15, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=min_lbl_width)
        self._tz.value_changed.connect(self.__changed)
        input_layout.addWidget(self._tz)

        self._dz = inputs.SliderSelectionWidget('Image distance:', 0.1, 10, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=min_lbl_width)
        self._dz.value_changed.connect(self.__changed)
        input_layout.addWidget(self._dz)

        self._border_slider = inputs.SliderSelectionWidget('Border width:', 0, 10, 10, 3,
            value_format_fx=lambda v: inputs.format_int(v, 3) + ' px', min_label_width=min_lbl_width)
        self._border_slider.value_changed.connect(self.__update_controls)
        input_layout.addWidget(self._border_slider)

        layout_border_color = QHBoxLayout()
        self._checkbox_border = inputs.CheckBoxWidget('Transparent border:', is_checked=True,
            min_label_width=min_lbl_width)
        self._checkbox_border.value_changed.connect(self.__update_controls)
        layout_border_color.addWidget(self._checkbox_border)
        self._color_border = inputs.ColorPickerWidget('Border color:', 
            min_label_width=120, initial_color=(0, 0, 0))
        self._color_border.value_changed.connect(self.__changed)
        layout_border_color.addWidget(self._color_border)
        layout_border_color.addStretch()
        input_layout.addLayout(layout_border_color)
        self._checkbox_border.setEnabled(self._border_slider.value() > 0)
        self._color_border.setEnabled(self._border_slider.value() > 0)

        layout_bg_color = QHBoxLayout()
        self._checkbox_bg = inputs.CheckBoxWidget('Transparent background:', is_checked=True,
            min_label_width=min_lbl_width)
        self._checkbox_bg.value_changed.connect(self.__update_controls)
        layout_bg_color.addWidget(self._checkbox_bg)
        self._color_bg = inputs.ColorPickerWidget('Background color:', 
            min_label_width=120, initial_color=(255, 255, 255))
        self._color_bg.value_changed.connect(self.__changed)
        layout_bg_color.addWidget(self._color_bg)
        layout_bg_color.addStretch()
        input_layout.addLayout(layout_bg_color)

        self._alpha_cb = inputs.CheckBoxWidget('Linear alpha interpolation:',
            is_checked=True, min_label_width=min_lbl_width)
        self._alpha_cb.value_changed.connect(self.__changed)
        self._alpha_cb.setEnabled(self._checkbox_bg.value() or self._checkbox_border.value())
        input_layout.addWidget(self._alpha_cb)

        self._viewer = imgview.ImageViewer()
        self._reset_scale = True

        self._fileio = inspection_widgets.ToolbarFileIOWidget(vertical=True, icon_size=QSize(30, 30))
        self._fileio.fileOpenRequest.connect(self.__load_request)
        self._fileio.fileSaveRequest.connect(self.__save_request)

        ctrl_layout = QHBoxLayout()
        ctrl_layout.addWidget(self._fileio)
        ctrl_layout.addWidget(inputs.VLine())
        ctrl_layout.addLayout(input_layout)

        main_layout = QVBoxLayout()
        main_layout.addLayout(ctrl_layout)
        main_layout.addWidget(inputs.HLine())
        main_layout.addWidget(self._viewer)

        self._main_widget.setLayout(main_layout)
        self.setCentralWidget(self._main_widget)
        self.resize(QSize(1280, 1024))

    def __load_images(self, filenames):
        if filenames is None or len(filenames) == 0:
            return
        images = [imutils.imread(fn) for fn in filenames]
        self.displayImageSequence(images)

    def __changed(self, value):
        ax = self._angle_x.value()
        ay = self._angle_y.value()
        az = self._angle_z.value()
        tx = self._tx.value()
        ty = self._ty.value()
        tz = self._tz.value()
        dz = self._dz.value()

        self._alpha_cb.setEnabled(self._checkbox_bg.value() or self._checkbox_border.value())
        alpha_interp = self._alpha_cb.value() if self._alpha_cb.isEnabled() else False

        border_width = int(self._border_slider.value())
        if border_width > 0:
            border_color = None if self._checkbox_border.value() else self._color_border.value()
            images = [imutils.pad(img, border_width, color=border_color) for img in self._images]
        else:
            images = self._images

        bg_color = (-1, -1, -1) if self._checkbox_bg.value() else (*self._color_bg.value(), 255)
        warped = imvis.render_image_sequence(
            images, rx=ax, ry=ay, rz=az, angles_in_deg=True,
            tx=tx, ty=ty, tz=tz, delta_z=dz, bg_color=bg_color,
            inter_alpha_linear=alpha_interp)
        self._vis_np = warped
        self._viewer.showImage(warped, reset_scale=self._reset_scale)
        self._reset_scale = False

    def __update_controls(self):
        self._color_bg.setEnabled(not self._checkbox_bg.value())
        if self._border_slider.value() > 0:
            self._checkbox_border.setEnabled(True)
            self._color_border.setEnabled(not self._checkbox_border.value())
        else:
            self._checkbox_border.setEnabled(False)
            self._color_border.setEnabled(False)
        self.__changed(None)


    def __load_request(self):
        filenames, _ = QFileDialog.getOpenFileNames(self, "Open image", "",
            'Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
            '', QFileDialog.DontUseNativeDialog)
        if filenames is not None and len(filenames) > 0:
            self.__load_images(filenames)

    def __save_request(self):
        if self._vis_np is None:
            return
        filename, _ = QFileDialog.getSaveFileName(self, "Save as...", "",
            'Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
            '', QFileDialog.DontUseNativeDialog)
        if filename is not None and len(filename) > 0:
            imutils.imsave(filename, self._vis_np)

    def displayImageSequence(self, imgs_np):
        self._images = imgs_np
        self.__changed(None)


def gui():
    # Prepare example data
    rgb = imutils.imread(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..' , '..', 'data' , 'flamingo.jpg'))
    # 3-channel grayscale
    gray_ = imutils.rgb2gray(rgb, is_bgr=False)
    gray = np.dstack((gray_, gray_, gray_))
    # 3 images with highlighted red, green, and blue channels
    def __highlight(img, c):
        res = img.copy()
        for i in range(res.shape[2]):
            if i != c:
                res[:, :, i] = 0
        return res
    channels = [__highlight(rgb, c) for c in range(3)]
    # Exemplary "image sequence"
    images = [rgb, *channels, gray]

    app = QApplication(['Image Sequence Plot'])
    main_widget = DemoApplication()
    main_widget.displayImageSequence(images)
    main_widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    gui()
