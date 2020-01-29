import os
import sys
import numpy as np
from vito import imutils

from iminspect import inputs, imgview, inspection_widgets

from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QVBoxLayout, QHBoxLayout, QFileDialog
from PyQt5.QtCore import QSize

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'gen'))
from vcp import imvis


class DemoApplication(QMainWindow):
    def __init__(self):
        super(DemoApplication, self).__init__()
        self._images = None
        self._vis_np = None
        self.__prepare_layout()

    def __prepare_layout(self):
        self._main_widget = QWidget()
        input_layout = QVBoxLayout()

        self._angle_x = inputs.SliderSelectionWidget('Camera rotation x:', -180, 180, 180, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=150)
        self._angle_x.value_changed.connect(self.__changed)
        input_layout.addWidget(self._angle_x)
        self._angle_y = inputs.SliderSelectionWidget('Camera rotation y:', -180, 180, 180, 10,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=150)
        self._angle_y.value_changed.connect(self.__changed)
        input_layout.addWidget(self._angle_y)
        self._angle_z = inputs.SliderSelectionWidget('Camera rotation z:', -180, 180, 180, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=150)
        self._angle_z.value_changed.connect(self.__changed)
        input_layout.addWidget(self._angle_z)


        self._tx = inputs.SliderSelectionWidget('Camera translation x:', -20, 20, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._tx.value_changed.connect(self.__changed)
        input_layout.addWidget(self._tx)
        self._ty = inputs.SliderSelectionWidget('Camera translation y:', -20, 20, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._ty.value_changed.connect(self.__changed)
        input_layout.addWidget(self._ty)
        self._tz = inputs.SliderSelectionWidget('Camera translation z:', -5, 15, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._tz.value_changed.connect(self.__changed)
        input_layout.addWidget(self._tz)

        self._dz = inputs.SliderSelectionWidget('Image distance:', 0.1, 10, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._dz.value_changed.connect(self.__changed)
        input_layout.addWidget(self._dz)

        
        #TODO border slider + color vs transparent
        #https://stackoverflow.com/questions/18257281/qt-color-picker-widget

        self._border_slider = inputs.SliderSelectionWidget('Border width:', 0, 10, 10, 0,
            value_format_fx=lambda v: inputs.format_int(v, 3) + ' px', min_label_width=150)
        self._border_slider.value_changed.connect(self.__update_controls)
        input_layout.addWidget(self._border_slider)

        layout_border_color = QHBoxLayout()
        self._checkbox_border = inputs.CheckBoxWidget('Transparent border:', is_checked=False,
            min_label_width=150)
        self._checkbox_border.value_changed.connect(self.__update_controls)
        layout_border_color.addWidget(self._checkbox_border)
        self._color_border = inputs.ColorPickerWidget('Border color:', 
            min_label_width=120, initial_color=(0, 0, 0))
        self._color_border.value_changed.connect(self.__changed)
        layout_border_color.addWidget(self._color_border)
        layout_border_color.addStretch()
        input_layout.addLayout(layout_border_color)
        self._checkbox_border.setEnabled(False)
        self._color_border.setEnabled(False)

        layout_bg_color = QHBoxLayout()
        self._checkbox_bg = inputs.CheckBoxWidget('Transparent background:', is_checked=False,
            min_label_width=150)
        self._checkbox_bg.value_changed.connect(self.__update_controls)
        layout_bg_color.addWidget(self._checkbox_bg)
        self._color_bg = inputs.ColorPickerWidget('Background color:', 
            min_label_width=120, initial_color=(255, 255, 255))
        self._color_bg.value_changed.connect(self.__changed)
        layout_bg_color.addWidget(self._color_bg)
        layout_bg_color.addStretch()
        input_layout.addLayout(layout_bg_color)

        self._viewer = imgview.ImageViewer()

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
        self.resize(QSize(640, 480))

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
        images = [imutils.pad(img, 3, color=None) for img in self._images] #TODO border slider
        warped = imvis.render_image_sequence(
            images, rx=ax, ry=ay, rz=az, angles_in_deg=True,#FIXME use colorbg
            tx=tx, ty=ty, tz=tz, delta_z=dz, bg_color=(-1, -1, -1) if self._checkbox_bg.get_input() else (255, 255, 255))
        self._vis_np = warped
        self._viewer.showImage(warped)

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
        filenames, _ = QFileDialog.getOpenFileNames(self, "Select file", "",
            'Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
            '', QFileDialog.DontUseNativeDialog)
        if filenames is not None and len(filenames) > 0:
            self.__load_images(filenames)

    def __save_request(self):
        if self._vis_np is None:
            return
        filename, _ = QFileDialog.getSaveFileName(self, "Select file", "",
            'Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
            '', QFileDialog.DontUseNativeDialog)
        if filename is not None:
            imutils.imsave(filename, self._vis_np)

    def displayImageSequence(self, imgs_np):
        self._images = imgs_np
        self.__changed(None)


def gui():
    rgb = imutils.imread(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..' , '..', 'data' , 'flamingo.jpg'))

    bgr = imutils.flip_layers(rgb)
    gray1 = imutils.rgb2gray(rgb, is_bgr=False)
    gray = np.dstack((gray1, gray1, gray1))
    images = [rgb, gray, bgr, gray]
    # Add (transparent) border
    #TODO make transparent again
    #FIXME rgb => widget; upon change (if transparent, add border) otherwise....... TODO extend GUI (save + colorize border, slider, range values, etc.)
    # images = [imutils.pad(img, 5, color=None) for img in images]
    # images = [imutils.pad(img, 5, color=(0, 0, 200)) for img in images]

    app = QApplication(['Visualize Image Sequence'])
    main_widget = DemoApplication()
    main_widget.displayImageSequence(images)
    #main_widget.displayImageSequence([imutils.pad(img, 5, color=None) for img in [gray1, gray1]])
    main_widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    gui()
