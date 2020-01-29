import os
import sys
import cv2
import numpy as np
from vito import imutils
from vito import cam_projections as prj

from iminspect import inputs, imgview, inspection_widgets

from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QVBoxLayout, QHBoxLayout, QFileDialog
from PyQt5.QtCore import QSize


sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'gen'))
import vcp.imvis as imvis


class DemoApplication(QMainWindow):
    def __init__(self):
        super(DemoApplication, self).__init__()
        self._images = None
        self._vis_np = None
        self.__prepare_layout()

    def __prepare_layout(self):
        self._main_widget = QWidget()
        input_layout = QVBoxLayout()

        #FIXME filenameSSS_open!!!
        self._fileselection = inputs.SelectDirEntryWidget('Select files to open:',
            inputs.SelectDirEntryType.FILENAME_OPEN, min_label_width=150,
            filters='Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
            relative_base_path=os.getcwd())
        self._fileselection.value_changed.connect(self.__load_images)
        input_layout.addWidget(self._fileselection)
        input_layout.addWidget(inputs.HLine())

        self._angle_x = inputs.SliderSelectionWidget('Angle x:', -180, 180, 361, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=150)
        self._angle_x.value_changed.connect(self.__changed)
        input_layout.addWidget(self._angle_x)
        self._angle_y = inputs.SliderSelectionWidget('Angle y:', -180, 180, 361, 10,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=150)
        self._angle_y.value_changed.connect(self.__changed)
        input_layout.addWidget(self._angle_y)
        self._angle_z = inputs.SliderSelectionWidget('Angle z:', -180, 180, 361, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=150)
        self._angle_z.value_changed.connect(self.__changed)
        input_layout.addWidget(self._angle_z)


        self._tx = inputs.SliderSelectionWidget('tx:', -10, 10, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._tx.value_changed.connect(self.__changed)
        input_layout.addWidget(self._tx)
        self._ty = inputs.SliderSelectionWidget('ty:', -10, 10, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._ty.value_changed.connect(self.__changed)
        input_layout.addWidget(self._ty)
        self._tz = inputs.SliderSelectionWidget('tz:', 0, 10, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._tz.value_changed.connect(self.__changed)
        input_layout.addWidget(self._tz)

        #https://stackoverflow.com/questions/18257281/qt-color-picker-widget

        self._dz = inputs.SliderSelectionWidget('Delta z:', 0.1, 10, 99, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._dz.value_changed.connect(self.__changed)
        input_layout.addWidget(self._dz)

        self._bgcb = inputs.CheckBoxWidget('Transparent background:', is_checked=False)
        self._bgcb.value_changed.connect(self.__changed)
        input_layout.addWidget(self._bgcb)

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
        warped = imvis.render_image_sequence(
            self._images, rx=ax, ry=ay, rz=az, angles_in_deg=True,
            tx=tx, ty=ty, tz=tz, delta_z=dz, bg_color=(-1, -1, -1) if self._bgcb.get_input() else (255, 255, 255))
        # warped = perspectiveView(self._img_np, ax, ay, az, tx, ty, tz, None if self._bgcb.get_input() else (255, 255, 255))
        self._vis_np = warped
        self._viewer.showImage(warped)

    def __load_request(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Select file", "",
            'Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
            '', QFileDialog.DontUseNativeDialog)
        #FIXME implement!if filename is not None:
            # img = imutils.imread(filename)
            # self.displayImage(img)

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
    images = [imutils.pad(img, 5, color=None) for img in images]
    # images = [imutils.pad(img, 5, color=(0, 0, 200)) for img in images]

    app = QApplication(['Visualize Image Sequence'])
    main_widget = DemoApplication()
    main_widget.displayImageSequence(images)
    #main_widget.displayImageSequence([imutils.pad(img, 5, color=None) for img in [gray1, gray1]])
    main_widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    gui()
