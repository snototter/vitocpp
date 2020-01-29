"""
Interactively adjust camera extrinsics to get a better intuition about perspective transformations.
"""
import os
import sys
from vito import imutils
from iminspect import inputs, imgview, inspection_widgets
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QVBoxLayout, QHBoxLayout, QFileDialog
from PyQt5.QtCore import QSize

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'gen'))
from vcp import imvis


class DemoApplication(QMainWindow):
    def __init__(self):
        super(DemoApplication, self).__init__()
        self._img_np = None
        self._vis_np = None
        self.__prepare_layout()

    def __prepare_layout(self):
        self._main_widget = QWidget()
        input_layout = QVBoxLayout()

        self._angle_x = inputs.SliderSelectionWidget('Camera rotation x:', -180, 180, 180, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=150)
        self._angle_x.value_changed.connect(self.__changed)
        input_layout.addWidget(self._angle_x)
        self._angle_y = inputs.SliderSelectionWidget('Camera rotation y:', -180, 180, 180, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=150)
        self._angle_y.value_changed.connect(self.__changed)
        input_layout.addWidget(self._angle_y)
        self._angle_z = inputs.SliderSelectionWidget('Camera rotation z:', -180, 180, 180, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + '°', min_label_width=150)
        self._angle_z.value_changed.connect(self.__changed)
        input_layout.addWidget(self._angle_z)

        self._tx = inputs.SliderSelectionWidget('Camera translation x:', -15, 15, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._tx.value_changed.connect(self.__changed)
        input_layout.addWidget(self._tx)
        self._ty = inputs.SliderSelectionWidget('Camera translation y:', -15, 15, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._ty.value_changed.connect(self.__changed)
        input_layout.addWidget(self._ty)
        self._tz = inputs.SliderSelectionWidget('Camera translation z:', 0, 15, 100, 0,
            value_format_fx=lambda v: inputs.format_float(v, after_comma=1), min_label_width=150)
        self._tz.value_changed.connect(self.__changed)
        input_layout.addWidget(self._tz)

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
        self.resize(QSize(1280, 720))

    def __load_request(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Select file", "",
            'Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
            '', QFileDialog.DontUseNativeDialog)
        if filename is not None:
            img = imutils.imread(filename)
            self.displayImage(img)

    def __save_request(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Select file", "",
            'Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
            '', QFileDialog.DontUseNativeDialog)
        if filename is not None:
            imutils.imsave(filename, self._vis_np)

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
        self._vis_np = warped
        self._viewer.showImage(warped)

    def displayImage(self, img_np):
        self._img_np = img_np
        self._vis_np = img_np
        self._viewer.showImage(img_np)


def gui():
    img = imutils.imread(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'data', 'flamingo.jpg'))
    app = QApplication(['Virtual Camera'])
    main_widget = DemoApplication()
    main_widget.displayImage(img)
    main_widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    gui()
