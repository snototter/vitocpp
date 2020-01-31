"""
Interactively adjust camera extrinsics to get a better intuition about perspective transformations.
"""
import os
import sys
from iminspect import inputs, imgview, inspection_widgets
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QVBoxLayout, QHBoxLayout, \
    QFileDialog, QShortcut
from PyQt5.QtCore import QSize
from PyQt5.QtGui import QKeySequence

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'gen'))
from vcp import imvis, imutils


class DemoApplication(QMainWindow):
    def __init__(self):
        super(DemoApplication, self).__init__()
        self._img_np = None
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
        min_lbl_width = 165

        self._slider_num_pyramid_levels = inputs.SliderSelectionWidget(
            'Pyramid levels:', 1, 10, 9, 3,
            value_format_fx=lambda v: inputs.format_int(v, 4),
            min_label_width=min_lbl_width)
        self._slider_num_pyramid_levels.value_changed.connect(self.__changed)
        input_layout.addWidget(self._slider_num_pyramid_levels)

        self._slider_num_bilateral_filters = inputs.SliderSelectionWidget(
            'Bilateral filters:', 1, 10, 9, 4,
            value_format_fx=lambda v: inputs.format_int(v, 4),
            min_label_width=min_lbl_width)
        self._slider_num_bilateral_filters.value_changed.connect(self.__changed)
        input_layout.addWidget(self._slider_num_bilateral_filters)

        self._slider_diameter_pixel_neighborhood = inputs.SliderSelectionWidget(
            'Pixel neighborhood:', 1, 41, 20, 7,
            value_format_fx=lambda v: inputs.format_int(v, 4) + ' px',
            min_label_width=min_lbl_width)
        self._slider_diameter_pixel_neighborhood.value_changed.connect(self.__changed)
        input_layout.addWidget(self._slider_diameter_pixel_neighborhood)

        self._slider_sigma_color = inputs.SliderSelectionWidget(
            'Sigma color:', 1, 41, 20, 9,
            value_format_fx=lambda v: inputs.format_float(v, digits=3, after_comma=1),
            min_label_width=min_lbl_width)
        self._slider_sigma_color.value_changed.connect(self.__changed)
        input_layout.addWidget(self._slider_sigma_color)

        self._slider_sigma_space = inputs.SliderSelectionWidget(
            'Sigma space:', 1, 41, 20, 7,
            value_format_fx=lambda v: inputs.format_float(v, digits=3, after_comma=1),
            min_label_width=min_lbl_width)
        self._slider_sigma_space.value_changed.connect(self.__changed)
        input_layout.addWidget(self._slider_sigma_space)
        
        self._slider_kernel_size_median = inputs.SliderSelectionWidget('Median kernel size:', 3, 43, 20, 7,
            value_format_fx=lambda v: inputs.format_int(v, 4) + ' px',
            min_label_width=min_lbl_width)
        self._slider_kernel_size_median.value_changed.connect(self.__changed)
        input_layout.addWidget(self._slider_kernel_size_median)
        
        self._slider_edge_block_size = inputs.SliderSelectionWidget('Edge block size:', 1, 63, 31, 11,
            value_format_fx=lambda v: inputs.format_int(v if v > 1 else 0, 4) + ' px',
            min_label_width=min_lbl_width)
        self._slider_edge_block_size.value_changed.connect(self.__changed)
        input_layout.addWidget(self._slider_edge_block_size)

        self._slider_px = inputs.SliderSelectionWidget('Pixelation block size:', 0, 50, 10, 0,
            value_format_fx=lambda v: inputs.format_int(v, 4) + ' px',
            min_label_width=min_lbl_width)
        self._slider_px.value_changed.connect(self.__changed)
        input_layout.addWidget(self._slider_px)
        # self._adjust_prj_cb = inputs.CheckBoxWidget('Adjust camera matrix:', is_checked=True)
        # self._adjust_prj_cb.value_changed.connect(self.__changed)
        # input_layout.addWidget(self._adjust_prj_cb)

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

    def __load_request(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Open image", "",
            'Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
            '', QFileDialog.DontUseNativeDialog)
        if filename is not None:
            img = imutils.imread(filename)
            self.displayImage(img)

    def __save_request(self):
        if self._vis_np is None:
            return
        filename, _ = QFileDialog.getSaveFileName(self, "Save as...", "",
            'Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
            '', QFileDialog.DontUseNativeDialog)
        if filename is not None:
            imutils.imsave(filename, self._vis_np)

    def __changed(self, value):
        if self._img_np is None:
            return
        num_pyramid_levels = int(self._slider_num_pyramid_levels.value())
        num_bilateral_filters = int(self._slider_num_bilateral_filters.value())
        diameter_pixel_neighborhood = int(self._slider_diameter_pixel_neighborhood.value())
        sigma_color = self._slider_sigma_color.value()
        sigma_space = self._slider_sigma_space.value()
        kernel_size_median = int(self._slider_kernel_size_median.value())
        edge_block_size = int(self._slider_edge_block_size.value())
        vis = imutils.cartoonify(self._img_np,
            num_pyramid_levels=num_pyramid_levels,
            num_bilateral_filters=num_bilateral_filters,
            diameter_pixel_neighborhood=diameter_pixel_neighborhood,
            sigma_color=sigma_color,
            sigma_space=sigma_space,
            kernel_size_median=kernel_size_median,
            edge_block_size=edge_block_size,
            is_rgb=True)
        if self._slider_px.value() > 0:
            vis = imutils.pixelate(vis, int(self._slider_px.value()))
        self._vis_np = vis
        self._viewer.showImage(vis, reset_scale=self._reset_scale)
        self._reset_scale = False

    def displayImage(self, img_np):
        self._img_np = img_np
        self._vis_np = img_np
        self.__changed(None)


def gui():
    img = imutils.imread(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'data', 'flamingo.jpg'))
    app = QApplication(['Cartoonification'])
    main_widget = DemoApplication()
    main_widget.displayImage(img)
    main_widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    gui()
