"""
Simple GUI to extrinsically calibrate a stream
"""
import os
import sys
import math
import threading
import numpy as np
import time
from vito import imutils
from iminspect import inputs, imgview, inspection_widgets
# from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QVBoxLayout, QHBoxLayout, \
#     QFileDialog, QShortcut, QDockWidget, QGridLayout
# from PyQt5.QtCore import QSize, QObject
# from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'gen'))
from vcp import imvis
from vcp import imutils
from vcp import best
from vcp import colormaps

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from calibration_utils import ExtrinsicsAprilTag


class Streamer(QThread):
    newFrameset = pyqtSignal(list)

    def __init__(self, folder, cfg_file):
        super(Streamer, self).__init__()
        self._cfg_file = cfg_file
        self._folder = folder
        # self._frame_callback = None
        self._capture = None
        self._keep_streaming = False
        self._timeout_dev_open = 10000
        self._timeout_frameset_wait = 1000
        self._setupStreams()
    
    def numStreams(self):
        return self._capture.num_streams()
    
    def streamLabels(self):
        return self._capture.frame_labels()
    
    def displayLabels(self):
        return [self._capture.frame_label(idx) + (' [Undist. & Rect.]' if self._capture.is_rectified(idx) else '')
            for idx in range(self.numStreams())]
    
    def getCapture(self):
        return self._capture

    def _setupStreams(self):
        print('\n\n\nLoading streaming configuration: {}'.format(self._cfg_file))
        self._capture = best.Capture()
        self._capture.load_libconfig(os.path.join(self._folder, self._cfg_file), rel_path_base_dir=self._folder)

        print('\nStarting to stream {} sinks from {} devices'.format(self._capture.num_streams(), self._capture.num_devices()))
        print('The configured capture yields the following streams:')
        print('  Labels:             {}'.format(self._capture.frame_labels()))
        print('  Configuration keys: {}'.format(self._capture.configuration_keys()))
        print('  Frame types:        {}\n'.format(self._capture.frame_types()))

        if not self._capture.open():
            raise RuntimeError('Cannot open devices')
        if not self._capture.start():
            raise RuntimeError('Cannot start streams')

        # Some cameras (especially our tested RGBD sensors) take quite long to provide the 
        # initial frameset, so it's recommended to wait a bit longer for the device to finish
        # initialization.
        if not self._capture.wait_for_frames(self._timeout_dev_open):
            raise RuntimeError("Didn't receive an initial frameset within {:d} seconds".format(self._timeout_dev_open // 1000))

        # Depending on your setup, some devices may return empty frames (e.g. not synchronized RealSense streams),
        # so we'll wait for the first "complete" frameset.
        while True:
            if not self._capture.wait_for_frames(1000.0):
                print('[WARNING]: wait_for_frames timed out')
                continue
            frames = self._capture.next()
            if any([f is None for f in frames]):
                print('[WARNING]: Skipping invalid frameset')
            else:
                break
    
    def startStream(self):
        if self._keep_streaming:
            print('[ERROR]: Streaming thread already running!')
            return
        self._keep_streaming = True
        self.start()
        # self._streaming_thread = threading.Thread(target=self._streaming_loop)
        # self._streaming_thread.start()

    def stopStream(self):
        self._keep_streaming = False
        # if self._streaming_thread:
        #     self._streaming_thread.join()
        #     self._streaming_thread = None
    
    def run(self):
        while self._keep_streaming and self._capture.all_devices_available():
            if not self._capture.wait_for_frames(self._timeout_frameset_wait):
                print('[WARNING]: wait_for_frames timed out')
                continue

            # Query the frames (since we know that all streams are available now)
            frames = self._capture.next()
            if any([f is None for f in frames]):
                print('[WARNING]: Skipping invalid frameset')
                continue

            # Colorize depth/infrared images for visualization
            def _col_rgb(f):
                return f #imutils.transform(f, 'histeq')

            def _col_depth(f):
                return imutils.transform(f, 'depth2surfnorm', 'surfnorm2rgb')
                #return imvis.pseudocolor(f, limits=[0, 5000], color_map=colormaps.colormap_turbo_rgb)

            def _col_ir(f):
                # RealSense infrared is provided (by default) in Y8 format
                if f.dtype == np.uint8:
                    return f
                return imvis.pseudocolor(f, limits=None, color_map=colormaps.colormap_turbo_rgb)

            vis_frames = [
                _col_depth(frames[idx])
                    if self._capture.is_depth(idx)
                    else (_col_ir(frames[idx])
                        if self._capture.is_infrared(idx)
                        else _col_rgb(frames[idx]))
                for idx in range(len(frames))]
            # vis_frames = [frames[0] for i in range(len(frames))]
            
            # # Resize
            # vis_frames = [imutils.fuzzy_resize(f, 0.75) for f in vis_frames]

            # # Overlay stream labels
            # if self._overlay_labels:
            #     vis_frames = [
            #         imvis.draw_text_box(vis_frames[idx], 
            #             self._capture.frame_label(idx) + (' [Undist. & Rect.]' if self._capture.is_rectified(idx) else ''),
            #             (vis_frames[idx].shape[1]//2, 10), 'north',
            #             bg_color=(0, 0, 0), font_color=(-1, -1, -1),
            #             font_scale=1.0, font_thickness=1,
            #             padding=5, fill_opacity=0.5)
            #         for idx in range(len(vis_frames))]

            self.newFrameset.emit(vis_frames)
            time.sleep(0.05)

        # Shut down gracefully (would be called upon desctruction anyways)
        if not self._capture.stop():
            raise RuntimeError('Cannot stop streams')
        if not self._capture.close():
            raise RuntimeError('Cannot close devices')


class StreamViewer(QWidget):
    streamVisibilityToggled = pyqtSignal(int, bool)  # Emitted whenever the user enables/disables this viewer

    def __init__(self, stream_idx, stream_label, parent=None):
        super(StreamViewer, self).__init__(parent)
        self._stream_idx = stream_idx
        font = QFont('Helvetica', 12, QFont.Bold)
        
        self._stream_label = QLabel('' if stream_label is None else stream_label, self)
        self._stream_label.setAlignment(Qt.AlignCenter)
        self._stream_label.setStyleSheet("QLabel {background-color: blue;}")
        self._stream_label.setFont(font)
        self._stream_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self._checkbox_active = QCheckBox('Active')
        self._checkbox_active.setChecked(True)
        self._checkbox_active.toggled.connect(self.updateVisibility)

        self._viewer = imgview.ImageViewer()

        lbl_layout = QHBoxLayout()
        lbl_layout.addWidget(self._stream_label)
        lbl_layout.addWidget(self._checkbox_active)

        layout = QVBoxLayout()
        layout.addLayout(lbl_layout)
        #TODO add calibration info label (rotation change, translation change, found tags, etc.)
        layout.addWidget(self._viewer)
        self.setLayout(layout)
    
    def updateVisibility(self):
        show = self._checkbox_active.isChecked()
        self._viewer.setVisible(show)
        self.streamVisibilityToggled.emit(self._stream_idx, show)
    
    def isActive(self):
        return self._checkbox_active.isChecked()

    def streamIndex(self):
        return self._stream_idx
    
    def setStreamLabel(self, label):
        self._stream_label.setText(label)
        self.update()
        #TODO set calibration

    def streamLabel(self):
        return self._stream_label.text()
    
    def showImage(self, img_np, reset_scale=True):
        # Only show image if this stream is enabled
        if self._checkbox_active.isChecked():
            self._viewer.showImage(img_np, reset_scale=reset_scale)
    
    def scaleToFitWindow(self):
        self._viewer.scaleToFitWindow()
    
    

class CalibApplication(QMainWindow):
    def __init__(self, streamer):
        super(CalibApplication, self).__init__()
        # self._img_np = None
        # self._vis_np = None
        self._streamer = streamer
        self._num_streams = streamer.numStreams()
        self._stream_display_labels = streamer.displayLabels()
        self._resize_viewers = True

        self._extrinsics_estimator = ExtrinsicsAprilTag(streamer.getCapture(), grid_limits=None)
        self.__prepare_layout()
        streamer.newFrameset.connect(self.displayFrameset)
        streamer.startStream()

        # # self.show()
        # # streamer.start_stream()
        # self.__prepare_shortcuts()

    # def __prepare_shortcuts(self):
    #     # Open file
    #     sc = QShortcut(QKeySequence('Ctrl+O'), self)
    #     sc.activated.connect(self.__load_request)
    #     # Save file
    #     sc = QShortcut(QKeySequence('Ctrl+S'), self)
    #     sc.activated.connect(self.__save_request)

    def rescale_viewers(self):
        self._resize_viewers = True

    def fit_viewers(self):
        for v in self._viewers:
            v['widget'].scaleToFitWindow()

    def __prepare_layout(self):
        self._num_viewer_rows = 2 if self._num_streams < 9 else 3 # FIXME hardcoded :-(
        self._num_viewer_columns = math.ceil(self._num_streams / self._num_viewer_rows)
        
        self._main_widget = QWidget()
        input_layout = QHBoxLayout()
        # min_lbl_width = 165

        btn = QPushButton('Original Image Size')
        btn.clicked.connect(self.rescale_viewers)
        input_layout.addWidget(btn)

        btn = QPushButton('Scale Images to Fit')
        btn.clicked.connect(self.fit_viewers)
        input_layout.addWidget(btn)

        #TODO options to overlay world coordinates, etc

        self._viewer_layout = QGridLayout()
        self._viewers = list()
        for i in range(self._num_streams):
            row = i // self._num_viewer_columns
            col = i % self._num_viewer_columns
            viewer = StreamViewer(i, self._stream_display_labels[i])
            viewer.streamVisibilityToggled.connect(self.streamVisibilityToggled)
            self._viewer_layout.addWidget(viewer, row, col)
            self._viewers.append({'widget': viewer, 'on-active': True})
        #TODO add grid/scroll area (?) at the bottom which holds all inactive/"hidden" streams
        # needed, because we want to activate them again sooner or later
        # Scroll area for inactive widgets
        # * layout
        self._inactive_scroll_layout = QHBoxLayout()
        # * widget (scroll area content)
        self._inactive_scroll_widget = QWidget()
        self._inactive_scroll_widget.setLayout(self._inactive_scroll_layout)
        # * scroll area
        self._inactive_scroll_area = QScrollArea()
        self._inactive_scroll_area.setWidgetResizable(True)
        self._inactive_scroll_area.setWidget(self._inactive_scroll_widget)
        self._inactive_scroll_area.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        main_layout = QVBoxLayout()
        main_layout.addLayout(input_layout)
        main_layout.addLayout(self._viewer_layout)
        main_layout.addWidget(self._inactive_scroll_area)
        
        self._main_widget.setLayout(main_layout)
        self.setCentralWidget(self._main_widget)
        self.resize(QSize(1280, 720))

    # def __load_request(self):
    #     filename, _ = QFileDialog.getOpenFileName(self, "Open image", "",
    #         'Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
    #         '', QFileDialog.DontUseNativeDialog)
    #     if filename is not None:
    #         img = imutils.imread(filename)
    #         self.displayImage(img)

#TODO enable saving
    # def __save_request(self):
    #     if self._vis_np is None:
    #         return
    #     filename, _ = QFileDialog.getSaveFileName(self, "Save as...", "",
    #         'Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
    #         '', QFileDialog.DontUseNativeDialog)
    #     if filename is not None:
    #         imutils.imsave(filename, self._vis_np)
  
    def displayFrameset(self, frames):
        assert len(frames) == len(self._viewers)
        for i in range(len(frames)):
            if frames[i] is not None:
                self._viewers[i]['widget'].showImage(frames[i], reset_scale=self._resize_viewers)
        self._resize_viewers = False
    
    def streamVisibilityToggled(self, stream_index, active):
        if active:
            print("Stream '{}' becomes active".format(self._viewers[stream_index]['widget'].streamLabel()))
            idx_layout_from = self._inactive_scroll_layout.indexOf(self._viewers[stream_index]['widget'])
            self._inactive_scroll_layout.takeAt(idx_layout_from)
            self._viewer_layout.addWidget(self._viewers[stream_index]['widget'])
            self._viewers[stream_index]['on-active'] = active
        else:
            print("Stream '{}' becomes inactive".format(self._viewers[stream_index]['widget'].streamLabel()))
            layout_from = self._viewer_layout
            idx_layout_from = layout_from.indexOf(self._viewers[stream_index]['widget'])
            # Swap viewers (move the toggled stream "at the end" of the layout)
            swap_idx = idx_layout_from + 1
            while swap_idx < layout_from.count():
                print('TODO swapping ', swap_idx, idx_layout_from)
                self._swapViewerWidgetsOnGrid(self._viewers[stream_index]['widget'], layout_from.itemAt(swap_idx).widget())
                idx_layout_from = swap_idx
                swap_idx += 1
            # Remove from grid
            layout_from.takeAt(idx_layout_from)
            # Add to inactive "list"
            self._inactive_scroll_layout.addWidget(self._viewers[stream_index]['widget'])
            self._viewers[stream_index]['on-active'] = active
    
    def _swapViewerWidgetsOnGrid(self, widget1, widget2):
        # Remember current position
        idx1 = self._viewer_layout.indexOf(widget1)
        row1, col1, row_span1, col_span1 = self._viewer_layout.getItemPosition(idx1)
        idx2 = self._viewer_layout.indexOf(widget2)
        row2, col2, row_span2, col_span2 = self._viewer_layout.getItemPosition(idx2)
        # Remove widgets from layout
        self._viewer_layout.takeAt(idx1)
        self._viewer_layout.takeAt(idx2)
        # Add them to the layout at the correct places
        self._viewer_layout.addWidget(widget2, row1, col1, row_span1, col_span1)
        self._viewer_layout.addWidget(widget1, row2, col2, row_span2, col_span2)
    
    def appAboutToQuit(self):
        self._streamer.stopStream()
        self._streamer.wait()


def gui():
    #TODO add argparse
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'data', 'data-best')
    cfg_file = 'webcam.cfg'#
    cfg_file = 'kinects.cfg'
    cfg_file = 'k4a.cfg'
    streamer = Streamer(folder, cfg_file)

    app = QApplication(['Calibrate Extrinsics'])
    main_widget = CalibApplication(streamer)
    app.aboutToQuit.connect(main_widget.appAboutToQuit)
    main_widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    gui()
    #TODO params:
    # april tag marker size, etc
    # step through capture vs live stream!
