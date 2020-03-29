"""
Simple GUI to extrinsically calibrate a stream
"""
import argparse
import os
import sys
import math
import threading
import numpy as np
import time
from vito import imutils, pyutils
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

    def __init__(self, folder, cfg_file, step_through):
        super(Streamer, self).__init__()
        self._cfg_file = cfg_file
        self._folder = folder
        self._step_through = step_through
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

    def isStepThrough(self):
        return self._step_through
    
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
        if self._step_through:
            return
        while self._keep_streaming and self._capture.all_devices_available():
            vis_frames = self.getNextFrameset()
            if vis_frames is not None:
                self.newFrameset.emit(vis_frames)
            time.sleep(0.05)

        # Shut down gracefully (would be called upon desctruction anyways)
        if not self._capture.stop():
            raise RuntimeError('Cannot stop streams')
        if not self._capture.close():
            raise RuntimeError('Cannot close devices')
    
    def getNextFrameset(self):
        if not self._capture.all_devices_available():
            return None
        if not self._capture.wait_for_frames(self._timeout_frameset_wait):
            print('[WARNING]: wait_for_frames timed out')
            return None
        # Query the frames (since we know that all streams are available now)
        frames = self._capture.next()
        if any([f is None for f in frames]):
            print('[WARNING]: Skipping invalid frameset')
            return None

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
            print('FIXME FIXME FIXME: convert to uint8!')
            return f.astype(np.uint8)
            # return imvis.pseudocolor(f, limits=None, color_map=colormaps.colormap_turbo_rgb)

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
        return vis_frames


class StreamViewer(QWidget):
    streamVisibilityToggled = pyqtSignal(int, bool)  # Emitted whenever the user enables/disables this viewer

    def __init__(self, stream_idx, stream_label, parent=None):
        super(StreamViewer, self).__init__(parent)
        self._stream_idx = stream_idx
        font = QFont('Helvetica', 12, QFont.Bold)
        
        self._stream_label = QLabel('' if stream_label is None else stream_label, self)
        self._stream_label.setAlignment(Qt.AlignCenter)
        # self._stream_label.setStyleSheet("QLabel {background-color: blue;}")
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
    def __init__(self, streamer, args):
        super(CalibApplication, self).__init__()
        # self._img_np = None
        # self._vis_np = None
        self._streamer = streamer
        self._num_streams = streamer.numStreams()
        self._stream_display_labels = streamer.displayLabels()
        self._resize_viewers = True
        self._args = args

        self._extrinsics_estimator = ExtrinsicsAprilTag(streamer.getCapture(), 
            args.tag_family, args.tag_size_mm, 
            grid_limits=None,
            pose_history_length=args.pose_history_length,
            pose_threshold_rotation=args.pose_threshold_rotation,
            pose_threshold_translation=args.pose_threshold_translation)

        self.prepareLayout()
        streamer.newFrameset.connect(self.displayFrameset)
        streamer.startStream()

        # If the stream should be stepped through, load the first frameset now:
        if streamer.isStepThrough():
            self.loadNextFrameset()

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

    def rescaleViewers(self):
        self._resize_viewers = True

    def fitViewers(self):
        for v in self._viewers:
            v['widget'].scaleToFitWindow()
    
    def loadNextFrameset(self):
        frames = self._streamer.getNextFrameset()
        if frames is None:
            print('Received empty frameset')
            self._btn_next.setEnabled(False)
        else:
            self.displayFrameset(frames)

    def prepareLayout(self):
        self._num_viewer_rows = 2 if self._num_streams < 9 else 3 # FIXME hardcoded :-(
        self._num_viewer_columns = math.ceil(self._num_streams / self._num_viewer_rows)
        
        self._main_widget = QWidget()
        input_layout = QHBoxLayout()
        # min_lbl_width = 165

        btn = QPushButton('Original Image Size')
        btn.clicked.connect(self.rescaleViewers)
        input_layout.addWidget(btn)

        btn = QPushButton('Scale Images to Fit')
        btn.clicked.connect(self.fitViewers)
        input_layout.addWidget(btn)

        if self._streamer.isStepThrough():
            self._btn_next = QPushButton('Next Frameset')
            self._btn_next.clicked.connect(self.loadNextFrameset)
            input_layout.addWidget(self._btn_next)

        #TODO options to overlay world coordinates, etc

        self._active_viewer_layout = QGridLayout()
        self._viewers = list()
        for i in range(self._num_streams):
            row = i // self._num_viewer_columns
            col = i % self._num_viewer_columns
            viewer = StreamViewer(i, self._stream_display_labels[i], parent=self)
            viewer.streamVisibilityToggled.connect(self.streamVisibilityToggled)
            self._active_viewer_layout.addWidget(viewer, row, col, 1, 1)
            self._viewers.append({'widget': viewer, 'on-active': True})
        #TODO add grid/scroll area (?) at the bottom which holds all inactive/"hidden" streams
        # needed, because we want to activate them again sooner or later
        self._active_viewer_widget = QWidget()
        self._active_viewer_widget.setLayout(self._active_viewer_layout)
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
        main_layout.addWidget(self._active_viewer_widget)
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

        # TODO enqueue??
        self._extrinsics_estimator.process_frameset(frames)
    
    def streamVisibilityToggled(self, stream_index, active):
        if active:
            print("Stream '{}' becomes active".format(self._viewers[stream_index]['widget'].streamLabel()))
            # Remove viewer widget from inactive list
            idx_layout_from = self._inactive_scroll_layout.indexOf(self._viewers[stream_index]['widget'])
            self._inactive_scroll_layout.takeAt(idx_layout_from)

        # Remove all viewer widgets from grid
        for widx in range(self._active_viewer_layout.count()-1, -1, -1):
            self._active_viewer_layout.takeAt(widx)
        # Re-insert only active viewers
        active_stream_indices = [idx for idx in range(self._num_streams) if self._viewers[idx]['widget'].isActive()]
        grid_idx = 0
        for sidx in active_stream_indices:
            row = grid_idx // self._num_viewer_columns
            col = grid_idx % self._num_viewer_columns
            self._active_viewer_layout.addWidget(self._viewers[sidx]['widget'], row, col, 1, 1)
            grid_idx += 1

        # Update status so we know, where it is placed currently:
        self._viewers[stream_index]['on-active'] = active

        # Add toggled viewer to "list" of inactive viewers (if it became inactive)
        if not active:
            print("Stream '{}' becomes inactive".format(self._viewers[stream_index]['widget'].streamLabel()))
            self._inactive_scroll_layout.addWidget(self._viewers[stream_index]['widget'])

        # if active:
        #     # Compute new position
        #     row = self._active_viewer_layout.count() // self._num_viewer_columns
        #     col = self._active_viewer_layout.count() % self._num_viewer_columns
        #     # Append to grid layout
        #     self._active_viewer_layout.addWidget(self._viewers[stream_index]['widget'], row, col, 1, 1)
        
            
    
    # def _swapViewerWidgetsOnGrid(self, widget1, widget2):
    #     print('FIXME: swapping {} with {}'.format(widget1.streamLabel(), widget2.streamLabel()))
    #     # Remember current position
    #     idx1 = self._viewer_layout.indexOf(widget1)
    #     row1, col1, row_span1, col_span1 = self._viewer_layout.getItemPosition(idx1)
    #     idx2 = self._viewer_layout.indexOf(widget2)
    #     row2, col2, row_span2, col_span2 = self._viewer_layout.getItemPosition(idx2)
    #     # Remove widgets from layout
    #     self._viewer_layout.takeAt(idx1)
    #     self._viewer_layout.takeAt(idx2)
    #     # Add them to the layout at the correct places
    #     self._viewer_layout.addWidget(widget2, row1, col1, row_span1, col_span1)
    #     self._viewer_layout.addWidget(widget1, row2, col2, row_span2, col_span2)
    
    def appAboutToQuit(self):
        self._streamer.stopStream()
        self._streamer.wait()


def parseArguments():
    parser = argparse.ArgumentParser()
    # parser.add_argument('config_file', action='store',
    #     help='Path to the stream configuration file (json or libconfig)')

    # AprilTag parameters
    parser.add_argument('tag_size_mm', action='store', type=pyutils.check_positive_real, #default=225.0,
        help='Size of the AprilTag in mm (the 8x8 square, not the 10x10)')
    parser.add_argument('--tag_family', action='store', help='AprilTag family', default='tag36h11')

    # Pose stability checks
    parser.add_argument('--pose-history-length', action='store', type=pyutils.check_positive_int, default=10,
        help='How many previous tag poses should be used to reason about stability?')
    parser.add_argument('--pose-threshold-rotation', action='store', type=pyutils.check_positive_real, default=0.1,
        help='Max. allowed angular deviation (in degrees) of a pose before considering a tag unstable.')
    parser.add_argument('--pose-threshold-translation', action='store', type=pyutils.check_positive_real, default=2.0,
        help='Max. allowed translation change (in mm) of a pose before considering a tag unstable')

    # # General visualization params
    # parser.add_argument('--overlay', action='store_true', dest='label_overlay',
    #     help='Enable camera label overlay')
    # parser.add_argument('--no-overlay', action='store_false', dest='label_overlay',
    #     help='Disable camera label overlay')
    # parser.set_defaults(label_overlay=True)
    # parser.add_argument('--paused', action='store_true', dest='start_paused',
    #     help='Wait for user interaction after each frame')
    # parser.add_argument('--no-paused', action='store_false', dest='start_paused',
    #     help="Don't wait for user interaction after each frame")
    # parser.set_defaults(start_paused=True)

    # # Params to visualize the world coordinate system
    # parser.add_argument('--world', action='store_true', dest='world_coords',
    #     help='Visualize world coordinate system after detecting markers.')
    # parser.add_argument('--no-world', action='store_false', dest='world_coords',
    #     help="Don't visualize the world coordinate system.")
    # parser.set_defaults(world_coords=True)
    # parser.add_argument('--axis-length', action='store', type=pyutils.check_positive_real, default=1000.0,
    #     help="Length of each axes (arrows in [mm]), when running with --world")
    # parser.add_argument('--grid-spacing', action='store', type=pyutils.check_positive_real, default=500.0,
    #     help="Size of each grid cell (in [mm]) when running with --world")
    # parser.add_argument('--grid-limits', action='store', nargs=4, type=float, default=[-1e4, -1e4, 1e4, 1e4],
    #     help="Limit the ground plane grid visualization to the region given by [x_min, y_min, x_max, y_max]")

    parser.add_argument('--step-through', action='store_true', dest='step_through',
        help="Step through recording manually (instead of live streaming)")
    parser.set_defaults(step_through=False)

    args = parser.parse_args()
    #FIXME remove:
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'data', 'data-best')
    cfg_file = 'webcam.cfg'#
    cfg_file = 'kinects.cfg'
    cfg_file = 'k4a.cfg'
    # cfg_file = 'image_sequence.cfg'
    args.config_file = os.path.join(folder, cfg_file)
    return args


def gui():
    args = parseArguments()

    abs_cfg = os.path.abspath(args.config_file)
    folder = os.path.dirname(abs_cfg)
    cfg_file = os.path.basename(args.config_file)
    streamer = Streamer(folder, cfg_file, args.step_through)
    # intrinsics = [streamer.getCapture().intrinsics(i) for i in range(streamer.getCapture().numStreams())]

    app = QApplication(['Calibrate Extrinsics'])
    main_widget = CalibApplication(streamer, args)
    app.aboutToQuit.connect(main_widget.appAboutToQuit)
    main_widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    gui()
    #TODO params:
    # april tag marker size, etc
    
    # step through capture vs live stream!
