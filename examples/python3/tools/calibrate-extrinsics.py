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

        # Post-process images
        def _prepare_rgb(f):
            return f #imutils.transform(f, 'histeq')

        def _prepare_depth(f):
            #return imutils.transform(f, 'depth2surfnorm', 'surfnorm2rgb')
            return imvis.pseudocolor(f, limits=[0, 5000], color_map=colormaps.colormap_turbo_rgb)

        def _prepare_ir(f):
            # Convert to uint8
            if f.dtype != np.uint8:
                f = (f.astype(np.float32) / np.max(f) * 255).astype(np.uint8)
            return imutils.transform(f, 'histeq')
            # return imvis.pseudocolor(f, limits=None, color_map=colormaps.colormap_turbo_rgb)

        processed_frames = [
            _prepare_depth(frames[idx])
                if self._capture.is_depth(idx)
                else (_prepare_ir(frames[idx])
                    if self._capture.is_infrared(idx)
                    else _prepare_rgb(frames[idx]))
            for idx in range(len(frames))]
        return processed_frames


class StreamViewer(QFrame):
    streamVisibilityToggled = pyqtSignal(int, bool)  # Emitted whenever the user enables/disables this viewer

    def __init__(self, stream_idx, stream_label, parent=None):
        super(StreamViewer, self).__init__(parent)
        self._stream_idx = stream_idx
        font_stream_label = QFont('Helvetica', 10, QFont.Bold)
        font_error_label = QFont('Monospace', 10)
        font_error_label.setStyleHint(QFont.TypeWriter)
        
        # Show the stream's label
        self._stream_label = QLabel('' if stream_label is None else stream_label, self)
        self._stream_label.setAlignment(Qt.AlignCenter)
        # self._stream_label.setStyleSheet("QLabel {background-color: blue;}")
        self._stream_label.setFont(font_stream_label)
        self._stream_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # A checkbox to toggle the display of this stream
        self._checkbox_active = QCheckBox('Active')
        self._checkbox_active.setChecked(True)
        self._checkbox_active.toggled.connect(self.updateVisibility)

        stream_lbl_layout = QHBoxLayout()
        stream_lbl_layout.addWidget(self._stream_label)
        stream_lbl_layout.addWidget(self._checkbox_active)

        # Custom "two-column" display
        self._tag_error_label_left = QLabel('No tag detected')
        self._tag_error_label_left.setAlignment(Qt.AlignLeft)
        self._tag_error_label_left.setFont(font_error_label)
        self._tag_error_label_right = QLabel('')
        self._tag_error_label_right.setAlignment(Qt.AlignLeft)
        self._tag_error_label_right.setFont(font_error_label)

        self._tag_error_divider = inputs.VLine()
        self._tag_error_divider.setVisible(False)

        error_lbl_layout = QHBoxLayout()
        error_lbl_layout.addWidget(self._tag_error_label_left)
        error_lbl_layout.addWidget(self._tag_error_divider)
        error_lbl_layout.addWidget(self._tag_error_label_right)

        # Two rows of text above the actual image content
        lbl_layout = QVBoxLayout()
        lbl_layout.addLayout(stream_lbl_layout)
        lbl_layout.addLayout(error_lbl_layout)

        # Actual image viewer
        self._viewer = imgview.ImageViewer()

        # This widget's "main" layout
        layout = QVBoxLayout()
        layout.addLayout(lbl_layout)
        #TODO add calibration info label (rotation change, translation change, found tags, etc.)
        layout.addWidget(self._viewer)
        self.setLayout(layout)

        # Add a border to this widget
        self.setFrameShape(QFrame.Panel)
    
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
    
    def setTagErrors(self, errors, is_stable):
        if len(errors) > 0:
            error_strings = ['Tag {:2d}: {:5.2f} deg, {:4.1f} mm, {:5.2f} px'.format(
                tag_id, errors[tag_id]['r_deg'], errors[tag_id]['t_mm'], errors[tag_id]['t_px'])
                for tag_id in errors]
            # # Alternate between two columns
            # el = error_strings[::2]
            # er = error_strings[1::2]
            # First half left, second half right
            les = len(error_strings)
            pivot = les // 2 + (les % 2)
            el = error_strings[:pivot]
            er = error_strings[pivot:]
            self._tag_error_label_left.setText('\n'.join(el))
            if len(er) > 0:
                self._tag_error_divider.setVisible(True)
                self._tag_error_label_right.setText('\n'.join(er))
            else:
                self._tag_error_divider.setVisible(False)
                self._tag_error_label_right.setText('')
        else:
            self._tag_error_label_left.setText('No tag detected')
            self._tag_error_label_right.setText('')
            self._tag_error_divider.setVisible(False)
        self._stream_label.setStyleSheet("QLabel {color: " + ('green' if is_stable else 'red') + ";}")

    def streamLabel(self):
        return self._stream_label.text()
    
    def showImage(self, img_np, reset_scale=True):
        # Only show image if this stream is enabled
        if self._checkbox_active.isChecked():
            self._viewer.showImage(img_np, reset_scale=reset_scale)
    
    def scaleToFitWindow(self):
        self._viewer.scaleToFitWindow()
    
class CameraPoseEstimator(QThread):
    processingDone = pyqtSignal(list, list)

    def __init__(self, streamer, args):
        super(CameraPoseEstimator, self).__init__()
        self._streamer = streamer

        # Visualization parameters
        self._draw_world_xyz = args.world_coords
        self._draw_groundplane = self._draw_world_xyz
        self._draw_horizon = self._draw_world_xyz
        self._draw_tags = self._draw_world_xyz
        self._axis_length = args.axis_length
        self._grid_spacing = args.grid_spacing
        self._grid_limits = args.grid_limits
        self._line_width = 3

        # Thread/lock parameters
        self._process_lock = threading.Lock()
        self._frameset_to_process = None
        self._continue_processing = False
        self._cv_frameset_available = QWaitCondition()
        self._mutex = QMutex()

        self._extrinsics_estimator = ExtrinsicsAprilTag(streamer.getCapture(), 
            args.tag_family, args.tag_size_mm, 
            pose_history_length=args.pose_history_length,
            pose_threshold_rotation=args.pose_threshold_rotation,
            pose_threshold_translation=args.pose_threshold_translation)
    
    def setVisualizationParameters(self, draw_world_xyz, draw_groundplane, draw_horizon,
            draw_tags, axis_length, grid_spacing, grid_limits, line_width):
        self._draw_world_xyz = draw_world_xyz
        self._draw_groundplane = draw_groundplane
        self._draw_horizon = draw_horizon
        self._draw_tags = draw_tags
        self._axis_length = axis_length
        self._grid_spacing = grid_spacing
        self._grid_limits = grid_limits
        self._line_width = int(line_width)

    def startProcessing(self):
        self._continue_processing = True
        self.start()

    def stopProcessing(self):
        self._mutex.lock()
        self._continue_processing = False
        self._cv_frameset_available.wakeAll()
        self._mutex.unlock()

    def enqueueFrameset(self, frames):
        # We only keep one frameset in the queue"
        self._mutex.lock()
        if self._frameset_to_process is None:
            self._frameset_to_process = frames
            self._cv_frameset_available.wakeAll()
        # else:
        #     print('Skipping frameset as pose estimator is busy!')
        self._mutex.unlock()

    def run(self):
        while self._continue_processing:
            self._mutex.lock()
            if self._frameset_to_process is None:
                self._cv_frameset_available.wait(self._mutex)
            frames = self._frameset_to_process
            #TODO If pose estimation doesn't take too long (unless you have many high resolution streams),
            # we could clear the frameset here already (so the next incoming frameset would be "enqueued").
            # self._frameset_to_process = None
            self._mutex.unlock()

            if frames is None:
                continue

            # Estimate the camera poses
            estimation_result = self._extrinsics_estimator.process_frameset(frames)
            # # We will update the error display labels with current estimation deltas
            # delta_strings = self._extrinsics_estimator.get_change_strings() #TODO get deltas instead of strings! also get stability, etc
            # Visualize the poses
            vis_frames = self._extrinsics_estimator.visualize_frameset(frames, estimation_result,
                draw_world_xyz=self._draw_world_xyz,
                draw_groundplane=self._draw_groundplane,
                draw_horizon=self._draw_horizon,
                draw_tags=self._draw_tags,
                axis_length=self._axis_length,
                grid_spacing=self._grid_spacing,
                grid_limits=self._grid_limits,
                line_width=self._line_width)
            self.processingDone.emit(vis_frames, estimation_result)
            # # # # [DEV] Dummy signal, just forwarding frames and adding some delay:
            # # # self.processingDone.emit(frames, list(), [list() for f in frames])
            # # # self.msleep(100)

            # Clear the processed frameset, so we can accept new incoming framesets
            self._mutex.lock()
            self._frameset_to_process = None
            self._mutex.unlock()


        

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
        self._filename_save = None
        # self._process_lock = threading.Lock()
        # self._pose_estimation_in_progress = False

        # self._extrinsics_estimator = ExtrinsicsAprilTag(streamer.getCapture(), 
        #     args.tag_family, args.tag_size_mm, 
        #     pose_history_length=args.pose_history_length,
        #     pose_threshold_rotation=args.pose_threshold_rotation,
        #     pose_threshold_translation=args.pose_threshold_translation)
        self._processing_thread = CameraPoseEstimator(streamer, args)
        self._processing_thread.processingDone.connect(self.displayPoseEstimates)
        self._processing_thread.startProcessing()

        self.prepareLayout()
        self.prepareShortcuts()
        streamer.newFrameset.connect(self._processing_thread.enqueueFrameset)
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
            v.scaleToFitWindow()
    
    def loadNextFrameset(self):
        frames = self._streamer.getNextFrameset()
        if frames is None:
            print('Received empty frameset')
            self._btn_next.setEnabled(False)
        else:
            self.displayFrameset(frames)

    def prepareLayout(self):
        # TODO (overkill) we could derive the number of rows/cols from the aspect ratio (but how to handle
        # captures with streams that have different aspect ratios).
        # Currently, keep it simple and straightforward:
        self._num_viewer_rows = 1 if self._num_streams < 3 else \
            (2 if self._num_streams < 9 else\
                (3 if self._num_streams < 12 else 4))
        self._num_viewer_columns = math.ceil(self._num_streams / self._num_viewer_rows)

        min_lbl_width = 120 # Minimum width of an input's label
        
        self._main_widget = QWidget()
        ctrl_layout = QHBoxLayout()

        self._filename_widget = inputs.SelectDirEntryWidget('Select file to save extrinsics:',
                inputs.SelectDirEntryType.FILENAME_SAVE,
                filters="XML (*.xml)", min_label_width=min_lbl_width)
        self._filename_widget.value_changed.connect(self.filenameSaveSelected)
        ctrl_layout.addWidget(self._filename_widget)

        self._btn_save_ext = QPushButton('Save Extrinsics')
        self._btn_save_ext.clicked.connect(self.saveExtrinsics)
        self._btn_save_ext.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._btn_save_ext.setEnabled(False)
        ctrl_layout.addWidget(self._btn_save_ext)

        if self._streamer.isStepThrough():
            self._btn_next = QPushButton('Next Frameset')
            self._btn_next.clicked.connect(self.loadNextFrameset)
            self._btn_next.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            ctrl_layout.addWidget(self._btn_next)
        
        # Controls to manipulate viewers and step through the stream
        btn_zoom_original = QPushButton('Original Image Size')
        btn_zoom_original.clicked.connect(self.rescaleViewers)

        btn_zoom_fit = QPushButton('Scale Images to Fit')
        btn_zoom_fit.clicked.connect(self.fitViewers)

        # Controls to modify visualization parameters
        self._draw_groundplane_cb = inputs.CheckBoxWidget('Draw groundplane:', is_checked=self._args.world_coords, min_label_width=min_lbl_width)
        self._draw_groundplane_cb.value_changed.connect(self.updateVisualizationOptions)

        self._grid_limits_widget = inputs.RoiSelectWidget('Groundplane ROI:', roi=[int(v) for v in self._args.grid_limits], min_label_width=min_lbl_width,
            box_labels=['X min:', 'Y min:', 'Width:', 'Height:'], support_image_selection=False)
        self._grid_limits_widget.value_changed.connect(self.updateVisualizationOptions)

        self._grid_spacing_widget = inputs.SliderSelectionWidget('Grid spacing:', 500, 5000, 18,
            value_format_fx=lambda v: '{:s} m'.format(inputs.format_float(v/1000, 5, 2)), min_label_width=min_lbl_width)
        self._grid_spacing_widget.value_changed.connect(self.updateVisualizationOptions)
        

        self._draw_xyz_cb = inputs.CheckBoxWidget('Draw world axis:', is_checked=self._args.world_coords, min_label_width=min_lbl_width)
        self._draw_xyz_cb.value_changed.connect(self.updateVisualizationOptions)

        self._axis_length_widget = inputs.SliderSelectionWidget('Axis length:', 500, 10000, 19,
            value_format_fx=lambda v: '{:s} m'.format(inputs.format_float(v/1000, 5, 2)), min_label_width=min_lbl_width)
        self._axis_length_widget.value_changed.connect(self.updateVisualizationOptions)

        self._line_width_widget = inputs.SliderSelectionWidget('Line thickness:', 1, 21, 10, initial_value=3,
            value_format_fx=lambda v: '{:s} px'.format(inputs.format_int(v, 2)), min_label_width=min_lbl_width)
        self._line_width_widget.value_changed.connect(self.updateVisualizationOptions)

        self._draw_horizon_cb = inputs.CheckBoxWidget('Draw horizon:', is_checked=self._args.world_coords, min_label_width=min_lbl_width)
        self._draw_horizon_cb.value_changed.connect(self.updateVisualizationOptions)

        self._draw_tags_cb = inputs.CheckBoxWidget('Draw tags:', is_checked=True, min_label_width=min_lbl_width)
        self._draw_tags_cb.value_changed.connect(self.updateVisualizationOptions)

        vis_option_layout = QGridLayout()
        vis_option_layout.addWidget(btn_zoom_original, 0, 0, 1, 5)
        vis_option_layout.addWidget(btn_zoom_fit, 0, 5, 1, 5)
        # vis_option_layout.addLayout(layout)

        # layout = QHBoxLayout()
        # layout.addWidget(self._draw_groundplane_cb)
        # layout.addWidget(self._grid_limits_widget)
        # layout.addWidget(self._grid_spacing_widget)
        # vis_option_layout.addLayout(layout)
        vis_option_layout.addWidget(self._draw_groundplane_cb, 1, 0, 1, 2)
        vis_option_layout.addWidget(self._grid_limits_widget, 1, 2, 1, 4)
        vis_option_layout.addWidget(self._grid_spacing_widget, 1, 6, 1, 4)

        # layout = QHBoxLayout()
        # layout.addWidget(self._draw_horizon_cb)
        # layout.addWidget(self._draw_xyz_cb)
        # layout.addWidget(self._axis_length_widget)
        # vis_option_layout.addLayout(layout)
        vis_option_layout.addWidget(self._draw_horizon_cb, 2, 0, 1, 2)
        vis_option_layout.addWidget(self._draw_xyz_cb, 2, 2, 1, 2)
        vis_option_layout.addWidget(self._axis_length_widget, 2, 4, 1, 6)

        # layout = QHBoxLayout()
        # layout.addWidget(self._draw_tags_cb)
        # layout.addWidget(self._line_width_widget)
        # vis_option_layout.addLayout(layout)
        vis_option_layout.addWidget(self._draw_tags_cb, 3, 0, 1, 4)
        vis_option_layout.addWidget(self._line_width_widget, 3, 4, 1, 6)


        
        vis_option_widget = QGroupBox('Visualization')
        vis_option_widget.setLayout(vis_option_layout)

        self._active_viewer_layout = QGridLayout()
        self._viewers = list()
        for i in range(self._num_streams):
            row = i // self._num_viewer_columns
            col = i % self._num_viewer_columns
            viewer = StreamViewer(i, self._stream_display_labels[i], parent=self)
            viewer.streamVisibilityToggled.connect(self.streamVisibilityToggled)
            self._active_viewer_layout.addWidget(viewer, row, col, 1, 1)
            self._viewers.append(viewer)
        #TODO add grid/scroll area (?) at the bottom which holds all inactive/"hidden" streams
        # needed, because we want to activate them again sooner or later
        self._active_viewer_widget = QGroupBox('Active Streams')
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
        # * group them all inside a groupbox
        self._inactive_groupbox = QGroupBox('Inactive Streams')
        group_layout = QVBoxLayout()
        group_layout.addWidget(self._inactive_scroll_area)
        self._inactive_groupbox.setLayout(group_layout)

        main_layout = QVBoxLayout()
        main_layout.addLayout(ctrl_layout)
        main_layout.addWidget(vis_option_widget)
        main_layout.addWidget(self._active_viewer_widget)
        main_layout.addWidget(self._inactive_groupbox)
        # main_layout.addWidget(self._inactive_scroll_area)
        
        self._main_widget.setLayout(main_layout)
        self.setCentralWidget(self._main_widget)
        self.resize(QSize(1280, 720))

        self.updateVisualizationOptions(None)

    def prepareShortcuts(self):
        sc = QShortcut(QKeySequence('Ctrl+F'), self)
        sc.activated.connect(self.fitViewers)
        sc = QShortcut(QKeySequence('Ctrl+1'), self)
        sc.activated.connect(self.rescaleViewers)
        sc = QShortcut(QKeySequence('Ctrl+S'), self)
        sc.activated.connect(self.saveExtrinsics)
        sc = QShortcut(QKeySequence('Ctrl+Shift+S'), self)
        sc.activated.connect(self.saveExtrinsicsAs)

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
  
    def displayPoseEstimates(self, frames, estimation_result):
        # Update the GUI
        for i in range(len(frames)):
            self._viewers[i].setTagErrors(estimation_result[i]['delta'], estimation_result[i]['stable'])
            self._viewers[i].showImage(frames[i], reset_scale=self._resize_viewers)
        self._resize_viewers = False
    
    def updateVisualizationOptions(self, value):
        self._processing_thread.setVisualizationParameters(
            self._draw_xyz_cb.value(), self._draw_groundplane_cb.value(),
            self._draw_horizon_cb.value(), self._draw_tags_cb.value(),
            self._axis_length_widget.value(), self._grid_spacing_widget.value(), self._grid_limits_widget.value(),
            self._line_width_widget.value())
    
    def filenameSaveSelected(self, filename):
        if filename is not None:
            self._filename_save = filename
        self._btn_save_ext.setEnabled(self._filename_save is not None)
    
    def saveExtrinsicsAs(self):
        self._filename_widget.open_dialog()
        self.saveExtrinsics()

    def saveExtrinsics(self):
        fn = self._filename_save
        if fn is None:
            print('Cannot save extrinsics - You must select a filename first!')
            return
        print('FIXME need to save the extrinsics to ', self._filename_save)

    # def enqueueFrameset(self, frames):
    #     self._process_lock.acquire()
    #     skip_frameset = self._pose_estimation_in_progress
    #     if not skip_frameset:
    #         self._pose_estimation_in_progress = True
    #     self._process_lock.release()

    #     if skip_frameset:
    #         print('Skipping frameset - pose estimation in progress')
    #         return
    #     print('Processing frameset')
    #     # Estimate the camera poses
    #     extrinsics = self._extrinsics_estimator.process_frameset(frames)
    #     # We will update the error display labels with current estimation deltas
    #     delta_strings = self._extrinsics_estimator.get_change_strings()
    #     # Visualize the poses
    #     vis_frames = self._extrinsics_estimator.visualize_frameset(frames, extrinsics,
    #         draw_world_coords=self._args.world_coords, axis_length=self._args.axis_length,
    #         grid_spacing=self._args.grid_spacing, grid_limits=self._args.grid_limits)
    #     # Update the GUI
    #     for i in range(len(vis_frames)):
    #         self._viewers[i].setTagErrorStrings(delta_strings[i])
    #         self._viewers[i].showImage(vis_frames[i], reset_scale=self._resize_viewers)
    #     self._resize_viewers = False

    #     self._process_lock.acquire()
    #     self._pose_estimation_in_progress = False
    #     self._process_lock.release()
    
    def streamVisibilityToggled(self, stream_index, active):
        if active:
            print("Stream '{}' becomes active".format(self._viewers[stream_index].streamLabel()))
            # Remove viewer widget from inactive list
            idx_layout_from = self._inactive_scroll_layout.indexOf(self._viewers[stream_index])
            self._inactive_scroll_layout.takeAt(idx_layout_from)

        # Remove all viewer widgets from grid
        for widx in range(self._active_viewer_layout.count()-1, -1, -1):
            self._active_viewer_layout.takeAt(widx)
        # Re-insert only active viewers
        active_stream_indices = [idx for idx in range(self._num_streams) if self._viewers[idx].isActive()]
        grid_idx = 0
        for sidx in active_stream_indices:
            row = grid_idx // self._num_viewer_columns
            col = grid_idx % self._num_viewer_columns
            self._active_viewer_layout.addWidget(self._viewers[sidx], row, col, 1, 1)
            grid_idx += 1

        # Add toggled viewer to "list" of inactive viewers (if it became inactive)
        if not active:
            print("Stream '{}' becomes inactive".format(self._viewers[stream_index].streamLabel()))
            self._inactive_scroll_layout.addWidget(self._viewers[stream_index])
    
    def appAboutToQuit(self):
        self._streamer.stopStream()
        self._streamer.wait()
        self._processing_thread.stopProcessing()
        self._processing_thread.wait()


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

    # Params to visualize the world coordinate system
    parser.add_argument('--world', action='store_true', dest='world_coords',
        help='Visualize world coordinate system after detecting markers.')
    parser.add_argument('--no-world', action='store_false', dest='world_coords',
        help="Don't visualize the world coordinate system.")
    parser.set_defaults(world_coords=True)
    parser.add_argument('--axis-length', action='store', type=pyutils.check_positive_real, default=1000.0,
        help="Length of each axes (arrows in [mm]), when running with --world")
    parser.add_argument('--grid-spacing', action='store', type=pyutils.check_positive_real, default=500.0,
        help="Size of each grid cell (in [mm]) when running with --world")
    parser.add_argument('--grid-limits', action='store', nargs=4, type=float, default=[-1e4, -1e4, 1e4, 1e4],
        help="Limit the ground plane grid visualization to the region given by [x_min, y_min, x_max, y_max]")

    parser.add_argument('--step-through', action='store_true', dest='step_through',
        help="Step through recording manually (instead of live streaming)")
    parser.set_defaults(step_through=False)

    args = parser.parse_args()
    # Convert grid limits to rectangle:
    args.grid_limits = [args.grid_limits[0], args.grid_limits[1], args.grid_limits[2]-args.grid_limits[0], args.grid_limits[3]-args.grid_limits[1]]

    #FIXME remove:
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'data', 'data-best')
    cfg_file = 'webcam.cfg'#
    # cfg_file = 'kinects.cfg'
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
