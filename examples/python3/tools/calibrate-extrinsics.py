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
from utils_ext_calib import ExtrinsicsAprilTag, split_pose

class Streamer(QThread):
    newFrameset = pyqtSignal(list)

    VIS_DEPTH_OPTIONS = [
        'Surface normals', 'Pseudocolor (Parula)', 'Pseudocolor (Turbo)',
        'Pseudocolor (Viridis)', 'Grayscale']
    VIS_DEPTH_SURFNORM_OPTION = 0

    VIS_IR_OPTIONS = [
        'Histogram Equalization', 'Pseudocolor (Parula)', 'Pseudocolor (Turbo)',
        'Pseudocolor (Viridis)', 'Grayscale']
    VIS_IR_HISTEQ_OPTION = 0

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
        self._depth_range_vis = [0, 5000]
        self._ir_range_vis = [0, 255]
        self._prepare_depth_fx = self._vis_depth_surfnorm
        self._prepare_ir_fx = self._vis_ir_histeq
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

        if self._step_through is None:
            self._step_through = any([self._capture.is_step_able_image_sequence(idx) for idx in range(self._capture.num_streams())])

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
        processed_frames = [
            self._prepare_depth_fx(frames[idx])
                if self._capture.is_depth(idx)
                else (self._prepare_ir_fx(frames[idx])
                    if self._capture.is_infrared(idx)
                    else self._prepare_rgb(frames[idx]))
            for idx in range(len(frames))]
        return processed_frames

    def setDepthVisualization(self, depth_vis_option, depth_range):
        if depth_vis_option == 'Surface normals':
            self._prepare_depth_fx = self._vis_depth_surfnorm
        elif depth_vis_option == 'Pseudocolor (Parula)':
            self._prepare_depth_fx = lambda f: self._vis_depth_colormap(f, colormaps.colormap_parula_rgb)
        elif depth_vis_option == 'Pseudocolor (Turbo)':
            self._prepare_depth_fx = lambda f: self._vis_depth_colormap(f, colormaps.colormap_turbo_rgb)
        elif depth_vis_option == 'Pseudocolor (Viridis)':
            self._prepare_depth_fx = lambda f: self._vis_depth_colormap(f, colormaps.colormap_viridis_rgb)
        elif depth_vis_option == 'Grayscale':
            self._prepare_depth_fx = lambda f: self._vis_depth_colormap(f, colormaps.colormap_gray)
        else:
            raise ValueError('Unknown depth visualization option: "{}"'.format(depth_vis_option))
        self._depth_range_vis = depth_range
    
    def setInfraredVisualization(self, ir_vis_option, ir_range):
        if ir_vis_option == 'Histogram Equalization':
            self._prepare_ir_fx = self._vis_ir_histeq
        elif ir_vis_option == 'Pseudocolor (Parula)':
            self._prepare_ir_fx = lambda f: self._vis_ir_colormap(f, colormaps.colormap_parula_rgb)
        elif ir_vis_option == 'Pseudocolor (Turbo)':
            self._prepare_ir_fx = lambda f: self._vis_ir_colormap(f, colormaps.colormap_turbo_rgb)
        elif ir_vis_option == 'Pseudocolor (Viridis)':
            self._prepare_ir_fx = lambda f: self._vis_ir_colormap(f, colormaps.colormap_viridis_rgb)
        elif ir_vis_option == 'Grayscale':
            self._prepare_ir_fx = lambda f: self._vis_ir_colormap(f, colormaps.colormap_gray)
        else:
            raise ValueError('Unknown infrared visualization option: "{}"'.format(ir_vis_option))
        self._ir_range_vis = ir_range
    
    def _prepare_rgb(self, f):
        # return imutils.transform(f, 'histeq')
        return f

    def _vis_ir_histeq(self, f):
        # Convert to uint8
        if f.dtype != np.uint8:
            f = (f.astype(np.float32) / np.max(f) * 255).astype(np.uint8)
        return imutils.transform(f, 'histeq')

    def _vis_ir_colormap(self, f, cm):
        return imvis.pseudocolor(f, limits=self._ir_range_vis, color_map=cm)

    def _vis_depth_surfnorm(self, f):
        return imutils.transform(f, 'depth2surfnorm', 'surfnorm2rgb')

    def _vis_depth_colormap(self, f, cm):
        return imvis.pseudocolor(f, limits=self._depth_range_vis, color_map=cm)



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
        self._stream_label.setFont(font_stream_label)
        self._stream_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # A checkbox to toggle the display of this stream
        self._checkbox_active = QCheckBox('Active')
        self._checkbox_active.setChecked(True)
        self._checkbox_active.toggled.connect(self.updateVisibility)

        stream_lbl_layout = QHBoxLayout()
        stream_lbl_layout.addWidget(self._stream_label)
        stream_lbl_layout.addWidget(self._checkbox_active)

        # Custom "two-column" label layout to display tag estimation errors
        self._tag_error_label_left = QLabel('<font color="red">No tag detected</font>')
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
            error_strings = ['Tag <font weight="bold">{:2d}</font>: <font color="{:s}">{:5.2f} deg</font>, <font color="{:s}">{:4.1f} mm</font>, {:5.2f} px'.format(
                tag_id, 
                'green' if errors[tag_id]['r_deg_stable'] else 'red',
                errors[tag_id]['r_deg'], 
                'green' if errors[tag_id]['t_mm_stable'] else 'red',
                errors[tag_id]['t_mm'], errors[tag_id]['t_px'])
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
            self._tag_error_label_left.setText('<font color="red">No tag detected</font>')
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
        self._draw_world_xyz = True
        self._draw_groundplane = True
        self._draw_horizon = True
        self._draw_tags = True
        self._axis_length = 1000.0
        self._arrow_head_fraction = 0.1
        self._grid_spacing = 500.0
        self._grid_limits = [-1e4, -1e4, 2e4, 2e4]
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
            draw_tags, axis_length, arrow_head_fraction, grid_spacing, grid_limits, line_width):
        self._draw_world_xyz = draw_world_xyz
        self._draw_groundplane = draw_groundplane
        self._draw_horizon = draw_horizon
        self._draw_tags = draw_tags
        self._axis_length = axis_length
        self._arrow_head_fraction = arrow_head_fraction
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
            # Update the stream's extrinsics
            for idx in range(len(frames)):
                R, t = estimation_result[idx]['ext_world']
                self._streamer.getCapture().set_extrinsics(idx, R, t)
            # Some sensor streams (e.g. depth) may have extrinsics set by the underlying (C++) sink, so
            # query the (potentially) updated extrinsics:
            for idx in range(len(frames)):
                # print('Query extrinsics again for stream ', idx, self._streamer.getCapture().frame_label(idx))
                R, t = self._streamer.getCapture().extrinsics(idx)
                estimation_result[idx]['ext_world'] = (R, t)

            # Visualize the tag poses, world reference coordinate system, etc.
            vis_frames = self._extrinsics_estimator.visualize_frameset(frames, estimation_result,
                draw_world_xyz=self._draw_world_xyz,
                draw_groundplane=self._draw_groundplane,
                draw_horizon=self._draw_horizon,
                draw_tags=self._draw_tags,
                axis_length=self._axis_length,
                arrow_head=self._arrow_head_fraction,
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

        # Make an initial guess on how to nicely align the streams:
        num_viewer_rows = 1 if self._num_streams < 3 else \
            (2 if self._num_streams < 9 else\
                (3 if self._num_streams < 12 else 4))
        self._num_viewer_columns = math.ceil(self._num_streams / num_viewer_rows)

        self.prepareLayout()
        self.prepareShortcuts()
        streamer.newFrameset.connect(self._processing_thread.enqueueFrameset)
        streamer.startStream()

        # If the stream should be stepped through, load the first frameset now:
        if streamer.isStepThrough():
            self.loadNextFrameset()

    def rescaleViewers(self):
        self._resize_viewers = True

    def fitViewers(self):
        for v in self._viewers:
            v.scaleToFitWindow()
    
    def loadNextFrameset(self):
        frames = self._streamer.getNextFrameset()
        if frames is None:
            print('[WARNING] Received empty frameset')
            self._btn_next.setEnabled(False)
        else:
            self.displayFrameset(frames)

    def prepareLayout(self):
        min_lbl_width = 90 # Minimum width of an input's label
        
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

        slider_viewer_columns = inputs.SliderSelectionWidget('Layout:', 1, self._num_streams, self._num_streams-1,
            initial_value=self._num_viewer_columns, value_format_fx=lambda v: '{:s} streams/row'.format(inputs.format_int(v, 2)), min_label_width=min_lbl_width)
        slider_viewer_columns.value_changed.connect(self.updateViewerLayout)

        self._depth_visualization_dropdown = inputs.DropDownSelectionWidget('Depth visualization:',
                [(i, Streamer.VIS_DEPTH_OPTIONS[i]) for i in range(len(Streamer.VIS_DEPTH_OPTIONS))],
                min_label_width=min_lbl_width)
        self._depth_visualization_dropdown.value_changed.connect(self.changeDepthVisualization)

        self._depth_range_widget = inputs.RangeSliderSelectionWidget('Depth range:', 0, self._args.max_depth,
            value_format_fx=lambda v: inputs.format_int(v, 5), min_label_width=min_lbl_width)
        self._depth_range_widget.value_changed.connect(self.changeDepthVisualization)

        self._ir_visualization_dropdown = inputs.DropDownSelectionWidget('Infrared visualization:',
                [(i, Streamer.VIS_IR_OPTIONS[i]) for i in range(len(Streamer.VIS_IR_OPTIONS))],
                min_label_width=min_lbl_width)
        self._ir_visualization_dropdown.value_changed.connect(self.changeInfraredVisualization)

        self._ir_range_widget = inputs.RangeSliderSelectionWidget('Infrared range:', 0, self._args.max_ir,
            value_format_fx=lambda v: inputs.format_int(v, 5), min_label_width=min_lbl_width)
        self._ir_range_widget.value_changed.connect(self.changeInfraredVisualization)

        

        # Controls to modify visualization parameters
        self._draw_groundplane_cb = inputs.CheckBoxWidget('Draw groundplane:', is_checked=False, min_label_width=min_lbl_width)
        self._draw_groundplane_cb.value_changed.connect(self.updateVisualizationOptions)

        self._grid_limits_widget = inputs.RoiSelectWidget('Groundplane:', roi=[int(v) for v in [-1e4, -1e4, 2e4, 2e4]], min_label_width=min_lbl_width,
            box_labels=['X min:', 'Y min:', 'Width:', 'Height:'], support_image_selection=False)
        self._grid_limits_widget.value_changed.connect(self.updateVisualizationOptions)

        self._grid_spacing_widget = inputs.SliderSelectionWidget('Grid spacing:', 500, 5000, 18, initial_value=1000,
            value_format_fx=lambda v: '{:s} m'.format(inputs.format_float(v/1000, 5, 2)), min_label_width=min_lbl_width)
        self._grid_spacing_widget.value_changed.connect(self.updateVisualizationOptions)

        self._draw_xyz_cb = inputs.CheckBoxWidget('Draw world axis:', is_checked=True, min_label_width=min_lbl_width)
        self._draw_xyz_cb.value_changed.connect(self.updateVisualizationOptions)

        self._axis_length_widget = inputs.SliderSelectionWidget('Axis length:', 500, 10000, 19, initial_value=1000,
            value_format_fx=lambda v: '{:s} m'.format(inputs.format_float(v/1000, 5, 2)), min_label_width=min_lbl_width)
        self._axis_length_widget.value_changed.connect(self.updateVisualizationOptions)

        self._axis_tip_widget = inputs.SliderSelectionWidget('Axis arrow tip:', 0.1, 0.8, 14, initial_value=0.15,
            value_format_fx=lambda v: '{:s} %'.format(inputs.format_int(int(100*v), 2)), min_label_width=min_lbl_width)
        self._axis_tip_widget.value_changed.connect(self.updateVisualizationOptions)

        self._line_width_widget = inputs.SliderSelectionWidget('Line thickness:', 1, 21, 10, initial_value=3,
            value_format_fx=lambda v: '{:s} px'.format(inputs.format_int(v, 2)), min_label_width=min_lbl_width)
        self._line_width_widget.value_changed.connect(self.updateVisualizationOptions)

        self._draw_horizon_cb = inputs.CheckBoxWidget('Draw horizon:', is_checked=True, min_label_width=min_lbl_width)
        self._draw_horizon_cb.value_changed.connect(self.updateVisualizationOptions)

        self._draw_tags_cb = inputs.CheckBoxWidget('Draw tags:', is_checked=True, min_label_width=min_lbl_width)
        self._draw_tags_cb.value_changed.connect(self.updateVisualizationOptions)

        vis_option_layout = QGridLayout()
        layout = QHBoxLayout()
        layout.addWidget(btn_zoom_original)
        layout.addWidget(btn_zoom_fit)
        layout.addWidget(slider_viewer_columns)
        vis_option_layout.addLayout(layout, 0, 0, 1, 9)

        layout = QHBoxLayout()
        layout.addWidget(self._depth_visualization_dropdown)
        layout.addWidget(self._depth_range_widget)
        layout.addWidget(self._ir_visualization_dropdown)
        layout.addWidget(self._ir_range_widget)
        vis_option_layout.addLayout(layout, 1, 0, 1, 9)
        # vis_option_layout.addWidget(slider_viewer_columns, 1, 0, 1, 5)
        # vis_option_layout.addWidget(dropdown_visualize_depth, 1, 5, 1, 4)

        layout = QHBoxLayout()
        layout.addWidget(self._draw_tags_cb)
        layout.addWidget(self._draw_xyz_cb)
        layout.addWidget(self._draw_groundplane_cb)
        layout.addWidget(self._draw_horizon_cb)
        vis_option_layout.addLayout(layout, 2, 0, 1, 9)

        vis_option_layout.addWidget(self._grid_limits_widget,  3, 0, 1, 5)
        vis_option_layout.addWidget(self._grid_spacing_widget, 3, 5, 1, 4)
        vis_option_layout.addWidget(self._axis_length_widget,  4, 0, 1, 3)
        vis_option_layout.addWidget(self._axis_tip_widget,     4, 3, 1, 3)
        vis_option_layout.addWidget(self._line_width_widget,   4, 6, 1, 3)
  
        vis_option_widget = QGroupBox('Visualization')
        vis_option_widget.setLayout(vis_option_layout)

        # Arrange the stream viewers
        self._active_viewer_layout = QGridLayout()
        self._viewers = list()
        for i in range(self._num_streams):
            row = i // self._num_viewer_columns
            col = i % self._num_viewer_columns
            viewer = StreamViewer(i, self._stream_display_labels[i], parent=self)
            viewer.streamVisibilityToggled.connect(self.streamVisibilityToggled)
            self._active_viewer_layout.addWidget(viewer, row, col, 1, 1)
            self._viewers.append(viewer)
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

        # Ensure the processing thread has the same parametrization as the UI inputs:
        self.updateVisualizationOptions(None)
        self.changeDepthVisualization(None)
        self.changeInfraredVisualization(None)
        # Set a default storage filename
        self._filename_widget.set_value('extrinsics.xml')

    def prepareShortcuts(self):
        sc = QShortcut(QKeySequence('Ctrl+F'), self)
        sc.activated.connect(self.fitViewers)
        sc = QShortcut(QKeySequence('Ctrl+1'), self)
        sc.activated.connect(self.rescaleViewers)
        sc = QShortcut(QKeySequence('Ctrl+S'), self)
        sc.activated.connect(self.saveExtrinsics)
        sc = QShortcut(QKeySequence('Ctrl+Shift+S'), self)
        sc.activated.connect(self.saveExtrinsicsAs)
        sc = QShortcut(QKeySequence('Ctrl+R'), self)
        sc.activated.connect(self.saveReplayConfig)

    # def __load_request(self):
    #     filename, _ = QFileDialog.getOpenFileName(self, "Open image", "",
    #         'Images (*.bmp *.jpg *.jpeg *.png *.ppm);;All Files (*.*)',
    #         '', QFileDialog.DontUseNativeDialog)
    #     if filename is not None:
    #         img = imutils.imread(filename)
    #         self.displayImage(img)
  
    def displayPoseEstimates(self, frames, estimation_result):
        # Update the GUI
        for i in range(len(frames)):
            self._viewers[i].setTagErrors(estimation_result[i]['delta'], estimation_result[i]['stable'])
            self._viewers[i].showImage(frames[i], reset_scale=self._resize_viewers)
        self._resize_viewers = False
    
    def updateVisualizationOptions(self, value):
        # Enable/disable UI inputs
        self._grid_limits_widget.setEnabled(self._draw_groundplane_cb.value())
        self._grid_spacing_widget.setEnabled(self._draw_groundplane_cb.value())
        self._axis_length_widget.setEnabled(self._draw_xyz_cb.value())
        self._axis_tip_widget.setEnabled(self._draw_xyz_cb.value())
        self._line_width_widget.setEnabled(self._draw_xyz_cb.value() or self._draw_tags_cb.value() \
            or self._draw_horizon_cb.value() or self._draw_groundplane_cb.value())

        self._processing_thread.setVisualizationParameters(
            self._draw_xyz_cb.value(), self._draw_groundplane_cb.value(),
            self._draw_horizon_cb.value(), self._draw_tags_cb.value(),
            self._axis_length_widget.value(), self._axis_tip_widget.value(),
            self._grid_spacing_widget.value(), self._grid_limits_widget.value(),
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
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText('Cannot save extrinsics - You must select a filename first!')
            msg.setWindowTitle('Error')
            msg.exec()
            return
        if self._streamer.getCapture().save_extrinsics(self._filename_save):
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)
            msg.setText('Extrinsics saved to "{:s}"'.format(self._filename_save))
            msg.setWindowTitle('Success')
            msg.exec()
        else:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText('Error saving extrinsics to "{:s}"'.format(self._filename_save))
            msg.setWindowTitle('Error')
            msg.exec()

    def saveReplayConfig(self):
        """Saves a "dummy" replay configuration (assuming each stream is dumped as an image sequence)
        so you don't have to build the configuration from scratch."""
        folder = 'output-ext-calib'
        # Assume that each stream will be dumped as an image sequence
        cap = self._streamer.getCapture()
        storage_params = dict()
        for idx in range(cap.num_streams()):
            lbl = cap.frame_label(idx)
            pn = cap.canonic_frame_label(idx)
            storage_params[lbl] = best.StreamStorageParams(
                best.StreamStorageParams.Type.ImageSequence, pn)

        if cap.save_replay_config(folder=folder, storage_params=storage_params, save_extrinsics=True):
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)
            msg.setText('Replay configuration saved to "{:s}"'.format(folder))
            msg.setWindowTitle('Success')
            msg.exec()
        else:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText('Error saving replay configuration to "{:s}"'.format(folder))
            msg.setWindowTitle('Error')
            msg.exec()

    def changeDepthVisualization(self, changed_value):
        rv = self._depth_range_widget.value()
        vo = self._depth_visualization_dropdown.value()
        self._depth_range_widget.setEnabled(vo[0] != Streamer.VIS_DEPTH_SURFNORM_OPTION)
        self._streamer.setDepthVisualization(Streamer.VIS_DEPTH_OPTIONS[vo[0]], rv)

    def changeInfraredVisualization(self, changed_value):
        rv = self._ir_range_widget.value()
        vo = self._ir_visualization_dropdown.value()
        self._ir_range_widget.setEnabled(vo[0] != Streamer.VIS_IR_HISTEQ_OPTION)
        self._streamer.setInfraredVisualization(Streamer.VIS_IR_OPTIONS[vo[0]], rv)

    def updateViewerLayout(self, num_columns):
        self._num_viewer_columns = int(num_columns)
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

    def streamVisibilityToggled(self, stream_index, active):
        if active:
            # print("Stream '{}' becomes active".format(self._viewers[stream_index].streamLabel()))
            # Remove viewer widget from inactive list
            idx_layout_from = self._inactive_scroll_layout.indexOf(self._viewers[stream_index])
            self._inactive_scroll_layout.takeAt(idx_layout_from)

        # Call helper to update the grid layout
        self.updateViewerLayout(self._num_viewer_columns)

        # Add toggled viewer to "list" of inactive viewers (if it became inactive)
        if not active:
            # print("Stream '{}' becomes inactive".format(self._viewers[stream_index].streamLabel()))
            self._inactive_scroll_layout.addWidget(self._viewers[stream_index])
    
    def appAboutToQuit(self):
        self._streamer.stopStream()
        self._streamer.wait()
        self._processing_thread.stopProcessing()
        self._processing_thread.wait()


def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('config_file', action='store',
        help='Path to the stream configuration file (json or libconfig)')

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
    parser.add_argument('--max-depth', action='store', type=pyutils.check_positive_int, default=5000,
        help="Expected maximum depth value (typically in Millimeters) to cut off visualization.")
    
    parser.add_argument('--max-ir', action='store', type=pyutils.check_positive_int, default=255,
        help="Expected maximum infrared value (reflectance) to cut off visualization.")

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
    parser.set_defaults(step_through=None)

    args = parser.parse_args()
    # Convert grid limits to rectangle:
    # args.grid_limits = [args.grid_limits[0], args.grid_limits[1], args.grid_limits[2]-args.grid_limits[0], args.grid_limits[3]-args.grid_limits[1]]

    return args


def gui():
    args = parseArguments()

    abs_cfg = os.path.abspath(args.config_file)
    folder = os.path.dirname(abs_cfg)
    cfg_file = os.path.basename(args.config_file)
    streamer = Streamer(folder, cfg_file, args.step_through)

    app = QApplication(['Calibrate Extrinsics'])
    main_widget = CalibApplication(streamer, args)
    app.aboutToQuit.connect(main_widget.appAboutToQuit)
    main_widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    gui()
