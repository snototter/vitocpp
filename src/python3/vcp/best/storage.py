import os
import sys
import skvideo.io
import multiprocessing as mp
import signal
import logging
from vito import imutils


def _store_snapshot(worker_id, queue, verbose, sigint_handler):
    """Worker process to store images retrieved from the 'queue' as still frames at the disk.
    Use this if you prefer high latency (filesystem overhead) and wasting disk space."""
    signal.signal(signal.SIGINT, sigint_handler)
    num = 0
    pid = os.getpid()
    while True:
        request = queue.get()
        if request is None:
            break
        image, filename = request
        imutils.imsave(filename, image, flip_channels=False)
        if verbose:
            mp.get_logger().info('vcp.best.storage.SnapshotStorage worker [{:d} - {:d}] saved image #{:d}: {:s}'.format(worker_id, pid, num, filename))
            num += 1
    if verbose:
        mp.get_logger().info('vcp.best.storage.SnapshotStorage worker [{:d} - {:d}] terminating'.format(worker_id, pid))
    queue.put(None) # To notify others of end-of-stream, too


def _append_video_frame(name, writer_args, queue, verbose, sigint_handler):
    """Worker process to append images to the given video writer."""
    signal.signal(signal.SIGINT, sigint_handler)
    frame_counter = 0
    writer = None
    should_split = writer_args['split_output'] is not None
    split_count = 0
    def _filename(cnt):
        if should_split:
            pth, ext = os.path.splitext(writer_args['filename'])
            return f'{pth}-{cnt:04d}{ext}'
        else:
            return writer_args['filename']
    try:
        while True:
            image = queue.get()
            if image is None:
                break
            if writer is None:
                #self.video_writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*fourcc), fps, resolution)
                # scikit-video provides an interface to ffmpeg which is way more reliable than OpenCV's video writer stuff
                writer = skvideo.io.FFmpegWriter(_filename(split_count),
                    inputdict = {'-r': str(writer_args['fps']), '-s':'{:d}x{:d}'.format(writer_args['width'], writer_args['height'])},
                    outputdict = {'-r': str(writer_args['fps']), '-crf':'17', # crf 0: lossless, 17: "visually lossless", 51: "worst quality ever"
                    '-c:v': 'libx264', '-preset':'ultrafast', '-tune': 'zerolatency'})
            writer.writeFrame(image)
            frame_counter += 1
            if should_split and frame_counter % writer_args['split_output'] == 0:
                writer.close()
                writer = None
                split_count += 1
                frame_counter = 0
    except Exception as e:
        mp.get_logger().error('vcp.best.storage.SingleVideoStorage: Error occured while appending video frame: {}'.format(e))
    finally:
        if verbose:
            mp.get_logger().info('vcp.best.storage.SingleVideoStorage finalizing video {:s}'.format(name))
        try:
            if writer is not None:
                writer.close()
        except Exception as e:
            mp.get_logger().error('vcp.best.storage.SingleVideoStorage: Error while closing the video: {}'.format(e))


class ImageSequenceStorage:
    def __init__(self, folder, file_extension='.png', flip_channels=False, verbose=False):
        self.queue = mp.Queue()
        self.worker = None
        self.frame_nr = 0
        self.folder = folder
        self.file_extension = file_extension
        self.flip_channels = flip_channels
        self.verbose = verbose
        self.sigint_received = False
        # Ensure the output path exists
        if not os.path.exists(folder):
            if verbose:
                mp.get_logger().info('vcp.best.storage.ImageSequenceStorage creating output path: {:s}'.format(folder))
            os.makedirs(folder)
        self.__start()

    def __del__(self):
        self.stop()

    def __start(self):
        self.worker = mp.Process(target=_store_snapshot, args=(0, self.queue, self.verbose, lambda signum, frame: self.__sigint_handler(0, signum, frame), ))
        self.worker.start()

    def stop(self):
        if self.worker.is_alive():
            self.queue.put(None)
            self.worker.join()
            self.queue.close()

    def __sigint_handler(self, terminated_worker, signum, frame):
        # Handle sigint as 'just a user intent' - i.e. "schedule" raising an Exception, so we don't screw up the worker process
        mp.get_logger().info('KeyboardInterrupt occured within vcp.best.storage.SnapshotStorage #{:d}'.format(terminated_worker))
        self.sigint_received = True

    def put_storage_request(self, image, filename=None):
        """Save the given image to disk (to <folder>/frame_<framenr><file_extension>).
        If you want to overwrite the automatically generated filename, pass a valid (string) filename.
        """
        if self.sigint_received:
            raise KeyboardInterrupt
        if self.flip_channels:
            image = imutils.flip_layers(image)
        if filename is None:
            filename = os.path.join(self.folder, 'frame_{:06d}{:s}'.format(self.frame_nr, self.file_extension))
            self.frame_nr += 1
        self.queue.put((image, filename))

    def __str__(self):
        return "ImageSequenceStorage ({})".format(self.folder)


class SingleVideoStorage:
    """Only stores mp4, x264"""
    def __init__(self, filename, fps, width, height, flip_channels=True, verbose=False, split_output=None):
        self.queue = mp.Queue()
        self.flip_channels = flip_channels
        self.filename = filename

        # Ensure the output path exists
        basename, _ = os.path.split(filename)
        if basename and not os.path.exists(basename):
            if verbose:
                mp.get_logger().info('vcp.best.storage.SingleVideoStorage creating output path: {:s}'.format(basename))
            os.makedirs(basename)

        video_writer_args = {
            'filename': filename,
            'fps': fps,
            'width': width,
            'height': height,
            'split_output': split_output
        }
        # #self.video_writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*fourcc), fps, resolution)
        # # scikit-video provides an interface to ffmpeg which is way more reliable than OpenCV's video writer stuff
        # self.video_writer = skvideo.io.FFmpegWriter(filename, 
        #     inputdict = {'-r': str(fps), '-s':'{:d}x{:d}'.format(width, height)},
        #     outputdict = {'-r': str(fps), '-crf':'17', # crf 0: lossless, 17: "visually lossless", 51: "worst quality ever"
        #     '-c:v': 'libx264', '-preset':'ultrafast', '-tune': 'zerolatency'})
        self.worker = mp.Process(target=_append_video_frame, args=(filename, video_writer_args, self.queue, verbose, self.__sigint_handler, ))
        self.sigint_received = False
        self.worker.start()

    def __del__(self):
        self.stop()
        
    def stop(self):
        if self.worker.is_alive():
            self.queue.put(None)
            self.worker.join()
            self.queue.close()

    def __sigint_handler(self, signum, frame):
        # Handle sigint as 'just a user intent' - i.e. "schedule" raising an Exception, so we don't screw up the worker process
        mp.get_logger().info('KeyboardInterrupt occured within vcp.best.storage.SingleVideoStorage: {:s}'.format(self.filename))
        self.sigint_received = True

    def put_storage_request(self, image):
        if self.sigint_received:
            raise KeyboardInterrupt
        if self.flip_channels:
            image = imutils.flip_layers(image)
        self.queue.put(image)

    def __str__(self):
        return "SingleVideoStorage ({})".format(self.filename)


class MultiVideoStorage:
    """Store multiple (color/grayscale) streams to video files. 
    Currently, we only render h264 encoded streams in mp4 containers."""
    def __init__(self, videos, verbose=False):
        """
        Starts a separate process for each incoming stream.

        :param videos: a dictionary of 'frame_label': video_param_dict, where
                'frame_label' is a unique identifier of the stream (you'll also
                use to append frames later on via @see put_storage_request()), and
                video_param_dict is a dictionary holding the video properties:
                filename, fps (framerate, double), width (int), height (int),
                flip_channels (bool, True if you need to change the order of the
                color channels).
        """
        def _create_video_storage(k):
            vk = videos[k]
            return SingleVideoStorage(vk['filename'], vk['fps'], vk['width'], vk['height'], vk['flip_channels'], verbose=verbose)
        self._video_stores = {k:_create_video_storage(k) for k in videos}
        if verbose:
            mp.get_logger().info('vcp.best.storage.MultiVideoStorage initialized workers for frame labels: [{}]'.format(', '.join([k for k in videos])))
    
    def __del__(self):
        self.stop()
    
    def stop(self):
        for k in self._video_stores:
            self._video_stores[k].stop()

    def put_storage_request(self, image_dict):
        """
        Append the given frames to the corresponding video worker processes.

        :param image_dict: a dictionary of 'frame_label': 'image', where 'frame_label'
                is the label you used to set up this MultiVideoStorage and 'image' is
                the image data, given as HxWxC numpy.ndarray.
        """
        for k in image_dict:
            self._video_stores[k].put_storage_request(image_dict[k])
