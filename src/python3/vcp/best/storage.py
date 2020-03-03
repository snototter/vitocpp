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
            logging.getLogger().info('vcp.best.storage.SnapshotStorage worker [{:d} - {:d}] saved image #{:d}: {:s}'.format(worker_id, pid, num, filename))
            num += 1
    if verbose:
        logging.getLogger().info('vcp.best.storage.SnapshotStorage worker [{:d} - {:d}] terminating'.format(worker_id, pid))
    queue.put(None) # To notify others of end-of-stream, too


def _append_video_frame(name, writer, queue, verbose, sigint_handler):
    #TODO doc, use logging
    signal.signal(signal.SIGINT, sigint_handler)
    num = 0
    try:
        while True:
            image = queue.get()
            if image is None:
                break
            writer.writeFrame(image)
            if verbose:
                print('[I] SingleVideoStorage worker appended frame #{:d} to {:s}'.format(num, name))
                num += 1
    except Exception as e:
        print('[E] Error occured while appending video frame: {}'.format(e))
    finally:
        if verbose:
            print('[I] SingleVideoStorage finalizing video {:s}'.format(name))
        try:
            writer.close()
        except Exception as e:
            print('[E] Error while closing the video: {}'.format(e))


class SnapshotStorage:
    #TODO doc, use logging
    def __init__(self, num_workers=1, verbose=False):
        self.queue = mp.Queue()
        self.num_workers = num_workers
        self.workers = list()
        self.verbose = verbose
        self.sigint_received = False
        self.__start()
    
    def __del__(self):
        self.__stop()

    def __start(self):
        self.workers = [mp.Process(target=_store_snapshot, args=(i, self.queue, self.verbose, lambda signum, frame: self.__sigint_handler(i, signum, frame), )) for i in range(self.num_workers)]
        for w in self.workers:
            w.start()
        
    def __stop(self):
        # if self.cleanup:
        self.queue.put(None)
        for i in range(self.num_workers):
            self.workers[i].join()
        self.queue.close()

    def __sigint_handler(self, terminated_worker, signum, frame):
        # Handle sigint as 'just a user intent' - i.e. "schedule" raising an Exception, so we don't screw up the worker process
        print('[W] KeyboardInterrupt occured within SnapshotStorage #{:d}'.format(terminated_worker))
        self.sigint_received = True
        

    def put_storage_request(self, image, filename):
        if self.sigint_received:
            raise KeyboardInterrupt
        self.queue.put((image, filename))

    def put_storage_requests(self, requests):
        if self.sigint_received:
            raise KeyboardInterrupt
        for req in requests:
            self.queue.put(req)


class SingleVideoStorage:
    """Only stores mp4, x264"""
    def __init__(self, filename, fps, width, height, flip_channels=True, verbose=False):
        self.queue = mp.Queue()
        self.flip_channels = flip_channels
        self.filename = filename

        # Ensure the output path exists
        basename, fn = os.path.split(filename)
        if basename and not os.path.exists(basename):
            if verbose:
                print('[I] SingleVideoStorage creating output path: {:s}'.format(basename))
            os.makedirs(basename)

        #self.video_writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*fourcc), fps, resolution)
        # scikit-video provides an interface to ffmpeg which is way more reliable than OpenCV's video writer stuff
        self.video_writer = skvideo.io.FFmpegWriter(filename, 
            inputdict = {'-r': str(fps), '-s':'{:d}x{:d}'.format(width, height)},
            outputdict = {'-r': str(fps), '-crf':'17', # crf 0: lossless, 17: "visually lossless", 51: "worst quality ever"
            '-c:v': 'libx264', '-preset':'ultrafast', '-tune': 'zerolatency'})
            #TODO currently, there seems to be a bug with my installation (cannot call video_writer.close() due to skvideo exception, so no video is created)
        self.worker = mp.Process(target=_append_video_frame, args=(filename, self.video_writer, self.queue, verbose, self.__sigint_handler, ))
        self.sigint_received = False
        self.worker.start()
    
    def __del__(self):
        self.__stop()
        
    def __stop(self):
        self.queue.put(None)
        self.worker.join()
        self.queue.close()

    def __sigint_handler(self, signum, frame):
        # Handle sigint as 'just a user intent' - i.e. "schedule" raising an Exception, so we don't screw up the worker process
        print('[W] KeyboardInterrupt occured within SingleVideoStorage: {:s}'.format(self.filename))        
        self.sigint_received = True

    def put_storage_request(self, image):
        if self.sigint_received:
            raise KeyboardInterrupt
        if self.flip_channels:
            image = imutils.flip_layers(image)
        self.queue.put(image)

#FIXME doc currently only mp4 + h264
class MultiVideoStorage:
    """Store multiple (color/grayscale) streams to video files."""
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
            logging.getLogger().info('vcp.best.storage.MultiVideoStorage initialized workers for frame labels: [{}]'.format(', '.join([k for k in videos])))

    def put_storage_request(self, image_dict):
        """
        Append the given frames to the corresponding video worker processes.

        :param image_dict: a dictionary of 'frame_label': 'image', where 'frame_label'
                is the label you used to set up this MultiVideoStorage and 'image' is
                the image data, given as HxWxC numpy.ndarray.
        """
        for k in image_dict:
            self._video_stores[k].put_storage_request(image_dict[k])
