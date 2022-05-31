from enum import Enum
from typing import List
import numpy as np
import datetime


class FrameType(Enum):
    Color = 1
    Depth = 2
    Intensity = 3

    def __str__(self):
        return self.name


class Frame(object):
    """A camera's (serializable) current frame."""
    def __init__(self, label: str, frame_data: np.ndarray,
                 frame_type: FrameType,
                 timestamp: datetime.datetime,
                 is_rectified: bool):
        self.label = label
        self.frame_data = frame_data
        self.frame_type = frame_type
        self.timestamp = timestamp
        self.is_rectified = is_rectified

    @property
    def is_color(self) -> bool:
        return self.frame_type == FrameType.Color

    @property
    def is_depth(self) -> bool:
        return self.frame_type == FrameType.Depth

    @property
    def is_intensity(self) -> bool:
        return self.frame_type == FrameType.Intensity

    def __repr__(self) -> str:
        dim = 'None' if self.frame_data is None else\
              'x'.join([str(s) for s in self.frame_data.shape])
        return f'{self.frame_type}: {dim} ({self.label})'


class Frameset(object):
    """A frameset contains the images of all connected cameras."""
    def __init__(self, frameset_number: int, timestamp: datetime.datetime = None):
        self.frameset_number = frameset_number
        self.timestamp = datetime.datetime.now() if timestamp is None else timestamp
        self.frames = list()

    @property
    def labels(self) -> List[str]:
        """
        Returns a list of (unique) labels for which you can
        query `Frame`s from this `Frameset`
        """
        return list(set([f.label for f in self.frames]))

    def frames_for_camera(self, label: str) -> List[Frame]:
        """
        Returns a list of `Frame`s for the given `cam_label` (i.e. the label
        you specified in the configuration file).
        """
        return [f for f in self.frames if f.label == label]

    def __getitem__(self, key) -> List[Frame]:
        return self.frames_for_camera(key)

    def __str__(self) -> str:
        return f'Frameset #{self.frameset_number} at {self.timestamp}, containing frames for {self.labels}'
