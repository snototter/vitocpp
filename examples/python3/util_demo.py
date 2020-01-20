#!/usr/bin/env python
# coding=utf-8
"""vcp::utils demo (it's better to use built-in python stuff instead of these utils; I just wrapped them to get familiar with pybind11)"""

import os
import sys
import time

# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src', 'python3'))
import vcp.utils as vutils


if __name__ == "__main__":
    help(vutils)
    vutils.log_debug('Debug message goes here')
    vutils.log_info('Status message goes here')
    vutils.log_warning('Warning message goes here')
    vutils.log_failure('Error message goes here')
    
    for dirent in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')):
        print("'{}' is{}an image".format(dirent, ' ' if vutils.is_image_file(dirent) else ' not '))
