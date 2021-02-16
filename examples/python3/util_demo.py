#!/usr/bin/env python
# coding=utf-8
"""vcp::utils demo (it's better to use built-in python stuff instead of these utils; I just wrapped them to get familiar with pybind11)"""

import os
import sys
import time

# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gen'))
import vcp.utils as vutils


if __name__ == "__main__":
    help(vutils)
    # Using C++ calls to print messages is overkill (but these were simple bindings
    # to get me started with pybind11)
    vutils.log_debug('Debug message goes here')
    vutils.log_info('Status message goes here')
    vutils.log_warning('Warning message goes here')
    vutils.log_failure('Error message goes here') # aliased as log_error()
    
    for dirent in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')):
        print("'{}' is{}an image".format(dirent, ' ' if vutils.is_image_file(dirent) else ' not '))
    

    print('\nStop watch demo:')
    watch1 = vutils.StopWatch()
    watch2 = vutils.StopWatch('some-label')

    for i in range(10):
        to_sleep = 0.01 + i*0.1
        watch1.start()
        watch2.start()
        time.sleep(to_sleep)
        print('Elapsed (should be {} ms), watch1: {} ms, watch2: {} ms'.format(to_sleep*1e3, watch1.elapsed(), watch2.elapsed()))
