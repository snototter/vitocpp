#!/usr/bin/env python
"""A demo application showing some basic vcp.config capabilities."""

import os
import sys

#def trace(frame, event, arg):
#    print("%s, %s:%d" % (event, frame.f_code.co_filename, frame.f_lineno))
#    return trace
#sys.settrace(trace)

# Add path to the vcp package
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src', 'python3'))
import vcp.config as vcfg
import vcp.utils as vutils


if __name__ == "__main__":
    for cf in ['../data/data-best/ipcam.cfg', '../data/data-best/k4a.cfg']:
        config = vcfg.Config()
        config.load_libconfig(cf)
        vutils.log_info('Loaded config file "{}"'.format(cf))
        for pn in config.list_parameters():
            print('          {}: {}'.format(pn, config[pn]))
        print()

