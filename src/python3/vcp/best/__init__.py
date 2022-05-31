#!/usr/bin/env python
# coding=utf-8
"""Streaming from multiple (supported) cameras."""

try:
    from vcp.best.best_cpp import *
except ImportError as e:
    print("[W] Could not load vcp::best C++ wrapper")
    print(str(e))

from vcp.best.storage import *
from vcp.best.streaming import *
