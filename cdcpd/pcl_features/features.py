from ctypes import *
import numpy as np
import os

PCL_LIB_PATH = os.path.join(os.path.dirname(__file__), 'libfeatures.so')
PCL_DLL = cdll.LoadLibrary(PCL_LIB_PATH)


def vfh(points):
    assert(len(points.shape) == 2)
    assert(points.shape[1] == 3)

    if points.dtype is not np.float32:
        points = points.astype(np.float32)

    feature = np.zeros(308, dtype=np.float32)
    points_ptr = points.astype(np.float32).ctypes.data_as(POINTER(c_float))
    feature_ptr = feature.ctypes.data_as(POINTER(c_float))

    PCL_DLL.vfh(feature_ptr, points_ptr, points.shape[0], points.shape[1])

    return feature
