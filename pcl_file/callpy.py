# 在python中调用C++代码，传入参数，获取返回值
import ctypes
import os

lib_path = "./build/libsus_vis_lib.so"
lib = ctypes.cdll.LoadLibrary(lib_path)
# 调用callsByPython方法，入参为2个string
lib.callsByPython.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
lib.callsByPython.restype = ctypes.c_char_p

# 调用callsByPython方法，入参为2个string("1630376940.000", "bbox")，无返回值
lib.callsByPython(b"1630376940.000", b"bbox")
