#!/usr/bin/env python3
import ctypes
from ctypes import util
import time
import os

x11 = ctypes.cdll.LoadLibrary(util.find_library('X11'))

class Display(ctypes.Structure):
    pass

x11.XOpenDisplay.restype = ctypes.POINTER(Display)
x11.XDefaultRootWindow.restype = ctypes.c_ulong

display = x11.XOpenDisplay(None)
if not display:
    print("Failed to open X display")
    exit(1)

root = x11.XDefaultRootWindow(display)

def get_children(window):
    root_return = ctypes.c_ulong()
    parent_return = ctypes.c_ulong()
    children_return = ctypes.POINTER(ctypes.c_ulong)()
    nchildren_return = ctypes.c_uint()
    status = x11.XQueryTree(display, window, ctypes.byref(root_return),
                            ctypes.byref(parent_return), ctypes.byref(children_return),
                            ctypes.byref(nchildren_return))
    if status == 0:
        return []
    children = []
    for i in range(nchildren_return.value):
        children.append(children_return[i])
    if nchildren_return.value > 0:
        x11.XFree(children_return)
    return children

def get_window_name(window):
    name_return = ctypes.c_char_p()
    status = x11.XFetchName(display, window, ctypes.byref(name_return))
    if status != 0 and name_return.value:
        name = name_return.value.decode('utf-8', 'ignore')
        x11.XFree(name_return)
        return name
    return None

def find_window(window, name_match):
    name = get_window_name(window)
    if name and name_match in name:
        return window
    for child in get_children(window):
        found = find_window(child, name_match)
        if found:
            return found
    return None

def arrange():
    # We want Gazebo at bottom-right: 960x520 at offset (960, 520)
    # Wait for Gazebo window to appear
    gazebo_win = None
    for i in range(30):
        gazebo_win = find_window(root, "Gazebo Sim")
        if gazebo_win:
            break
        time.sleep(1)

    if gazebo_win:
        print(f"Found Gazebo window: {gazebo_win}. Moving to bottom-right.")
        x11.XMoveResizeWindow(display, gazebo_win, 960, 520, 960, 520)
        x11.XFlush(display)
    else:
        print("Gazebo window not found")

if __name__ == '__main__':
    arrange()
