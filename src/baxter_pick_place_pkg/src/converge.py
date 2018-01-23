#!/usr/bin/env python

# This class is receives the centerpoint of the
class Converge:
    def __init__(self):
        self._frame_height
        self._frame_width
        self._frame_center_h
        self._frame_center_w
        self._contour_center_h
        self._contour_center_w

        ns = '/robot/limb/' + limb + '/'
        _goal_listener = rospy.Subscriber(ns + 'topic', MsgString?, self._on_completion, queue_size=1, tcp_nodelay=True)
    # receives the height and width of the camera on Baxter's camera
    def set_frame_dimensions(self, h_arg=200, w_arg=320):
        if h_arg > 0 or w_arg > 0:
            self._frame_height = h_arg
            self._frame_center_h = h_arg/2
            self._frame_height = w_arg
            self._frame_center_w = w_arg/2
        else:
            debug(2, "Error: ", "set_frame_dimensions in Converge received one or more 0 values")
            end_program()
    set_contour_center(self, input_arg)
