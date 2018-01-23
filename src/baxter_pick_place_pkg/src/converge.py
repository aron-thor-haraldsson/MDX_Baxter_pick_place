#!/usr/bin/env python

import numpy as np

# This class is receives the centerpoint of the
class Converge:
    def __init__(self):
        self._frame_height=False
        self._frame_width=False
        self._frame_center_h=False
        self._frame_center_w=False
        self._contour_center_h=False
        self._contour_center_w=False
        self._contour_report=False
        self._preferred_shape=False
        self._preferred_confidence=False

        #ns = '/robot/limb/' + limb + '/'
        #_goal_listener = rospy.Subscriber(ns + 'topic', MsgString?, self._on_completion, queue_size=1, tcp_nodelay=True)
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
    def get_frame_center(self):
        return [self._frame_center_h, self._frame_center_w]
    def set_contour_report(self, report_arg=False):
        self._contour_report = report_arg
    def get_contour_report(self):
        return self._contour_report
    def set_search_for_shape(self, shape_arg="TRI", conf_arg=80):
        report = self.get_contour_report()
        quantity = 0
        shape = np.shape(report)
        if shape == (3,):
            quantity = 1
        elif shape[1] == 3:
            quantity = shape[0]
        if quantity == 1:
            curr = self.check_report_line(report, shape_arg, conf_arg)
            if curr:
                self._contour_center_h, self._contour_center_w = curr
        elif quantity > 1:
            for i in range(quantity):
                curr = self.check_report_line(report[i-1], shape_arg, conf_arg)
                if curr:
                    self._contour_center_h, self._contour_center_w = curr
                    break
    def check_report_line(self, report_line_arg, shape_arg, conf_arg):
        if report_line_arg[1] == shape_arg and report_line_arg[2] > conf_arg:
            return report_line_arg[0]
        else:
            return False
    def get_search_for_shape(self):
        h, y = self._contour_center_h, self._contour_center_w
        self.reset_all()
        return h, y

    def reset_all(self):
        self._contour_report = False
        self._contour_center_h = False
        self._contour_center_w = False
        self._preferred_shape = False
        self._preferred_confidence = False
