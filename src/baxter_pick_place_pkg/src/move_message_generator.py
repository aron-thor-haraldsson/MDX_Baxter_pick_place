#!/usr/bin/env python

import numpy as np

# This class is receives the centerpoint of the
class Move_Message:
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
            debug(2, "Error: ", "set_frame_dimensions in Move_Message received one or more 0 values")
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
        if shape == (0,) or shape == () or shape == False:
            quantity = 0
        elif shape == (3,):
            quantity = 1
        elif shape[1] == 3:
            quantity = shape[0]
        if quantity == 0:
            curr = False, False
            self._contour_center_h, self._contour_center_w = curr
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
        return self._contour_center_h, self._contour_center_w

    def reset_all(self):
        self._contour_report = False
        self._contour_center_h = False
        self._contour_center_w = False
        self._preferred_shape = False
        self._preferred_confidence = False

    def calculate_yx_diff(self):
        y_cont, x_cont = self.get_search_for_shape()
        y_frame, x_frame = self.get_frame_center()
        y_diff = y_frame - y_cont
        x_diff = x_cont- x_frame
        y_diff = y_diff - 50
        x_diff = x_diff -17 # a higher value results in a more positive Baxter arm movement in x direction(forward)
        if y_cont == False or x_cont == False:
            return -1000, -1000
        else:
            if abs(x_diff) < 2:
                x_diff = 0
            if abs(x_diff) < 2:
                y_diff = 0
            return y_diff, x_diff

    def build_move_command(self):
        y, x = self.calculate_yx_diff()
        self.reset_all()
        #print y
        #print x
        div = 1500.0

        if abs(y) == -1000 or x == -1000:
            return [-1000, -1000, -1000]
        else:

            x = x/div
            y = y/div
            print x, y
            if abs(x) < 0.0055:
                x = 0.0
            if abs(y) < 0.0055:
                y = 0.0
            print x, y
            if y == 0 and x == 0:
                return [0.0, 0.0, 0.0]
            elif y == True:
                return [x, 0, 0.0]
            elif x == True:
                return [0, y, 0.0]
            else:
                return [x, y, 0.0]

