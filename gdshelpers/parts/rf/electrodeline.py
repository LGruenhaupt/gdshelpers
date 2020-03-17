from __future__ import print_function, division

import collections

import numpy as np
import numpy.linalg as linalg
import scipy.interpolate
import shapely.geometry
import shapely.affinity
import shapely.ops
import shapely.validation

from gdshelpers.parts.electrodeport import Electrodeport
from gdshelpers.helpers import find_line_intersection, normalize_phase

class Electrodeline(object):
    def __init__(self, origin, angle, width):
        self._current_port = Electrodeport(origin, angle, width)
        self._in_port = self._current_port.inverted_direction.copy()
        self._segments = list()

    @classmethod
    def make_at_port(cls, port, **kargs):
        port_param = port.copy()
        port_param.set_port_properties(**kargs)
        return cls(**port_param.get_parameters())

    @property
    def x(self):
        return self._current_port.origin[0]

    @property
    def y(self):
        return self._current_port.origin[1]

    @property
    def origin(self):
        return self._current_port.origin

    @property
    def angle(self):
        return self._current_port.angle

    @property
    def width(self):
        return self._current_port.width

    @width.setter
    def width(self, width):
        self._current_port.width = width

    @property
    def current_port(self):
        return self._current_port.copy()

    def parallel_offset(self, offset):
        """
        Returns a new port, which offset in parallel from this port.

        :param offset: Offset from the center of the port. Positive is left of the port.
        :type offset: float
        :return: The new offset port
        :rtype: Port
        """
        port = self._current_port
        offset = [offset * np.cos(self.angle + np.pi / 2), offset * np.sin(self.angle + np.pi / 2)]
        port.origin = port.origin + offset
        return port

    def longitudinal_offset(self, offset):
        """
        Returns a new port, which offset in in direction of this port.

        :param offset: Offset from the end of the port. Positive is the direction, the port is pointing.
        :type offset: float
        :return: The new offset port
        :rtype: Port
        """

        port = self._current_port
        offset = [offset * np.cos(self.angle), offset * np.sin(self.angle)]
        port.origin = port.origin + offset
        return port

    # Add alias for current port
    port = current_port

    @property
    def in_port(self):
        return self._in_port.copy()

    @property
    def length(self):
        return sum((length for port, obj, length in self._segments))

    @property
    def length_last_segment(self):
        if not len(self._segments):
            return 0
        return self._segments[-1][2]

    def get_shapely_object(self):
        """
        Get a shapely object which forms this path.
        """
        return shapely.ops.cascaded_union([obj for port, obj, length in self._segments])

    def get_segments(self):
        """
        Returns the list of tuples, containing their ports and shapely objects.
        """
        return [(port.copy(), obj, length) for port, obj, length in self._segments]

    def add_straight_segment(self, length, final_width=None, **kargs):
        self._current_port.set_port_properties(**kargs)
        final_width = final_width or self.width

        endpoint = shapely.geometry.Point(length, 0)

        if not np.isclose(length, 0):
            assert length >= 0, 'Length of straight segment must not be negative'
            polygon1 = shapely.geometry.Polygon(([0, (self.width) / 2.], [length, (final_width) / 2.],
                                                [length, (-final_width) / 2.], [0, (-self.width) / 2.]))

            polygon1 = shapely.affinity.rotate(polygon1, self.angle, origin=[0, 0], use_radians=True)
            polygon1 = shapely.affinity.translate(polygon1, self.x, self.y)

            self._segments.append((self._current_port.copy(), polygon1, length))


        endpoint = shapely.affinity.rotate(endpoint, self.angle, origin=[0, 0], use_radians=True)
        endpoint = shapely.affinity.translate(endpoint, self.x, self.y)

        self._current_port.origin = endpoint.coords[0]
        self._current_port.width = final_width

        return self

    def add_bend(self, angle, radius, final_width=None, n_points=50, **kargs):
        # If number of points is None, default to 128
        n_points = n_points if n_points else 50

        self._current_port.set_port_properties(**kargs)
        final_width = final_width or self.width

        angle = normalize_phase(angle, zero_to_two_pi=True) - (0 if angle > 0 else 2 * np.pi)

        if not np.isclose(radius, 0) and not np.isclose(angle, 0) and radius > 0:
            if angle > 0:
                circle_center = (-np.sin(self.angle) * radius, np.cos(self.angle) * radius) + self.current_port.origin
                start_angle = -np.pi / 2 + self.angle
            else:
                circle_center = (np.sin(self.angle) * radius, -np.cos(self.angle) * radius) + self.current_port.origin
                start_angle = np.pi / 2 + self.angle

            end_angle = start_angle + angle

            # Calculate the points needed for this angle
            points = max(int(abs(end_angle - start_angle) / (np.pi / 2) * n_points), 2)

            phi = np.linspace(start_angle, end_angle, points)

            
            # first branch
            upper_radius_points = np.linspace(radius + self.width / 2, radius + final_width / 2 , points)
            upper_line_points = np.array([upper_radius_points * np.cos(phi),
                                          upper_radius_points * np.sin(phi)]).T + circle_center

            lower_radius_points = np.linspace(radius - self.width / 2 , radius - final_width / 2, points)
            lower_line_points = np.array([lower_radius_points * np.cos(phi),
                                          lower_radius_points * np.sin(phi)]).T + circle_center

            polygon = shapely.geometry.Polygon(np.concatenate([upper_line_points, lower_line_points[::-1, :]]))
            self._segments.append((self._current_port.copy(), polygon, abs(angle) * radius))

            endpoint = shapely.geometry.Point(radius * np.cos(end_angle) + circle_center[0],
                                              radius * np.sin(end_angle) + circle_center[1])

            self._current_port.origin = endpoint.coords[0]

        self._current_port.width = final_width
        self._current_port.angle += angle

        # assert self._segments[-1][1].is_valid, \
        #     'Invalid polygon generated: %s' % shapely.validation.explain_validity(self._segments[-1][1])

        return self

    def add_contactpad(self, angle_pad, width_pad, length_pad, offset_x, offset_y, **kargs):


        self._current_port.set_port_properties(**kargs)

        polygon1 = shapely.geometry.Polygon(([length_pad, -width_pad/2],
                                             [length_pad, +width_pad/2],
                                             [0, width_pad/2],
                                             [0, -width_pad/2]))

        polygon1 = shapely.affinity.rotate(polygon1, self.angle + angle_pad, origin=[0, 0], use_radians=True)
        polygon1 = shapely.affinity.translate(polygon1, self.x, self.y)

        self._segments.append((self._current_port.copy(), polygon1, length_pad))

        endpoint = shapely.geometry.Point([length_pad + offset_y, offset_x])
        endpoint = shapely.affinity.rotate(endpoint, self.angle + angle_pad, origin=[0, 0], use_radians=True)
        endpoint = shapely.affinity.translate(endpoint, self.x, self.y)

        #print(endpoint.coords[0])

        self._current_port.origin = endpoint.coords[0]
        self._current_port.width = width_pad
        self._current_port.angle = self.angle + angle_pad

        ### sorry. I have to get angle and origin right :-(

        return self

    def add_tree(self, angle_tree, width_tree, length_tree, offset, distance, n_branch, **kargs):

        self._current_port.set_port_properties(**kargs)

        for i in np.linspace(0, n_branch, n_branch):
            polygon1 = shapely.geometry.Polygon(([length_tree/2 , width_tree/2+ offset + i* distance],
                                                 [-length_tree/2, width_tree/2+ offset + i* distance],
                                                 [-length_tree/2, -width_tree/2+ offset + i* distance],
                                                 [length_tree/2, -width_tree/2+ offset + i* distance]))

            polygon1 = shapely.affinity.rotate(polygon1, self.angle + angle_tree, origin=[0, 0], use_radians=True)
            polygon1 = shapely.affinity.translate(polygon1, self.x, self.y)

            self._segments.append((self._current_port.copy(), polygon1, length_tree))


        self._current_port.width = self.width
        self._current_port.angle = self.angle

        return self








