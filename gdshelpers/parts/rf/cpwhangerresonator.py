from __future__ import print_function, division

import math
import numpy as np
import shapely.geometry
import shapely.affinity
import shapely.validation

from gdshelpers.parts.cpwport import CPWport


class CPWhangerresonator(object):
    def __init__(self, origin, angle, width, spacing):
        self._current_port = CPWport(origin, angle, width, spacing)
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
    def spacing(self):
        return self._current_port.spacing

    @property
    def current_port(self):
        return self._current_port.copy()

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

    def add_cpw_hangerresonator(self, width_r, spacing_r, length, coupling_distance, **kargs):
        self._current_port.set_port_properties(**kargs)

        endpoint = shapely.geometry.Point(length, coupling_distance)

        # define straight region
        polygon1 = shapely.geometry.Polygon(
            ([0, coupling_distance + width_r / 2 + spacing_r], [length, coupling_distance + width_r / 2 + spacing_r],
             [length, coupling_distance + width_r / 2], [0, coupling_distance + width_r / 2]))
        polygon1 = shapely.affinity.rotate(polygon1, self.angle, origin=[0, 0], use_radians=True)
        polygon1 = shapely.affinity.translate(polygon1, self.x, self.y)
        self._segments.append((self._current_port.copy(), polygon1, length))

        polygon2 = shapely.geometry.Polygon(
            ([0, coupling_distance - width_r / 2], [length, coupling_distance - width_r / 2],
             [length, coupling_distance - width_r / 2 - spacing_r], [0, coupling_distance - width_r / 2 - spacing_r]))
        polygon2 = shapely.affinity.rotate(polygon2, self.angle, origin=[0, 0], use_radians=True)
        polygon2 = shapely.affinity.translate(polygon2, self.x, self.y)
        self._segments.append((self._current_port.copy(), polygon2, length))

        # define bottom region
        polygon3 = shapely.geometry.Polygon(
            ([0, coupling_distance + width_r / 2 + spacing_r], [-np.sign(length)*spacing_r, coupling_distance + width_r / 2 + spacing_r],
             [-np.sign(length)*spacing_r, coupling_distance - width_r / 2 - spacing_r], [0, coupling_distance - width_r / 2 - spacing_r]))
        polygon3 = shapely.affinity.rotate(polygon3, self.angle, origin=[0, 0], use_radians=True)
        polygon3 = shapely.affinity.translate(polygon3, self.x, self.y)
        self._segments.append((self._current_port.copy(), polygon3, length))

        endpoint = shapely.affinity.rotate(endpoint, np.sign(length)*self.angle, origin=[0, 0], use_radians=True)
        endpoint = shapely.affinity.translate(endpoint, self.x, self.y)

        self._current_port.origin = endpoint.coords[0]
        self._current_port.width = width_r
        self._current_port.spacing = spacing_r
        return self













