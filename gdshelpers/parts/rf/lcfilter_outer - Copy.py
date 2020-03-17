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
from gdshelpers.parts.electrodeline import Electrodeline
from gdshelpers.helpers import find_line_intersection, normalize_phase
from gdshelpers.geometry import geometric_union


"""
	def __init__(self, origin, angle, width,
                 spiral_width=4.5,
                 spiral_radius=10., spiral_factor=6., spiral_n=4.,
                 base_length=300., base_width=250.,
                 trunk_width=20., trunk_length=250.,
                 tree_width=3.5, tree_length=240., tree_offset=5., tree_distance=8., tree_branch=30):
"""
class LCfilter_outer(object):				 
    def __init__(self, origin, angle, width, spiral_width = 4.5,
                 spiral_radius=10., spiral_factor=7., spiral_n=3,
                 base_length=300., base_width=250.,
                 trunk_width=10., trunk_length=250.,
                 tree_width=2.5, tree_length=300., tree_offset=5., tree_distance=8., tree_branch=30):
				 
		self._origin_port = Electrodeport(origin, angle, width)
        self.spiral_radius = spiral_radius
        self.spiral_factor = spiral_factor
        self.spiral_n = spiral_n
        self.spiral_width = spiral_width

        self.base_length = base_length
        self.base_width = base_width

        self.trunk_width = trunk_width
        self.trunk_length = trunk_length

        self.tree_width = tree_width
        self.tree_length = tree_length
        self.tree_offset = tree_offset
        self.tree_distance = tree_distance
        self.tree_branch = tree_branch

    @classmethod
    def make_at_port(cls, port, **kwargs):
        default_port_param = dict(port.get_parameters())
        default_port_param.update(kwargs)
        del default_port_param['origin']
        del default_port_param['angle']
        del default_port_param['width']

        return cls(port.origin, port.angle, port.width, **default_port_param)

    ###
    # Let's allow the user to change the values
    # hidden in _origin_port. Hence the internal use
    # of a Port is transparent.
    @property
    def origin(self):
        return self._origin_port.origin

    @origin.setter
    def origin(self, origin):
        self._origin_port.origin = origin

    @property
    def angle(self):
        return self._origin_port.angle

    @angle.setter
    def angle(self, angle):
        self._origin_port.angle = angle

    @property
    def width(self):
        return self._origin_port.trunk_width

    @width.setter
    def width(self, width):
        self._origin_port.width = width

    @property
    def port(self):
        angle = self.angle
        offset = self.base_length + 2*(self.spiral_radius+self.spiral_factor*(self.spiral_n+1.))+self.spiral_radius + self.trunk_length + 20 -self.spiral_radius-self.spiral_factor*(self.spiral_n+1.)
        deltax = offset * np.cos(angle)
        deltay = offset * np.sin(angle)
        self._origin_port.width = self.trunk_width
        return self._origin_port.deltax(deltax).deltay(deltay)

    def get_shapely_object(self):
        LCpath_outer = Electrodeline.make_at_port(self._origin_port)

        LCpath_outer.longitudinal_offset(offset=-self.spiral_radius-self.spiral_factor*(self.spiral_n+1.))

        LCpath_outer.add_contactpad(angle_pad=0, width_pad=self.base_width + 2*(self.spiral_radius+self.spiral_factor*(self.spiral_n+1.)),
                                    length_pad=self.base_length + 2*(self.spiral_radius+self.spiral_factor*(self.spiral_n+1.))+self.spiral_radius,
                                    offset_x=0, offset_y=0)
        LCpath_outer.add_contactpad(angle_pad=0, width_pad=self.trunk_width,
                                    length_pad=20., offset_x=0,
                                    offset_y=0)

        LCpath_outer.add_straight_segment(length=self.trunk_length)

        LCpath_outer.add_tree(angle_tree=np.pi/2, width_tree=self.tree_width, length_tree=self.tree_length, offset=self.tree_offset, distance=self.tree_distance, n_branch=self.tree_branch)

        return geometric_union([LCpath_outer])








