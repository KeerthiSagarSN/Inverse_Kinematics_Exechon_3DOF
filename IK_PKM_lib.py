# -*- coding: utf-8 -*-
"""
Created on Fri Nov 09 05:32:10 2018

@author: keerthi
"""

# Class to compute the inverse kinematics for the PKM...
from euler_inverse import euler_inverse
from numpy import math,shape, zeros
from math import sin, cos, atan2, pow, sqrt
from numpy import pi as pi
from numpy import mat,matrix,vstack,hstack
from IK_PKM_params import IK_PKM_params
from numpy.linalg import inv as inv


from visualbuild_v2 import revolute_joint, link_generate, prismatic_joint
from visualbuild_v2 import tripod_plot, spherical_joint, vector_plot, plane_plot,kinematics_plot,point_plot,line_plot
from visualbuild_v2 import coordinate_vector,equilateral_triangle,plane_lineaxis

import numpy as np
from linearalgebra import V_unit,V_ang,check_ndarray

