# -*- coding: utf-8 -*-
"""
Created on Fri Nov 09 05:46:04 2018

@author: keerthi
"""

import matplotlib
import numpy as np
import math
from linearalgebra import V_unit,V_mod, R_Rod, newpoint_V, R_X, R_Y, R_Z,check_ndarray,R_L,R_Rod
from mpl_toolkits.mplot3d import Axes3D
from scipy.linalg import norm
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import matplotlib.pyplot as plt
import itertools


from mpl_toolkits.mplot3d import art3d
from mpl_toolkits.mplot3d import proj3d
import numpy as np
import matplotlib
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import art3d
from mpl_toolkits.mplot3d import proj3d
from itertools import product
import copy
from zyz_euler import zyz_euler
