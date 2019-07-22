# -*- coding: utf-8 -*-
"""
Created on Mon Aug 28 13:23:15 2017

@author: Keerthi
"""

from sympy import init_printing
init_printing(use_latex='mathjax',pretty_print = False)
from sympy.solvers import solve

from sympy.matrices import Matrix,eye
from matplotlib import pyplot as plt

# Scalar variables imported from Sympy.abc library
from sympy.abc import a,b,c,d,e,f,g,h,l, theta
from sympy import sin,cos,tan,pi,acos,asin,atan
from sympy import Symbol
from sympy import symbols
from sympy.vector import CoordSys3D
# For profiling

import cProfile
from sympy import lambdify
# Creating Reference frame 
from sympy.vector import CoordSys3D,express
#from sympy.physics.vector import magnitude

from mpl_toolkits.mplot3d import Axes3D
from linearalgebra import R_Rod_sympy # Rodriguez Rotation formula in sympy format
import numpy as np

# Reference Frame
O = CoordSys3D('O')
P = CoordSys3D('P')
#P = O.locate_new('P',)
'''
P = CoordSys3D('P')
hx = Symbol('hx')
hz = Symbol('hz')
'''
#A = N.orientnew('A','Axis',(theta,N.z))


# Vectors declaration
v_a2,v_norm2_unit,v_norm1,v_norm2,v_ra,v_a,OS,OSP,OB,OC,OA,OP,PO_ij,PO_jk,N_ij,n_PO_jk,n_PO_jk,OD,BD,AE,DP,PE,PR = symbols('v_a2 v_norm2_unit v_norm1 v_norm2 v_ra v_a OS OSP OB OC OA OP PO_{ij} PO_{jk} N_{ij} n_{PO_{ij}} n_{PO_{jk}} OD BD AE DP PE PR') # PR - Random vector with tail at P
# Length declaration
alpha,alpha1,alpha2,l_A,l_C,l_B,l_PF,l_PG,l_CD,l_AE, l_PH, hx, hz= symbols('alpha alpha1 alpha2 l_A l_C l_B l_{PF} l_{PG} l_{CD} l_{AE} l_{PH} hx hz')
# User defined points
s_x,s_y,s_z = symbols('s_x s_y s_z')
# Random vector cefficient
r_x,r_y,r_z = symbols('r_x r_y r_z')

p_f,p_g,p_h = symbols('p_f p_g p_h')
PF,PG,PH = symbols('PF PG PH')
OF,OG,OH = symbols('OF OG OH')


# SUbsitution below
# Unit vector

hx = 0.2828
hz = -0.2
pa = -0.1523
pc = 0.2523
pb = 0.1324
l_a = -0.4434
l_b = 0.3455
l_c = 0.7998

# Ground points
OA_subs = -0.4434*O.j
OC_subs = 0.7798*O.j
OB_subs = 0.3455*O.i

delt_A = 1
delt_1_B = 1
delt_2_B = 1

OS_subs = 0.02*O.i + 0.7*O.j + 1.02*O.k
#OS_subs =1.02*O.k

O_PO_S = OS_subs.dot(O.j)
O_PO_S_subs = O_PO_S*(O.j)
PO_S_subs = OS_subs - O_PO_S_subs

n_PO_S_subs = PO_S_subs/PO_S_subs.magnitude()

OSP_subs = (OS_subs.magnitude() + hz)*n_PO_S_subs

v_ra_subs = -1*O.i # Random vector

# Vector between OS and OB

BS_subs = OS_subs - OB_subs

#Perpendicular vector between BS_subs and OS_subs

v_norm1_BS_OS = BS_subs.cross(PO_S_subs)

# Perpendicular vector between OSP and v_ra

v_norm1_subs = v_norm1_BS_OS.cross(PO_S_subs)

# Perpendicular vector between OSP and v_norm1
#v_norm2_subs = OSP_subs.cross(v_norm1_subs)

# Unit vector
v_norm1_unit_subs = v_norm1_subs/v_norm1_subs.magnitude()
# Get length of the vector
OSP_P_v_subs = -hx*v_norm1_unit_subs 

OSP_P_subs = OSP_subs + OSP_P_v_subs

# Projected point

OSP_P_subs_proj = OSP_P_subs - ((OSP_P_subs - O.i).dot(O.j))*(O.j)

#alpha1 = acos((OSP_P_subs_proj.dot(1*O.i))/OSP_P_subs_proj.magnitude())
alpha1 = acos((OSP_P_subs.dot(1*O.i))/OSP_P_subs.magnitude())

#v_u_subs = OSP_P_subs_proj/OSP_P_subs_proj.magnitude()
v_u_subs = OSP_P_subs/OSP_P_subs.magnitude()


l_12_A = 0.1023*v_u_subs # Get the length of the links along the unit vector
l_12_C = 0.1523*v_u_subs # Vector direction of link CD

O_P2_A_subs = OA_subs + l_12_A
O_P2_C_subs = OC_subs + l_12_C



O_P5_B_subs = OSP_P_subs + (pb*v_norm1_unit_subs)

k5B = v_norm1_BS_OS/v_norm1_BS_OS.magnitude()

k5B_unit = k5B/k5B.magnitude()

O_P4_A_subs = OSP_P_subs + (pa)*(k5B_unit)
O_P4_C_subs = OSP_P_subs + (pc)*(k5B_unit)



# dummy point
O_OSP_P_Dum = OSP_subs + (pc)*(k5B_unit)



# Check parallel

dum1 = O_P4_A_subs - O_P4_C_subs
dum2 = OSP_subs

alpha2 = acos((dum2.dot(k5B_unit))/(dum2.magnitude())*(k5B_unit.magnitude()))



def get_vector(V,O): # Get the vector and the coordinate system
    V_new = np.empty(shape = (1,3))
    V_new[0,0] = V.coeff(O.i)
    V_new[0,1] = V.coeff(O.j)
    V_new[0,2] = V.coeff(O.k)
    
    return V_new
    


def plot_line(V1,V2,color_in):
    #color1 = str(color_in)
    ax.plot([V1[0,0],V2[0,0]],[V1[0,1],V2[0,1]],[V1[0,2],V2[0,2]],c = color_in)
    return ax


def plot_pt(V1,color_in):
    ax.plot()

fig = plt.figure()
ax = Axes3D(fig)
ax.axis('equal')     



# PLot all the points first


orig = np.empty(shape=(1,3))
orig[0,0] = 0
orig[0,1] = 0
orig[0,2] = 0

z_axis = np.empty(shape=(1,3))
z_axis[0,0] = 0
z_axis[0,1] = 0
z_axis[0,2] = 2
# Plot Point OC

plot_line(get_vector(OC_subs,O),orig,'r') # Plot  axis line between O and C
plot_line(get_vector(OA_subs,O),orig,'r') # Plot  axis line between O and A
plot_line(get_vector(OB_subs,O),orig,'r') # Plot  axis line between O and B

plot_line(get_vector(OA_subs,O),get_vector(O_P2_A_subs,O),'b') # Plot  axis line between C and D

plot_line(get_vector(OC_subs,O),get_vector(O_P2_C_subs,O),'b') # Plot  axis line between D and F

plot_line(get_vector(O_P2_A_subs,O),get_vector(O_P4_A_subs,O),'b') # Plot  axis line between F and G

plot_line(get_vector(O_P2_C_subs,O),get_vector(O_P4_C_subs,O),'b') # Plot  axis line between F and H

plot_line(get_vector(O_P4_A_subs,O),get_vector(O_P4_C_subs,O),'b') # Plot  axis line between G and H

plot_line(get_vector(O_P4_A_subs,O),get_vector(O_P5_B_subs,O),'b') # Plot  axis line between G and E

plot_line(get_vector(O_P4_C_subs,O),get_vector(O_P5_B_subs,O),'b') # Plot  axis line between E and A


plot_line(get_vector(OB_subs,O),get_vector(O_P5_B_subs,O),'b') 


plot_line(get_vector(OB_subs,O),get_vector(OS_subs,O),'k') 

plot_line(orig,get_vector(OS_subs,O),'k') 

plot_line(get_vector(OSP_subs,O),get_vector(O_OSP_P_Dum,O),'k') 

plot_line(get_vector(OSP_P_subs,O),orig,'k') 

plot_line(z_axis,orig,'k') 



'''
plot_line(get_vector(OH_subs,O),get_vector(OB_subs,O),'b') # Plot  axis line between H and B
'''


OSP_P_pl2 = get_vector(OA_subs,O)
ax.scatter3D(OSP_P_pl2[0,0],OSP_P_pl2[0,1],OSP_P_pl2[0,2],c = 'r')



OSP_P_pl3 = get_vector(OB_subs,O)
ax.scatter3D(OSP_P_pl3[0,0],OSP_P_pl3[0,1],OSP_P_pl3[0,2],c = 'r')



OSP_P_pl4 = get_vector(OC_subs,O)
ax.scatter3D(OSP_P_pl4[0,0],OSP_P_pl4[0,1],OSP_P_pl4[0,2],c = 'r')

OSP_P_pl5 = get_vector(O_P5_B_subs,O)
ax.scatter3D(OSP_P_pl5[0,0],OSP_P_pl5[0,1],OSP_P_pl5[0,2],c = 'm')

OSP_P_pl6 = get_vector(O_P4_A_subs,O)
ax.scatter3D(OSP_P_pl6[0,0],OSP_P_pl6[0,1],OSP_P_pl6[0,2],c = 'pink')

OSP_P_pl7 = get_vector(O_P4_C_subs,O)
ax.scatter3D(OSP_P_pl7[0,0],OSP_P_pl7[0,1],OSP_P_pl7[0,2],c = 'pink')

OSP_P_pl8 = get_vector(O_P2_C_subs,O)


ax.scatter3D(orig[0,0],orig[0,1],orig[0,2],c = 'k')



OSP_P_pl = get_vector(OSP_P_subs,O)
ax.scatter3D(OSP_P_pl[0,0],OSP_P_pl[0,1],OSP_P_pl[0,2],c = 'cyan')
#ax.scatter3D(0.8200,0.0136,0.69515,c = 'cyan')

OSP_P1_pl = get_vector(OSP_subs,O)
ax.scatter3D(OSP_P1_pl[0,0],OSP_P1_pl[0,1],OSP_P1_pl[0,2],c = 'b')
#OH_pt = get_vector(OH_subs,O)
#ax.scatter3D(OH_pt[0,0],OH_pt[0,1],OH_pt[0,2],c='g')
OS_pt = get_vector(OS_subs,O)
ax.scatter3D(OS_pt[0,0],OS_pt[0,1],OS_pt[0,2],c='y')
