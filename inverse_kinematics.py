# -*- coding: utf-8 -*-
"""
Created on Mon Aug 28 13:23:15 2017

@author: Keerthi
"""

from sympy import init_printing
init_printing(use_latex='mathjax',pretty_print = False)
from sympy.solvers import solve


from matplotlib import pyplot as plt

# Scalar variables imported from Sympy.abc library
from sympy.abc import a,b,c,d,e,f,g,h,l, theta
from sympy import sin,cos,tan,pi,acos,asin
from sympy import Symbol
from sympy import symbols

# For profiling

import cProfile
from sympy import lambdify
# Creating Reference frame 
from sympy.vector import CoordSys3D,express
#from sympy.physics.vector import magnitude

from mpl_toolkits.mplot3d import Axes3D

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
OB,OC,OA,OP,PO_ij,PO_jk,N_ij,n_PO_jk,n_PO_jk,OD,BD,AE,DP,PE,PR = symbols('OB OC OA OP PO_{ij} PO_{jk} N_{ij} n_{PO_{ij}} n_{PO_{jk}} OD BD AE DP PE PR') # PR - Random vector with tail at P
# Length declaration
l_A,l_C,l_B,l_PF,l_PG,l_CD,l_AE, l_PH= symbols('l_A l_C l_B l_{PF} l_{PG} l_{CD} l_{AE} l_{PH}')
# User defined points
p_x,p_y,p_z = symbols('p_x p_y p_z')
# Random vector cefficient
r_x,r_y,r_z = symbols('r_x r_y r_z')


# Ground points
OA = l_A*O.i
OC = -l_C*O.i
OB = l_B*O.j

OP = p_x*O.i + p_y*O.j + p_z*O.k

# Projection of point P onto the plane ij
# Get the normal vector for the plane
N_ij = (O.i).cross(O.j)
PO_ij = OP - ((OP - O.i).dot(N_ij))*N_ij

# Unit vector
n_PO_ij = PO_ij.normalize()

# Projection of point P onto the plane jk
# Get the normal vector for the plane
N_jk = (O.j).cross(O.k)
PO_jk = OP - ((OP - O.k).dot(N_jk))*N_jk
# Unit vector
n_PO_jk = PO_jk.normalize()

# Projection of point P onto the plane jk
# Get the normal vector for the plane
N_jk = (O.j).cross(O.k)
PO_jk = OP - ((OP - O.k).dot(N_jk))*N_jk


OD = OC + l_CD*n_PO_jk
OE = OA + l_AE*n_PO_jk

CD = l_CD*n_PO_jk
AE = l_AE*n_PO_jk


PD = OD - OP
PE = OE - OP

OF = OP + l_P*(PD)
OG = OP + l_P*(PE)


# Take cross product
# SOme random vector
OR = r_x*O.i + r_y*O.j + r_z*O.k
PR = OR - OP 

# Take the cross product between random vector and PG
PG = OG - OP
PF = OF - OP
n_PR_PG = (PR).cross(PG)

# Take the cross product between perpendicular vector and PG
n_n_PG = n_PR_PG.cross(PG)

# FInally identify point H
# Check which of the vector are co-planar
check_coplanar = n_PR_PG.dot(PF.cross(PG))
if check_coplanar == 0: # Means all three vectors are co-planar
    OH = OP + l_PH*n_PR_PG
if check_coplanar != 0:
    OH = OP + l_PH*n_n_PG

BH = OH - OB

# Substitution cases below

#OA = l_A*O.i
OA_subs = OA.subs([(l_A,0.4434)])
#OC = -l_A*O.i
OC_subs = OC.subs([(l_C,0.7798)])
#OB = l_B*O.j
OB_subs = OB.subs([(l_B,0.3455)])

#OP = p_x*O.i + p_y*O.j + p_z*O.k
OP_subs = OP.subs([(p_x,0),(p_y,0.2),(p_z,0.50)]) # PTs: 0.2828,0,-1.2



# Projection of point P onto the plane ij
# Get the normal vector for the plane
N_ij_subs = (O.i).cross(O.j)
PO_ij_subs = OP_subs - ((OP_subs - O.i).dot(N_ij_subs))*N_ij_subs

# Unit vector
n_PO_ij_subs = PO_ij_subs.normalize()

# Projection of point P onto the plane jk
# Get the normal vector for the plane
N_jk_subs = (O.j).cross(O.k)
PO_jk_subs = OP_subs - ((OP_subs - O.k).dot(N_jk_subs))*N_jk_subs
# Unit vector
n_PO_jk_subs = PO_jk_subs.normalize()

# Projection of point P onto the plane jk
# Get the normal vector for the plane
N_jk_subs = (O.j).cross(O.k)
PO_jk_subs = OP_subs - ((OP_subs - O.k).dot(N_jk_subs))*N_jk_subs


OD = OC_subs + l_CD*n_PO_jk_subs
OD_subs = OD.subs([(l_CD,0.1023)])
OE = OA_subs + l_AE*n_PO_jk_subs
OE_subs = OE.subs([(l_AE,0.1523)])

CD = l_CD*n_PO_jk_subs
CD_subs = CD.subs([(l_CD,0.1023)])
AE = l_AE*n_PO_jk_subs
AE_subs = AE.subs([(l_AE,0.1523)])


PD_subs = OD_subs - OP_subs
PE_subs = OE_subs - OP_subs

OF = OP_subs + l_PF*(PD_subs)
OF_subs = OF.subs([(l_PF,0.1523)])
OG = OP_subs + l_PG*(PE_subs)
OG_subs = OG.subs([(l_PG,0.2523)])


# Take cross product
# SOme random vector
OR = r_x*O.i + r_y*O.j + r_z*O.k
OR_subs = OR.subs([(r_x,0.100),(r_y,0.350),(r_z,0.69)])
PR_subs = OR_subs - OP_subs 

# Take the cross product between random vector and PG
PG_subs = OG_subs - OP_subs
PF_subs = OF_subs - OP_subs
n_PR_PG_subs = (PR_subs).cross(PG_subs)

# Take the cross product between perpendicular vector and PG
n_n_PG_subs = n_PR_PG_subs.cross(PG_subs)

# FInally identify point H
# Check which of the vector are co-planar
check_coplanar = n_PR_PG_subs.dot(PF.cross(PG_subs))
if check_coplanar == 0: # Means all three vectors are co-planar
    OH = OP_subs + l_PH*n_PR_PG_subs
    OH_subs = OH.subs([(l_PH,0.1324)])
if check_coplanar != 0:
    OH = OP_subs + l_PH*n_n_PG_subs
    OH_subs = OH.subs([(l_PH,0.1324)])

BH_subs = OH_subs - OB_subs

# Use the same expression as below to get the coefficients for the 
OC_subs.coeff(O.i)

# 




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

# PLot all the points first


orig = np.empty(shape=(1,3))
orig[0,0] = 0
orig[0,1] = 0
orig[0,2] = 0
# Plot Point OC

plot_line(get_vector(OC_subs,O),orig,'r') # Plot  axis line between O and C
plot_line(get_vector(OA_subs,O),orig,'r') # Plot  axis line between O and A
plot_line(get_vector(OB_subs,O),orig,'r') # Plot  axis line between O and B

plot_line(get_vector(OC_subs,O),get_vector(OD_subs,O),'b') # Plot  axis line between C and D

plot_line(get_vector(OD_subs,O),get_vector(OF_subs,O),'b') # Plot  axis line between D and F

plot_line(get_vector(OF_subs,O),get_vector(OG_subs,O),'k') # Plot  axis line between F and G

plot_line(get_vector(OF_subs,O),get_vector(OH_subs,O),'k') # Plot  axis line between F and H

plot_line(get_vector(OG_subs,O),get_vector(OH_subs,O),'k') # Plot  axis line between G and H

plot_line(get_vector(OG_subs,O),get_vector(OE_subs,O),'b') # Plot  axis line between G and E

plot_line(get_vector(OE_subs,O),get_vector(OA_subs,O),'b') # Plot  axis line between E and A

plot_line(get_vector(OH_subs,O),get_vector(OB_subs,O),'b') # Plot  axis line between H and B

plot_line(get_vector(OH_subs,O),get_vector(OB_subs,O),'b') # Plot  axis line between H and B
OP_pl = get_vector(OP_subs,O)
#ax.scatter3D(OP_pl[0,0],OP_pl[0,1],OP_pl[0,2],c = 'r')
OH_pt = get_vector(OH_subs,O)
ax.scatter3D(OH_pt[0,0],OH_pt[0,1],OH_pt[0,2],c='g')



    

'''

S_O_coef = [0.02,0.7,1.02]
P_1_A_coef = [0,-0.4434,0]
P_1_C_coef = [0,0.4434,0]
P_1_B_coef = [0.3455,0,0]

# Input coordinates of the spherical joint in the base frame
S_O = S_O_coef[0]*O.i + S_O_coef[1]*O.j + S_O_coef[2]*O.k

h = [0.2828,0,-0.2]

P_1_A = P_1_A_coef[0]*O.i + P_1_A_coef[1]*O.j + P_1_A_coef[2]*O.k
P_1_C = P_1_C_coef[0]*O.i + P_1_C_coef[1]*O.j + P_1_C_coef[2]*O.k
P_1_B = P_1_B_coef[0]*O.i + P_1_B_coef[1]*O.j + P_1_B_coef[2]*O.k

# Angles
alph1,alph2,beta,alpha = symbols('alph1 alph2 beta alpha')

alph1 = (S_O.dot(-1*O.i))/((S_O.magnitude())*(O.i.magnitude()))

alph2 = (S_O.dot(-1*O.i))/((S_O.magnitude())*(O.i.magnitude()))


S_P_O = S_O.normalize()*(S_O.magnitude() - h[2])

P_1_B_S = (S_O_coef[0] - P_1_B_coef[0])*O.i + (S_O_coef[1] - P_1_B_coef[1])*O.j + (S_O_coef[2] - P_1_B_coef[2])*O.k


k_5_B = S_O.cross(P_1_B_S)

k_2_A = k_5_B.cross(S_O)

k_1_A = P_1_A.normalize()
k_1_C = k_1_A

k_1_B = P_1_B.normalize() 
# Input coordinates of the spherical joint in the platform frame

p_A_coef = -0.1523
p_B_coef = 0.1324
p_C_coef = 0.2523



# Get the coordinate of the point P
P_loc = S_P_O + h[0]*k_2_A 
P = O.locate_new('P',P_loc)

P_4_A = p_A_coef*P.j
P_5_B = p_B_coef*P.i
P_4_C = p_C_coef*P.j   

alpha = (P_loc.dot(-1*O.i))/((P_loc.magnitude())*(O.i.magnitude()))



l_12_A = 0.1023
l_12_C = 0.1523


P_2_A = P_1_A + (l_12_A)*(P_loc.normalize())
P_2_C = P_1_C + (l_12_C)*(P_loc.normalize())
'''



