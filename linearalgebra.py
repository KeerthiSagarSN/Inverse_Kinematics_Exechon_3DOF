# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 17:34:17 2016

@author: keerthi
"""

# Rotation Matrix definitions
import numpy as np
import math
from sympy import sqrt


# Rotation around X axis
# Angle in degrees
def R_X(ang):
    return np.matrix([[1, 0, 0], [0, math.cos(math.radians(ang)), -math.sin(math.radians(ang))], [0, math.sin(math.radians(ang)), math.cos(math.radians(ang))] ])
        
    
# Rotation around Y axis
# Angle in degrees
def R_Y(ang):
    return np.matrix([[math.cos(math.radians(ang)), 0, math.sin(math.radians(ang))], [0, 1, 0], [-math.sin(math.radians(ang)), 0, math.cos(math.radians(ang))] ])
 
# Rotation around Y axis
# Angle in degrees
def R_Z(ang):
    return np.matrix([[math.cos(math.radians(ang)), -math.sin(math.radians(ang)), 0], [math.sin(math.radians(ang)), math.cos(math.radians(ang)), 0], [0, 0, 1]])

# General Rotation matrix

def R_r(angx, angy, angz):
    return R_X(angx)*R_Y(angy)*R_Z(angz)

# Module/ Magnitude of a vector
def V_mod(V):
    # Square root 
    return np.linalg.norm(V)


def V_npcheck(V):
    if type(V).__module__ == np.__name__ : # Chck if numpy element if not change as numpy element
        Vnew = V
        #print('Numpy array')
    else:
        Vnew = np.array(V)
        #print('Not numpy array')
    return Vnew


# unit vector of a vector V

def V_unit(V):
    # V_norm = V/V_mod
    V = V_npcheck(V) # Check if numpy array
    
    all_zeros = not np.any(V)
    if all_zeros:
        return V
    else:
         return V*(np.power(V_mod(V),-1))
        

# Unit vector of a Vector S

def S_unit(S):
    # S_norm = S/S_mod
    S = V_npcheck(S)# Check if nummpy aarray
    
    all_zeros = not np.any(V)
    if all_zeros:
        return S
    else:
        S = np.matrix(S)
        
        return S*(np.power(S_mod(S),-1))    
    
 # Angle between two vectors
def V_ang(V1,V2):
    # V1.V2 = |V1||V2|cos(tet)
    # Angle in radians
       
    return np.arccos(np.clip(np.dot(V_unit(V1), V_unit(V2)), -1.0, 1.0))
    

# Skew Matrix

def V_skew(V):
    
    return
    
# Sign of the vector basis
# m,n indicates the X and y plane
def V_sign(V1,m,n):
    if (len(V1) > 1) and (len(V2) > 1):
        V1_msign = np.sign(V1[m])
        V1_nsign = np.sign(V1[n])
        return V1_msign, V1_nsign
        #return V1_ysign = np.sign(V1[1])
    
        

# Angle of 2 Vector when no direction is  given between two vectors - XY Plane - Counterclockwise

def V_angxy(V1,V2):
    if (len(V1) > 1) and (len(V2) > 1):
       m = 0
       n = 1
       V1_sign = V_sign(V1,m,n)
       V2_sign = V_sign(V2,m,n)
       sign_xy = np.concatenate((V1_sign,V2_sign),axis = 0)
       V1_xy = (V1[m],V1[n])
       V2_xy = (V2[m],V1[n])
       # X coordinate  
       ang_xy = V_ang(V1_xy,V2_xy)
       return ang_xy,sign_xy
    else:
        print ('Vector size is not 2 Dimensional, Enter a 2D size vector')
        
# Angle of 2 Vector when no direction is  given between two vectors - YZ Plane - Counterclockwise
def V_angyz(V1,V2):
    if (len(V1) > 2) and (len(V2) > 2):
       m = 1
       n = 2
       V1_sign = V_sign(V1,m,n)
       V2_sign = V_sign(V2,m,n)
       sign_yz = np.concatenate((V1_sign,V2_sign),axis = 0)
       V1_yz = (V1[m],V1[n])
       V2_yz = (V2[m],V1[n])
       # X coordinate  
       ang_yz = V_ang(V1_yz,V2_yz)
       return ang_yz,sign_yz
    else:
        print ('Vector size is not 3 Dimensional, Enter a 3D size vector')       

# Angle of 2 Vector when no direction is  given between two vectors - XZ Plane - Counterclockwise
def V_angxz(V1,V2):
    if (len(V1) > 2) and (len(V2) > 2):
       m = 0
       n = 2
       V1_sign = V_sign(V1,m,n)
       V2_sign = V_sign(V2,m,n)
       sign_xz = np.concatenate((V1_sign,V2_sign),axis = 0)
       V1_xz = (V1[m],V1[n])
       V2_xz = (V2[m],V1[n])
       # X coordinate  
       ang_xz = V_ang(V1_xz,V2_xz)
       return ang_xz,sign_xz
    else:
        print ('Vector size is not 3 Dimensional, Enter a 3D size vector')             
    
     #X Coordinate



    
# Determinant of two vectors - 2D Vector
def V_det(V1,V2):
    V = [V1[0:2],V2[0:2]]
    return np.linalg.det(V)
    
    
# Transpose of a matrix
def V_transpose(V1):
    V1 = np.matrix(V1)
    return np.transpose(V1)
    

def V_affine(angx,angy,angz,P0,PN,PV,dim):
    # Transforming the new point to origin point
    P = np.array(P0 - PN)
    R_mat = R_r(angx,angy,angz)
    M = np.hstack([R_mat,P])
    if dim == 2:
        newrow = np.array([[0,0,1]])
    if dim == 3:
        newrow = np.array([[0,0,0,1]])
    M = np.vstack([M,newrow])        
    PV = V_transpose(PV)
    PV = np.vstack([PV,1])
    A = M*PV
    return A
    

# Creating offset of a vector (Line segment)    
def V_offset2d(P1,P2,dist):
#    P1 = np.array([[0,0]])
#    P2 = np.array([[0,5]])
#    dist = 5
    V = np.zeros([2,1])
    V[0,0] = (P2[0,0] - P1[0,0])
    V[1,0] = (P2[0,1] - P1[0,1])
    V = np.matrix(V)
    Vmat = np.matrix([[P1[0,0],P2[0,0]],[P1[0,1],P2[0,1]]]) 
    l = V_mod(V)*1.0 # Magnitude of the vector
    crmat = np.matrix([[0,-1],[1,0]]) # Unit normal vector matrix
    
    Voff = (crmat*V)/(l)*(1.0) # Normal unit vector
    Vn = Vmat + Voff # Offset points
    Vn = np.transpose(Vn)
    Vn = np.array(Vn)
    magV = l # Magnitude of the vector
    return Vn
    
    
# Scaling a vector- when vector is give
def V_scale(V,scale):
    V = V_npcheck(V) # Check if numpy array
    V_uni = V_unit(V) # Get the unit vector
    V_scal = scale*V_uni
    return V_scal
    
    
        

# When two points are given
def V_scalep(P1,P2,scale):
    
    P1 = V_npcheck(P1)
    P2 = V_npcheck(P2)
    V = P2 - P1
    V_uni = V_unit(V)
    V = scale*V_uni
    return V
# Round matrix with given direction





    
#def M_round(M):
#    
#    return

      
# Homogeneous matrix
def H_mat(T):
    H_mat = np.zeros(shape = [4,4])
    H_mat[0,0] = 1
    H_mat[0,1] = 0
    H_mat[0,2] = 0
    H_mat[0,3] = T[0,0]
    H_mat[1,0] = 0
    H_mat[1,1] = 1
    H_mat[1,2] = 0
    H_mat[1,3] = T[1,0]
    H_mat[2,0] = 0
    H_mat[2,1] = 0
    H_mat[2,2] = 1
    H_mat[2,3] = T[2,0]

# Rodriguez rotation matrix - Only when axis is not parallel to basis vector axis - x,y,z -Axis is parallel and offset to basis axis
def R_Rod(teta,v_axis):
    # I - Identity matrix 3x3
    # v_axis should be a matrix of dimension 3x1
    # Check size of the v_axis matrix
    if np.shape(v_axis)[0] == 1 and np.shape(v_axis)[1] == 3: # Means we need to transpose
        v_axis = np.transpose(v_axis)
    print (v_axis)
    
    I = np.identity(3)

    
    # The cross product matrix - Check wikipedia - Rodriguez rotation formula for reference
    K = np.zeros(shape = [3,3])
    K[0,0] = 0
    K[1,1] = 0
    K[2,2] = 0
    K[0,1] = -1*v_axis[2,0]
    K[0,2] = v_axis[1,0]
    K[1,0] = v_axis[2,0]
    K[1,2] = -1*v_axis[0,0]
    K[2,0] = -1*v_axis[1,0]
    K[2,1] = v_axis[0,0]
    R_Rod = I + math.sin(math.radians(teta))*K + (1- math.cos(math.radians(teta)))*np.dot(K,K)
    return R_Rod

# Rodriguez rotation matrix: Sympy version
def R_Rod_sympy(teta,v_axis,O): # Angle (teta) in radians, axis (v_axis), coordinate system(O)
    from sympy.matrices import eye,Matrix
    from sympy import sin,cos,tan,pi,acos,asin
    from sympy.vector import CoordSys3D
    I = eye(3)
    
    # The cross product matrix
    K = Matrix(([0,-v_axis.coeff(O.k),v_axis.coeff(O.j)],[v_axis.coeff(O.k),0,-v_axis.coeff(O.i)],[-v_axis.coeff(O.j),v_axis.coeff(O.i),0]))
    R_Rod_sym = I + sin(teta)*K + (1-cos(teta))*(K*K)
    return R_Rod_sym
# Translation of the point 

def T_pt(inp_mat,disp_mat):
    trans_mat = np.zeros(shape = [3,1])
    trans_mat = inp_mat + disp_mat
        
    return trans_mat


# Translating a point to the origin
def T_orig(p):
    D = np.identity(4)
    D[0,3] = -1*p[0,0]
    D[1,3] = -1*p[1,0]
    D[2,3] = -1*p[2,0] 
    return D


def Transl(i,j,k,pt):
    pt = np.matrix(pt)
    pt = np.transpose(pt)
    D = np.identity(4)
    D[0,3] = i
    D[1,3] = j
    D[2,3] = k
    # pt is input as  3x1 matrix need to convert it into 4x1 matrix to perform multiplication with 4x4 and 4x1
    dd = np.zeros(shape=[1,1])
    dd[0,0] = 1
    pt = np.append(pt,dd,axis = 0)
    Td = np.dot(D,pt)
    Td = Td[0:-1]
    Td = np.transpose(Td)
    Td = np.asarray(Td)
    Td = Td[0]
    return Td
# Rotation matrix about an arbitrary line - Multiply this matrix with the point to get Rotated point
# Teta - angle, l_p1 - First point on the line-axis to rotate, l_p2 - Second point on the line axis to rotate
def R_L(teta,l_p1,l_p2):
    v_axis = l_p2 - l_p1
    # Translate the point to the origin
    D = T_orig(l_p1)
    # Rodriguez rotation matrix
    R_rod = R_Rod(teta,V_unit(v_axis))
    # Rodriguez Rotation matrix is 3x3, but the entire Rotation matrix is 4x4
    endrow = np.zeros(shape = [1,3])
    R_rod = np.append(R_rod,endrow,axis = 0)
    endcolumn = np.zeros(shape = [4,1])
    endcolumn[3,0] = 1
    R_rod = np.append(R_rod,endcolumn,axis = 1)
    # Final rotation matrix
    R_rot = np.dot(np.linalg.inv(D),np.dot(R_rod,D))
    return R_rot
    # Step 1 : Translate point to origin:
        
# Given two points - Gets the slope of the line between them (2D):
def slope_L(p1,p2):
    slp = ((p2[0,1] - p1[0,1])*math.pow((p2[0,0] - p1[0,0]),-1)) 
    return slp # Slope of a line , m = (y2-y1)/(x2-x1)

# Given starting point, slope and distance between the two points required - Find the new point on the line
def newpoint_L(p1,p2,m,d): # Input arguments - p1 - Initial point, m - slope, d - distance between initial point and the required/output point
    from scipy.optimize import fsolve
    import math
    import numpy as np
    x1 = p1[0]
    y1 = p1[1]
    x0 = p2[0]
    y0 = p2[1]
    def equations(p,x1,y1,m,d):
        x,y = p
        return ((x-x1)**2 + (y-y1)**2 - d**2, (y-y1) - m*(x-x1) - 0) # Euclidean distance formula and slope equation
    
    x,y = fsolve(equations,(x0,y0),args = (x1,y1,m,d))
    p2 = np.zeros(shape = [1,2])
    p2[0,0] = x
    p2[0,1] = y
    return p2

# Given a vector, and one point, find the other point of the vector
def newpoint_V(p1,v1):
    import numpy as np
    p2 = np.copy(p1)
    p2[0,0] = p1[0,0] - v1[0,0]
    p2[0,1] = p1[0,1] - v1[0,1]
    p2[0,2] = p1[0,2] - v1[0,2]
    return p2

# Given a 3d circle with its axis vector and radius, get the shortest point on the 3d circle from a given desired point. Also specify the distance
# Ref: https://www.geometrictools.com/Documentation/DistanceToCircle3.pdf
def shortest_3dcircle_point(circle_axis,circle_centre,radius,p1):
    import numpy as np
    # Shortest point on the circle from the given point
    print('p1 isssss')
    print(p1)
    print(np.shape(p1))
    print('circle centre')
    print(circle_centre)
    print(np.shape(circle_centre))
    delt = p1 - circle_centre
    print('deltttt')
    print(delt)
    print(np.shape(delt))
    N = circle_axis
    print('NNNNNNNNNissssss')
    print(N)
    print(np.shape(N))
    
    print('dottttt')
    print(np.dot(N,delt))
    p_min = circle_centre + (radius*(delt - ((np.dot(N,delt))*N))/(np.linalg.norm(delt - ((np.dot(N,delt))*N))))
    
    dist = np.linalg.norm(p1-p_min)
    print('disttt')
    print(dist)
    
    return p_min,dist


# Check and flatten an n-dimensional array to 1D array 

def check_ndarray(p_in):
    from numpy import ndim,hstack
    if p_in.ndim > 1: # Check if the array is multidimensional
        p1 = p_in.tolist() # Flatten the array/matrix
        p2 = hstack((p1))
    else:
        p2 = p_in
    return p2

    