# -*- coding: utf-8 -*-
"""
Created on Tue Nov 13 23:15:03 2018

@author: keerthi
"""

def euler_angles(alpha,beta,gamma,euler_angle_type): # 0: XYZ euler angle, 1: ZYZ euler angle type
    from numpy import sin,cos,zeros
    
    
    # Compute sines and cosines
    s1 = sin(alpha)
    c1 = cos(alpha)
    s2 = sin(beta)
    c2 = cos(beta)
    s3 = sin(gamma)
    c3 = cos(gamma)
    
    # XYZ Euler angle rotation # Doing R_X()R_Y()R_Z() would give the same result. But timeit suggests it is 4 times slower
    # XYZ and ZYZ euler angle rotation matrix are exactly the same. If you want to give rotations in x,y,z
    # just feed it into the euler angle function, it would give the same rotational multiplication matrix
    if euler_angle_type == 0:
        r = zeros(shape = [3,3])
        r[0,0] = c2*c3 
        r[0,1] = -c2*s3
        r[0,2] = s2
        r[1,0] = s1*s2*c3 + c1*s3
        r[1,1] = -s1*s2*s3 + c1*c3
        r[1,2] = -s1*c2
        r[2,0] = -s2*c3*c1 + s1*s3
        r[2,1] = s3*s2*c1 + s1*c3
        r[2,2] = c1*c2
        
        
    if euler_angle_type == 1:
        
        # Compute rotation matrix.
        r = zeros(shape = [3,3])
        r[0,0] = c1*c2*c3 - s1*s3
        r[0,1] = -c1*c2*s3 - s1*c3
        r[0,2] = c1*s2
        r[1,0] = s1*c2*c3 + c1*s3
        r[1,1] = -s1*c2*s3 + c1*c3
        r[1,2] = s1*s2
        r[2,0] = -s2*c3
        r[2,1] = s2*s3
        r[2,2] = c2

    return r