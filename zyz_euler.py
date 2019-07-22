# -*- coding: utf-8 -*-
"""
Created on Tue Nov 13 23:15:03 2018

@author: keerthi
"""

def zyz_euler(alpha,beta,gamma):
    from numpy import sin,cos,zeros
    
    # Compute sines and cosines
    s1 = sin(alpha)
    c1 = cos(alpha)
    s2 = sin(beta)
    c2 = cos(beta)
    s3 = sin(gamma)
    c3 = cos(gamma)
    
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