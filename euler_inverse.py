# -*- coding: utf-8 -*-
"""
Created on Thu Sep 27 00:48:35 2018

@author: keerthi
"""
from numpy import math,shape, zeros
from math import sin, cos, atan2, pow, sqrt, pi, asin, acos

def euler_inverse(H):
    
    
    # Temporary variable - for notation shortening.
    r = H[0:3,0:3]
   
    # Compute angles.
    beta = atan2(sqrt(pow(r[2,0],2) + pow(r[2,1],2)), r[2,2])
    # Beta may vary between <0 and 180>.
    if (beta == 0):
        alpha = 0
        gamma = atan2(-r[0,1],r[0,0])
    elif (beta == pi):
        alpha = 0
        gamma = atan2(r[0,1], -r[0,0])
    else:
        s_beta = sin(beta)
        alpha = atan2(r[1,2]/s_beta, r[0,2]/s_beta)
        gamma = atan2(r[2,1]/s_beta, -r[2,0]/s_beta)
    
    # Return computed solution.
    Thetas = [alpha, beta, gamma]
    return Thetas

