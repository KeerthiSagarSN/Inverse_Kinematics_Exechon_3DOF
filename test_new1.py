# -*- coding: utf-8 -*-
"""
Created on Wed Nov 14 04:38:53 2018

@author: keerthi
"""
import numpy as np

new_yaxis = -np.cross(new_xaxis, new_zaxis)

# new axes:
nnx, nny, nnz = new_xaxis, new_yaxis, new_zaxis
# old axes:
nox, noy, noz = np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=float).reshape(3, -1)

# ulgiest rotation matrix you can imagine
top = [np.dot(nnx, n) for n in [nox, noy, noz]]
mid = [np.dot(nny, n) for n in [nox, noy, noz]]
bot = [np.dot(nnz, n) for n in [nox, noy, noz]]

def newit(vec):
    xn = sum([p*q for p,q in zip(top, vec)])
    yn = sum([p*q for p,q in zip(mid, vec)])
    zn = sum([p*q for p,q in zip(bot, vec)])
    return np.hstack((xn, yn, zn))


'''
nnx:         array([-0.22139284, -0.73049229,  0.64603887])
newit(nnx):  array([ 1.,  0.,  0.])

nny:         array([ 0.88747002,  0.1236673 ,  0.44396325])
newit(nny):  array([ 0.,  1.,  0.])

nnz:         array([-0.40420561,  0.67163042,  0.62091095])
newit(nnz:   array([ 0.,  0.,  1.])
'''