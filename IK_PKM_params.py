# -*- coding: utf-8 -*-
"""
Created on Tue Sep 25 10:50:47 2018

@author: keerthi
"""
import numpy
from numpy import math,shape, zeros
from math import sin, cos, atan2, pow, sqrt
from numpy import pi as pi
from numpy import mat,matrix,vstack,hstack
class IK_PKM_params:
    

           
    def params_pkm1(self,pkm_no):
        
        if pkm_no == 1:
            
            
            # End effector head
            scale_head = 2
            self.head_side = 0.12188*scale_head
            self.head_radius = self.head_side/sqrt(3)
            self.head_height = self.head_radius*(sqrt(3)/2.0)
            # Lower platform
            self.lA = -0.05
            #self.lA = -0.4434
            #self.lA = 0
            #self.lB = 0.18
            self.lB = 0.18
            #self.lC = 0.05
            
            self.lC = 0.05
            #self.lC = 0.7798
            
            # Upper platform.
            self.uA = -0.05
            #self.uA = -0.1523
            self.uB = 0.086
            #self.uB = 0.1324
            self.uC = 0.05
            #self.uC = 0.2523
            
            self.hA = 0
            self.hC = 0
            
            self.l_12_A = 0
            self.l_12_C = 0
            
            # Maximum length of prismatic joints (CAD Measurement)
            self.qA_max = 0.150
            self.qB_max = 0.150
            self.qC_max = 0.150
            
            
            
            
            # Transformation between upper platform and middle of the wrist.
            # Rotation by -pi/2 around y axis.
            #self.P_S_P = [0.0; 0; 0.1];
            #P_S_R = [0, 0, -1; 0 1 0; 1, 0, 0];
            self.P_S_P_x = 0
            #self.P_S_P_z_1 = 0.09
            self.P_S_P_z_1 = 0
            self.P_S_P_z_2 = 0
            self.P_S_P = matrix([[self.P_S_P_x],[ 0],[ (self.P_S_P_z_1 + self.P_S_P_z_2)]]) # Measured value
            #self.P_S_P = numpy.mat('0.2828; 0; -0.2') #Publication value
            #P_S_R = [0, 1.0, 0; -1.0, 0, 0; 0, 0, 1];
            self.P_S_R = mat('1.0, 0, 0; 0, 1.0, 0; 0, 0, 1')
            self.P_S_T = vstack((hstack((self.P_S_R, self.P_S_P)),[ 0, 0, 0, 1]))
            
            # Traslation between middle of the spherical wrist and wrist
            # (PKM end-effector).
            #self.W_S_P = mat('0.0; 0; -0.0725') # MEasured quantity from Poles
            self.W_S_P = mat('0.0; 0; 0') # MEasured quantity from Poles
            #self.W_S_P = mat('0.0; 0; 0') # Publication value
            # The wrist frame is rotated by pi/2 around the Z axis in relation to
            # the upper platform frame
            self.W_S_R = mat('1, 0, 0; 0, 1, 0; 0, 0, 1')
            self.W_S_T = vstack((hstack((self.W_S_R, self.W_S_P)),[ 0, 0, 0, 1]))
            
            # Initialization of upper thyk alpha angle limit.
            self.upper_alpha_thyk_angle_limit = [30.0, 50.0, 30.0]
            # Initialization of lower thyk alpha angle limit.
            self.lower_alpha_thyk_angle_limit = [-30.0, -50.0, -30.0]
            # Initialization of upper thyk beta angle limit.
            self.upper_beta_thyk_angle_limit = [40.0, 50.0, 40.0]
            # Initialization of lower thyk beta angle limit.
            self.lower_beta_thyk_angle_limit = [-30.0, -50.0, -30.0]
            
            #  Initialization of parameters describing the synchronisation positions of first three parallel PM axes (A=0,B=1,C=2).
            self.synchro_positions = [0.242, 0.2435, 0.242, 0.0, 0.0, 0.0]
            
            #  Initialization of parameters related to conversion from motor positions to joints.
            #  Parameters for conversion for linear DOFs are:
            # * Lead roller screw is equal to 5mm.
            # * The encoder has 500 CPT (Counts per turn).
            # * Quadcounts = 4 x Encoder Counts.
            # * The gear ratio is equal to 9.
            linear_mp2i_ratio = 0.005 / (4.0 * 500 * 9.0)
            #  Parameters for conversion for rotational DOFs are:
            # * The encoder has 2000 CPT (Counts per turn).
            # * Quadcounts = 4 x Encoder Counts.
            # * The gear ratio is equal to 100.
            rotational_mp2i_ratio = 2*pi / (4.0 * 2000 * 100)
            
            # Moog motor mp2i ratio.
            moog_mp2i_ratio = 2*pi / (4.0 * 4096 * 100)
            
            self.mp2i_ratios = [linear_mp2i_ratio, linear_mp2i_ratio, linear_mp2i_ratio, rotational_mp2i_ratio, moog_mp2i_ratio, rotational_mp2i_ratio]
            
            # Initialization of upper motors limits vector
            self.upper_motor_pos_limits = [8000, 8000, 8000, 383000, 2000, 260000]
            
            # Initialization of lower motors limits vector.
            self.lower_motor_pos_limits = [-350000, -350000, -350000, -352000, -380000, -270000]
            
        return self
    
    def params_pkm2(self,pkm_no):
        
        if pkm_no ==2:
            # End effector head
            # End effector head
            scale_head = 2
            self.head_side = 0.12188*scale_head
            self.head_radius = self.head_side/sqrt(3)
            self.head_height = self.head_radius*(sqrt(3)/2.0)
            # Lower platform
            self.lA = -0.05
            #self.lA = -0.4434
            #self.lA = 0
            self.lB = 0.18
            #self.lB = 0.3455
            #self.lC = 0.05
            
            self.lC = 0.05
            #self.lC = 0.7798
            
            # Upper platform.
            self.uA = -0.05
            #self.uA = -0.1523
            self.uB = 0.086
            #self.uB = 0.1324
            self.uC = 0.05
            #self.uC = 0.2523
            
            self.hA = 0
            self.hC = 0
            
            self.l_12_A = 0
            self.l_12_C = 0
            
            
            
            # Transformation between upper platform and middle of the wrist.
            # Rotation by -pi/2 around y axis.
            #self.P_S_P = [0.0; 0; 0.1];
            #P_S_R = [0, 0, -1; 0 1 0; 1, 0, 0];
            self.P_S_P_x = 0
            self.P_S_P_z_1 = 0.09
            self.P_S_P_z_2 = 0
            self.P_S_P = matrix([[self.P_S_P_x],[ 0],[ (self.P_S_P_z_1 + self.P_S_P_z_2)]]) # Measured value
            #self.P_S_P = numpy.mat('0.2828; 0; -0.2') #Publication value
            #P_S_R = [0, 1.0, 0; -1.0, 0, 0; 0, 0, 1];
            self.P_S_R = mat('1.0, 0, 0; 0, 1.0, 0; 0, 0, 1')
            self.P_S_T = vstack((hstack((self.P_S_R, self.P_S_P)),[ 0, 0, 0, 1]))
            
            # Traslation between middle of the spherical wrist and wrist
            # (PKM end-effector).
            #self.W_S_P = mat('0.0; 0; -0.0725') # MEasured quantity from Poles
            self.W_S_P = mat('0.0; 0; 0') # MEasured quantity from Poles
            #self.W_S_P = mat('0.0; 0; 0') # Publication value
            # The wrist frame is rotated by pi/2 around the Z axis in relation to
            # the upper platform frame
            self.W_S_R = mat('1, 0, 0; 0, 1, 0; 0, 0, 1')
            self.W_S_T = vstack((hstack((self.W_S_R, self.W_S_P)),[ 0, 0, 0, 1]))
            
            # Initialization of upper thyk alpha angle limit.
            self.upper_alpha_thyk_angle_limit = [30.0, 50.0, 30.0]
            # Initialization of lower thyk alpha angle limit.
            self.lower_alpha_thyk_angle_limit = [-30.0, -50.0, -30.0]
            # Initialization of upper thyk beta angle limit.
            self.upper_beta_thyk_angle_limit = [40.0, 50.0, 40.0]
            # Initialization of lower thyk beta angle limit.
            self.lower_beta_thyk_angle_limit = [-30.0, -50.0, -30.0]
            
            #  Initialization of parameters describing the synchronisation positions of first three parallel PM axes (A=0,B=1,C=2).
            self.synchro_positions = [0.242, 0.2435, 0.242, 0.0, 0.0, 0.0]
            
            #  Initialization of parameters related to conversion from motor positions to joints.
            #  Parameters for conversion for linear DOFs are:
            # * Lead roller screw is equal to 5mm.
            # * The encoder has 500 CPT (Counts per turn).
            # * Quadcounts = 4 x Encoder Counts.
            # * The gear ratio is equal to 9.
            linear_mp2i_ratio = 0.005 / (4.0 * 500 * 9.0)
            #  Parameters for conversion for rotational DOFs are:
            # * The encoder has 2000 CPT (Counts per turn).
            # * Quadcounts = 4 x Encoder Counts.
            # * The gear ratio is equal to 100.
            rotational_mp2i_ratio = 2*pi / (4.0 * 2000 * 100)
            
            # Moog motor mp2i ratio.
            moog_mp2i_ratio = 2*pi / (4.0 * 4096 * 100)
            
            self.mp2i_ratios = [linear_mp2i_ratio, linear_mp2i_ratio, linear_mp2i_ratio, rotational_mp2i_ratio, moog_mp2i_ratio, rotational_mp2i_ratio]
            
            # Initialization of upper motors limits vector
            self.upper_motor_pos_limits = [8000, 8000, 8000, 383000, 2000, 260000]
            
            # Initialization of lower motors limits vector.
            self.lower_motor_pos_limits = [-350000, -350000, -350000, -352000, -380000, -270000]
            
        return self
        
        
        
        