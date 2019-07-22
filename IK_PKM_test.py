# -*- coding: utf-8 -*-
"""
Created on Tue Sep 25 09:35:47 2018

@author: keerthi
"""

from IK_PKM_lib import *




class IK_PKM:
    
    
    
# Constructor
    
    def __init__(self):
        

        # All fixed parameteres for the PKM
        #self.params = params
        # Current position of the spherical joint
        #self.current_SW_thetas = current_SW_thetas
        self.current_SW_thetas = [0.0, 0.0, 0.0]
        
        
# Solves inverse kinematics for PKM1.
# O_W_T - a Homogeneous matrix with pose of the
# end-effector (W) in the base (O) reference frame.    +
    
    def inverse_kinematics_pkm(self,ax,O_W_T_desired,T_PKM_O,EF_pose,EF_pose_input,EF_z_OS,pkm_no):
        
        
        # Set parameters for PKM1.
        obj_params = IK_PKM_params()
        if pkm_no == 1:
            self.params = obj_params.params_pkm1(pkm_no)
        if pkm_no == 2:
            self.params = obj_params.params_pkm2(pkm_no)
        #Solve kinematics.
        # Solves inverse kinematics.
        # O_W_T - a Homogeneous matrix with pose of the
        # end-effector (W) in the base (O) reference frame.
        #self.current_SW_thetas
        
        #print('Required pose of the wrist')
        #print(O_W_T_desired[0:3,3])
        #print('euler angles')
        #print(euler_inverse(O_W_T_desired))
        
        
        # Compute the required O_S_T - pose of the spherical wrist middle (S) in
        # global reference frame (O).
        O_S_T_desired = O_W_T_desired*self.params.W_S_T
        
        #print('Required pose of the spherical wrist center',O_S_T_desired[0:3,3],'euler angles',euler_inverse(O_S_T_desired))
        
        # Extract translation.
        O_S_P_desired = O_S_T_desired[0:3,3]
        #print('OSPdesired')
        #print(O_S_P_desired)
        #print(shape(O_S_P_desired))
        
        # Compute solution for delta_B1 and delta_B2 equal to 1.
        # Compute e basing only on translation O_S_P from O_S_T.
        e = self.PKM_S_to_e_for_deltas(O_S_P_desired, 1, 1)
        # Compute inverse PKM transform.
        joints,OP_4_A,OP_4_C,OP_5_B,OP_2_A,OP_2_C  = self.PKM_inverse_from_e_for_deltas(e, 1, 1)
            
        	# Compute upper platform pose (O_P_T = He_inv).
        O_P_T = self.PKM_O_P_T_from_e(e)
          
        #print('Computed upper platform pose',O_P_T[0:3,3],'euler angles', euler_inverse(O_P_T))
                
        # Variable used for testing purposes.
        upp_plat_orient = euler_inverse(O_P_T)
                
        # Compute the pose of wrist (S) on the base of upper platform pose.
        #print('O_P_T')
        #print(O_P_T)
        #print('P_S_T')
        #print(self.params.P_S_T)
        O_S_T_computed = O_P_T * self.params.P_S_T
        
        #print('Computed pose of wrist center'),,'euler angles', euler_inverse(O_S_T_computed))
        #print('Computed pose of wrist center')
        #print(O_S_T_computed[0:3,3])
        #print('euler angles')
        #print(euler_inverse(O_S_T_computed))
        
        # Compute the desired "twist of the wrist".
        # Transformation from computed OST to desired OST.    
        #O_S_R_desired = O_S_T_desired(1:3,1:3);
        #O_S_R_computed = O_S_T_computed(1:3,1:3);
        
        #print('O_S_T_computed')
        #print(O_S_T_computed)
        
        #print('O_S_T_desired')
        #print(O_S_T_desired)
        wrist_twist = inv(O_S_T_computed)*O_S_T_desired
        #print('wrist_twist')
        #print(wrist_twist)
        
        #print('Computed twist of the wrist',wrist_twist[0:3,3],'euler angles', zyz_euler_inverse(wrist_twist))
        #print('Computed twist of wrist center')
        #print(wrist_twist[0:3,3])
        #print('euler angles')
        #print(euler_inverse(wrist_twist))

        # Compute the inverse transform of the spherical wrist
        # basing on its "twist" and current joints.
        thetas = self.SW_inverse(wrist_twist, self.current_SW_thetas)
        #print('thetas')
        #print(thetas)
        # Check cartesian pose.
        self.check_cartesian_pose(O_P_T)
        
        # Compute motors in order to check limits.
        joints = hstack((joints, thetas))
        motors = self.i2mp_transform (joints)
        self.check_motor_position(motors)
         
        # Return the whole vector - the upp_platf_orient is only for
        # test purposes.
        joints = hstack((joints, upp_plat_orient))
         
        # Remember "current" thetas.
        self.current_SW_thetas = thetas
        
        
        solution = self.inverse_kinematics_for_deltas(O_S_P_desired, 1,1)
        s_gamma = O_S_P_desired[1]/(np.math.sqrt(np.math.pow(O_S_P_desired[1],2) + np.math.pow(O_S_P_desired[2],2)))
        
        gamma = np.math.asin(s_gamma)
        
        O_P_x = O_S_P_desired[0,0] - self.params.P_S_P[0]*solution[0] + self.params.P_S_P[2]*solution[1]*solution[3]
        O_P_y = O_S_P_desired[1,0] + self.params.P_S_P[2]*solution[2]
        O_P_z = O_S_P_desired[2,0] - self.params.P_S_P[0]*solution[1] - self.params.P_S_P[2]*solution[0]*solution[3]
        
        OP = np.zeros(shape = (3))                
        OP[0] = O_P_x[0]
        OP[1] = O_P_y[0]
        OP[2] = O_P_z[0]
        
        
        k_5_B = np.array([-1*solution[2]*solution[1],solution[3],solution[2]*solution[0]])
        k_2_A = np.array([solution[0],0,solution[1]])
        
        
        #print('e_h')
        e_h = np.cross(k_2_A,k_5_B)
        #print(e_h)
        #print('OP')
        #print(OP)
        #print('Spherical joint on OP')
        
        OP_eh = OP + self.params.P_S_P_z_1*e_h
        
        
        
       
        #print('OP_eh')
        
        #print(OP_eh)
        
        #print('shape of OP_eh')
        #print(np.shape(OP_eh))
        
        
        #print('TTRREE')
        #print(OP_4_A)
        #print(OP_4_C)
        #print(OP_5_B)
        
        #OP_4_C = np.array([,,])
        
        
        
        #print('O_P_X')
        #print(O_P_x)
        
        #print('O_P_Y')
        #print(O_P_y)
        
        #print('O_P_Z')
        #print(O_P_z)
        
        #print('s_gam')
        #print(sin(gamma))
        
        #print('c_gam')
        #print(cos(gamma))
        
        ax = self.inverse_kinematics_plot(ax,T_PKM_O,self.params,solution,O_S_P_desired,OP,OP_4_C,OP_4_A,OP_5_B,OP_2_A,OP_2_C,k_2_A,k_5_B,OP_eh,EF_pose,EF_z_OS,O_W_T_desired[0:3,3],EF_pose_input)
        
        return joints,ax
    
    
    def inverse_kinematics_plot(self,ax,T_PKM_O,params,solution,O_S_P,OP,OP_4_C,OP_4_A,OP_5_B,OP_2_A,OP_2_C,k_2_A,k_5_B,OP_eh,EF_pose,EF_z_OS,OS_computed,EF_pose_input):
        
        #matplotlib.use('TkAgg') 
        #fig = plt.figure()
        #ax=fig.gca(projection='3d')
        '''
        ax_frame = kinematics_plot()
        ax = ax_frame.plot_frame()
        '''
        #T_PKM_O = np.matrix([[1,0,0,0.5],[0,1,0,0.5],[0,0,1,0.5],[0,0,0,1]])
        print('T_PKM_O')
        print(T_PKM_O)
        
        plane1 = plane_plot() # Base plane containing the revolute joints and the spherical joints
        plane1.generate_plane(ax,np.array([0,0,0]),np.array([0,0,1]),2,2)
        
        
        
        origin_test = T_PKM_O*(np.matrix([[0],[0],[0],[1]]))
        ax.scatter(origin_test[0,0],origin_test[1,0],origin_test[2,0],s=10)
        
        
        rev1 = revolute_joint() # First revolute joint
        pA = T_PKM_O*np.matrix([[0],[params.lA],[0],[1]])
        pA_axis = T_PKM_O*(np.matrix([[0],[1],[0],[1]])) - origin_test
        rev1.revolute(ax,pA[0:3,:],V_unit(pA_axis[0:3,:]),0.05,0.02,1)
        
        rev2 = revolute_joint() # Second revolute joint
        pC = T_PKM_O*np.matrix([[0],[params.lC],[0],[1]])
        pC_axis = T_PKM_O*np.matrix([[0],[-1],[0],[1]]) -  origin_test
        rev2.revolute(ax,pC[0:3,:],V_unit(pC_axis[0:3,0]),0.05,0.02,1)
        
        sph1 = spherical_joint() # First spherical joint
        p_sph1 = T_PKM_O*np.matrix([[params.lB],[0],[0],[1]])
        sph1.generate_sphere(ax,p_sph1[0:3,:],0.04,1,0,0)
        
        print('Check this::: angle between pc,pb')
        
        print(np.dot(np.transpose(pA[0:3,:]-origin_test[0:3,:]),p_sph1[0:3,:]-origin_test[0:3,:]))
        
        rev3 = revolute_joint()
        p3 = T_PKM_O*np.reshape(np.hstack((OP_2_A,[1])),(-1,1))
        p3_axis = T_PKM_O*np.reshape(np.hstack((k_2_A,[1])),(-1,1)) -  origin_test
        rev3.revolute(ax,p3[0:3,:],V_unit(p3_axis[0:3,:]),0.05,0.02,1)
        
        
        
        
        rev4 = revolute_joint()
        p4 = T_PKM_O*np.reshape(np.hstack((OP_2_C,[1])),(-1,1))
        p4_axis = T_PKM_O*np.reshape(np.hstack((k_2_A,[1])),(-1,1)) -  origin_test
        rev4.revolute(ax,p4[0:3,:],V_unit(p4_axis[0:3,:]),0.05,0.02,1)
        

        
        
        rev5 = revolute_joint()
        p5 = T_PKM_O*np.reshape(np.hstack((OP_4_A,[1])),(-1,1))
        p5_axis = T_PKM_O*np.reshape(np.hstack((k_2_A,[1])),(-1,1)) -  origin_test
        rev5.revolute(ax,p5[0:3,:],V_unit(p5_axis[0:3,:]),0.05,0.02,1)
        
        
        rev6 = revolute_joint()
        p6 = T_PKM_O*np.reshape(np.hstack((OP_4_C,[1])),(-1,1))
        p6_axis = T_PKM_O*np.reshape(np.hstack((k_2_A,[1])),(-1,1)) -  origin_test
        rev6.revolute(ax,p6[0:3,:],V_unit(p6_axis[0:3,:]),0.05,0.02,1)
        
        
        rev7 = revolute_joint()
        p7 = T_PKM_O*np.reshape(np.hstack((OP_5_B,[1])),(-1,1))
        p7_axis = T_PKM_O*np.reshape(np.hstack((k_5_B,[1])),(-1,1)) -  origin_test
        rev7.revolute(ax,p7[0:3,:],V_unit(p7_axis[0:3,:]),0.05,0.02,1)
        
        print('Angle between the two vectors issss')
        print(k_5_B)
        print(np.shape(k_5_B))
        print(k_2_A)
        print(np.shape(k_2_A))
        #print(p7_axis[0:3,:])
        
        k_5_B_1 = np.reshape(k_5_B,(-1,1))
        k_2_A_1 = np.reshape(k_2_A,(-1,1))
        
        #print(np.dot(np.transpose(T_PKM_O[0:3,0:3]*k_5_B_1),T_PKM_O[0:3,0:3]*k_2_A_1))
        
        
        link_l12_C = link_generate()
        link_l12_C.link_rect_plot(ax,rev2,rev4,0.4,0.04)
        link_l12_A = link_generate()
        link_l12_A.link_rect_plot(ax,rev1,rev3,0.4,0.04)
        
        
        link_l12_C = link_generate()
        link_l12_C.link_rect_plot(ax,rev4,rev2,0.4,0.04)
        
        link_l_24_A = prismatic_joint()
        link_l_24_A.generate_link(ax,rev3,rev5,0.04)
        
        link_l_24_C = prismatic_joint()
        link_l_24_C.generate_link(ax,rev4,rev6,0.04)
        
        
              
        link_l_15_B = prismatic_joint()
        link_l_15_B.generate_link(ax,sph1,rev7,0.04)
        
        link_p_AC = link_generate()
        link_p_AC.link_rect_plot(ax,rev5,rev6,0.4,0.02)
        
        link_p_AB = link_generate()
        link_p_AB.link_rect_plot(ax,rev5,rev7,0.4,0.02)
        
        link_p_BC = link_generate()
        link_p_BC.link_rect_plot(ax,rev6,rev7,0.4,0.02)
        
        link_p_h = line_plot()
        p_OP = T_PKM_O*np.reshape(np.hstack((OP,[1])),(-1,1))
        p_OP_eh = T_PKM_O*np.reshape(np.hstack((OP_eh,[1])),(-1,1))
        link_p_h.generate_line(ax,p_OP[0:3,:],p_OP_eh[0:3,:])
        
        wrist_spherical = spherical_joint()
        p_OS_computed = T_PKM_O*np.reshape(np.hstack((OS_computed,[1])),(-1,1))
        wrist_spherical.generate_sphere(ax,p_OS_computed[0:3,:],0.04,1,0,0)
        
        rev8 = revolute_joint()
        EF_z_OS = EF_z_OS.tolist()
        EF_z_OS = np.hstack(EF_z_OS)
        
        p_pose = EF_pose_input
        #print('EF_pose')
        #print(EF_pose[0:3])
        #print('EF_z_OS')
        #print(EF_z_OS)
        #print(np.shape(EF_z_OS))
        p_EF_z_OS = T_PKM_O*np.reshape(np.hstack((EF_z_OS,[1])),(-1,1))
        p_EF_z_OS_ax = (T_PKM_O*np.reshape(np.hstack((p_pose[0:3] - check_ndarray(p_EF_z_OS[0:3,:]),[1])),(-1,1))) - origin_test
        
        rev8.revolute(ax,p_EF_z_OS[0:3,:],p_EF_z_OS_ax[0:3,:],0.05,0.02,0)

        
        
        link_EF_z_OS_EF = link_generate()
        link_EF_z_OS_EF.link_cyl_plot(ax,wrist_spherical,rev8)
        #link_EF_z_OS_EF.link_rect_plot(ax,wrist_spherical,rev8,0.4,0.02)
        #link_EF_z_OS_EF.link_rect_plot(ax,rev7,rev8,0.4,0.02)
        
        end_effector_z_OS = point_plot()
        p_EF_z_OS = T_PKM_O*np.reshape(np.hstack((EF_z_OS,[1])),(-1,1))
        end_effector_z_OS.generate_point(ax,p_EF_z_OS[0:3,:])
        
        tripod_1 = tripod_plot()
        tripod_1.generate_tripod(ax,rev5.p_ini,rev6.p_ini,rev7.p_ini)
        
        '''
        coordv2 = coordinate_vector()
        pose_1 = T_PKM_O[0:3,0:3]*zyz_euler(EF_pose[3:4])
        coordv1.generate_coordinate_vector(ax,pose[0:3,:],0)
        
        print('EF_pose is')
        print(EF_pose)
        print(np.shape(EF_pose))
        '''
        
        end_effector_head = equilateral_triangle()
        
        end_effector_head.generate_triangle(ax,p_pose,self.params.head_radius,self.params.head_height,0)
        
        link_EF_z_OS_EF_pose = link_generate()
        link_EF_z_OS_EF_pose.link_cyl_plot(ax,rev8,end_effector_head)
        

        
        end_effector_spherical = point_plot()
        p_O_S_P = T_PKM_O*np.vstack((O_S_P,[1]))
        end_effector_spherical.generate_point(ax,p_O_S_P[0:3,:])
        
        end_effector_P = point_plot()
        p_OP = T_PKM_O*np.reshape(np.hstack((OP,[1])),(-1,1))
        end_effector_P.generate_point(ax,p_OP[0:3,:])
        


        
        return ax
        
        
        
        
        
        
# Computes solution for given O_S_P point and values of delta.
        # O_S_P is a vector containing the position of the spherical
        # wrist middle (S) in the base (O) reference frame.
    def inverse_kinematics_for_deltas(self, O_S_P, delta_B1, delta_B2):
        # Compute e basing only on translation O_S_P from O_S_T.
        e = self.PKM_S_to_e_for_deltas(O_S_P, delta_B1, delta_B2)
        # Compute inverse PKM transform.
        [joints,OP_4_A,OP_4_C,OP_5_B,OP_2_A,OP_2_C] = self.PKM_inverse_from_e_for_deltas(e, delta_B1, delta_B2)

        # Return computed solution.
        #solution = [delta_B1, delta_B2, e, joints];
        solution = hstack((e, joints))
        #print('Solution is:::::')
        #print(solution[0])
        #print(solution[1])
        return solution # inverse_kinematics_for_deltas
        
        
# Computes platform pose on the base of given _O_S_P.
# O_S_P Position of the middle of the spherical wrist (S) in the O (lower PM platform) reference frame.
# Platform pose is returned in the form of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
    def PKM_S_to_e_for_deltas(self, O_S_P, delta_B1, delta_B2):
        #self.params.P_S_P(1)
        # Temporary variables used for computations of alpha.
        t0_sq = O_S_P[0,0] * O_S_P[0,0] + O_S_P[2,0] * O_S_P[2,0]
        hx_sq = self.params.P_S_P[0,0] * self.params.P_S_P[0,0]
        
        # Compute sine and cosine of the alpha angle.
        s_alpha = (delta_B1 * O_S_P[2,0] * sqrt(t0_sq - hx_sq) + self.params.P_S_P[0,0] * O_S_P[0,0]) / ((t0_sq)*(1.0))
        c_alpha = (-delta_B1 * O_S_P[0,0] * sqrt(t0_sq - hx_sq) + self.params.P_S_P[0,0] * O_S_P[2,0]) / ((t0_sq)*(1.0))

        # Compute sine and cosine of the beta angle.
        t6 = (delta_B1 * (t0_sq - self.params.lB * O_S_P[0,0]) * sqrt(t0_sq - hx_sq) + self.params.lB * self.params.P_S_P[0,0] * O_S_P[2,0]) / ((t0_sq)*(1.0))
        
        s_beta = -delta_B2 * O_S_P[1,0] / (sqrt(pow(t6,2) + pow(O_S_P[1,0],2)))
        c_beta = delta_B2 * t6 / (sqrt(pow(t6 ,2) + pow(O_S_P[1,0],2)))
        
        # Compute h.
        h = (delta_B2 * (O_S_P[1,0] * O_S_P[1,0] + (delta_B1 * t6 * sqrt(t0_sq - hx_sq)))) / (((sqrt(t6 * t6 + O_S_P[1,0] * O_S_P[1,0])))) - self.params.P_S_P[2,0]
        
        # Return upper platform pose.
        e = [s_alpha, c_alpha, s_beta, c_beta, h]
        #print('Waiiiiiiiiiiiiiiiiittttttttttttt')
        #print(e)
        return e # PKM_S_to_e

# Computes position S basing on given e.
# e - the platform pose in the form of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
# Used for verification purposes.
    def PKM_e_to_S(self, e):
        # "Retrieve" values from e.
        s_alpha = e[0]
        c_alpha = e[1]
        s_beta = e[2]
        c_beta = e[3]
        h = e[4]

        # Compute S.
        Sx = self.params.P_S_P[0,0]*s_alpha - (h+self.params.P_S_P[0,2])*(c_beta*c_alpha) + (c_alpha*self.params.lB*s_beta)*(c_alpha*s_beta)
        Sy = -(h+self.params.P_S_P[0,2])*s_beta + (c_alpha*self.params.lB*s_beta)*(-c_beta)
        Sz = self.params.P_S_P[0,0]*c_alpha - (h+self.params.P_S_P[0,2])*(-c_beta*s_alpha) + (c_alpha*self.params.lB*s_beta)*(-s_alpha*s_beta)

        # Print results.
        S = [Sx, Sy, Sz]
        return S # PKM_S_to_e
        
        
# Computes values of PM joints basing on given e.
# e - the platform pose in the form of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
# Returns joints in the form of vector <qA,qB,qC>.
    def PKM_inverse_from_e_for_deltas(self, e, delta_B1, delta_B2):
        # "Retrieve" values from e.
        s_alpha = e[0]
        c_alpha = e[1]
        s_beta = e[2]
        c_beta = e[3]
        h = e[4]
        delta_A = 1
        delta_C = 1

        # Compute temporary variables.
        t1 = self.params.lB * s_alpha - self.params.uB
        t2 = self.params.lB * c_alpha * c_beta + h
        t3 = self.params.lB * c_alpha - t2 * c_beta
        
        from scipy.spatial import distance
        # Intermediate joints
        OP_4_A = np.array([c_alpha*(t3-(self.params.uA*s_beta)-(self.params.hA*c_beta)),((self.params.uA*c_beta)-(t2*s_beta)-(self.params.hA*s_beta)),((-s_alpha)*((t3)-(self.params.uA*s_beta)-(self.params.hA*c_beta)))])
        OP_4_C = np.array([c_alpha*(t3-(self.params.uC*s_beta)-(self.params.hC*c_beta)),((self.params.uC*c_beta)-(t2*s_beta)-(self.params.hC*s_beta)),((-s_alpha)*((t3)-(self.params.uC*s_beta)-(self.params.hC*c_beta)))])
 
        OP_5_B = np.array([((-t1*s_alpha)+(self.params.lB)-(t2*c_alpha*c_beta)),(-t2*s_beta),((t2*s_alpha*c_beta)-(t1*c_alpha))])       
        
        OP_2_A = np.array([-self.params.l_12_A*c_alpha,self.params.lA,self.params.l_12_A*s_alpha])
        
        #print('Neww lenght1 is')
        #print(distance.euclidean(OP_2_A,OP_4_A))
        
        
        
        OP_2_C = np.array([-self.params.l_12_C*c_alpha,self.params.lC,self.params.l_12_C*s_alpha])
        
        #print('Neww lenght2 is')
        #print(distance.euclidean(OP_2_C,OP_4_C))
        
        #print('AAAAAAGT')
        # Compute joints.
        qB = sqrt(pow(t1 ,2) + pow(t2,2))
        #print('qB is')
        #print(qB)
        qA = sqrt(pow((t3 - self.params.uA * s_beta - self.params.hA*(c_beta) + delta_A*self.params.l_12_A),2) + pow((t2 * s_beta - self.params.uA * c_beta +  self.params.hA*(s_beta)+self.params.lA),2))
        #print('qA is')
        #print(qA)
        qC = sqrt(pow((t3 - self.params.uC * s_beta - self.params.hC*(c_beta) + delta_C*self.params.l_12_C),2) + pow((t2 * s_beta - self.params.uC * c_beta + self.params.hC*(s_beta)+self.params.lC),2))
        #print('qC is')
        #print(qC)
        # Return vector with joints.
        joints = [qA, qB, qC]
        #print('Joints are')
        #print(joints)
        return joints,OP_4_A,OP_4_C,OP_5_B,OP_2_A,OP_2_C # PKM_inverse_from_e

# Computes matrix O_P_T, representing the position  and orientation of upper platform (P) in relation to the lower one (O).
# This is in fact the inverted He matrix from the M.Zoppi & D.Zlatanov.
#_e - Platform pose in the form of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
    def PKM_O_P_T_from_e(self, e):
        # "Retrieve" values from e.
        s_alpha = e[0]
        c_alpha = e[1]
        s_beta = e[2]
        c_beta = e[3]
        h = e[4]
        # Compute matrix representing the location and orientation of upper platform.
        # Rotation matrix.
        O_P_T = zeros(shape = (4,4))
        O_P_T[0, 0] = s_alpha
        O_P_T[0, 1] = -s_beta*c_alpha
        O_P_T[0, 2] = -c_beta*c_alpha
        O_P_T[1, 0] = 0
        O_P_T[1, 1] = c_beta
        O_P_T[1, 2] = -s_beta
        O_P_T[2, 0] = c_alpha
        O_P_T[2, 1] = s_beta*s_alpha
        O_P_T[2, 2] = c_beta*s_alpha
        # Translation matrix.
        O_P_T[0, 3] = (pow(c_alpha,2)*(pow(s_beta,2))*(self.params.lB)) - (h*c_beta*c_alpha)
        O_P_T[1, 3] = -c_alpha*s_beta*c_beta*self.params.lB - h*s_beta
        O_P_T[2, 3] = (-1*pow(s_beta,2)*(s_alpha)*(c_alpha)*self.params.lB) + (s_alpha*c_beta*h)
        # Last row.
        O_P_T[3, 0] = 0.0
        O_P_T[3, 1] = 0.0
        O_P_T[3, 2] = 0.0
        O_P_T[3, 3] = 1.0
        
        return O_P_T

        
# Computes matrix P_O_T, representing the position  and orientation of upper platform (P) in relation to the lower one (O).
# e - Platform pose in the form of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
    def PKM_He_from_e(self, e):
        # "Retrieve" values from e.
        s_alpha = e[0]
        c_alpha = e[1]
        s_beta = e[2]
        c_beta = e[3]
        h = e[4]
        # Compute matrix representing the location and orientation of upper platform.
        O_P_T[0,0] = s_alpha
        O_P_T[0,1] = 0.0
        O_P_T[0,2] = c_alpha
        O_P_T[0,3] = 0.0
        O_P_T[1,0] = -s_beta * c_alpha
        O_P_T[1,1] = c_beta
        O_P_T[1,2] = s_beta * s_alpha
        O_P_T[1,3] = c_alpha * self.params.lB * s_beta
        O_P_T[2,0] = -c_beta * c_alpha
        O_P_T[2,1] = -s_beta
        O_P_T[2,2] = c_beta * s_alpha
        O_P_T[2,3] = -h
        O_P_T[3,0] = 0.0
        O_P_T[3,1] = 0.0
        O_P_T[3,2] = 0.0
        O_P_T[3,3] = 1.0
        
        
        return O_P_T
        
# Computes SW inverse kinematics transform - values of SW thetas
# basing on desired twist of spherical wrist.
# The orientation is described as Z-Y-Z Euler angles.
# Returns joints (thatas) in the form of vector <q1,q2,q3>.        
# wrist_twist Position of end of spherical wrist (W) with relation to
# its base, which is in fact upper platform reference frame (P) (the twist of the wrist).
# current_SW_thetas Current (in fact previously desired) joint
# values.
    def SW_inverse(self, wrist_twist, current_SW_thetas):
        # current_SW_thetas

        # Variables.
        # phi, theta, psi, dist;
        #      phi2, theta2, psi2, dist2;
        #    thetas;
      
        if (wrist_twist[2,2] == 1):
            # If u33 = 1 then theta is 0.
            theta = 0
            # Infinite number of solutions: only the phi + psi value can be computed, thus phi is equal to the previous one.
            phi = current_SW_thetas(1)
            psi = atan2(wrist_twist(2,1), wrist_twist[0,0]) - phi
            thetas = [phi, theta, psi]
        elif (wrist_twist[2,2] == -1):
                # If u33 = -1 then theta is equal to pi.
                theta = pi
                # Infinite number of solutions: only the phi - psi value can be computed, thus phi is equal to the previous one.
                phi = current_SW_thetas(1)
                psi = - atan2(-wrist_twist[0,1], -wrist_twist[0,0]) + phi
                #std::cout<<"CASE II: u33=-1 => ["<<phi<<", "<<theta<<", "<<psi<<"]\n";
                thetas = [phi, theta, psi]
        else:
                # Two possible solutions.
                # First solution.
                theta = atan2(sqrt(1 - wrist_twist[2,2]*wrist_twist[2,2]), wrist_twist[2,2])
                phi = atan2(wrist_twist[1,2], wrist_twist[0,2])
                psi = atan2(wrist_twist[2,1], -wrist_twist[2,0])
                
                #print('current_SW_thetas')
                #print(current_SW_thetas)
                #print('phi')
                #print(phi)
                # Compute maximal delta.
                dist = max(max(abs(phi - current_SW_thetas[0]), abs(theta - current_SW_thetas[1])), abs(psi - current_SW_thetas[2]))
                print(dist)
                
                # Second solution.
                theta2 = atan2(-sqrt(1 - wrist_twist[2,2]*wrist_twist[2,2]), wrist_twist[2,2])
                phi2 = atan2(-wrist_twist[1,2], -wrist_twist[0,2])
                psi2 = atan2(-wrist_twist[2,1], wrist_twist[2,0])
                
                #std::cout<<"CASE IV: atan(u33, -sqrt(1-u33^3)) => ["<<phi2<<", "<<theta2<<", "<<psi2<<"]\n";
                # Compute maximal delta.
                dist2 = max(max(abs(phi2 - current_SW_thetas[0]), abs(theta2 - current_SW_thetas[1])), abs(psi2 - current_SW_thetas[2]))
                
                # Select best solution.
                if (dist < dist2):
                    thetas = [phi, theta, psi]
                else:
                    thetas = [phi2, theta2, psi2]
                     
            
        # Return vector with thetas.
        return thetas
  
   
        # Checks whether given Cartesian pose is valid - but in this case computations are based on the upper PM platform pose computed in the IK.
        # O_P_T - upper PM platform pose.
    def check_cartesian_pose(self, O_P_T):
        
        
        # Location of the lower platform A,B, and C points (middle of rotation) - all in relation to the O (global PKM reference frame).
        O_lA_T = matrix([[ 1, 0, 0, 0],[ 0, 1, 0, self.params.lA],[ 0, 0, 1, 0],[ 0, 0, 0, 1]])
        O_lB_T = matrix([[1, 0, 0, self.params.lB],[0, 1, 0, 0],[0, 0, 1, 0], [0, 0, 0, 1]])
        O_lC_T = matrix([[ 1, 0, 0, 0], [0, 1, 0, self.params.lC], [0, 0, 1, 0], [0, 0, 0, 1]])
    
        # Compute location of the upper platform A,B, and C points (middle of rotation) - all in relation to the O (global PKM reference frame).
        uA = matrix([[1, 0, 0, 0],[ 0, 1, 0, self.params.uA],[ 0, 0, 1, 0],[ 0, 0, 0, 1]])
        uB = matrix([[1, 0, 0, self.params.uB],[ 0, 1, 0, 0],[ 0, 0, 1, 0],[ 0, 0, 0, 1]])
        uC = matrix([[1, 0, 0, 0],[ 0, 1, 0, self.params.uC],[ 0, 0, 1, 0],[ 0, 0, 0, 1]])
    
        O_uA_T = O_P_T * uA
        O_uB_T = O_P_T * uB
        O_uC_T = O_P_T * uC
         
         # Compute angles related to the inner and outer gimbals.
        uA_lA_T = O_uA_T - O_lA_T
        uB_lB_T = O_uB_T - O_lB_T
        uC_lC_T = O_uC_T - O_lC_T
     
        # Thyk alpha = rotation around the x axes of legs A, B, and C of the upper platform: alpha = arc tan (|y|/|z|).
        thyk_alpha = zeros(shape = (3))
        thyk_beta = zeros(shape = (3))
        thyk_alpha[0] = atan2 (uA_lA_T[1,3], uA_lA_T[2,3]) * 180.0 / pi
        thyk_alpha[1] = atan2 (uB_lB_T[1,3], uB_lB_T[2,3]) * 180.0 / pi
        thyk_alpha[2] = atan2 (uC_lC_T[1,3], uC_lC_T[2,3]) * 180.0 / pi
        #thyk_alpha
        print('I am heree')
        # Check thyk alpha angle.
        for i in xrange(3):
            if thyk_alpha[i] > self.params.upper_alpha_thyk_angle_limit[i]:
                
                print('I new')
                #print(thyk_alpha[i])
                # Throw adequate error
                
                #err = MException('CartesianPose:UpperThykAlphaLimit',['Upper Thyk Alpha ' int2str(i) ' Limit Exceeded'])
                 
                #print(['Upper Thyk Alpha ', i ,' Limit Exceeded: ', num2str(thyk_alpha(i), 5), ' > ', num2str(self.params.upper_alpha_thyk_angle_limit(i), 3)])
                #input('?')
            elif thyk_alpha[i] < self.params.lower_alpha_thyk_angle_limit[i]:
                
                print('I new2')
                #print(thyk_alpha[i])
                 # Throw adequate error
                #err = MException('CartesianPose:LowerThykAlphaLimit',['Lower Thyk Alpha ' int2str(i) ' Limit Exceeded'])
                 
                #print(['Lower Thyk Alpha ', i ,' Limit Exceeded: ', num2str(thyk_alpha(i), 5), ' < ', num2str(self.params.lower_alpha_thyk_angle_limit(i), 3)])
                #input('?')
             
    
        # Thyk beta = rotation around the y axes of legs A, B, and C of the upper platform: alpha = arc tan (|x|/|z|).
        thyk_beta[0] = atan2(uA_lA_T[0,3], uA_lA_T[2,3]) * 180.0 / (pi)*(1.0)
        thyk_beta[1] = atan2(uB_lB_T[0,3], uB_lB_T[2,3]) * 180.0 / (pi)*(1.0)
        thyk_beta[2] = atan2(uC_lC_T[0,3], uC_lC_T[2,3]) * 180.0 / (pi)*(1.0)
        #thyk_beta
     
        # Check thyk beta angle.
        for i in xrange(3):
            if thyk_beta[i] > self.params.upper_beta_thyk_angle_limit[i]:
                
                print('I new3')
                #print(thyk_beta[i])
                # Throw adequate error
                #err = MException('CartesianPose:UpperThykBetaLimit', ['Upper Thyk Beta ' int2str(i) ' Limit Exceeded'])
                 #throw(err)
                #print(['Upper Thyk Beta ',i, ' Limit Exceeded: ', num2str(thyk_beta(i), 5), ' > ', num2str(self.params.upper_beta_thyk_angle_limit(i), 3)]);
                #input('?')
            elif thyk_beta[i] < self.params.lower_beta_thyk_angle_limit[i]:
                 # Throw adequate error
                #err = MException('CartesianPose:LowerThykBetaLimit', ['Lower Thyk Beta ' int2str(i) ' Limit Exceeded'])
                #throw(err)
                #print(['Lower Thyk Beta ', i, ' Limit Exceeded: ', num2str(thyk_beta(i), 5), ' < ', num2str(self.params.lower_beta_thyk_angle_limit(i), 3)]);
                #input('?')
                print('I new4')
                #print(thyk_beta[i])




        # Computes motor increments from internal coordinates.
        # joints - current joints settings
        # Returns computed motor increment.
    def i2mp_transform(self, joints):
        # Compute desired motor positions for linear axes.
        motor_position = [-1,-1,-1,-1,-1,-1]
        
        #print('Ridicu')
        #print(joints)
        #print(shape(joints))
        
        for i in xrange(3):
            motor_position[i] = (self.params.synchro_positions[i] - joints[i]) / ((self.params.mp2i_ratios[i])*(1.0))
                
        # Compute desired motor positions for rotary axes.
        for i in xrange(3,6,1):
            motor_position[i] = joints[i] / (self.params.mp2i_ratios[i]*(1.0))
        

        return motor_position
        
        # Checks whether given motor increments are valid.
        # motor_position - Motor position to be validated.
    def check_motor_position(self, motor_position):
        # Check upper limit for every motor.
        for i in xrange(6):
             if motor_position[i] > self.params.upper_motor_pos_limits[i]:
                 print('I new5')
                 #print(motor_position[i])
                 # Throw adequate error
                 #err = MException('Motors:UpperMotorLimit', ['Upper Motor ' int2str(i) ' Limit Exceeded'])
                 #throw(err)
                 #print(['Upper Motor ',i, ' Limit Exceeded: ', num2str(motor_position[i], 4), ' > ', num2str(self.params.upper_motor_pos_limits[i], 2)])
                 #input('?')
             elif motor_position[i] < self.params.lower_motor_pos_limits[i]:
                 print('I new6')
                 #print(motor_position[i])
                 # Throw adequate error
                 #err = MException('Motors:LowerMotorLimit', ['Lower Motor ' int2str(i) ' Limit Exceeded'])
                 #throw(err)
                 #print(['Lower Motor ',i, ' Limit Exceeded: ', num2str(motor_position(i), 4), ' < ', num2str(self.params.lower_motor_pos_limits(i), 2)])
                 #input('?')
             
        
# inverse_kinematics_transform_from_surface (an equivalent of M.Zoppi's inverse_7dof) is a function solving the inverse kinematics with as assigned variables:
#  O_W_P - coordinates of the desired head center point in space
#  expressed in the ground reference frame.
#  d - a unit-lenght vector tangent to the surface to which the head
#  must adhere.
#  n - a unit-lenght vector orthogonal to d and orthogonal to the
#  tangent plane to the surface to which the head must adhere.

#    def inverse_kinematics_transform_from_surface(self, O_W_P, d, n):
        # Compute thw W pose on the base of given vectors and position.
#        O_W_R = [d; n; cross(d,n)]
#        O_W_T = [O_W_R, O_W_P; 0 0 0 1]
        
#        self.inverse_kinematics_transform(O_W_T)

#        return thetas
        