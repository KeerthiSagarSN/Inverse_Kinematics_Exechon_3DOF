# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 17:34:17 2016

@author: keerthi
"""

# Header library
from visualbuild_v2_lib import *

# Joint visualisations
'''
r_cylinder = 5
#extr = 50\
matplotlib.use('TkAgg') 
fig = plt.figure()
ax=fig.gca(projection='3d')
ax.set_aspect('equal')
'''

'''
#ax.set_aspect('equal')

'''


class Empty_Class:
    pass
class kinematics_plot():
    
    def plot_frame(self):
        
        from visualbuild_v2_lib import *
        matplotlib.use('Qt5Agg') 
        fig = plt.figure()
        ax=fig.gca(projection='3d')
        ax.set_aspect('equal')
        ax.patch.set_facecolor("white")
        ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
        ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
        ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
        # Setting the axis limits here
        xmin = -0.30 # Previous -1.5
        xmax = 0.30
        ymin = -0.30
        ymax = 0.30
        zmin = -0.1
        zmax = 0.6
        ax.set_xlim([xmin,xmax])
        ax.set_ylim([ymin,ymax])
        ax.set_zlim([zmin,zmax])
        #ax.margins(x=1, y=1, z = 1)
        #ax.grid(False)
        ax.view_init(15, 300)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m]')

        return ax
    
 
# All functions below are for generation of the caps of the revolute cylinders along an axis
# For reference of the function:  https://stackoverflow.com/questions/18228966/how-can-matplotlib-2d-patches-be-transformed-to-3d-with-arbitrary-normals
class revolute_joint():
      
    # Creating a cylinder surface for revoulute joint
# Reference Link - Cylinder about arbitrary axis : http://mathforum.org/library/drmath/view/51734.html
#http://stackoverflow.com/questions/32317247/how-to-draw-a-cylinder-using-matplotlib-along-length-of-point-x1-y1-and-x2-y2
# Input for cylinder is the 3D vector of the axis and the radius of the cylinder
    def revolute(self,ax,p_ini,v_ax,extr,R,link_axis): # ax is the 3d axes of the image, p_ini is the initial plot, v_ax is a unit vector of the axis of the revolute joint
        # link_axis = 0: Link is aligned along the revolute joint, link_axis = 1: Link is aligned along the perpendicular direction
        # Input parameters
        
        #extr = 0.05
        
        #R = 0.02
        
        p_ini = check_ndarray(np.array(p_ini))
        v_ax = check_ndarray(np.array(v_ax))
        self.v_axis = v_ax
        
        v_unit = V_unit(self.v_axis)
        self.p_ini = p_ini
        self.p1 = self.p_ini + (extr/2.0)*(v_unit)
        self.p0 = self.p_ini - (extr/2.0)*(v_unit)
        
        #p1 = -p1
        # Create a random unit vector not in the same direction as v_unit
        
        not_v_unit = np.array([1,0,0])
        
        # Checking if random unit vector lies along the unit vector
        if (not_v_unit == v_unit).all():
            not_v_unit = np.array([0,1,0])
        
        # Vector perpendicular to v_unit
        n1 = np.cross(v_unit,not_v_unit)
        # Normalizing the perpendicular vector
        n1 /= norm(n1)
        # Common unit vector perpendicular to v_unit and n1
        n2 = np.cross(v_unit,n1)
        # Creating surface with this unit vector just as a circle equation helps
        # extr can take value of length of the vector but in our case a fixed extrusion for the revolute joint is followed
        
        
        
        if link_axis == 0:
            self.p1_l_ax = self.p_ini + (extr/2.0)*(v_unit)
            self.p0_l_ax = self.p_ini - (extr/2.0)*(v_unit)
        elif link_axis ==1:
            self.p1_l_ax = self.p_ini + (extr/2.0)*(n2)
            self.p0_l_ax = self.p_ini - (extr/2.0)*(n2)
        
        
        
        t = np.linspace(0, extr/2, 100)
        theta = np.linspace(0, 2 * np.pi, 100)
        #use meshgrid to make 2d arrays
        t, theta = np.meshgrid(t, theta)
        #generate coordinates for surface
        x_cyl, y_cyl, z_cyl = [self.p0[i] + v_unit[i]*t + (R * np.sin(theta) * n1[i]) + (R * np.cos(theta) * n2[i] + v_unit[i] * t) for i in [0, 1, 2]]
        ax.plot_wireframe(x_cyl, y_cyl, z_cyl,color = "k",rstride = 30,cstride = 30)
        
                # Caps for the cylinder
        # Reference link: End caps of the revolute cylinder
        # Reference for function: https://stackoverflow.com/questions/18228966/how-can-matplotlib-2d-patches-be-transformed-to-3d-with-arbitrary-normals

        normal = v_unit
        
        c1 = Circle((0,0), R, facecolor = 'k', alpha = 0.6)
        ax.add_patch(c1)

        self.pathpatch_2d_to_3d(c1, z = 0, normal = normal)
        self.pathpatch_translate(c1,(self.p0[0],self.p0[1],self.p0[2]))
        
        c2 = Circle((0,0), R, facecolor = 'k', alpha = 0.6)
        ax.add_patch(c2)
        self.pathpatch_2d_to_3d(c2, z = 0, normal = normal)
        self.pathpatch_translate(c2,(self.p1[0],self.p1[1],self.p1[2]))
        
        '''
        x_cyl, y_cyl, z_cyl = [self.p0[i] + v_unit[i]*t + (R * np.sin(theta) * n1[i]) + (R * np.cos(theta) * n2[i] + v_unit[i] * t) for i in [0, 1, 2]]
        ax.plot_wireframe(x_cyl, y_cyl, z_cyl)
        '''
        #ax.add_patch(c2)        

        #cyl_per = np.cross()
        return ax
    

    def rotation_matrix(self,v1,v2):
        
        '''
        Calculates the rotation matrix that changes v1 into v2.
        '''
        
        if np.any(v1) and np.any(v2):
            v1 = np.divide(v1,np.linalg.norm(v1))
            v2 = np.divide(v2,np.linalg.norm(v2))
        
            cos_angle=np.dot(v1,v2)
            d=np.cross(v1,v2)
            sin_angle=np.linalg.norm(d)
        
            if sin_angle == 0:
                M = np.identity(3) if cos_angle>0. else -np.identity(3)
            else:
                d = np.divide(d,sin_angle)
        
                eye = np.eye(3)
                ddt = np.outer(d, d)
                skew = np.array([[    0,  d[2],  -d[1]],
                              [-d[2],     0,  d[0]],
                              [d[1], -d[0],    0]], dtype=np.float64)
        
                M = ddt + cos_angle * (eye - ddt) + sin_angle * skew
        
            return M
        else:
            raise ValueError('One of the two vectors is a zero vector')
    
    def pathpatch_2d_to_3d(self,pathpatch, z = 0, normal = 'z'):
        '''
        Transforms a 2D Patch to a 3D patch using the given normal vector.
    
        The patch is projected into they XY plane, rotated about the origin
        and finally translated by z.
        '''
        if type(normal) is str: #Translate strings to normal vectors
            index = "xyz".index(normal)
            normal = np.roll((1,0,0), index)
    
        path = pathpatch.get_path() #Get the path and the associated transform
        trans = pathpatch.get_patch_transform()
    
        path = trans.transform_path(path) #Apply the transform
    
        pathpatch.__class__ = art3d.PathPatch3D #Change the class
        pathpatch._code3d = path.codes #Copy the codes
        pathpatch._facecolor3d = pathpatch.get_facecolor #Get the face color    
    
        verts = path.vertices #Get the vertices in 2D
    
        M = self.rotation_matrix(normal,(0, 0, 1)) #Get the rotation matrix
    
        pathpatch._segment3d = np.array([np.dot(M, (x, y, 0)) + (0, 0, z) for x, y in verts])
        
    

    
    
    def pathpatch_translate(self,pathpatch, delta):
        """
        Translates the 3D pathpatch by the amount delta.
        """
        pathpatch._segment3d += delta        
        
        
        
class link_generate():

        
    def link_rect_plot(self,ax,joint1,joint2,prism_val,width):
        
        
        # Constants of the size of the links
        
        w = width*(prism_val) # prism_val denotes the prismatic link/not if prism_val = 1 for link, and = 2, for prismatic joint
        #extr = np.linalg.norm(p_ini-p_out) # In mm (unit) i guess
        
         
        cuboid_pts = self.link(joint1,joint2,prism_val,w)
        
        # Aligning axis of the link along the v_axis
        #
        
        
        vert_plt = []
        for i in xrange(0,24,4):
            X = []
            Y = []
            Z = []
            X = cuboid_pts[i:i+4,0].tolist()
            X = list(itertools.chain.from_iterable(X))
            Y = cuboid_pts[i:i+4,1].tolist()
            Y = list(itertools.chain.from_iterable(Y))
            Z = cuboid_pts[i:i+4,2].tolist()
            Z = list(itertools.chain.from_iterable(Z))
            vert_plt += [zip(X,Y,Z)]
                    
        ax.add_collection3d(Poly3DCollection(vert_plt,facecolors = 'r',alpha = 0.5))
        ax.add_collection3d(Line3DCollection(vert_plt, colors='k', linewidths=1, linestyles='-'))
        #ax.scatter3D(p_ini[0,0],p_ini[0,1],p_ini[0,2],c = 'cyan')
        #ax.scatter3D(p_out[0,0],p_out[0,1],p_out[0,2],c = 'cyan')
        #ax.plot([joint1.p_ini[0,0],joint2.p_ini[0,0]],[joint1.p_ini[0,1],joint2.p_ini[0,1]],[joint1.p_ini[0,2],joint2.p_ini[0,2]],c='g')
        #ax.set_autoscaley_on(True)
        return ax
        #plt.show()
    def link(self,joint1,joint2,prism_val,w): # Get the basic link along the unit direction along the centre
    
        # Constants
        #w = 0.02    
        # Create a cuboid structure
        v_axis = joint2.p_ini - joint1.p_ini
        v_unit = V_unit(v_axis)
        v_p1_p0 = joint1.p1_l_ax - joint1.p0_l_ax
        v_unit_p1_p0 = V_unit(v_p1_p0)
        v_norm = np.cross(v_unit_p1_p0,v_unit) # This is the first normal vector
        
        p0_unit_1 = V_unit(joint1.p0_l_ax - joint1.p_ini)
        p0_1 = joint1.p_ini + (prism_val/3.0)*(np.linalg.norm(joint1.p0_l_ax - joint1.p_ini))*p0_unit_1
        
        
        p1_unit_1 = V_unit(joint1.p1_l_ax - joint1.p_ini)
        p1_1 = joint1.p_ini + (prism_val/3.0)*(np.linalg.norm(joint1.p1_l_ax - joint1.p_ini))*p1_unit_1
        
        
        
        
        p0_unit_2 = V_unit(joint2.p0_l_ax - joint2.p_ini)
        p0_2 = joint2.p_ini + (prism_val/3.0)*(np.linalg.norm(joint2.p0_l_ax - joint2.p_ini))*p0_unit_2
        
        
        p1_unit_2 = V_unit(joint2.p1_l_ax - joint2.p_ini)
        p1_2 = joint2.p_ini + (prism_val/3.0)*(np.linalg.norm(joint2.p1_l_ax - joint2.p_ini))*p1_unit_2
        
        # We need to check if the cross product produces a zero vector, meaning both p_k and p_1 are scaled versions of each other
        '''
        if not np.any(v_norm):
            p_k = np.matrix([-20,200,100]) # Assign another unit vector in totally different direction
            v_norm = np.cross(p_k,p_1_unit)
        '''
        # Step 2 Get the cross product of the two vectors
        #v_norm_2 = np.cross(p_1_unit,v_norm)
        # Step 3 Both vectors provide the direction of the unit axis vector
        # First get the unit vector of the normal vector
        # We need to scale the vector 
        # ALl these vectors are unit vectors
        
        # In here we take into account the size of the rectangle that we require
        # And we scale it appropritately
        # POints along the first joint
        j1_p1 = p1_1 + (v_norm*w)
        j1_p2 = p1_1 - (v_norm*w)
        j1_p3 = p0_1 + (v_norm*w)
        j1_p4 = p0_1 - (v_norm*w)
        
        
        
        # POints along the second joint
        j2_p1 = p1_2 + (v_norm*w)
        j2_p2 = p1_2 - (v_norm*w)
        j2_p3 = p0_2 + (v_norm*w)
        j2_p4 = p0_2 - (v_norm*w)
        
        cube_pts = np.matrix([-1,-1,-1])
        
        # Get 4 coordinates of the rectangle near point p1- Rectangular cap 1
              
        cube_pts  = np.vstack((cube_pts,j1_p1))  # Get new coordinate points
        cube_pts  = np.vstack((cube_pts,j1_p2)) # Get new coordinate points
        cube_pts  = np.vstack((cube_pts,j1_p4)) # Get new coordinate points            
        cube_pts  = np.vstack((cube_pts,j1_p3)) # Get new coordinate points
        


        # Get 4 coordinates of the rectangle near point p1- Rectangular cap 2
                
        cube_pts  = np.vstack((cube_pts,j2_p1))  # Get new coordinate points
        cube_pts  = np.vstack((cube_pts,j2_p2)) # Get new coordinate points
        cube_pts  = np.vstack((cube_pts,j2_p4)) # Get new coordinate points            
        cube_pts  = np.vstack((cube_pts,j2_p3)) # Get new coordinate points    
        
        
        # Get 4 coordinates of the rectangle near point p1- Rectangular cap 3
                
        cube_pts  = np.vstack((cube_pts,j1_p4))  # Get new coordinate points
        cube_pts  = np.vstack((cube_pts,j1_p3)) # Get new coordinate points
        cube_pts  = np.vstack((cube_pts,j2_p3)) # Get new coordinate points            
        cube_pts  = np.vstack((cube_pts,j2_p4)) # Get new coordinate points
        
        # Get 4 coordinates of the rectangle near point p1- Rectangular cap 4
                
        cube_pts  = np.vstack((cube_pts,j1_p1))  # Get new coordinate points
        cube_pts  = np.vstack((cube_pts,j1_p2)) # Get new coordinate points
        cube_pts  = np.vstack((cube_pts,j2_p2)) # Get new coordinate points            
        cube_pts  = np.vstack((cube_pts,j2_p1)) # Get new coordinate points  
        
        # Get 4 coordinates of the rectangle near point p1- Rectangular cap 5
                
        cube_pts  = np.vstack((cube_pts,j1_p1))  # Get new coordinate points
        cube_pts  = np.vstack((cube_pts,j1_p3)) # Get new coordinate points
        cube_pts  = np.vstack((cube_pts,j2_p3)) # Get new coordinate points            
        cube_pts  = np.vstack((cube_pts,j2_p1)) # Get new coordinate points 
        
        
        # Get 4 coordinates of the rectangle near point p1- Rectangular cap 6
                
        cube_pts  = np.vstack((cube_pts,j1_p2))  # Get new coordinate points
        cube_pts  = np.vstack((cube_pts,j1_p4)) # Get new coordinate points
        cube_pts  = np.vstack((cube_pts,j2_p4)) # Get new coordinate points            
        cube_pts  = np.vstack((cube_pts,j2_p2)) # Get new coordinate points 
        
        
        # Delete the first element
        cube_pts = cube_pts[1:]
        #axis_sign = np.sign(v_axis[0,2])
        # Hypotenuse starting point of the rectangle
        #p0 = np.matrix([0,0,0])
        # Starting from left corner
        #x_link = [p0[0,0] + w/2,p0[0,0] - w/2,p0[0,0] - w/2,p0[0,0] + w/2,p0[0,0] + w/2,p0[0,0] + w/2,p0[0,0] + w/2,p0[0,0] + w/2,p0[0,0] + w/2,p0[0,0] - w/2,p0[0,0] - w/2,p0[0,0] + w/2,p0[0,0] - w/2,p0[0,0] - w/2,p0[0,0] - w/2,p0[0,0] - w/2,p0[0,0] - w/2,p0[0,0] + w/2,p0[0,0] + w/2,p0[0,0] - w/2,p0[0,0] - w/2,p0[0,0] - w/2,p0[0,0] + w/2,p0[0,0] + w/2,p0[0,0] - w/2] 
        #y_link = [p0[0,1] + l/2,p0[0,1] + l/2,p0[0,1] - l/2,p0[0,1] - l/2,p0[0,1] + l/2,p0[0,1] + l/2,p0[0,1] - l/2,p0[0,1] - l/2,p0[0,1] + l/2,p0[0,1] + l/2,p0[0,1] + l/2,p0[0,1] + l/2,p0[0,1] + l/2,p0[0,1] - l/2,p0[0,1] - l/2,p0[0,1] + l/2,p0[0,1] - l/2,p0[0,1] - l/2,p0[0,1] - l/2,p0[0,1] - l/2,p0[0,1] - l/2,p0[0,1] + l/2,p0[0,1] + l/2,p0[0,1] - l/2,p0[0,1] - l/2]
        #z_link = [p0[0,2],p0[0,2],p0[0,2],p0[0,2],p0[0,2],p0[0,2] + axis_sign*(extr),p0[0,2] + axis_sign*(extr),p0[0,2],p0[0,2],p0[0,2],p0[0,2] + axis_sign*(extr),p0[0,2] + axis_sign*(extr),p0[0,2],p0[0,2],p0[0,2] + axis_sign*(extr),p0[0,2] + axis_sign*(extr),p0[0,2],p0[0,2] + axis_sign*(extr),p0[0,2] + axis_sign*(extr),p0[0,2],p0[0,2] + axis_sign*(extr),p0[0,2] + axis_sign*(extr),p0[0,2] + axis_sign*(extr),p0[0,2] + axis_sign*(extr),p0[0,2] + axis_sign*(extr)]
        
        XX = cube_pts[:,0].tolist()
        XX = list(itertools.chain.from_iterable(XX))
        YY = cube_pts[:,1].tolist()
        YY = list(itertools.chain.from_iterable(YY))
        ZZ = cube_pts[:,2].tolist()
        ZZ = list(itertools.chain.from_iterable(ZZ))
        #ax.scatter3D(XX,YY,ZZ,c = 'g')
        
        
        return cube_pts
    
    def link_cyl_plot(self,ax,joint1,joint2): # ax is the 3d axes of the image, p_ini is the initial plot, v_ax is a unit vector of the axis of the revolute joint
        # link_axis = 0: Link is aligned along the revolute joint, link_axis = 1: Link is aligned along the perpendicular direction
        # Input parameters

        self.p1 = check_ndarray(joint2.p_ini)
        self.p0 = check_ndarray(joint1.p_ini)

        
        extr = np.linalg.norm(self.p1-self.p0)
        
        R = 0.015
        
        
        #print('joint1p_ini')
        #print(joint1.p_ini)
        #print('joint2.p_ini')
        #print(joint2.p_ini)
        
        self.v_axis = self.p1 - self.p0
        
        v_unit = V_unit(self.v_axis)
                
        #p1 = -p1
        # Create a random unit vector not in the same direction as v_unit
        not_v_unit = np.array([1,0,0])
        # Checking if random unit vector lies along the unit vector
        if (not_v_unit == v_unit).all():
            not_v_unit = np.array([0,1,0])
        # Vector perpendicular to v_unit
        n1 = np.cross(v_unit,not_v_unit)
        # Normalizing the perpendicular vector
        n1 /= norm(n1)
        # Common unit vector perpendicular to v_unit and n1
        n2 = np.cross(v_unit,n1)
        # Creating surface with this unit vector just as a circle equation helps
        # extr can take value of length of the vector but in our case a fixed extrusion for the revolute joint is followed
        
        
         
        
        
        t = np.linspace(0, extr/2, 100)
        theta = np.linspace(0, 2 * np.pi, 100)
        #use meshgrid to make 2d arrays
        t, theta = np.meshgrid(t, theta)
        #generate coordinates for surface
        x_cyl, y_cyl, z_cyl = [self.p0[i] + v_unit[i]*t + (R * np.sin(theta) * n1[i]) + (R * np.cos(theta) * n2[i] + v_unit[i] * t) for i in [0, 1, 2]]
        ax.plot_wireframe(x_cyl, y_cyl, z_cyl,color = 'r',facecolor = 'k',alpha = 0.2)
        
                # Caps for the cylinder
        # Reference link: End caps of the revolute cylinder
        # Reference for function: https://stackoverflow.com/questions/18228966/how-can-matplotlib-2d-patches-be-transformed-to-3d-with-arbitrary-normals

        normal = v_unit
        
        c1 = Circle((0,0), R, facecolor = 'r', alpha = 0.5)
        ax.add_patch(c1)

        self.pathpatch_2d_to_3d(c1, z = 0, normal = normal)
        self.pathpatch_translate(c1,(self.p0[0],self.p0[1],self.p0[2]))
        
        c2 = Circle((0,0), R, facecolor = 'r', alpha = 0.5)
        ax.add_patch(c2)
        self.pathpatch_2d_to_3d(c2, z = 0, normal = normal)
        self.pathpatch_translate(c2,(self.p1[0],self.p1[1],self.p1[2]))
        
        '''
        x_cyl, y_cyl, z_cyl = [self.p0[i] + v_unit[i]*t + (R * np.sin(theta) * n1[i]) + (R * np.cos(theta) * n2[i] + v_unit[i] * t) for i in [0, 1, 2]]
        ax.plot_wireframe(x_cyl, y_cyl, z_cyl)
        '''
        #ax.add_patch(c2)        

        #cyl_per = np.cross()
        return ax
    

    def rotation_matrix(self,v1,v2):
        
        '''
        Calculates the rotation matrix that changes v1 into v2.
        '''
        
        if np.any(v1) and np.any(v2):
            v1 = np.divide(v1,np.linalg.norm(v1))
            v2 = np.divide(v2,np.linalg.norm(v2))
        
            cos_angle=np.dot(v1,v2)
            d=np.cross(v1,v2)
            sin_angle=np.linalg.norm(d)
        
            if sin_angle == 0:
                M = np.identity(3) if cos_angle>0. else -np.identity(3)
            else:
                d = np.divide(d,sin_angle)
        
                eye = np.eye(3)
                ddt = np.outer(d, d)
                skew = np.array([[    0,  d[2],  -d[1]],
                              [-d[2],     0,  d[0]],
                              [d[1], -d[0],    0]], dtype=np.float64)
        
                M = ddt + cos_angle * (eye - ddt) + sin_angle * skew
        
            return M
        else:
            raise ValueError('One of the two vectors is a zero vector')
    
    def pathpatch_2d_to_3d(self,pathpatch, z = 0, normal = 'z'):
        '''
        Transforms a 2D Patch to a 3D patch using the given normal vector.
    
        The patch is projected into they XY plane, rotated about the origin
        and finally translated by z.
        '''
        if type(normal) is str: #Translate strings to normal vectors
            index = "xyz".index(normal)
            normal = np.roll((1,0,0), index)
    
        path = pathpatch.get_path() #Get the path and the associated transform
        trans = pathpatch.get_patch_transform()
    
        path = trans.transform_path(path) #Apply the transform
    
        pathpatch.__class__ = art3d.PathPatch3D #Change the class
        pathpatch._code3d = path.codes #Copy the codes
        pathpatch._facecolor3d = pathpatch.get_facecolor #Get the face color    
    
        verts = path.vertices #Get the vertices in 2D
    
        M = self.rotation_matrix(normal,(0, 0, 1)) #Get the rotation matrix
    
        pathpatch._segment3d = np.array([np.dot(M, (x, y, 0)) + (0, 0, z) for x, y in verts])
        
    

    
    
    def pathpatch_translate(self,pathpatch, delta):
        """
        Translates the 3D pathpatch by the amount delta.
        """
        pathpatch._segment3d += delta
    
class prismatic_joint():
    def generate_link(self,ax,joint1,joint2,width):
        
        '''
        joint1.p_ini = check_ndarray(joint1.p_ini)
        joint1.p0_l_ax = check_ndarray(joint1.p0_l_ax)
        joint1.p1_l_ax = check_ndarray(joint1.p1_l_ax)
        
        joint2.p_ini = check_ndarray(joint2.p_ini)
        joint2.p0_l_ax = check_ndarray(joint2.p0_l_ax)
        joint2.p1_l_ax = check_ndarray(joint2.p1_l_ax)
        '''
        
        
        
        #joint1.p1_l_ax = check_ndarray(joint1.p1_l_ax)
        
        
        joint1_copy = copy.deepcopy(joint1) # Deep copy of the object to change the initial points
        '''
        joint1_copy.p_ini = check_ndarray(joint1_copy.p_ini)
        joint1_copy.p0_l_ax = check_ndarray(joint1_copy.p0_l_ax)
        joint1_copy.p1_l_ax = check_ndarray(joint1_copy.p1_l_ax)
        '''
        joint1_copy.p_ini = (joint1.p_ini + joint2.p_ini)/2.0
        joint1_copy.p0_l_ax = (joint1.p0_l_ax + joint2.p0_l_ax)/2.0
        joint1_copy.p1_l_ax = (joint1.p1_l_ax + joint2.p1_l_ax)/2.0

        
        self.firsthalf = link_generate()
        self.firsthalf.link_rect_plot(ax,joint1,joint1_copy,0.7,width)
        
        self.secondhalf = link_generate()
        self.secondhalf.link_rect_plot(ax,joint1_copy,joint2,0.3,width)
       
        
        return ax

# Reference material
#https://stackoverflow.com/questions/11140163/python-matplotlib-plotting-a-3d-cube-a-sphere-and-a-vector        
class spherical_joint():
    def generate_sphere(self,ax,p_ini,r,i,j,k): # p_ini - Centre of the sphere, r - radius of the sphere, i,j,k - Axis along which link will be attached [1,0,0] or [0,1,0] or [0,0,1]
        u, v = np.mgrid[0:2*np.pi:10j, 0:np.pi:10j]
        x = p_ini[0]+r*np.cos(u)*np.sin(v)
        y = p_ini[1]+r*np.sin(u)*np.sin(v)
        z = p_ini[2]+r*np.cos(v)
        ax.plot_wireframe(x, y, z, color="k")
        p_ini = check_ndarray(p_ini)
        self.p_ini = p_ini
        '''
        if (i ==1 and j==0 and k==0):
            self.p0_l_ax = self.p_ini - [0.005,0,0]
            self.p1_l_ax = self.p_ini + [0.005,0,0]
        elif (i ==0 and j==1 and k==0):
            self.p0_l_ax = self.p_ini - [0,0.005,0]
            self.p1_l_ax = self.p_ini + [0,0.005,0]
        if (i ==0 and j==0 and k==1):
            self.p0_l_ax = self.p_ini - [0,0,0.005]
            self.p1_l_ax = self.p_ini + [0,0,0.005]
        '''
        return ax


class vector_plot():
    def generate_vector(self,ax,v1,v2):
        ax.quiver(v1[0],v1[1],v1[2],v2[0],v2[1],v2[2],length=1.0)
        
        return ax

class plane_plot():
    def generate_plane(self,ax,p1,normal,range_u,range_v):
        # a plane is a*x+b*y+c*z+d=0
        # [a,b,c] is the normal. Thus, we have to calculate
        # d and we're set
        d = -p1.dot(normal)
        
        # create x,y
        xx, yy = np.meshgrid(range(-2,3), range(-1,3))
        xx = xx/10.0
        yy = yy/10.0
        # calculate corresponding z
        z = (-normal[0] * xx - normal[1] * yy - d) * 1. /normal[2]
        
        ax.plot_wireframe(xx,yy,z,alpha=0.4)
        return ax

class plane_lineaxis():
    def generate_plane(self,ax,p1,p2,v_axis,range_length):
        
        # Constants of the size of the links
        
        #w = width*(prism_val) # prism_val denotes the prismatic link/not if prism_val = 1 for link, and = 2, for prismatic joint
        #extr = np.linalg.norm(p_ini-p_out) # In mm (unit) i guess
        
        vert_plt = []         
        
        p3 = p1 + V_unit(v_axis)*range_length
        p4 = p2 + V_unit(v_axis)*range_length
        #cuboid_pts = self.link(joint1,joint2,prism_val,w)
        vert_plt = np.vstack((p1,p2,p3,p4))
        vert_plt = vert_plt.tolist()
        #X = np.zeros(shape=[1,4])
        #Y = np.zeros(shape=[1,4])
        #Z = np.zeros(shape=[1,4])
        X = np.array([p1[0],p2[0],p4[0],p3[0],p1[0]]).tolist()
        Y = np.array([p1[1],p2[1],p4[1],p3[1],p1[1]]).tolist()
        Z = np.array([p1[2],p2[2],p4[2],p3[2],p1[2]]).tolist()
        
        vert_plt = [zip(X,Y,Z)]
        #ax.add_collection3d(Poly3DCollection(vert_plt,facecolors = 'red',alpha = 0.9))
        ax.add_collection3d(Line3DCollection(vert_plt, colors='k', linewidths=1, linestyles='-'))
        '''
        vert_plt = []
        for i in xrange(0,4,1):
            X = []
            Y = []
            Z = []
            X = cuboid_pts[i:i+4,0].tolist()
            X = list(itertools.chain.from_iterable(X))
            Y = cuboid_pts[i:i+4,1].tolist()
            Y = list(itertools.chain.from_iterable(Y))
            Z = cuboid_pts[i:i+4,2].tolist()
            Z = list(itertools.chain.from_iterable(Z))
            vert_plt += [zip(X,Y,Z)]
        '''            
        #ax.add_collection3d(Poly3DCollection(vert_plt,facecolors = 'r',alpha = 0.5))
        #ax.add_collection3d(Line3DCollection(vert_plt, colors='k', linewidths=1, linestyles='-'))
        #ax.scatter3D(p_ini[0,0],p_ini[0,1],p_ini[0,2],c = 'cyan')
        #ax.scatter3D(p_out[0,0],p_out[0,1],p_out[0,2],c = 'cyan')
        #ax.plot([joint1.p_ini[0,0],joint2.p_ini[0,0]],[joint1.p_ini[0,1],joint2.p_ini[0,1]],[joint1.p_ini[0,2],joint2.p_ini[0,2]],c='g')
        #ax.set_autoscaley_on(True)
        return ax
        
        
class point_plot():
    def generate_point(self,ax,p1):
        # a plane is a*x+b*y+c*z+d=0
        # [a,b,c] is the normal. Thus, we have to calculate
        # d and we're set
        p1 = check_ndarray(p1) # Flatten array
            

        #print(p2.ndim)
        
        ax.scatter(p1[0],p1[1],p1[2],'k',s = 20)
        return ax



class line_plot():
    def generate_line(self,ax,p1,p2):
        p1 = check_ndarray(p1) # Flatten array
        p2 = check_ndarray(p2) # Flatten array
        ax.plot([p1[0],p2[0]],[p1[1],p2[1]],[p1[2],p2[2]],'k-')
        
        return ax


class tripod_plot():
    def generate_tripod(self,ax,p1,p2,p3): # Three vertices of the tripod. To get the thickness generate links between the links with link_generate() class
        
        
        tripod_pts = np.vstack((p1,p2))
        tripod_pts = np.vstack((tripod_pts,p3))
        '''
        XX = tripod_pts[:,0].tolist()
        print('XXXX')
        print(XX)
        XX = list(itertools.chain.from_iterable(XX))
        YY = tripod_pts[:,1].tolist()
        YY = list(itertools.chain.from_iterable(YY))
        ZZ = tripod_pts[:,2].tolist()
        ZZ = list(itertools.chain.from_iterable(ZZ))
        '''
        vert_plt = []
        X = []
        Y = []
        Z = []
        for i in range(0,3,4):

            X = tripod_pts[i:i+3,0].tolist()
            #X = list(itertools.chain.from_iterable(X))
            Y = tripod_pts[i:i+3,1].tolist()
            #Y = list(itertools.chain.from_iterable(Y))
            Z = tripod_pts[i:i+3,2].tolist()
            #Z = list(itertools.chain.from_iterable(Z))
            vert_plt += [zip(X,Y,Z)]
                    
        ax.add_collection3d(Poly3DCollection(vert_plt,facecolors = 'r',alpha = 0.4))
        ax.add_collection3d(Line3DCollection(vert_plt, colors='k', linewidths=1, linestyles=':'))
        
        return ax


class coordinate_vector():
    def generate_coordinate_vector(self,ax,pose,euler_angle_type): # Euler angle type: 0 - XYZ 1 - ZYZ rotation
        
    
        pose = check_ndarray(pose)
        length_v = 0.005
        length_arrow = 25
        tail = np.matrix([[pose[0]],[pose[1]],[pose[2]]])
        #tail = np.matrix([pose[0],pose[1],pose[2]])
        
        if euler_angle_type == 0:
            #Rot_matrix = R_X(math.degrees(pose[3]))*R_Y(math.degrees(pose[4]))*R_Z(math.degrees(pose[5]))
            Rot_matrix = zyz_euler(-pose[3],-pose[4],-pose[5])
            #print('Here1')
            #print(Rot_matrix)
        elif euler_angle_type ==1:
            #Rot_matrix = R_X(math.degrees(pose[3]))*R_Y(math.degrees(pose[4]))*R_Z(math.degrees(pose[5]))
            Rot_matrix = zyz_euler(-pose[3],-pose[4],-pose[5])
            #print('Here2')
            #print(Rot_matrix)
        else:
            raise ValueError('generate_coordinate_vector accepts only value 0 or 1. Enter "0" for XYZ euler angle rotation or "1" for ZYZ euler angle rotation')
        #Rot_i = Rot_matrix*np.matrix([[length_v],[0],[0]])
        Rot_i = [Rot_matrix[0,0:3]*length_v]
        Rot_i = np.transpose(Rot_i)
        #print('Rot_i')
        #print(Rot_i)
        head_v_i = Rot_i + tail
        '''
        print('pose')
        print(pose[0])
        print(pose[1])
        print(pose[2])
        '''
        
        #Rot_j = Rot_matrix*np.matrix([[0],[length_v],[0]])
        Rot_j = [Rot_matrix[1,0:3]*length_v]
        Rot_j = np.transpose(Rot_j)
        head_v_j = Rot_j + tail
        #Rot_k = Rot_matrix*np.matrix([[0],[0],[length_v]])
        Rot_k = [Rot_matrix[2,0:3]*length_v]
        Rot_k = np.transpose(Rot_k)
        head_v_k = Rot_k + tail
        
        ax.quiver([head_v_i[0,0],head_v_j[0,0],head_v_k[0,0]],[head_v_i[1,0],head_v_j[1,0],head_v_k[1,0]],[head_v_i[2,0],head_v_j[2,0],head_v_k[2,0]],[Rot_i[0,0],Rot_j[0,0],Rot_k[0,0]],[Rot_i[1,0],Rot_j[1,0],Rot_k[1,0]],[Rot_i[2,0],Rot_j[2,0],Rot_k[2,0]],color=['b','lime','k'],length=length_arrow)
             
        
        return ax
class gen_coordinate_vector():
    def generate_coordinate_vector(self,ax,pose,Rot_matrix): # Euler angle type: 0 - XYZ 1 - ZYZ rotation
        
    
        pose = check_ndarray(pose)
        length_v = 0.005
        length_arrow = 17
        tail = np.matrix([[pose[0]],[pose[1]],[pose[2]]])

        Rot_i = Rot_matrix[0,0:3]*length_v
        Rot_i = np.transpose(Rot_i)
        #print('Rot_i')
        #print(Rot_i)
        head_v_i = Rot_i + tail
        #print('head_v_i')
        #print(head_v_i)
        '''
        print('pose')
        print(pose[0])
        print(pose[1])
        print(pose[2])
        '''
        
        #Rot_j = Rot_matrix*np.matrix([[0],[length_v],[0]])
        Rot_j = Rot_matrix[1,0:3]*length_v
        Rot_j = np.transpose(Rot_j)
        head_v_j = Rot_j + tail
        #print('head_v_j')
        #print(head_v_j)
        #Rot_k = Rot_matrix*np.matrix([[0],[0],[length_v]])
        Rot_k = Rot_matrix[2,0:3]*length_v
        Rot_k = np.transpose(Rot_k)
        head_v_k = Rot_k + tail
        #print('head_v_k')
        #print(head_v_k)
        ax.quiver([head_v_i[0,0],head_v_j[0,0],head_v_k[0,0]],[head_v_i[1,0],head_v_j[1,0],head_v_k[1,0]],[head_v_i[2,0],head_v_j[2,0],head_v_k[2,0]],[Rot_i[0,0],Rot_j[0,0],Rot_k[0,0]],[Rot_i[1,0],Rot_j[1,0],Rot_k[1,0]],[Rot_i[2,0],Rot_j[2,0],Rot_k[2,0]],color=['b','lime','k'],length=length_arrow)
             
        
        return ax
    
class equilateral_triangle():
    def generate_triangle(self,ax,pose,radius,height,euler_angle_type): # Pose of centroid centre. 0 - XYZ euler angle, 1 - ZYZ euler angle type
        
        # Circumscribed circle in the origin
        centroid_ini = np.matrix([[0],[0],[0]])
        self.p_ini = pose[0:3]
        centroid_1 = np.matrix([[pose[0]],[pose[1]],[pose[2]]])
        # Vertex 1 along the "x" or "i" axis
        vert_1_ini = np.matrix([[radius/math.sqrt(3)],[0],[0]]) 
        
        vert_2_ini = R_Z(120)*vert_1_ini
        
        vert_3_ini = R_Z(120)*vert_2_ini
        
        
        
        # Diplaced equilateral triangle
        '''
        eq_triangle_coord_vector = coordinate_vector()
        ax = eq_triangle_coord_vector.generate_coordinate_vector(ax,pose,euler_angle_type)
        '''
        if euler_angle_type == 0:
            
            
            Rot_matrix = zyz_euler(pose[3],pose[4],pose[5])
            #Rot_matrix = R_X(np.degrees(-pose[3]))*R_Y(np.degrees(-pose[4]))*R_Z(np.degrees(-pose[5]))
            Disp_matrix = np.matrix([[pose[0]],[pose[1]],[pose[2]]])
            row_4 = np.matrix([[0,0,0,1]])
            
            Homog_matrix = np.vstack((np.hstack((Rot_matrix,Disp_matrix)),row_4))
            #print('Homog_matrix')
            #print(Homog_matrix)
        
            vert_1 = Homog_matrix*np.vstack((vert_1_ini,[1]))
            vert_2 = Homog_matrix*np.vstack((vert_2_ini,[1]))
            vert_3 = Homog_matrix*np.vstack((vert_3_ini,[1]))
            centroid_1 = Homog_matrix*np.vstack((centroid_ini,[1]))
            

            
            #print('Side is')
            #print(np.linalg.norm((vert_1-vert_2)))
            
            #print('Side 2 is')
            #print(np.linalg.norm((vert_2-vert_3)))
            
            
        elif euler_angle_type == 1:
            
            Rot_matrix = zyz_euler(pose[3],pose[4],pose[5])
            Disp_matrix = np.matrix([[pose[0]],[pose[1]],[pose[2]]])
            row_4 = np.matrix([[0,0,0,1]])
            
            Homog_matrix = np.vstack((np.hstack((Rot_matrix,Disp_matrix)),row_4))
            
        
            vert_1 = Homog_matrix*np.vstack((vert_1_ini,[1]))
            vert_2 = Homog_matrix*np.vstack((vert_2_ini,[1]))
            vert_3 = Homog_matrix*np.vstack((vert_3_ini,[1]))
            centroid_1 = Homog_matrix*np.vstack((centroid_ini,[1]))
            

        
        else: 
            
            raise ValueError('Euler angle value as "1" or "0"')
        
        #vert_1 = centroid_1 + Rot_matrix
            
        ax.plot([vert_1[0,0],vert_2[0,0]],[vert_1[1,0],vert_2[1,0]],[vert_1[2,0],vert_2[2,0]],'-')
        ax.plot([vert_2[0,0],vert_3[0,0]],[vert_2[1,0],vert_3[1,0]],[vert_2[2,0],vert_3[2,0]],'-')
        ax.plot([vert_1[0,0],vert_3[0,0]],[vert_1[1,0],vert_3[1,0]],[vert_1[2,0],vert_3[2,0]],'-')
        
        ax.scatter(centroid_1[0,0],centroid_1[1,0],centroid_1[2,0],'r',s = 10)
        
        
        
        
        
        return ax
        
        
class ScrewsPlot():
    
    def rotation_matrix(self,v1,v2):
        
        '''
        Calculates the rotation matrix that changes v1 into v2.
        '''
        
        if np.any(v1) and np.any(v2):
            v1 = np.divide(v1,np.linalg.norm(v1))
            v2 = np.divide(v2,np.linalg.norm(v2))
        
            cos_angle=np.dot(v1,v2)
            d=np.cross(v1,v2)
            sin_angle=np.linalg.norm(d)
        
            if sin_angle == 0:
                M = np.identity(3) if cos_angle>0. else -np.identity(3)
            else:
                d = np.divide(d,sin_angle)
        
                eye = np.eye(3)
                ddt = np.outer(d, d)
                skew = np.array([[    0,  d[2],  -d[1]],
                              [-d[2],     0,  d[0]],
                              [d[1], -d[0],    0]], dtype=np.float64)
        
                M = ddt + cos_angle * (eye - ddt) + sin_angle * skew
        
            return M
        else:
            raise ValueError('One of the two vectors is a zero vector')
    def pathpatch_2d_to_3d(self,pathpatch, z = 0, normal = 'z'):
        '''
        Transforms a 2D Patch to a 3D patch using the given normal vector.
    
        The patch is projected into they XY plane, rotated about the origin
        and finally translated by z.
        '''
        if type(normal) is str: #Translate strings to normal vectors
            index = "xyz".index(normal)
            normal = np.roll((1,0,0), index)
    
        path = pathpatch.get_path() #Get the path and the associated transform
        trans = pathpatch.get_patch_transform()
    
        path = trans.transform_path(path) #Apply the transform
    
        pathpatch.__class__ = art3d.PathPatch3D #Change the class
        pathpatch._code3d = path.codes #Copy the codes
        pathpatch._facecolor3d = pathpatch.get_facecolor #Get the face color    
    
        verts = path.vertices #Get the vertices in 2D
    
        M = self.rotation_matrix(normal,(0, 0, 1)) #Get the rotation matrix
    
        pathpatch._segment3d = np.array([np.dot(M, (x, y, 0)) + (0, 0, z) for x, y in verts])
        
    

    
    
    def pathpatch_translate(self,pathpatch, delta):
        """
        Translates the 3D pathpatch by the amount delta.
        """
        pathpatch._segment3d += delta  
    
    def Concurrent_screws(self,ax,p_cen,radius,normal_inp): # ax - graphics pallete p_cen should be 3x1, rad - is radius, normal - 3x1
        
        c1 = Circle((0,0), radius, facecolor = 'grey', alpha = 0.6)
        ax.add_patch(c1)
        normal = [normal_inp[0,0],normal_inp[0,1],normal_inp[0,2]]
        self.pathpatch_2d_to_3d(c1, z = 0, normal = normal)
        self.pathpatch_translate(c1,(p_cen[0,0],p_cen[0,1],p_cen[0,2]))
        
        v_circle_rand = np.cross(normal_inp,np.matrix([-1,0,1]))
        v_circle = np.cross(v_circle_rand,normal_inp)
        
        pt_circle = p_cen + radius*V_unit(v_circle)
        ax.quiver(p_cen[0,0],p_cen[0,1],p_cen[0,2],pt_circle[0,0],pt_circle[0,1],pt_circle[0,2],color='k')
        
        for i in range(30,360,30):
            
            pt_circle1 = R_Rod(np.radians(i),normal_inp)*np.transpose(pt_circle)
            pt_circle1 = np.transpose(pt_circle1)
            pt_circle = pt_circle1
            ax.quiver(p_cen[0,0],p_cen[0,1],p_cen[0,2],pt_circle1[0,0],pt_circle1[0,1],pt_circle1[0,2],color='k')
                       
        
        return ax
    
    def Parallel_screws(self,ax,pt_start, pt_ref,number_screws,length_screws,dist_bw_screws,direction_screw,color_input): # Generate parallel Screws, pt_ref is a point on the plane on which the arrows should liee
        
        from copy import deepcopy
        direction_screw = V_unit(direction_screw)

        
        
        quiv_start = deepcopy(pt_start)
        quiv_end = quiv_start + length_screws*direction_screw
        ax.plot((quiv_start[0,0],quiv_end[0,0]),(quiv_start[0,1],quiv_end[0,1]),(quiv_start[0,2],quiv_end[0,2]),color = color_input)
        
        first_per = np.cross(V_unit(pt_ref-pt_start),direction_screw) # Cross product between the start point and the reference point (point on the plance)
        per_direc = np.cross(first_per,direction_screw)
        
        for i in range(number_screws):
            
            queue_start = None
            queue_start = quiv_start[-1] + dist_bw_screws*per_direc
            
            quiv_start = np.vstack((quiv_start,queue_start))
            queue_end = None
            queue_end = quiv_end[-1] + dist_bw_screws*per_direc
            
            quiv_end = np.vstack((quiv_end,queue_end))
            
        for i in range(number_screws):
            ax.plot((quiv_start[i,0],quiv_end[i,0]),(quiv_start[i,1],quiv_end[i,1]),(quiv_start[i,2],quiv_end[i,2]),color = color_input)
            

            pt_start = quiv_end[i] + ((0.05)*direction_screw)
            ax = self.generate_cone(ax,np.array([pt_start[0,0],pt_start[0,1],pt_start[0,2]]),np.array([quiv_end[i,0], quiv_end[i,1], quiv_end[i,2]]),0,0.02,8,'k')
            #ax.scatter3D(quiv_start[i,0],quiv_start[i,1],quiv_start[i,2])
            #ax.scatter3D(quiv_end[i,0],quiv_end[i,1],quiv_end[i,2])
            #ax.plot((quiv_start[i,0],quiv_end[i,0]),(quiv_start[i,1],quiv_end[i,1]),(quiv_start[i,2],quiv_end[i,2]))
            #ax.plot(quiv_start[i,0],quiv_start[i,1],quiv_start[i,2],quiv_end[i,0],quiv_end[i,1],quiv_end[i,2],color='k')

        return ax
    
    
    def Couple(self,ax,p0,p1):
        print p0
        print np.shape(p0)
        print p1
        print np.shape(p1)
        ax = self.generate_cone(ax,1.1*p1,p1,0,0.05,5,'r') # Generate the cone part
        couple_base = Empty_Class() # Generate the base part
        couple_base.p_ini = p0
        v_ax = V_unit(p1 - p0)
        extr = np.linalg.norm(p1-p0)
        p_rand = np.array([-1,0,-1])
        v_cross_rand = np.cross(V_unit(p1-p0),p_rand)
        v_unit = np.cross(v_ax,v_cross_rand)
        couple_base.p1_l_ax = couple_base.p_ini + (extr/2.0)*(v_unit)
        couple_base.p0_l_ax = couple_base.p_ini - (extr/2.0)*(v_unit)
        couple_ceil = Empty_Class() # Generate the ceiling part
        couple_ceil.p_ini = p1        
        couple_ceil.p1_l_ax = couple_ceil.p_ini + (extr/2.0)*(v_unit)
        couple_ceil.p0_l_ax = couple_ceil.p_ini - (extr/2.0)*(v_unit)
        
        link_couple = link_generate()
        ax = link_couple.link_rect_plot(ax,couple_base,couple_ceil,1,0.01) 
        
        
        
        return ax
    
    def generate_cone(self,ax,p0, p1, R0, R1,no_of_sides_cone, color):
        from linearalgebra import V_unit
        from numpy.linalg import norm as norm_np
        """
        Based on https://stackoverflow.com/a/39823124/190597 (astrokeat)
        """       
        # vector in direction of axis
        v = p1 - p0        
        # find magnitude of vector
        mag = norm(v)
        # unit vector in direction of axis
        v = V_unit(v)
        # make some vector not in the same direction as v
        not_v = np.array([1, 1, 0])
        if (v == not_v).all():
            not_v = np.array([0, 1, 0])
        # make vector perpendicular to v
        n1 = np.cross(v, not_v)        
        # print n1,'\t',norm(n1)
        # normalize n1
        n1 = V_unit(n1)
        # make unit vector perpendicular to v and n1
        n2 = np.cross(v, n1)
        # surface ranges over t from 0 to length of axis and 0 to 2*pi
        #no_of_sides_cone = 8
        t = np.linspace(0, mag, no_of_sides_cone)
        theta = np.linspace(0, 2 * np.pi, no_of_sides_cone)
        # use meshgrid to make 2d arrays
        t, theta = np.meshgrid(t, theta)
        R = np.linspace(R0, R1, no_of_sides_cone)
        # generate coordinates for surface
        X, Y, Z = [p0[i] + v[i] * t + R *
                   np.sin(theta) * n1[i] + R * np.cos(theta) * n2[i] for i in [0, 1, 2]]
        ax.plot_surface(X, Y, Z, facecolor=color, color = color,linewidth=0, antialiased=True,alpha = 0.5)
        return ax
    
    
    
    
    

    
    
    


             


'''

#Test parameters
p_in = np.array([0,-1.0,0])
v_axis = np.array([0,1,0])


j1 = revolute_joint()
j1.revolute(ax,p_in,v_axis)
'''

'''
p_in2 = np.array([1,0,0])
v_axis_2 = np.array([1,0,0])


j2 = revolute_joint()
j2.revolute(ax,p_in2,v_axis_2)


cc = link_generate()
cc.link_plot(j1,j2,1)


dd = prismatic_joint()
dd.generate_link(ax,j1,j2)

ee = spherical_joint()
ee.generate_sphere(ax,p_in,1)

ff = vector_plot()
ff.generate_vector(ax,p_in,p_in2)


point  = np.array([1, 2, 3])
normal = np.array([1, 1, 2])

gg = plane_plot()
gg.generate_plane(ax,point,normal,2,2)


p2 = np.zeros(shape = (1,3))
p2[0,0] = 0
p2[0,1] = 0
p2[0,2] = 0


p3 = np.zeros(shape = (1,3))
p3[0,0] = 0
p3[0,1] = 0.5
p3[0,2] = 0


aa = prismatic_joint()
aa.generate_link(p2,p3)

# Create a connection link

x1,x2,x3,x4,x5,x6,x7,x8,x9,x10 = revolute(p1,p0,r_revolute,extr)

ax.plot_surface(x1, x2, x3)


a1 = np.array([10,10,100])
a0 = np.array([0,0,200])
normal = a0 - a1
p33 = Circle((a1[0],a1[1]), r_revolute, facecolor = 'r', alpha = 0.5)
ax.add_patch(p33)
pathpatch_2d_to_3d(p33, z = a1[2], normal = normal)



p44 = Circle((a0[0],a0[1]), r_revolute, facecolor = 'r', alpha = 0.5)
ax.add_patch(p44)
pathpatch_2d_to_3d(p44, z = a0[2], normal = normal)
'''




    
    

  

