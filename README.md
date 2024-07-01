# Inverse_Kinematics_Exechon_3DOF
A visual planner of the Exechon - 3 DOF parallel manipulator.

Exechon parallel manipulator has 3 linear actuators, where two actuators lie on a rotating plane and the third actuator along a spherical joint.
The detailed kinematic equations have been described in the research paper " Zoppi, Matteo, Dimiter Zlatanov, and Rezia Molfino. "Kinematics analysis of the Exechon tripod." ASME 2010 international design engineering technical conferences and computers and information in engineering conference. American Society of Mechanical Engineers, 2010."

A hybrid parallel serial architecture of Exechon- with Serial arm was developed for the EU FP7 "SwarmItFix" project. This kinematic planner was developed with native Python, Matplotlib and math to obtain a lightweight simulator to visualize the plan of the manipulator. Only the Exechon plan is available in this repository, a much detailed version of the entire Parallel-Serial chain will be available after a planned publication. 

# Prerequisites

Python 2.7, Numpy, Matplotlib, Qt5 library

# Instructions

1. Clone the repository to your account.
2. The main planner file is "test_planner_PKM_visual.py"
3. Run the planner file to generate the visual representation of the actuated joints of the Exechon.

# Input

1. Inputs are represented as p_x,p_y,p_z (Line 56), which in-turn represents the position of the off-set spherical joint over the exechon platform in 3D space.
2. The parameters of the link lengths can be modified in "IK_PKM_params.py". Naming convention is similar to the research paper. Research paper is attached for reference in this current repository.

# Output

1. Visual output (Matplotlib graphs)

1.1 (x,y,z) = [0,0,0.35]

![alt text](https://github.com/KeerthiSagarSN/Inverse_Kinematics_Exechon_3DOF/blob/master/Output_images/x_0_y_0_z_0.35_img1.png)

![alt text](https://github.com/KeerthiSagarSN/Inverse_Kinematics_Exechon_3DOF/blob/master/Output_images/x_0_y_0_z_0.35_img2.png)

![alt text](https://github.com/KeerthiSagarSN/Inverse_Kinematics_Exechon_3DOF/blob/master/Output_images/x_0_y_0_z_0.35_img3.png)


1.2 (x,y,z) = [0,0.2,0.35]

![alt text](https://github.com/KeerthiSagarSN/Inverse_Kinematics_Exechon_3DOF/blob/master/Output_images/x_0_y_0.2_z_0.35_img1.png)

![alt text](https://github.com/KeerthiSagarSN/Inverse_Kinematics_Exechon_3DOF/blob/master/Output_images/x_0_y_0.2_z_0.35_img2.png)

![alt text](https://github.com/KeerthiSagarSN/Inverse_Kinematics_Exechon_3DOF/blob/master/Output_images/x_0_y_0.2_z_0.35_img3.png)


1.3 (x,y,z) = [0.05,0.2,0.30]


![alt text](https://github.com/KeerthiSagarSN/Inverse_Kinematics_Exechon_3DOF/blob/master/Output_images/x_0.05_y_0.2_z_0.30_img1.png)

![alt text](https://github.com/KeerthiSagarSN/Inverse_Kinematics_Exechon_3DOF/blob/master/Output_images/x_0.05_y_0.2_z_0.30_img2.png)

![alt text](https://github.com/KeerthiSagarSN/Inverse_Kinematics_Exechon_3DOF/blob/master/Output_images/x_0.05_y_0.2_z_0.30_img3.png)


2. Joints (Actuated joint length), in this case, the stroke of each linear actuator to achieve the desired pose for the platform and the spherical joint.


If you found this code useful, we would be grateful if you cite the following reference:

[1] Sagar, Keerthi, Vishal Ramadoss, and Matteo Zoppi. "Towards High Dynamic Operations With Parallel-Serial Hybrid Robots." International Design Engineering Technical Conferences and Computers and Information in Engineering Conference. Vol. 87363. American Society of Mechanical Engineers, 2023.

[2] Zoppi, Matteo, Dimiter Zlatanov, and Rezia Molfino. "Kinematics analysis of the Exechon tripod." International design engineering technical conferences and computers and information in engineering conference. Vol. 44106. 2010.

Bibtex:

```
@inproceedings{sagar2023towards,
  title={Towards High Dynamic Operations With Parallel-Serial Hybrid Robots},
  author={Sagar, Keerthi and Ramadoss, Vishal and Zoppi, Matteo},
  booktitle={International Design Engineering Technical Conferences and Computers and Information in Engineering Conference},
  volume={87363},
  pages={V008T08A064},
  year={2023},
  organization={American Society of Mechanical Engineers}
}
```
```
@inproceedings{zoppi2010kinematics,
  title={Kinematics analysis of the Exechon tripod},
  author={Zoppi, Matteo and Zlatanov, Dimiter and Molfino, Rezia},
  booktitle={International design engineering technical conferences and computers and information in engineering conference},
  volume={44106},
  pages={1381--1388},
  year={2010}
}

```

You can find the video accompanying for the article [here](https://www.youtube.com/watch?v=V-pb8RU4bpY)






