# Introduction

## degree of freedom

<img src="diagrams/diagram-13.png" alt="drawing" width="400"/>

# Spatial Descriptions and Transformation Matrices

## rotation matrix

<img src="diagrams/diagram-9.png" alt="drawing" width="400"/>

## ref

1. Spatial Descriptions and Transformation Matrices for Robotic Manipulators https://www.youtube.com/watch?v=4Y1_y9DI_Hw
2. Spacial Descriptions and Transformations - Worked Example https://www.youtube.com/watch?v=r244ADkuexk

screw asix

prismatic

# kinematics

the forward kinematics problem is to find the configuration of the end effector ([b]-frame) relative to the [s]-frame given the vector of joint angles theta (joint variable)

forward kinematics: x = e(q)
inverse kinematics: q = e'(x)
ref: https://www.youtube.com/watch?v=jtei695t4VY

## DH table to transformation matrix

<img src="diagrams/diagram-17.png" alt="drawing" width="600"/>

<img src="diagrams/diagram-18.png" alt="drawing" width="600"/>

<img src="diagrams/diagram-4.png" alt="drawing" width="400"/>

DH table: https://www.youtube.com/watch?v=DPO9Se6ZqN0&list=PLZaGkBteQK3HQFSWDM7-yRQWTd86DeDIY&index=7

<img src="diagrams/diagram-3.png" alt="drawing" width="400"/>

## vector of joint variables

joint variables (q)
q = [q1, q2, q3]
for revolute joints, q_i = theta_i
for prismatic joints, q_i = distance_i

# inverse kinematics

Given the desired position and orientation of the tool relative to the station, compute the set of joint angles which wili achieve this desired result

## solvability

it is difficult to solve if number of degree larger than 6
<img src="diagrams/diagram-10.png" alt="drawing" width="400"/>

# jacobian matrix and singularities

jacobian matrix is the velocity of the joint in the system

e.g. V = [x', y', z'] (cartesian space) = [v, w] (joint space where v = linear and w = angular) = J [theat1, theat2]

ref: https://www.youtube.com/watch?v=h2YM0CDzDl4&t=458s

## general force vectors

general force = [F, N], where F is a 3 x 1 force vector and N is a 3 x 1 moment vector

## VELOCITY "PROPAGATION" FROM LINK TO LINK

the angular velocity of link i + 1 is the same as that of link i plus a new component caused by rotational velocity at joint i + 1

## Skew-symmetric matrices and the vector cross-product

<img src="diagrams/diagram-11.png" alt="drawing" width="400"/>

# Dymanic

There are two problems related to the dynamics of a manipulator that we wish to solve.

1. In the first problem, we are given a trajectory point, e, and ë, and we wish to find the required vector of joint torques, r.

2. The second problem is to calculate how the mechanism will move under application of a set of joint torques. That is, given a torque vector, r, calculate the resulting motion of the manipulator, P, P' and P''.

## notation

    •	θ or q is used to represent the joint position
    •	θ̇ or q̇ is used to represent the joint velocity
    •	θ̈ or q̈ is used to represent the joint acceleration

## manipulator's dynamic equations (The state-space equation)

<img src="diagrams/diagram-5.png" alt="drawing" width="400"/>

## Inertia Tensor

https://www.youtube.com/watch?v=Ch-VTxTIt0E

double integrals: https://tutorial.math.lamar.edu/classes/calciii/DoubleIntegrals.aspx

### inertia tensor

- tensor of gives us an idea about how the mass is distributed in a rigid body
- also it is the resistance of an object to changes in its motion or state of rest

### intertia matrix M(q)

- the inertia matrix is symmetric and positive definite

### Mass moment of inertia

a body’s resistance to a change in its rotation direction or the angular momentum

## newton-eulr equation approach

```
^{i}W_i = R * q' * ^{i}Z_i
^{i}W'_i = R * ^{i}W'_i-1 * ^{i}Z_i
^{i}v_i = R * ^{i-1}vC_i-1
^{i}vC_i = ^{i}w'\_i * P + ^{i}w_i^2 * P + ^{i}v_i
^{i}F_i  = M * ^{i}v_Ci
torque = Z component of ^{i}n_i or M(p) * p'' + V(p, p') + G(p) (6.59 formula)
```

### ^{1}F_1

F refers to the force acting on a body or link 1 in a robotic manipulator, expressed in the frame attached to link 1.

### ^{1}N_1

N refers to the torque or moment acting on link 1, expressed in the same frame attached to link 1.

## Lagrange approach

https://www.youtube.com/watch?v=zO5o-tAwOPc

L = K-P (k = kinetic energy and P = potential energy)
K = 0.5*m*v^2 (where m = mass, v = velocity)
k(q, q') = 0.5*q'T*M(q)\*q'

<img src="diagrams/diagram-14.png" alt="drawing" width="400"/>

<img src="diagrams/diagram-15.png" alt="drawing" width="400"/>

<img src="diagrams/diagram-16.png" alt="drawing" width="400"/>

ref: https://www.youtube.com/watch?v=QN-Awth50aA

### potential energy formular

p = mass \* height

## Newton-Euler vs Lagrange approach

in the Lagrangian formulation we treat the manipulator as a whole and perform the analysis using a Lagrangian function (the difference between the kinetic energy and the potential energy). In contrast, in the Newton-Euler formulation we treat each link of the robot in turn, and write down the equations describing its linear motion and its angular motion. Of course, since each link is coupled to other links, these equations that describe each link contain coupling forces and torques that appear also in the equations that describe neighboring links. By doing a so-called forward-backward recursion, we are able to determine all of these coupling terms and eventually to arrive at a description of the manipulator as a whole. Thus we see that the philosophy of the Newton-Euler formulation is quite different from that of the Lagrangian formulation.

calculation rotational velocity (w) -> linear acceleration -> calcuatte force and torque on a link

## static force

https://www.youtube.com/watch?v=R7FGiA9WF-Y

# trajectory generation

time history of position, velocity and acceleration for each degree of freedom

ref: https://www.youtube.com/watch?v=TXHM9xsRUkA&list=PLZaGkBteQK3HQFSWDM7-yRQWTd86DeDIY&index=15

## path generation methods

### joint space schemes

#### cubic polynomials

<img src="diagrams/diagram-6.png" alt="drawing" width="400"/>

#### parabolic blends

<img src="diagrams/diagram-7.png" alt="drawing" width="400"/>

<img src="diagrams/diagram-8.png" alt="drawing" width="400"/>

### cartesian schemes

## linear control

https://www.bilibili.com/video/BV1p7411m7Za?p=21&spm_id_from=pageDriver

open-loop control/ feedforward control
there is no sensing of the actual joint position to close a feedback loop

- if there is ever any error in the joint position, however, this open-loop approach cannot recover

closed-loop control

- with feedback

# manipulator-mechanism design

## compliance

“Compliance” is a measure of the susceptibility of a structure to move as a result of an external force. The greater the compliance (i.e., the lower the stiffness), the more easily the structure moves as a result of an applied force.

## stiffness and deflections

<img src="diagrams/diagram-12.png" alt="drawing" width="400"/>

## others

body-frame transformation, it postmultiplies M -> M \* e

# youtube reference

Compound Transformation Matrices and Inverse Transformation Matrices - Robotic Basics
https://www.youtube.com/watch?v=sm7d1A2npWA

exam
https://www.youtube.com/watch?v=dASdcqgBlqw

# others

1. https://www.youtube.com/watch?v=_GIk3vhiU3c
2. https://www.pdfdrive.com
3. https://see.stanford.edu/materials/aiircs223a/solution4.pdf
4. https://courses.skule.ca
