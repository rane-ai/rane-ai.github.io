---
layout: post
title:  "Notes on Rotations"
---

## Introduction

Rotations are a central object in geometry processing and of particular interest, *pose estimation*. However, they can be complicated to understand and work with so this post seeks to walk through all things rotations with a focus on 3D. A simple implementation of the Iterative Closest Point (ICP) algorithm using the different parameterisations reported here is available to help understand this material/enable experimentation. 

<p align="center">
  <img src="/assets/coordinate_frame.png" alt="drawing" width="30%"/>
  <em>In 3D an object has 3 angles roll, pitch and yaw</em>
</p>

## Representations

There exist several different ways to represent rotations mathematically. This post will consider rotation matrices and quaternions. We shall start off with the most common and my favourite: matrices, a non-minimal representation. 

# Rotation Matrices/Euler Angles

In 3D, the physical concepts of roll, pitch and yaw - as sketched above - can be encoded in 3 corresponding matrices as follows:

$$
R_{roll} = 
\begin{bmatrix}
 1  & 0           &  0           \\
 0  & \cos(\gamma) & -\sin(\gamma) \\
 0  & \sin(\gamma) &  \cos(\gamma)
\end{bmatrix}
\quad
R_{pitch} = 
\begin{bmatrix}
\cos(\beta)   & 0 & \sin(\beta) \\
 0            & 1 & 0           \\
-\sin(\beta)  & 0 & \cos(\beta) 
\end{bmatrix}
\quad
R_{yaw} = 
\begin{bmatrix}
\cos(\alpha) & -\sin(\alpha) & 0 \\
\sin(\alpha) &  \cos(\alpha) & 0 \\
0           & 0              & 1
\end{bmatrix}
$$

where $$ \alpha,\beta,\gamma $$ are radian angles. A combined matrix, $$R$$, is formed by taking the product of these individual per-axis matrices e.g.  

$$  
R(\alpha,\beta,\gamma) = R_{yaw}(\alpha)R_{pitch}(\beta)R_{roll}(\gamma)
$$  

when multiplied in this order the angles are known as Euler angles. Note, it is possible to multiply in a different order but the above order is common. Why is this non-minimal? Physically, there are 3 angles which are represented by 9 numbers in matrix form. A rotation can be applied to a point by left multiplication:

$$
{}^wX = {}^wR_l{}^lX
$$

which transforms the point $$X$$ from coordinate frame $$l$$ into frame $$w$$ - the *from* and *to* frames are encoded in the sub/super scripts of $$R$$ respectively. Rotations act in an a counter-clockwise manner, to rotate the other way $$R^{-1}$$ (which is equal to $$R^T$$) may be applied. Conveniently, a rotation matrix can be easily visualised by plotting its columns as a coordinate frame (after rooting the vectors at a common point). The X, Y and Z axes are simply the 1st, 2nd and 3rd columns respectively. See the following plots of and identity rotation and one which corresponds to (10, 20, 30) degs of rotation. 

<p align="center">
  <img src="/assets/coordinate_viz2.png" alt="drawing" width="50%"/>
  <em>LHS: I, RHS: Euler angles of 10,20,30deg</em>
</p>

Given that this is how a rotation matrix is built, it should be clear that not *any old* 3x3 matrix is a rotation matrix! Rotations have a specific structure, in particular $$R$$ is orthonormal. What this means is that its columns are of unit magnitude, mutually perpendicular, $$R^{-1}=R^T$$ and additionally for a right handed frame $$\operatorname{det}(R)=+1$$.

OK, so say you have a 3x3 matrix which is not quite orthogonal, can I form the nearest rotation? Yes, via the SVD of the matrix. Given some, $$R^{approx}$$, computed from noisy data, the proper rotation can be found as follows:

$$ 
R^{approx} = U \Sigma V
$$

then

$$
R = UV
$$ 

is orthonormal. If the determinant is negative, negating the singular vector corresponding to the smallest singular value will yield a +1 $$\operatorname{det}$$.

This way to represent angles has some issues though. A well known problem called *gimbal lock* can occur, which corresponds to there being a loss of 1 degree of freedom in the rotation. This is a singularity where the input angles $$ (\alpha,\beta,\gamma) = (0, \pm\pi/2, 0)$$ which effectively results in $$\alpha$$ and $$\gamma$$ having the same effect, see [4]. Another potential issue happens, numerically, when composing several matrices in that the resultant matrix could be non-orthogonal, as a result of rounding etc, thus scaling the object being rotated. This condition, referred to as *drift*, needs to be checked and remedied via re-orthogonalised. However, applying rotations is straightforward and they are easy to visualise - I typically work with them when doing geometry processing. 

Interestingly, given some, $$R$$, adding to it some increment e.g. $$R' = R + \Delta X$$ does not guarantee that $$R'$$ is a rotation. However, it does work if $$\Delta X$$ is structured correctly as we shall see in a later section. Immediately, this means that interpolation between 2 rotation matrices is not straightforward ... In addition to this, how do we optimise w.r.t. a rotation matrix in such as way that the structure is respected? This is not clear. Naively, a first thought would be to unpack a rotation into a 9x1 vector, optimise the elements individually then project to the nearest matrix using the SVD... this sounds like it could be very wasteful though... Also, it is intellectually dissatisfying that it takes 9 numbers to represent 3 physical attributes. Another way would be to convert back to Euler angles, adjust and reconstruct a matrix - this is possible but still suffers from gimbal lock. This brings us on to quaternions!

# Quaternions

Loosely, Eulers rotation theorem says that the action of a rotation matrix upon an object is equivalent to the object rotating by some angle $$\theta$$ around an axis $$\boldsymbol{u}$$: this is called an *axis-angle* representation. The most popular form of axis-angle methods are the unit magnitude *quaternions*.

Quaternions are 4D complex numbers of the form

$$
\boldsymbol{q} = a + bi + cj + dk
$$

where $$i,j,k$$ are imaginary units which combine according to a set of rules, see [5]. The conjugate is given by

$$
\boldsymbol{q^*} = a - bi - cj - dk
$$

while the norm is given by 

$$
||\boldsymbol{q}|| = \sqrt{(a^2 + b^2 + c^2 + d^2)}
$$

A point $$X$$ can be rotated via the product

$$
{}^w\boldsymbol{p} = \boldsymbol{q} {}^l \boldsymbol{p} \boldsymbol{q^*} 
$$

where $$\boldsymbol{p}$$ is the (pure) quaternion whose imaginary part has been set to $$X$$. Similarly to rotation matrices, composition of multiple rotations is done through quaternion-multiplication, the output of which is another unit magnitude quaternion. The format of quaternion written as above does not reveal detail of the axis and angle. This can be seen through the alternative form:

$$
\boldsymbol{q} = \cos(\theta/2) + \boldsymbol{u}\sin(\theta/2)
$$

with

$$
\boldsymbol{u} = iu_x + ju_y + ku_z
$$

The axis-angle parameters, $$\theta$$ and $$\boldsymbol{u}$$ can be recovered straightforwardly from this format. Note, to rotate in the opposite direction the inverse quaternion may be applied. $$\boldsymbol{q}^{-1} = \boldsymbol{q}^*$$. It is possible to convert a quaternion into a rotation matrix, as follows:

$$
R = 
\begin{bmatrix}
1 - 2c^2 - 2d^2 & 2bc - 2da       & 2bd + 2cd        \\
2bc + 2da       & 1 - 2b^2 - 2d^2 & 2cd - 2ba        \\
2bd - 2ca       & 2cd + 2ba       & 1 - 2b^2 - 2c^2  \\
\end{bmatrix}
$$

Quaternions have serveral advantages over rotation matrices. They are a minimal representation as a result of the unit magnitude constraint - the 4th component can always be recovered from the other three. Importantly, quaternions don't suffer from gimbal lock - this makes them a popular choice in computer graphics. Interpolation is possible through the Spherical Interpolation (SLERP) algoritm. In fact, one way to interpolate rotation matrices is to convert to a quaternion, carry out SLERP and convert back! Additionally, it is easier to see how optimsation w.r.t. a quaternion would work - this is covered in the ICP demo. However, depending upon the implementation, the aforementioned problems of drift also apply to quaternions and needs to be handled. 

So I have covered two very different looking approaches and it may seem strange that they both describe rotations. It turns out that rotation matrices and quaternions are both parameterisations of the Special Orthogonal Group SO(3) - methods making use of SO(3) are now common in the literature, hence the following section. This is of interest for optimisation w.r.t a rotation matrix as understanding SO(3) reveals the form of an incremental rotation.


# **S**pecial **O**rthogonal Group of dimension **3**: SO(3)

So why does adding an increment to a rotation matrix not work in general? This stems from the fact that rotation matrices do not belong to a vector space, rather they belong to a *Group*, in particular SO(3). Adding an increment could result in *jumping out* of the space of rotation matrices. This is of interest in the context of optimisation (w.r.t a rotation matrix) where normally increments are added to some intial value to drive down the loss. 

There are several papers on this topic but few are as accessible as the excellent Sola et al [1] to which I refer the reader for an in-depth analysis. With pragmatism in mind, I will simply summarise the main results from [1] in order to have sufficient working knowledge to apply the described methods.
 
<p align="center">
  <img src="/assets/manifold.png" alt="drawing" width="30%"/>
  <em>Manifold, M, Inspired by [1]</em>
</p>

At a high level, rotation matrices are said to exist on an abstract mathematical object known as a *manifold*. This can be thought of as a smooth, everywhere differentiable, surface. This manifold is not a vector space. However, *locally*, over a small area, the tangent (hyper) plane, $$T_xM$$, resembles a vector space of angular velocities - see sketch. An commonly used example to visualise this, is to consider the earth which, *globally*, is a sphere but *locally* is well approximated as being planar. Quaternions are another parameterisation of SO(3) and have their own tangent plane representation. However since optimising w.r.t. a quaternion is clearer - the remainder of this post will focus on rotation matrices where this theory is more important. When researching this topic I found the tutorial [7] quite helpful.

It turns out that the $$\operatorname{exp}$$ and $$\operatorname{log}$$ operators have a central role in mapping from the tangent space to the manifold and vice versa respectively. At first glance these functions may seem unexpected. However, [1] develop the theory underlying SO(3) from a familiar starting place in (unit magnitude) complex numbers, which describe a unit circle. As is well known, in exponential form, complex multiplication can be viewed as 2D rotation in the complex plane - this is where $$\operatorname{exp}$$ comes into play. The extension then to 3D via (unit) Quaternions becomes easier to follow. 

<!--- Tanget plane --->
The following theory assumes that the tangent plane is evaluated at the identity rotation on the manifold and only *small*, incremental, rotations are of interest. Elements of the tanget space known as the, Lie Algebra, have the form of skew-symmetric matrices:  

$$
[\boldsymbol{\omega}]_x = 
\begin{bmatrix}
 0        & -\omega_x &  \omega_y  \\
 \omega_x &  0        & -\omega_z  \\
-\omega_y &  \omega_z & 0 
\end{bmatrix}
$$  

where $${\omega}_i$$ is the derivative of angle, i.e. angular velocity. Drummond [2] arrives at this result through considering how the form of a small perturbation of the identity rotation, $$I$$ would appear. Additionaly, [2] notes that the tangent plane looks the same at any point on the manifold. If we want to evaluate the tangent plane at another point we can do so by starting off with $$I + \Delta $$ where $$\Delta$$ is a member of the tangent space followed by left multiplying by the evaluation point on the manifold, $$R$$, resulting in $$R+R\Delta$$.

As you can see, there are only 3 unique values, so this can be effectively flattened into a 3-vector as $$\boldsymbol{\omega}$$. The "flattening" operator is referred to as *vee*, [ $$]^v$$, while its inverse as *hat*, [ ]^. Interestingly, elements, $$[\boldsymbol{\omega}]_x$$, can be further broken down as a linear combination of so-called *generators* i.e. fundamental building blocks.  

$$
[\boldsymbol{\omega}]_x = \omega_x E_x + \omega_y E_y + \omega_z E_z 
$$  

The generators have the following form:

$$
E_x = 
\begin{bmatrix}
0 & 0 &  0 \\
0 & 0 & -1 \\
0 & 1 &  0
\end{bmatrix}
\quad
E_y = 
\begin{bmatrix}
 0 & 0 & 1 \\
 0 & 0 & 0 \\
-1 & 0 & 0
\end{bmatrix}
\quad
E_z = 
\begin{bmatrix}
0 & -1 & 0 \\
1 &  0 & 0 \\
0 &  0 & 0
\end{bmatrix}
$$

which somewhat mirror the per-axis rotation matrices above. [3] shows that the generators are in fact the derivative of the per-axis rotations evaluated at the Identity. We can map from the tangent space to the manifold through a matrix exponential as follows:

$$
R = \operatorname{exp( [\boldsymbol{u}\theta]_x  )} = I + [\boldsymbol{u}]_x sin(\theta) + [\boldsymbol{u}]_x^2 (1 - cos(\theta))
$$

commonly known as the Rodrigues rotation formula. Through the $$Log$$ operator, manifold elements are mapped to the tangent space as follows:

$$
Log(R) = [ \frac{\theta(R - R^T)}{2 sin(\theta)} ]^v
$$

where $$\theta$$ is given by:

$$
\theta = cos^{-1} \frac{ (\mathrm{Tr}(R)-1) }{2}
$$

This means that incremental rotations can be represented by an angular velocity $$  \boldsymbol{\omega} =[ \omega_x,  \omega_y,  \omega_z] $$ - this is minimal.  The important point here is that elements of the tanget plane live in a vector space and so we can add angular velocities and map back to the manifold to obtain a valid rotation matrix. *This* is the mechanism by which we can optimise w.r.t. a rotation matrix.


## Demo

In order to demo the above parameterisations a simple version of ICP was implemented in Python with the approach closely following [5]. It is definitely not optimised and has been written with a straightforward understanding in mind. As well as optimising for rotation, the translation is also adjusted (i.e. full 6 degree of freedom rigid body transformation) - the translation aspects were not covered here but can be readily understood in [1]. 


## References

[1] Sola, J., Deray, J. and Atchuthan, D., 2018. A micro Lie theory for state estimation in robotics. arXiv preprint arXiv:1812.01537 

[2] Drummond, T., 2014. Lie groups, Lie algebras, projective geometry and optimization for 3D Geometry, Engineering and Computer Vision  

[3] Eade, E., 2013. Lie groups for 2d and 3d transformations. URL http://ethaneade.com/lie.pdf, revised Dec, 117, p.118  

[4] Wyss-Gallifent J, MATH431: Gimbal Lock, URL (http://www.math.umd.edu/~immortal/MATH431/book/ch_gimballock.pdf)

[5] Kuipers, J.B., 1999. Quaternions and rotation sequences: a primer with applications to orbits, aerospace, and virtual reality. Princeton university press.

[6] Grisetti, G., Guadagnino, T., Aloise, I., Colosi, M., Della Corte, B. and Schlegel, D., 2020. Least squares optimization: From theory to practice. Robotics, 9(3), p.51

[7] Manifolds #1 - Introducing Manifolds https://youtu.be/GqRoiZgd6N8