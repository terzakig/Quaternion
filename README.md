# Quaternion
A Quaternion class that implements general functionality with focus on the Modified Rodrigues Parametrization (MRPs) 

## Description
This is a Quaternion class (`Quaternion.h`) that implements standard functionality such as quaternin products, conversions from unit quaternions to rotation matrices vice versa, elementary interpolation (_SLERP_) differentiation (in terms of _axis-angle_ and _modified Rodrigues_ parameters) and all typical operations (_addition_, _quaternion multiplication_, _scalar multiplication_). The class is fully templated in terms of precision, so it can automatically handle conversions in operations or initializations from other quaternion objects or variables in different precision.

Probably the most important feature of the class is that it implements the analytical rotation and quaternion derivatives in terms of _Modified Rodrigues Parameters_ (MRPs) as well as the implicit quaternion updates from perturbations in MRPs. This way, the quaternion can be employed in iterative optimization problems without even having to comvert to/from MRPs.

The class also provides all the necessary mahinery required to manipulate quaternion without the need to use any additional library (e.g., addition, multiplication, interpolation, exponentiation and logarithm, conversion to rotation matrix, ininitialization from various parameteric and non-parametric forms such as rotation matrices, axis-angle parameters, Gibbs vectors, MRPs). Thus, all input parameters and results are provided/returned in simple arrays in order to be easily ported to matrix structures of the linear algebra library of choice (The code in **Example2** makes use of OpenCV `cv::Matx` and `cv::Vec` objects for standard linear algebra operations - primarily inversions).

For intuition behind the implementation, see ["Modified Rodrigues Parameters: An Efficient Parametrization of Orientation in 3D Vision and Graphics"_ by G. Terzakis, M. Lourakis and D. Ait-Boudaoud](https://link.springer.com/article/10.1007/s10851-017-0765-x). 

## Examples

There are two examples: The first one (_Example1_) contains simple examples of nearly all methods included in the class, so it is a good place to start for one who wants to get to know the class. The second example (_Example2-LM_) is the implementation of an iterative solution of the classic _rotation averaging_ problem in which an unknown rotation matrix `X` is estimated from multiple "noisy" rotation matrices that where somehow sampled from the same orientation. The **Levenberg-Marquardt** is employed using **MRPs**( for differentiation of the cost function, while **OpenCV** is used to provide the necessary linear algebra operations (addition, multiplication and inversion). Thus, the second example illustrates how to use the `Quaternion` class with OpenCV objects and furthermore, it provides an example on the implicit use of MRPs in an iterative method.  

## MATLAB

Almost all the methods included in the `Quaternion` class have a corresponding Matlab script in directory _MATLAB_. Most implementations are identical if not similar, espcially where _MRPs_ are involved. They can be used to confirm the results obtained with C++. The code is fairly intuitive owing to the vectorization with MATLAB arrays. 
