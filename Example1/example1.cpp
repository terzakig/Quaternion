/*
 *                     George Terzakis, 2017
 * 
 * 		    University of Portsmouth
 * 
 *     Code examples for the Quaternions class implementede based on:
 * 
 * "Modified Rodrifues Parameters: An Efficient Representation of Orientation in 3D Vision and Graphics"
 * 	               G. Terzakis, M. Lourakis and D. Ait-Boudaoud
 * 
 * This file contains **simple** examples using all (or nearly all) the methods provided in the Quaternion class. 
 * Feel free to verify the results against the Matlab implementations.
 * See the rotation averaging example (example2) for more sophisticated code that refines a rotation matrix over an iterative method.
*/

#include <iostream>

using namespace std;

#include "../Quaternion.h"
using namespace RigidTransforms;


// A helper function for printing-out standard 1D arrays as matrices (matlab style)
template<typename P>
void PrintMatrix(const P* mat, int rows, int cols)
{
  cout <<"[";
  
  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      cout <<mat[r * cols + c];
      if (c < cols - 1) cout <<", ";
    }
    if (r < rows - 1) cout <<";"<<endl;
  }
  cout <<"]"<<endl;
  
}


int main() 
{

  // Create a floating point quaternion (default)
  Quaternion<> q1 = {1, 2, 3, 4};
  
  // Create a double-precision quaternion from a Gibbs-vector
  float g[3] = {1, 2, 3};
  Quaternion<double> q2(g, QUATERNION_GIBBS);   // parameter is a Gibbs vector
  // Quaternion<double> q2(g, QUATERNION_AA);   // parameter is an Axis-angle vector
  // Quaternion<double> q2(g, QUATERNION_MRPs); // parameter is an MRP vector
  

  // Create  third quaternion as the difference of the first two
  Quaternion<> q3 = q2 - q1;
  //Quaternion<> q3 = q1 + q2; // addition respectively
  
  
  
  // normalize the non-unit quaternions q1 and q3
  q1.unit();
  q3.unit();
  
  // Obtain the rotation matrices R1, R2, R3 in 9 - arrays
  double R1[9], R2[9], R3[9];
  q1.RotationMatrix(R1);
  q2.RotationMatrix(R2);
  q3.RotationMatrix(R3);
  
  // Finally, create a fourth quaternion from the rotation matrix of the third
  Quaternion<> q4(R3);
  
  
  // print out the quaternions
  cout<<"Quaternion #1 : "<<q1<<endl;
  cout<<"Quaternion #2 : "<<q2<<endl;
  cout<<"Quaternion #3 : "<<q3<<endl;
  cout<<"Quaternion #4 : "<<q4<<endl;
  
  // Now obtain the dot-produce of q3 and a4 as Euclidean vectors (should be 1)
  cout<<"Dot product of q3 and q4 : "<<q3.Dot(q4)<<endl;
  
  // obtain the Quaternion product of q2 and q4:
  Quaternion<double> q5 = q2.Mul(q3);
  
  cout <<"Quaternion #5 (product of #2 and #3): "<<q5<<endl;

  
  // Scalar multiplication
  cout<<"Quaternion #5 multiplied by 10 : "<<q5.Muls(10)<<endl;
  
  // Derivatives QRT quaternion components
  double dr12dscalar = q1.RotationJacWRTquat(0, 1, 0);
  cout <<"Derivative of r12 WRT quaternion scalar : "<<dr12dscalar<<endl;
  
  // Get the derivative of the rotation matrix in terms of the scalar part:
  float dRds[9]; // 9 = 3x3
  q1.RotationJacWRTscalar(dRds);
  cout <<"The derivative of the R1 in terms of q1.s : "<<endl;
  PrintMatrix(dRds, 3, 3);
  
  // Get the derivative(s) R1 in terms of the vector part
  double dRdv1[9], dRdv2[9], dRdv3[9];
  q1.RotationJacWRTvector(0, dRdv1);
  q1.RotationJacWRTvector(1, dRdv2);
  q1.RotationJacWRTvector(2, dRdv3);
  
  
  cout <<"The derivative of R1 in terms of q1.v1 : "<<endl;
  PrintMatrix(dRdv1, 3, 3);
  
  cout <<"The derivative of R1 in terms of q1.v2 : "<<endl;
  PrintMatrix(dRdv2, 3, 3);
  
  cout <<"The derivative of R1 in terms of q1.v3 : "<<endl;
  PrintMatrix(dRdv3, 3, 3);
  
  // Derivative WRT MRPs
  float dRdpsi1[9], dRdpsi2[9], dRdpsi3[9];
  q1.RotationJacWRTMRPs(0, dRdpsi1);
  q1.RotationJacWRTMRPs(1, dRdpsi2);
  q1.RotationJacWRTMRPs(2, dRdpsi3);
  
  cout <<"The derivative of R1 in terms of MRP #1  : "<<endl;
  PrintMatrix(dRdpsi1, 3, 3);
  
  cout <<"The derivative of R1 in terms of MRP #2  : "<<endl;
  PrintMatrix(dRdpsi2, 3, 3);
  
  cout <<"The derivative of R1 in terms of MRP #3  : "<<endl;
  PrintMatrix(dRdpsi3, 3, 3);
  
  // Get the Jacobian of R1*v hwere v is a 3D vector in an array 
  float v[3] = {1, 2, 3};
  double rot_v[3];
  q1.RotatedVectorJacWRTMRPs(v, rot_v);
  
  cout <<"The rotated vector (by R1) derivative: "<<endl;
  PrintMatrix(rot_v, 3, 3);
  
  // Get the Jacobian of R1'*v hwere v is a 3D vector in an array 
  double rot_trans_v[3];
  q1.RotatedTranspVectorJacWRTMRPs(v, rot_trans_v);
  
  cout <<"The rotated vector (by R1') derivative: "<<endl;
  PrintMatrix(rot_trans_v, 3, 3);
  
  
  
  // Finally, trying SLERP (midpoint in this case-easily verified)
  auto q = Quaternion<>::SLERP(q1, q3, 0.5);
  
  cout <<"The SLERP-ed quaternion at 1/2 between q1 = "<<q1<<" and q3 = "<<q3<<" is : " <<q<<endl;
  
  
  // Testing the Jacobians of Rotation matrix elements WRT Modified Rodrigues Parameters
  float drijdp[3]; // cache for the 1x3 Jacobian 
  for (int r = 0; r < 3; r++)
  {
   for (int c = 0; c < 3; c++)
   {
     cout<<"Rotation Element ("<<r<<", "<<c<<" ) Jacobian :"<<endl;
     q1.RotationElementJacWRTMRPs(r, c, drijdp);
     PrintMatrix(drijdp, 1, 3 );
   }
  }
  
  
  // The Jacobian of the Quaternion scalar WRT Axis-Angle Parameters
  double Js[3];
  q1.QuaternionScalarJacWRTAA(Js);
  cout << "The Quaternion scalar Jacobian in terms of axis-angle parameters : "<<endl;
  PrintMatrix(Js, 1, 3);
  
  double Jv[9];
  q1.QuaternionVectorJacWRTAA(Jv);
  cout << "The Quaternion vector Jacobian in terms of axis-angle parameters : "<<endl;
  PrintMatrix(Jv, 3, 3);
  
  
return 1;  
}

