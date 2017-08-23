/*
 *		George Terzakis, 2017 
 * 
 * 	`	University of Portsmouth 
 * 
 *      A sample use of the Quaternion class implemented based on the contents of:
 *  
 * "Modified Rodrigues Parameters: An Efficient representation of Orientation in 3D Vision and Graphics"
 * 			G. Terzakis, M. Lourakis and D. Ait-Boudaoud
 * 
 * 
 * Here the Quaternion class is used in the context of the most representative rotation averaging problem:
 * 
 * minimize{ Sum(X-Ri) }
 *    X
 * s.t. det(X)=1
 * 
 * 
 * where Ri are "noisy" estimates of the same rotation matrix.
 * The optimization is done using the Levenberg-Marquardt heuristic over the Gauss-Newton method,
 * while the descent directioons are decided by the rotation matrix element gradients in terms of
 * the respective modified Rodrigues parameters.
 * NOTE: OpenCV is employed for standard algebra operations such as addition, multiplication and inversion. 
 * 	 This is a good example on how to use the Quaternion class with OpenCV Matx and Vec objects.
*/
#include <cv.hpp>
//#include <highgui.hpp>
#include <core.hpp>

#include <random>
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

// a function that computes the sum of chordal distances between a matrix and a numeber of other rotation matrices
double SquaredError(const cv::Matx<double, 3, 3> &X, const std::vector<cv::Matx<double, 3, 3>> &Rs)
{
  double sum = 0;
  for (int i = 0; i < Rs.size(); i++)
  {
    sum += cv::norm(X - Rs[i], cv::NORM_L2SQR);
  }
  
  return sum;
}


int main() 
{
  // random generator
  std::default_random_engine gen_;

  // Creating a ground truth rotation matrix
  cv::Matx<double, 3, 3> Rt;
  // The ground-truth Quaternion:
  cv::Vec<double, 3> u(M_PI/4, M_PI/6, -M_PI/10); // an axis-angle vector (
  Quaternion<double> qt(u.val, QUATERNION_AA);
  // store the rotation matrix in Rt
  qt.RotationMatrix(Rt.val);
  
  int N = 100;
  // Now creating 50 "noisy" rotations from Rt
  std::vector<cv::Matx<double, 3, 3>> data;
  // The Gaussian distribution to sample the noise from 
  double sigma = 0.1; // that's roughly 5.7 degrees...
  normal_distribution<double> NormDist(0, sigma);
  cv::Matx<double, 3, 3> R_noisy; // cache for noisy rotation matrices
  for (int i = 0; i < N; i++)
  {
    // create a "noise" vector to be added to the ground-truth axis-angle vector
    cv::Vec<double, 3> n( NormDist(gen_),
			  NormDist(gen_),
			  NormDist(gen_)
			);
    // Create the noisy quaternion from a noisy axis angle vector
    Quaternion<double> q_noisy( (n + u).val, QUATERNION_AA);
    // Now obtain the respective matrix
    q_noisy.RotationMatrix(R_noisy.val);
    // add it to the data
    data.push_back(R_noisy);
    
  }
  
  // Done generating data!
  
  // //////////// Solving the simplest rotation averaging problem : ///////////////////
  //
  //                 minimize {Sumi(norm(X-Ri)^2)}
  // (i.e., minimize the average chordal distance between the unknown rotation matrix and the data.
  
  // Preparing the Levenberg-Marquardt algorithm
  double lambda = 0.01; // damping factor
  // Create an initial estimate (identity)
  cv::Matx<double, 3, 3> X;
  Quaternion<double> q;
  // assign the rotation matrix of the current estimate to X
  q.RotationMatrix(X.val);
  
  // Average the data rotation matrices (this is a one-off computation)
  cv::Matx<double, 3, 3> avg = cv::Matx<double, 3, 3>::zeros();
  for (int i = 0; i < data.size(); i++)
  {
    avg += data[i];
  }
  avg *= 1.0 / data.size();
  
  // Get the error for this estimate
  double errorsq = SquaredError(X, data);
  double initial_errorsq = errorsq; // keep the initial error for the stats in the end...
  double best_errorsq = errorsq; // best error
  double prev_errorsq = 9999999999.9; // change in error
  int timeout = 30; // timeout in 30 iterations
  
  int step = 0;   // a step counter
  double tolerance = 10E-6;  // tolerance
  bool stop = best_errorsq < tolerance; // stop flag
  
  while (!stop)
  {
    // Compute the new information matrix and information vector
    cv::Matx<double, 3, 3> Omega; // system information matrix
    cv::Vec<double, 3> ksi;       // system information vector
    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
	cv::Matx<double, 1, 3> Jrc;
	// get the element Jacobian
	q.RotationElementJacWRTMRPs(r, c, Jrc.val);
	// Now accumulate the Gram-matrix
	Omega += Jrc.t() * Jrc;
	ksi += Jrc.t() * avg(r, c);
      }
    }
    
    // Now loop until we improve our estimate
    bool improvement = false;
    while (!improvement && !stop)
    {
      // augment the diagonal of Omega
      cv::Matx<double, 3, 3> Omegaaug = Omega + lambda * cv::Matx<double, 3, 3>::eye(); 
      // Solve for the perturbation in the quaternion's MRPs
      cv::Matx<double, 3, 3> Omegainv;
      cv::invert(Omegaaug, Omegainv, cv::DECOMP_LU);
      cv::Vec<double, 3> deltapsi_temp = Omegainv * ksi;
      // Update the quaternion. NOTE: No need to use MRPs explicitly!
      Quaternion<double> q_temp = q.UpdateFromDeltaMRPs(deltapsi_temp.val);
      // get the new rotation matrix
      cv::Matx<double, 3, 3> X_temp;
      q_temp.RotationMatrix(X_temp.val);
      // compute new error
      errorsq = SquaredError(X_temp, data);
    
      if (errorsq < best_errorsq)
      {	
	best_errorsq = errorsq;
	X = X_temp;
	q = q_temp;
     
	lambda /= 10;
	
	improvement = true;
      }
      else
      {
	lambda *= 10;
      }
    
      stop =  fabs(errorsq - prev_errorsq) < 10e-6 || errorsq < tolerance || step > timeout;
      
      prev_errorsq = errorsq;
      step++;
    }
  }
  
  cout <<"Initial sum of squared errors :  "<<initial_errorsq<<" ( Average Chordal Distance: "<< sqrt(initial_errorsq / N)<<" )"<<endl;
  cout <<"Final sum of squared errors :  "<<best_errorsq<<" ( Average Chordal Distance: "<< sqrt(best_errorsq / N)<<" )"<<endl;
  cout <<"Total number of iterations : "<<step<<endl;
  cout <<"Ground truth rotation : "<<Rt<<endl;
  cout <<"Estimated rotation : "<<X<<endl;
   
return 1;  
}

