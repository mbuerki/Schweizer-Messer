// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

// Helpful functions from libsm
#include <sm/eigen/gtest.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

TEST(QuaternionAlgebraTestSuite, testRotation)
{

  try {
  using namespace sm::kinematics;
  for(int i = 0; i < 1000; i++)
    {
      // Create a random quaternion
      Eigen::Vector4d q_a_b = quatRandom();
      
      // Create a random point in R3
      Eigen::Vector3d v_b;
      v_b.setRandom();
      
      Eigen::Vector3d v_a1 = quat2r(q_a_b) * v_b;
      Eigen::Vector3d v_a2 = quatRotate(q_a_b, v_b);
      
      sm::eigen::assertNear(v_a1, v_a2, 1e-10,SM_SOURCE_FILE_POS, "The rotation matrix And shortcut rotations are not equal");
      
    }
  } catch(const std::exception & e)
    {
      std::cout << "Exception: " << e.what() << std::endl;
    }
  
}



TEST(QuaternionAlgebraTestSuite, testJacobian)
{
  // I'm not completely sure how to do this...
  

}

TEST(RotationExpressionNodeTestSuites,testQuatLogJacobian)
{
	try {

	    using namespace sm::kinematics;
	    Eigen::Vector4d initialValue = quatIdentity();

	    double eps = 1e-7;

	    Eigen::Vector3d AAInitial = sm::kinematics::qlog(initialValue);

	    //std::cout << std::setprecision(15) << "Initial Value: " << std::endl << initialValue << std::endl;
	    //std::cout << std::setprecision(15) << "Initial AA: " << std::endl << AAInitial << std::endl;

	    Eigen::MatrixXd Jest = Eigen::MatrixXd(3,4);

    	double dx = 2*eps;

	    for(int c = 0; c < 4; c++)
	    {
	    	Eigen::Vector4d updatedQuat;
	    	updatedQuat = initialValue;
	    	updatedQuat(c) += eps;
	    	//std::cout << "Updated quat: " << std::endl << updatedQuat << std::endl;
	    	Eigen::Vector3d AAUpdatedPlus = sm::kinematics::qlog(updatedQuat);
	    	//std::cout << "AAUpdatedPlus: " << std::endl << AAUpdatedPlus << std::endl;
	    	updatedQuat = initialValue;
	    	updatedQuat(c) -= eps;
	    	Eigen::Vector3d AAUpdatedMinus = sm::kinematics::qlog(updatedQuat);

	    	Eigen::Vector3d diffAA =  AAUpdatedPlus - AAUpdatedMinus;
	    	//std::cout << "Diff AA: " << std::endl << diffAA << std::endl;
	    	Jest(0,c) = diffAA(0) / dx;
	    	Jest(1,c) = diffAA(1) / dx;
	    	Jest(2,c) = diffAA(2) / dx;
	    }

	    Eigen::MatrixXd J = sm::kinematics::quatLogJacobian(initialValue);

		std::cout << "Jest" << std::endl << Jest << std::endl;
		std::cout << "J" << std::endl << J << std::endl;

		sm::eigen::assertNear(J, Jest, 1e-6, SM_SOURCE_FILE_POS, "Testing the quat log Jacobian");

	}
	catch(std::exception const & e)
    {
        FAIL() << e.what();
    }

}
