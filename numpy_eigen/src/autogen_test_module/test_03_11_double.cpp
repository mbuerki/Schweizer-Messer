#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 3, 11> test_double_03_11(const Eigen::Matrix<double, 3, 11> & M)
{
	return M;
}
void export_double_03_11()
{
	boost::python::def("test_double_03_11",test_double_03_11);
}

