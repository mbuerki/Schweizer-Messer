#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 11, 7> test_double_11_07(const Eigen::Matrix<double, 11, 7> & M)
{
	return M;
}
void export_double_11_07()
{
	boost::python::def("test_double_11_07",test_double_11_07);
}

