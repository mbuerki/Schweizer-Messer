// This file automatically generated by create_export_module.py
#include <NumpyEigenConverter.hpp>


void import_07_03_int()
{
	// Without this import, the converter will segfault
	import_array();
	NumpyEigenConverter<Eigen::Matrix< int, 7, 3 > >::register_converter();
}

