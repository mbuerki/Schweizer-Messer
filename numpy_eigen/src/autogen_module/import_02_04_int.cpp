// This file automatically generated by create_export_module.py
#include <NumpyEigenConverter.hpp>


void import_02_04_int()
{
	// Without this import, the converter will segfault
	import_array();
	NumpyEigenConverter<Eigen::Matrix< int, 2, 4 > >::register_converter();
}

