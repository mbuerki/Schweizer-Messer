// This file automatically generated by create_export_module.py
#include <NumpyEigenConverter.hpp>


void import_11_11_float()
{
	// Without this import, the converter will segfault
	import_array();
	NumpyEigenConverter<Eigen::Matrix< float, 11, 11 > >::register_converter();
}

