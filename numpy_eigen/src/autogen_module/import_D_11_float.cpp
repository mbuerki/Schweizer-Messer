// This file automatically generated by create_export_module.py
#include <NumpyEigenConverter.hpp>


void import_D_11_float()
{
	// Without this import, the converter will segfault
	import_array();
	NumpyEigenConverter<Eigen::Matrix< float, Eigen::Dynamic, 11 > >::register_converter();
}

