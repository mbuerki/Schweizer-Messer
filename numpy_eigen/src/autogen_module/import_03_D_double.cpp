// This file automatically generated by create_export_module.py
#include <NumpyEigenConverter.hpp>


void import_03_D_double()
{
	// Without this import, the converter will segfault
	import_array();
	NumpyEigenConverter<Eigen::Matrix< double, 3, Eigen::Dynamic > >::register_converter();
}

