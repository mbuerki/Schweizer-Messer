// This file automatically generated by create_export_module.py
#include <NumpyEigenConverter.hpp>


void import_06_07_double()
{
	// Without this import, the converter will segfault
	import_array();
	NumpyEigenConverter<Eigen::Matrix< double, 6, 7 > >::register_converter();
}

