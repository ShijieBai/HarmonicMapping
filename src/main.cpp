#include "HarmonicMapping.h"

int main() {

	Mapping::HarmonicMapping mapping;

	mapping.read_obj("D:/A-work/Harmonic_mapping/HarmonicMapping/IO/patch2.obj");

	mapping.find_corner_and_bnd_nodes();

	mapping.map_boundary();

	mapping.map_quad();

	mapping.write_uv_obj("D:/A-work/Harmonic_mapping/HarmonicMapping/IO/uv.obj");

	return 0;
}