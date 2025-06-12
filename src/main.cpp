#include "HarmonicMapping.h"

int main() {
	Mapping::HarmonicMapping mapping;

	mapping.read_obj("D:/A-work/Harmonic_mapping/Mapping/HarmonicMapping/IO/patch2.obj");

	mapping.build_edge_hash();
	
	mapping.find_boundary_node();

	mapping.find_corner();

	return 0;
}