#include "HarmonicMapping.h"

namespace Mapping {

	void Mesh::read_obj(const std::string& obj_file) {
		std::ifstream file(obj_file);
		if (!file.is_open()) {
			std::cerr << "Error: Could not open file " << obj_file << std::endl;
		}

		std::string line;
		while (getline(file, line)) {
			// Skip comments and empty lines
			if (line.empty() || line[0] == '#') continue;

			std::istringstream iss(line);
			std::string type;
			iss >> type;
			if (type == "v") {
				Vector3d vertex;
				iss >> vertex[0] >> vertex[1] >> vertex[2];
				nodes_.push_back(vertex);
			}
			else if (type == "f") {
				std::vector<int> faceIndices;
				std::string faceData;
				while (iss >> faceData) {
					// Handle formats like "v", "v/vt", or "v/vt/vn"
					size_t pos = faceData.find('/');
					if (pos != std::string::npos) {
						faceData = faceData.substr(0, pos);
					}
					try {
						int index = stoi(faceData);
						faceIndices.push_back(index - 1);
					}
					catch (...) {
						std::cerr << "Warning: Invalid face data: " << faceData << std::endl;
					}
				}
				if (faceIndices.size() >= 3) {
					VectorXi face(faceIndices.size());
					for (int i = 0; i < faceIndices.size(); ++i) {
						face[i] = faceIndices[i];
					}
					elements_.push_back(face);
				}
			}
		}
		file.close();
	}

	void HarmonicMapping::read_obj(const std::string& obj_file) {
		mesh_.read_obj(obj_file);
	}

	void HarmonicMapping::build_edge_hash() {
		for (const auto& tri : mesh_.elements_) {
			for (int i = 0; i < 3; ++i) {
				int v0 = tri[i];
				int v1 = tri[(i + 1) % 3];
				auto edge = (v0 < v1) ? std::make_pair(v0, v1) : std::make_pair(v1, v0);
				edge_counts_[edge]++;
			}
		}
	}

	void HarmonicMapping::find_boundary_node() {
		if (edge_counts_.empty()) {
			build_edge_hash();
		}
		for (const auto& entry : edge_counts_) {
			if (entry.second == 1) {
				bnd_nodes_.insert(entry.first.first);
				bnd_nodes_.insert(entry.first.second);
			}
		}
	}

	void HarmonicMapping::find_corner() {
		if (edge_counts_.empty()) {
			build_edge_hash();
		}
		std::unordered_map<int, std::vector<int>> boundaryAdj;

		for (const auto& entry : edge_counts_) {
			if (entry.second == 1) {  // Boundary edge
				int v0 = entry.first.first;
				int v1 = entry.first.second;
				boundaryAdj[v0].push_back(v1);
				boundaryAdj[v1].push_back(v0);
			}
		}
		
		for (int v : bnd_nodes_) {
			const auto& neighbors = boundaryAdj[v];
			if (neighbors.size() != 2) continue;  // Only consider vertices with 2 boundary neighbors

			int vPrev = neighbors[0];
			int vNext = neighbors[1];

			Vector3d vecPrev = mesh_.nodes_[vPrev] - mesh_.nodes_[v];
			Vector3d vecNext = mesh_.nodes_[vNext] - mesh_.nodes_[v];

			// Compute angle (in radians, then convert to degrees)
			double angle = acos(vecPrev.normalized().dot(vecNext.normalized())) * (180.0 / M_PI);

			if (angle < 120) {
				corners_.push_back(v);
			}
		}		
	}
}