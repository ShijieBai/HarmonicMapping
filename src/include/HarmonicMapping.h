#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <math.h>
#include <iostream>

using Vector3d = Eigen::Vector3d;
using VectorXi = Eigen::VectorXi;

namespace Mapping {

	struct EdgeHash {
		size_t operator()(const std::pair<int, int>& edge) const {
			return std::hash<int>()(edge.first) ^ std::hash<int>()(edge.second);
		}
	};

	class Mesh {
	public:
		std::vector<Vector3d> nodes_;
		std::vector<VectorXi> elements_;

		void read_obj(const std::string& obj_file);
	};

	class HarmonicMapping {
		Mesh mesh_;
		std::unordered_map<std::pair<int, int>, int, EdgeHash> edge_counts_;
		std::unordered_set<int> bnd_nodes_{};
		std::vector<int> corners_{};
	public:

		void read_obj(const std::string& obj_file);

		void build_edge_hash();

		void find_corner();

		void find_boundary_node();


	};











}