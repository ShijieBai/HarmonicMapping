#pragma once
#include <Eigen/Dense>

using Vector3d = Eigen::Vector3d;
using VectorXi = Eigen::VectorXi;

namespace Mapping {

	class Node {
		Vector3d coord_;


	};

	class Mesh {
	public:
		std::vector<Vector3d> nodes_;
		std::vector<VectorXi> elements_;

		void read_obj(const std::string &obj_file);
	};











}