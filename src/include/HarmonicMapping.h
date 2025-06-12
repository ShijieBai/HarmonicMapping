#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <math.h>
#include <iostream>

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using VectorXi = Eigen::VectorXi;

namespace Mapping {

    struct Mesh {
        std::vector<Vector3d> nodes_;
        std::vector<VectorXi> elements_;
        void read_obj(const std::string &obj_file);
        void write_obj(const std::string &obj_file);
    };

    class HarmonicMapping {
        Mesh mesh_;
        std::vector<std::vector<int>> bnd_nodes_{};
        std::vector<int> corners_{};
        std::vector<std::vector<Vector2d>> bnd_uv_{};

    public:
        HarmonicMapping() = default;

        HarmonicMapping(const HarmonicMapping &) = delete;

        void read_obj(const std::string &obj_file);

        void write_obj(const std::string &obj_file);

        void find_corner_and_bnd_nodes(const int threshold = 120);

        // map boundary to uv space
        void map_boundary();

        void map_quad();
    };

} // namespace Mapping