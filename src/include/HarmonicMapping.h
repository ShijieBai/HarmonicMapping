#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <math.h>
#include <iostream>

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using VectorXi = Eigen::VectorXi;

namespace Mapping {

    struct Node {
        Vector3d coord_{};
        Vector2d uv_{0.5, 0.5};
        bool is_boundary_{};
        Node(Vector3d &coord);
    };

    struct Edge {
        int to_{};
        double weight_{};
        Edge(const int to);
        bool operator<(const Edge &other) const;
        bool operator==(const Edge &other) const;
    };

    struct Mesh {
        std::vector<Node> nodes_;
        std::vector<VectorXi> elements_;
        void read_obj(const std::string &obj_file);
        void write_obj(const std::string &obj_file);
        void write_uv_obj(const std::string &obj_file);
    };

    class HarmonicMapping {
        Mesh mesh_;
        std::vector<std::vector<int>> bnd_nodes_{}; // interval between corners
        std::vector<int> corners_{};  
    public:
        HarmonicMapping() = default;

        HarmonicMapping(const HarmonicMapping &) = delete;

        void read_obj(const std::string &obj_file);

        void write_obj(const std::string &obj_file);

        void write_uv_obj(const std::string &obj_file);

        void find_corner_and_bnd_nodes(const int threshold = 120);

        // map boundary to uv space
        void map_boundary();

        void map_quad();

        void cal_weight(const int p, Edge &edge, const std::set<int> &eles);
    };

} // namespace Mapping