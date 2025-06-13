#include "HarmonicMapping.h"
#include <stack>
#include <tbb/parallel_for.h>

namespace Mapping {

    Node::Node(Vector3d &coord) : coord_(coord) {}

    bool Edge::operator<(const Edge &other) const {
        return to_ < other.to_;
    }

    bool Edge::operator==(const Edge &other) const {
        return to_ == other.to_;
    }

    Edge::Edge(const int to) : to_(to) {}

    void Mesh::read_obj(const std::string &obj_file) {
        std::ifstream file(obj_file);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << obj_file << std::endl;
        }

        std::string line;
        while (getline(file, line)) {
            // Skip comments and empty lines
            if (line.empty() || line[0] == '#')
                continue;

            std::istringstream iss(line);
            std::string type;
            iss >> type;
            if (type == "v") {
                Vector3d vertex;
                iss >> vertex[0] >> vertex[1] >> vertex[2];
                nodes_.push_back(vertex);
            } else if (type == "f") {
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
                    } catch (...) {
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

    void Mesh::write_obj(const std::string &obj_file) {
        std::ofstream fileOBJ(obj_file.c_str(), std::ios::out);

        fileOBJ << "# Object name" << std::endl;
        fileOBJ << "o " << obj_file << std::endl;
        fileOBJ << std::endl;

        fileOBJ << "# Begin list of vertices" << std::endl;
        int nnodes = nodes_.size();
        for (int i = 0; i < nnodes; i++) {
            fileOBJ << "v ";
            for (auto &p : nodes_[i].coord_) {
                fileOBJ << p << " ";
            }
            fileOBJ << std::endl;
        }

        fileOBJ << "# End list of vertices" << std::endl;
        fileOBJ << std::endl;

        fileOBJ << "# Begin list of faces" << std::endl;
        int nele = elements_.size();
        for (int i = 0; i < nele; i++) {
            fileOBJ << "f ";
            for (auto &id : elements_[i]) {
                fileOBJ << id + 1 << " ";
            }
            fileOBJ << std::endl;
        }
        fileOBJ << "# End list of faces" << std::endl;
        fileOBJ << std::endl;
        fileOBJ.close();
    }

    void Mesh::write_uv_obj(const std::string &obj_file) {
        std::ofstream fileOBJ(obj_file.c_str(), std::ios::out);

        fileOBJ << "# Object name" << std::endl;
        fileOBJ << "o " << obj_file << std::endl;
        fileOBJ << std::endl;

        fileOBJ << "# Begin list of vertices" << std::endl;
        int nnodes = nodes_.size();
        for (int i = 0; i < nnodes; i++) {
            fileOBJ << "v ";
            for (auto &p : nodes_[i].coord_) {
                fileOBJ << p << " ";
            }
            fileOBJ << std::endl;
        }

        fileOBJ << "# End list of vertices" << std::endl;
        fileOBJ << std::endl;

        fileOBJ << "# Begin list of vertices uv" << std::endl;
        for (const auto &node : nodes_) {
            fileOBJ << "vt " << node.uv_[0] << " " << node.uv_[1] << "\n";
        }
        fileOBJ << "# End list of vertices uv" << std::endl;
        fileOBJ << std::endl;

        fileOBJ << "# Begin list of faces" << std::endl;
        int nele = elements_.size();
        for (int i = 0; i < nele; i++) {
            fileOBJ << "f ";
            for (auto &id : elements_[i]) {
                fileOBJ << id + 1 << "/" << id + 1 << " ";
            }
            fileOBJ << std::endl;
        }
        fileOBJ << "# End list of faces" << std::endl;
        fileOBJ << std::endl;
        fileOBJ.close();
    }

    void HarmonicMapping::read_obj(const std::string &obj_file) {
        mesh_.read_obj(obj_file);
    }

    void HarmonicMapping::write_obj(const std::string &obj_file) {
        mesh_.write_obj(obj_file);
    }

    void HarmonicMapping::write_uv_obj(const std::string &obj_file) {
        mesh_.write_uv_obj(obj_file);
    }

    void HarmonicMapping::find_corner_and_bnd_nodes(const int threshold) {
        struct EdgeHash {
            size_t operator()(const std::pair<int, int> &edge) const {
                return std::hash<int>()(edge.first) ^ std::hash<int>()(edge.second);
            }
        };

        std::unordered_map<std::pair<int, int>, int, EdgeHash> edge_counts;
        for (const auto &tri : mesh_.elements_) {
            for (int i = 0; i < 3; ++i) {
                int v0 = tri[i];
                int v1 = tri[(i + 1) % 3];
                auto edge = (v0 < v1) ? std::make_pair(v0, v1) : std::make_pair(v1, v0);
                edge_counts[edge]++;
            }
        }

        std::unordered_set<int> bnd_nodes;
        for (const auto &entry : edge_counts) {
            if (entry.second == 1) {
                bnd_nodes.insert(entry.first.first);
                bnd_nodes.insert(entry.first.second);
                mesh_.nodes_[entry.first.first].is_boundary_ = true;
                mesh_.nodes_[entry.first.second].is_boundary_ = true;
            }
        }

        std::unordered_map<int, std::vector<int>>
            boundaryAdj; // get the neighbors bnd nodes of every bnd nodes

        for (const auto &entry : edge_counts) {
            if (entry.second == 1) { // Boundary edge
                int v0 = entry.first.first;
                int v1 = entry.first.second;
                boundaryAdj[v0].push_back(v1);
                boundaryAdj[v1].push_back(v0);
            }
        }

        for (int v : bnd_nodes) {
            const auto &neighbors = boundaryAdj[v];
            if (neighbors.size() != 2)
                continue; // Only consider vertices with 2 boundary neighbors

            int vPrev = neighbors[0];
            int vNext = neighbors[1];

            Vector3d vecPrev = mesh_.nodes_[vPrev].coord_ - mesh_.nodes_[v].coord_;
            Vector3d vecNext = mesh_.nodes_[vNext].coord_ - mesh_.nodes_[v].coord_;

            double angle = acos(vecPrev.normalized().dot(vecNext.normalized())) * (180.0 / M_PI);
            if (angle < threshold) {
                corners_.emplace_back(v);
            }
        }
        // arrange the bnd nodes in order
        int n_corner = corners_.size();
        std::stack<int> stack;
        std::unordered_set<int> visited;
        auto min_it =
            std::min_element(corners_.begin(), corners_.end(), [this](const int &a, const int &b) {
                return mesh_.nodes_[a].coord_.squaredNorm() < mesh_.nodes_[b].coord_.squaredNorm();
            });
        stack.push(*min_it); // 以角点最小点为起始点
        std::unordered_set<int> corner_set;
        corner_set.insert(corners_.begin(), corners_.end());
        corners_.clear();
        corners_.reserve(n_corner);
        bnd_nodes_.resize(n_corner);
        int ith{};
        while (!stack.empty()) {
            auto &bnd_loop = bnd_nodes_[ith];
            int v = stack.top();
            stack.pop();
            if (visited.count(v))
                continue;
            visited.insert(v);
            if (corner_set.count(v)) {
                if (!corners_.empty()) {
                    ith++;
                }
                corners_.emplace_back(v);
            } else {
                bnd_loop.emplace_back(v);
            }
            const auto &neighbors = boundaryAdj[v];
            for (int i : neighbors) {
                if (!visited.count(i)) {
                    stack.push(i);
                }
            }
        }
    }

    void HarmonicMapping::map_boundary() {
        mesh_.nodes_[corners_[0]].uv_ = Vector2d{0, 0};
        mesh_.nodes_[corners_[1]].uv_ = Vector2d{0, 1};
        mesh_.nodes_[corners_[2]].uv_ = Vector2d{1, 1};
        mesh_.nodes_[corners_[3]].uv_ = Vector2d{1, 0};

        int n_path = bnd_nodes_.size();
        for (int k = 0; k < n_path; k++) {
            auto &path = bnd_nodes_[k];
            int pre_index{corners_[k]};
            int end_index{};
            if (k == n_path - 1) {
                end_index = corners_[0];
            } else {
                end_index = corners_[k + 1];
            }
            int nn = path.size();
            std::vector<double> lengths;
            lengths.resize(nn);
            for (int i = 0; i < nn; i++) {
                if (i == 0) {
                    lengths[i] = (mesh_.nodes_[path[i]].coord_ - mesh_.nodes_[pre_index].coord_).norm();

                } else {
                    lengths[i] = lengths[i - 1] +
                                 (mesh_.nodes_[path[i]].coord_ - mesh_.nodes_[pre_index].coord_).norm();
                }
                pre_index = path[i];
            }

            lengths.emplace_back(lengths.back() +
                                 (mesh_.nodes_[end_index].coord_ - mesh_.nodes_[pre_index].coord_).norm());

            auto &path_length = lengths.back();
            for (int i = 0; i < nn; i++) {
                switch (k) {
                case 0: {
                    mesh_.nodes_[path[i]].uv_ = {0, lengths[i] / path_length};
                    break;
                }
                case 1: {
                    mesh_.nodes_[path[i]].uv_ = {lengths[i] / path_length, 1};
                    break;
                }
                case 2: {
                    mesh_.nodes_[path[i]].uv_ = {1, 1.0 - lengths[i] / path_length};
                    break;
                }
                case 3: {
                    mesh_.nodes_[path[i]].uv_ = {1.0 - lengths[i] / path_length, 0};
                    break;
                }
                default: {
                    std::cerr << "Error: index out of scope " << std::endl;
                }
                }
            }
        }
    }

    void HarmonicMapping::map_quad() {
        // build the nodes-edge and edge-ele relationship
        std::map<int, std::vector<Edge>> node_edges_map;
        std::map<std::pair<int, int>, std::set<int>> edge_eles_map;
        int nele = mesh_.elements_.size();
        for (int k = 0; k < nele; k++) {
            auto &ele = mesh_.elements_[k];
            for (int i = 0; i < 3; ++i) {
                int v0 = ele[i];
                int v1 = ele[(i + 1) % 3];
                auto edge = (v0 < v1) ? std::make_pair(v0, v1) : std::make_pair(v1, v0);
                node_edges_map[v0].emplace_back(v1);
                node_edges_map[v1].emplace_back(v0);
                edge_eles_map[edge].insert(k);
            }
        }

        // remove duplicates
        for (auto &[index, edges] : node_edges_map) {
            std::sort(edges.begin(), edges.end());
            auto last = std::unique(edges.begin(), edges.end());
            edges.erase(last, edges.end());
        }

        // cal the weight for every internal node neighbor edges
        int nnodes = mesh_.nodes_.size();
        for (int i = 0; i < nnodes; i++) {
            auto &node = mesh_.nodes_[i];
            if (node.is_boundary_) {
                continue;
            }
            for (auto &edge : node_edges_map[i]) {
                auto edge_p = (i < edge.to_) ? std::make_pair(i, edge.to_) : std::make_pair(edge.to_, i);
                const auto &eles = edge_eles_map[edge_p];
                cal_edge_weight(i, edge, eles);
            }
        }

        // iteration for internal nodes in uv space
        int iter{};
        int max_iter{20};
        while (iter < max_iter) {
            for (int i = 0; i < nnodes; i++) { // traversal internal nodes
                auto &node = mesh_.nodes_[i];
                if (node.is_boundary_) {
                    continue;
                }
                Vector2d new_uv{0, 0};
                double sum_weight{};
                for (auto &edge : node_edges_map[i]) {
                    sum_weight += edge.weight_;
                    new_uv += edge.weight_ * mesh_.nodes_[edge.to_].uv_;
                }
                new_uv = new_uv.array() / sum_weight;
                // ensure the nv in the range 0-1
                new_uv[0] = std::max(0.0, new_uv[0]);
                new_uv[1] = std::max(0.0, new_uv[1]);
                new_uv[0] = std::min(1.0, new_uv[0]);
                new_uv[1] = std::min(1.0, new_uv[1]);
                node.uv_ = new_uv;
            }
            iter++;
        }
    }

    void HarmonicMapping::cal_edge_weight(const int A, Edge &edge, const std::set<int> &eles) {
        auto &nodes = mesh_.nodes_;
        std::vector<double> thetas;
        for (auto index : eles) {
            auto &ele = mesh_.elements_[index];
            int B{}, C{};
            if (ele[0] == A) {
                B = ele[1];
                C = ele[2];
            } else if (ele[1] == A) {
                B = ele[0];
                C = ele[2];
            } else if (ele[2] == A) {
                B = ele[0];
                C = ele[1];
            } else {
                throw("the index not int the triangle");
            }
            double a = (nodes[B].coord_ - nodes[C].coord_).norm();
            double b = (nodes[A].coord_ - nodes[C].coord_).norm();
            double c = (nodes[A].coord_ - nodes[B].coord_).norm();

            double theta = acos((b * b + c * c - a * a) / (2 * b * c)) / 2.0;
            thetas.emplace_back(theta);
        }
        if (thetas.size() != 2) {
            std::cerr << "cal weight for internal edge!" << std::endl;
        }
        double length = (nodes[A].coord_ - nodes[edge.to_].coord_).norm();
        edge.weight_ = (tan(thetas[0]) + tan(thetas[1])) / length;
    }

} // namespace Mapping