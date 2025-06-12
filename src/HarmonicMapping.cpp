#include "HarmonicMapping.h"
#include <stack>
#include <tbb/parallel_for.h>

namespace Mapping {

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
            for (auto &p : nodes_[i]) {
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

    void HarmonicMapping::read_obj(const std::string &obj_file) {
        mesh_.read_obj(obj_file);
    }

    void HarmonicMapping::write_obj(const std::string &obj_file) {
        mesh_.write_obj(obj_file);
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

            Vector3d vecPrev = mesh_.nodes_[vPrev] - mesh_.nodes_[v];
            Vector3d vecNext = mesh_.nodes_[vNext] - mesh_.nodes_[v];

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
                return mesh_.nodes_[a].squaredNorm() < mesh_.nodes_[b].squaredNorm();
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
        std::vector<Vector2d> corner_uv;
        corner_uv.reserve(corners_.size());
        corner_uv.emplace_back(Vector2d{0, 0});
        corner_uv.emplace_back(Vector2d{0, 1});
        corner_uv.emplace_back(Vector2d{1, 1});
        corner_uv.emplace_back(Vector2d{1, 0});

        int n_path = bnd_nodes_.size();
        bnd_uv_.resize(n_path);

        for (int k = 0; k < n_path; k++) {
            auto &path = bnd_nodes_[k];
            int pre_index{corners_[k]};
            int nn = path.size();
            std::vector<double> lengths;
            lengths.resize(nn);
            for (int i = 0; i < nn; i++) {
                if (i == 0) {
                    lengths[i] = (mesh_.nodes_[path[i]] - mesh_.nodes_[pre_index]).norm();
                } else {
                    lengths[i] = lengths[i - 1] + (mesh_.nodes_[path[i]] - mesh_.nodes_[pre_index]).norm();
                }
                pre_index = path[i];
            }

            auto &path_uv = bnd_uv_[k];
            path_uv.resize(nn);
            auto &path_length = lengths.back();
            for (int i = 0; i < nn; i++) {
                switch (k) {
                case 0: {
                    path_uv[i] = {0, lengths[i] / path_length};
                    break;
                }
                case 1: {
                    path_uv[i] = {lengths[i] / path_length, 1};
                    break;
                }
                case 2: {
                    path_uv[i] = {1, lengths[i] / path_length};
                    break;
                }
                case 3: {
                    path_uv[i] = {lengths[i] / path_length, 0};
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






    }

} // namespace Mapping