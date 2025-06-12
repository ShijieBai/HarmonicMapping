#include "HarmonicMapping.h"
#include <iostream>
#include <fstream>

namespace Mapping {


	void Mesh::read_obj(const std::string& obj_file){
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





}