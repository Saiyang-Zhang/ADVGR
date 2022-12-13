#pragma once

namespace Tmpl8 {

    const double BRIGHTNESS = 2.0f * 3.1415926f;

    class BoundingBox
    {
        BoundingBox() = default;
        BoundingBox(Cube& s) {

        }
        BoundingBox(Quad& s) {

        }
        BoundingBox(Sphere& s) {

        }
        BoundingBox(Triangle& s) {

        }

        float3 minP, maxP;
    };
   
    class TriangleMesh
    {
    public:
        TriangleMesh() = default;
        TriangleMesh(const string& file_name) {
            read_from_obj_file(file_name);
        }

        void read_from_obj_file(const string& file_name) {
            // Open the file
            ifstream file(file_name);
            if (!file.is_open()) {
                cerr << "Error: Unable to open file " << file_name << endl;
                return;
            }

            // Read the file line by line
            string line;
            while (getline(file, line)) {
                stringstream ss(line);

                // Split the line into tokens
                string token;
                ss >> token;

                // Parse the tokens
                if (token == "v") {
                    // Parse vertex coordinates
                    float3 vertex;
                    ss >> vertex.x >> vertex.y >> vertex.z;
                    vertices.push_back(vertex);
                }
                else if (token == "f") {
                    // Parse triangle vertices
                    Triangle triangle;
                    ss >> triangle.v0 >> triangle.v1 >> triangle.v2;

                    // OBJ indices are 1-based, so we need to subtract 1 to get 0-based indices
                    triangle.v0--;
                    triangle.v1--;
                    triangle.v2--;
                    triangles.push_back(triangle);
                }
            }
        }        
        vector<float3> vertices;
        vector<Triangle> triangles;
    };

}
