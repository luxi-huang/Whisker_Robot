#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <iostream>
using namespace std;

vector<vector<float>> read_csv(string filename) {
    ifstream classFile(filename);

    if(!classFile.is_open()) throw runtime_error("Could not open file");
    vector<vector<float>> classData;
    string line;
    string field;
    int count = 0;
    while(getline(classFile,line)) {
        istringstream s(line);
        classData.push_back({});
        while (getline(s,field, ',')) {
            float val;
            val = stoi(field);
            classData.back().push_back(val);
        }
    }
    return classData;
}

int main() {
    vector<vector<float>> collision = read_csv("/home/luxi/github_projects/Whisker_Robot/data/output/test/kinematics/c/RD1.csv");
    vector<vector<float>> x_val = read_csv("/home/luxi/github_projects/Whisker_Robot/data/output/test/kinematics/x/RD1.csv");
    vector<vector<float>> y_val = read_csv("/home/luxi/github_projects/Whisker_Robot/data/output/test/kinematics/x/RD1.csv");
    vector<vector<float>> z_val = read_csv("/home/luxi/github_projects/Whisker_Robot/data/output/test/kinematics/x/RD1.csv");

    int row = collision.size();
    int count = 0;

    vector<vector<float>> points;

    for (int i=0; i < row; i++){
        points.push_back({});
        for (int j = 0; j < 20; j++) {
            if (collision[i][j] == 1) {   
                    float x_point = (x_val[i][j] + x_val[i][j+1]) / 2;
                    float y_point = (y_val[i][j] + y_val[i][j+1]) / 2;
                    float z_point = (z_val[i][j] + z_val[i][j+1]) / 2;
            } 
        }
        
    }

    cout << count;  
}