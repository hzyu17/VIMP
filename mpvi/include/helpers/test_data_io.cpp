#include "data_io.h"

using namespace std;
using namespace Eigen;

int main(){
    DataIO data_io{};

    string filename("data.csv");

    MatrixXd data;
    data = MatrixXd::Random(10, 10);

    data_io.saveData(filename, data);
     
    return 0;
}