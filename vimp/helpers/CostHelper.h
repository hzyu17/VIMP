/**
 * @file CostHelper.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief helpers class to record the costs.
 * @version 0.1
 * @date 2023-04-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <matplot/matplot.h>

using namespace std;
using namespace matplot;

namespace vimp{
class CostHelper{
public:
    CostHelper(){}
    ~CostHelper(){}
    CostHelper(int max_iter):_max_iter(max_iter), 
                             _control_energy_costs(max_iter),
                             _collision_costs(max_iter),
                             _total_costs(max_iter)
    {
        for (int i=0; i< _max_iter; i++){
            _control_energy_costs[i] = 0.0;
            _collision_costs[i] = 0.0;
            _total_costs[i] = 0.0;
        }
    }

    void add_cost(const int & i, const double & col_cost, const double & control_cost){
        _control_energy_costs[i] = control_cost;
        _collision_costs[i] = col_cost;
        _total_costs[i] = control_cost + col_cost;
    }

    void save_costs(const string& filename){
        int cost_size = _control_energy_costs.size();
        MatrixXd total_costs(3, cost_size);
        for (int i=0; i<cost_size; i++){
            total_costs(0, i) = _control_energy_costs[i];
            total_costs(1, i) = _collision_costs[i];
            total_costs(2, i) = _total_costs[i];
        }
        _mio.saveData(filename, total_costs);
    }

    void plot_costs(){
        
        figure();
        std::vector<double> x = linspace(0, _max_iter, _max_iter);
        plot(x, _control_energy_costs, "r");
        hold(on);
        plot(x, _collision_costs, "b");
        plot(x, _total_costs, "k");
        show();
    }

private:
    int _max_iter;
    vector<double> _control_energy_costs;
    vector<double> _collision_costs;
    vector<double> _total_costs;
    MatrixIO _mio;
};
}