/**
 * @file line_searcher.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief A template code for line search backtracking algorithm.
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

namespace vimp{

template <typename Optimizer>
class LineSearch{

public:
    LineSearch(){}
    virtual ~LineSearch(){}

    LineSearch(double beta, const Optimizer& opt): _beta(beta),_p_opt(std::make_shared<Optimizer>(opt)){}

    void optimize(){

    }

    double line_search(double step_size, int max_iter){
        int iter = 0;
        while (iter < max_iter){
            
            iter ++;
        }
    }

private:
    double _beta;
    std::shared_ptr<Optimizer> _p_opt;
}

}// namespace vimp