/**
 * @file random.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief A wrapper for random numbers in c++.
 * @version 0.1
 * @date 2023-01-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <iostream>
#include <random>

class Random{

public:
    Random(){};

    int randint(int range_from, int range_to) {
        std::mt19937                        generator(_rand_dev());
        std::uniform_int_distribution<>    distr(range_from, range_to);
        return distr(generator);
    }

    double rand_double(double range_from, double range_to) {
        std::mt19937                        generator(_rand_dev());
        std::uniform_real_distribution<>    distr(range_from, range_to);
        return distr(generator);
    }

private:
    std::random_device  _rand_dev;

};
