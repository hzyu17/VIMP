/**
 * @file run_experiments.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Run experiment according to the input argument.
 * @version 0.1
 * @date 2023-04-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <argparse/argparse.hpp>

int main(int argc, char *argv[]) {
  argparse::ArgumentParser program("program_name");

  program.add_argument("square")
    .help("display the square of a given integer")
    .scan<'i', int>();

  try {
    program.parse_args(argc, argv);
  }
  catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  auto input = program.get<int>("square");
  std::cout << (input * input) << std::endl;

  return 0;
}