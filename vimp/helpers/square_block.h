/**
 * @file block_index.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief A class describing \b square block matrix related notions.
 * @version 0.1
 * @date 2023-01-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

namespace vimp{
class SquareBlock{
public:
    SquareBlock(){}
    SquareBlock(int start_row, int end_row):
    _start_row(start_row),
    _end_row(end_row){
        _nrow = end_row - start_row + 1;
    }

    int s_r(){
        return _start_row;
    }

    int e_r(){
        return _end_row;
    }

    int nrows(){
        return _nrow;
    }

private:
    int _start_row;
    int _end_row;
    int _nrow;
};
}
