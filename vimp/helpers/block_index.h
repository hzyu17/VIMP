/**
 * @file block_index.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief A class describing block matrix related notions.
 * @version 0.1
 * @date 2023-01-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

namespace vimp{
class BlockIndex{
public:
    BlockIndex(int start_row, int end_row, int start_col, int end_col):
    _start_row(start_row),
    _end_row(end_row){
        _nrow = end_row - start_row + 1;
        _ncol = end_row - start_row + 1;
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

    int ncols(){
        return _ncol;
    }

private:
    int _start_row;
    int _end_row;
    int _nrow;
};
}
