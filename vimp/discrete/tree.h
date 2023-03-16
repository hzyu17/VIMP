/**
 * @file tree.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Tree structure.
 * @version 0.1
 * @date 2023-02-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */

namespace vimp{

template <typename NodeType>
class TreeNode{

public: 
    TreeNode(){};


};

template <typename DataType>
class Tree{
using Node = TreeNode<DataType>;
public:
    Tree(){};
    Tree(Node* root){ _root = root };

    Node* _root;

};

}