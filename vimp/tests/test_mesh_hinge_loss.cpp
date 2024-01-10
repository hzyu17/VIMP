/**
 * @file test_mesh_hinge_loss.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test plotting a hinge loss mesh.
 * @version 0.1
 * @date 2022-08-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#define STRING(x) #x
#define XSTRING(x) STRING(x)

#include "helpers/hinge2Dhelper.h"
#include <gtest/gtest.h>


TEST(Test_meshgrid, hinge_mesh){
    gvi::MatrixIO m_io;
    std::string source_root{XSTRING(SOURCE_ROOT)};
    MatrixXd grid_X = m_io.load_csv(source_root+"/data/sdf_grid_x.csv");
    MatrixXd grid_Y = m_io.load_csv(source_root+"/data/sdf_grid_y.csv");

    MatrixXd field = m_io.load_csv(source_root+"/maps/2dpR/map0/field_multiobs_map0.csv");

    Vector2d origin(-20, -10);
    double cell_size = 0.1;
    double eps = 2.0;

    gpmp2::PlanarSDF sdf = gpmp2::PlanarSDF(origin, cell_size, field);  
    
    MatrixXd grid_hingeloss = mesh_hingeloss(grid_X, grid_Y, sdf, eps);

    m_io.saveData(source_root+"/vimp/data/mesh_hingeloss.csv", grid_hingeloss);

    ASSERT_EQ(0, 0);

}