/*
 * GraphBLAS Template Library, Version 2.0
 *
 * Copyright 2018 Carnegie Mellon University, Battelle Memorial Institute, and
 * Authors. All Rights Reserved.
 *
 * THIS MATERIAL WAS PREPARED AS AN ACCOUNT OF WORK SPONSORED BY AN AGENCY OF
 * THE UNITED STATES GOVERNMENT.  NEITHER THE UNITED STATES GOVERNMENT NOR THE
 * UNITED STATES DEPARTMENT OF ENERGY, NOR THE UNITED STATES DEPARTMENT OF
 * DEFENSE, NOR CARNEGIE MELLON UNIVERSITY, NOR BATTELLE, NOR ANY OF THEIR
 * EMPLOYEES, NOR ANY JURISDICTION OR ORGANIZATION THAT HAS COOPERATED IN THE
 * DEVELOPMENT OF THESE MATERIALS, MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR
 * ASSUMES ANY LEGAL LIABILITY OR RESPONSIBILITY FOR THE ACCURACY, COMPLETENESS,
 * OR USEFULNESS OR ANY INFORMATION, APPARATUS, PRODUCT, SOFTWARE, OR PROCESS
 * DISCLOSED, OR REPRESENTS THAT ITS USE WOULD NOT INFRINGE PRIVATELY OWNED
 * RIGHTS..
 *
 * Released under a BSD (SEI)-style license, please see license.txt or contact
 * permission@sei.cmu.edu for full terms.
 *
 * This release is an update of:
 *
 * 1. GraphBLAS Template Library (GBTL)
 * (https://github.com/cmu-sei/gbtl/blob/1.0.0/LICENSE) Copyright 2015 Carnegie
 * Mellon University and The Trustees of Indiana. DM17-0037, DM-0002659
 *
 * DM18-0559
 */

#include <iostream>

#include <graphblas/graphblas.hpp>

using namespace grb;

#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE lil_sparse_matrix_test_suite

#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

//****************************************************************************
// LIL basic constructor
BOOST_AUTO_TEST_CASE(lil_test_construction_basic)
{
    IndexType M = 7;
    IndexType N = 4;
    backend::LilSparseMatrix<double> m1(M, N);

    IndexType num_rows(m1.nrows());
    IndexType num_cols(m1.ncols());

    BOOST_CHECK_EQUAL(num_rows, M);
    BOOST_CHECK_EQUAL(num_cols, N);
}


//****************************************************************************
// LIL constructor from dense matrix
BOOST_AUTO_TEST_CASE(lil_test_construction_dense)
{
    std::vector<std::vector<double>> mat = {{6, 0, 0, 4},
                                            {7, 0, 0, 0},
                                            {0, 0, 9, 4},
                                            {2, 5, 0, 3},
                                            {2, 0, 0, 1},
                                            {0, 0, 0, 0},
                                            {0, 1, 0, 2}};

    backend::LilSparseMatrix<double> m1(mat);

    IndexType M = mat.size();
    IndexType N = mat[0].size();
    IndexType num_rows(m1.nrows());
    IndexType num_cols(m1.ncols());

    BOOST_CHECK_EQUAL(num_rows, M);
    BOOST_CHECK_EQUAL(num_cols, N);
    for (IndexType i = 0; i < M; i++)
    {
        for (IndexType j = 0; j < N; j++)
        {
            BOOST_CHECK_EQUAL(m1.extractElement(i, j), mat[i][j]);
        }
    }
}

//****************************************************************************
// LIL constructor from dense matrix, implied zeros
BOOST_AUTO_TEST_CASE(lil_test_construction_dense_zero)
{
    std::vector<std::vector<double>> mat = {{6, 0, 0, 4},
                                            {7, 0, 0, 0},
                                            {0, 0, 9, 4},
                                            {2, 5, 0, 3},
                                            {2, 0, 0, 1},
                                            {0, 0, 0, 0},
                                            {0, 1, 0, 2}};

    backend::LilSparseMatrix<double> m1(mat, 0);

    IndexType M = mat.size();
    IndexType N = mat[0].size();
    IndexType num_rows(m1.nrows());
    IndexType num_cols(m1.ncols());

    BOOST_CHECK_EQUAL(num_rows, M);
    BOOST_CHECK_EQUAL(num_cols, N);
    for (IndexType i = 0; i < M; i++)
    {
        for (IndexType j = 0; j < N; j++)
        {
            if (mat[i][j] != 0)
            {
                BOOST_CHECK_EQUAL(m1.extractElement(i, j), mat[i][j]);
            }
        }
    }
}

//****************************************************************************
// LIL constructor from copy
BOOST_AUTO_TEST_CASE(lil_test_construction_copy)
{
    std::vector<std::vector<double>> mat = {{6, 0, 0, 4},
                                            {7, 0, 0, 0},
                                            {0, 0, 9, 4},
                                            {2, 5, 0, 3},
                                            {2, 0, 0, 1},
                                            {0, 0, 0, 0},
                                            {0, 1, 0, 2}};

    backend::LilSparseMatrix<double> m1(mat, 0);

    backend::LilSparseMatrix<double> m2(m1);

    BOOST_CHECK_EQUAL(m1, m2);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(lil_test_assign_to_implied_zero)
{
    std::vector<std::vector<double>> mat = {{6, 0, 0, 4},
                                            {7, 0, 0, 0},
                                            {0, 0, 9, 4},
                                            {2, 5, 0, 3},
                                            {2, 0, 0, 1},
                                            {0, 0, 0, 0},
                                            {0, 1, 0, 2}};

    backend::LilSparseMatrix<double> m1(mat, 0);

    mat[0][1] = 8;
    m1.setElement(0, 1, 8);
    BOOST_CHECK_EQUAL(m1.extractElement(0, 1), mat[0][1]);

    backend::LilSparseMatrix<double> m2(mat, 0);
    BOOST_CHECK_EQUAL(m1, m2);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(lil_test_assign_to_nonzero_element)
{
    std::vector<std::vector<double>> mat = {{6, 0, 0, 4},
                                            {7, 0, 0, 0},
                                            {0, 0, 9, 4},
                                            {2, 5, 0, 3},
                                            {2, 0, 0, 1},
                                            {0, 0, 0, 0},
                                            {0, 1, 0, 2}};

    backend::LilSparseMatrix<double> m1(mat, 0);

    mat[0][0] = 8;
    m1.setElement(0, 0, 8);
    BOOST_CHECK_EQUAL(m1.extractElement(0, 0), mat[0][0]);

    backend::LilSparseMatrix<double> m2(mat, 0);
    BOOST_CHECK_EQUAL(m1, m2);
}


//****************************************************************************
BOOST_AUTO_TEST_CASE(lil_test_get_set_col)
{
    std::vector<std::vector<double>> mat = {{6, 0, 0, 4},
                                            {7, 0, 0, 0},
                                            {0, 0, 9, 4},
                                            {2, 5, 0, 3},
                                            {2, 0, 0, 1},
                                            {0, 0, 0, 0},
                                            {0, 1, 0, 2}};

    backend::LilSparseMatrix<double> m1(mat, 0);

    auto col = m1.getCol(0);
    BOOST_CHECK_EQUAL(4UL, col.size());

    col = m1.getCol(1);
    BOOST_CHECK_EQUAL(2UL, col.size());

    col = m1.getCol(2);
    BOOST_CHECK_EQUAL(1UL, col.size());

    col = m1.getCol(3);
    BOOST_CHECK_EQUAL(5UL, col.size());

    BOOST_CHECK_EQUAL(12UL, m1.nvals());
    col.clear();
    m1.setCol(0, col);
    col = m1.getCol(0);
    BOOST_CHECK_EQUAL(0UL, col.size());
    BOOST_CHECK_EQUAL(8UL, m1.nvals());

    col.clear();
    m1.setCol(1, col);
    BOOST_CHECK_EQUAL(6UL, m1.nvals());
    m1.setCol(2, col);
    BOOST_CHECK_EQUAL(5UL, m1.nvals());
    m1.setCol(3, col);
    BOOST_CHECK_EQUAL(0UL, m1.nvals());


    std::vector<std::vector<double>> mat2= {{0, 1, 0, 3, 0, 5, 0, 7, 0, 1, 0, 3, 0, 5, 0, 7},
                                            {0, 0, 2, 3, 0, 0, 6, 7, 0, 0, 2, 3, 0, 0, 6, 7},
                                            {0, 0, 0, 0, 4, 5, 6, 7, 0, 0, 0, 0, 4, 5, 6, 7},
                                            {0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 8, 8, 8, 8, 8, 8}};

    backend::LilSparseMatrix<double> m2(mat2, 0);
    backend::LilSparseMatrix<double> m3(4, 1);
    for (IndexType ci = 0; ci < 16; ++ci)
    {
        IndexType nz(0);
        //m3.setCol(0, m2.getCol(ci));
        auto c = m2.getCol(ci);
        m3.setCol(0, c);

        for (IndexType ri = 0; ri < 4; ++ri)
        {
            if (mat2[ri][ci] == 0)
            {
                ++nz;
            }
            else
            {
                BOOST_CHECK_EQUAL(mat2[ri][ci], m3.extractElement(ri, 0));
            }
        }
        BOOST_CHECK_EQUAL(m3.nvals(), 4UL - nz);
    }
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(lil_test_get_set_row)
{
    std::vector<std::vector<double>> mat = {{6, 7, 0, 2, 2, 0, 0},
                                            {0, 0, 0, 5, 0, 0, 1},
                                            {0, 0, 9, 0, 0, 0, 0},
                                            {4, 0, 4, 3, 1, 0, 2}};

    backend::LilSparseMatrix<double> m1(mat, 0);

    BOOST_CHECK_EQUAL(4UL, m1[0].size());
    BOOST_CHECK_EQUAL(2UL, m1[1].size());
    BOOST_CHECK_EQUAL(1UL, m1[2].size());
    BOOST_CHECK_EQUAL(5UL, m1[3].size());

    BOOST_CHECK_EQUAL(12UL, m1.nvals());

    std::vector<std::tuple<grb::IndexType, double>> row;
    m1.setRow(0, row);

    BOOST_CHECK_EQUAL(0UL, m1[0].size());
    BOOST_CHECK_EQUAL(8UL, m1.nvals());

    row.clear();
    m1.setRow(1, row);
    BOOST_CHECK_EQUAL(6UL, m1.nvals());
    m1.setRow(2, row);
    BOOST_CHECK_EQUAL(5UL, m1.nvals());
    m1.setRow(3, row);
    BOOST_CHECK_EQUAL(0UL, m1.nvals());


    std::vector<std::vector<double>> mat2= {{0, 0, 0, 0},
                                            {0, 0, 0, 1},
                                            {0, 0, 2, 0},
                                            {0, 0, 2, 1},
                                            {0, 4, 0, 0},
                                            {0, 4, 0, 1},
                                            {0, 4, 2, 0},
                                            {0, 4, 2, 1},
                                            {8, 0, 0, 0},
                                            {8, 0, 0, 1},
                                            {8, 0, 2, 0},
                                            {8, 0, 2, 1},
                                            {8, 4, 0, 0},
                                            {8, 4, 0, 1},
                                            {8, 4, 2, 0},
                                            {8, 4, 2, 1}};

    backend::LilSparseMatrix<double> m2(mat2, 0);
    backend::LilSparseMatrix<double> m3(1, 4);
    for (IndexType row_idx = 0; row_idx < 16; ++row_idx)
    {
        IndexType nz(0);
        auto &c = m2[row_idx];
        m3.setRow(0, c);

        for (IndexType col_idx = 0; col_idx < 4; ++col_idx)
        {
            if (mat2[row_idx][col_idx] == 0)
            {
                ++nz;
            }
            else
            {
                BOOST_CHECK_EQUAL(mat2[row_idx][col_idx], m3.extractElement(0, col_idx));
            }
        }
        BOOST_CHECK_EQUAL(m3.nvals(), 4UL - nz);
    }
}

BOOST_AUTO_TEST_SUITE_END()
