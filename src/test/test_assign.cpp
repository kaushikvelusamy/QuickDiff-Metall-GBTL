/*
 * GraphBLAS Template Library (GBTL), Version 3.0
 *
 * Copyright 2020 Carnegie Mellon University, Battelle Memorial Institute, and
 * Authors.
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
 * RIGHTS.
 *
 * Released under a BSD-style license, please see LICENSE file or contact
 * permission@sei.cmu.edu for full terms.
 *
 * [DISTRIBUTION STATEMENT A] This material has been approved for public release
 * and unlimited distribution.  Please see Copyright notice for non-US
 * Government use and distribution.
 *
 * This Software includes and/or makes use of the following Third-Party Software
 * subject to its own license:
 *
 * 1. Boost Unit Test Framework
 * (https://www.boost.org/doc/libs/1_45_0/libs/test/doc/html/utf.html)
 * Copyright 2001 Boost software license, Gennadiy Rozental.
 *
 * DM20-0442
 */

//#define GRAPHBLAS_LOGGING_LEVEL 2

#include <graphblas/graphblas.hpp>

using namespace grb;

#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE assign_test_suite

#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

//****************************************************************************
// 4.3.7.1 Standard Vector tests
//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_test_bad_dimensions)
{
    IndexArrayType i_w = {1, 2, 3};
    std::vector<double> v_w  = {1, 2, 3};
    Vector<double> w(4);
    w.build(i_w, v_w);

    IndexArrayType i_a    = {1};
    std::vector<double> v_a = {99};
    Vector<double> u(2);
    u.build(i_a, v_a);

    IndexArrayType vect_I({1,3,2});

    BOOST_CHECK_THROW(
            (assign(w,
                    grb::NoMask(),
                    grb::NoAccumulate(),
                    u,
                    vect_I)),
            DimensionException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_test_index_out_of_bounds)
{
    // Testing 4.3.7.1

    IndexArrayType i_c = {1, 2, 3};
    std::vector<double> v_c  = {1, 2, 3};
    Vector<double> w(4);
    w.build(i_c, v_c);

    IndexArrayType i_a    = {1};
    std::vector<double> v_a = {99};
    Vector<double> u(2);
    u.build(i_a, v_a);

    IndexArrayType vect_I({0,4});

    BOOST_CHECK_THROW(
            (assign(w,
                    grb::NoMask(),
                    grb::NoAccumulate(),
                    u,
                    vect_I)),
            IndexOutOfBoundsException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_test_no_mask_no_accum)
{
    // w = { 8, 9, 1, - }
    IndexArrayType i_c = {0, 1, 2};
    std::vector<double> v_c = {8, 9, 1   };
    Vector<double> c(4); // Gets is four entries
    c.build(i_c, v_c);

    // a = { -, 98, 99 }
    IndexArrayType i_a = {1, 2};
    std::vector<double> v_a = {   98, 99};
    Vector<double> a(3);
    a.build(i_a, v_a);

    IndexArrayType vect_I({1,2,3});

    IndexArrayType i_result      = {0,     2,  3};
    std::vector<double> v_result = {8,    98, 99};
    Vector<double> result(4);
    result.build(i_result, v_result);

    assign(c,
           grb::NoMask(),
           grb::NoAccumulate(),
           a,
           vect_I);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_test_accum)
{
    IndexArrayType i_c = {0, 1, 2};
    std::vector<double> v_c = {8, 9, 1   };
    Vector<double> c(4);
    c.build(i_c, v_c);

    IndexArrayType i_a = {1, 2};
    std::vector<double> v_a = {   98, 99};
    Vector<double> a(3);
    a.build(i_a, v_a);

    IndexArrayType vect_I({1,2,3});

    IndexArrayType i_result      = {0, 1,  2,  3};
    std::vector<double> v_result = {8, 9, 99, 99};
    Vector<double> result(4);
    result.build(i_result, v_result);

    assign(c, grb::NoMask(), grb::Plus<double>(),
           a, vect_I);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_test_allindices_no_accum)
{
    IndexArrayType i_c = {0, 1, 2};
    std::vector<double> v_c = {8, 9, 1   };
    Vector<double> c(4);
    c.build(i_c, v_c);

    IndexArrayType i_a = {1, 2, 3};
    std::vector<double> v_a = {   98, 99, 100};
    Vector<double> a(4);
    a.build(i_a, v_a);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           a, grb::AllIndices());

    BOOST_CHECK_EQUAL(c, a);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_test_allindices_accum)
{
    IndexArrayType i_c = {0, 1, 2};
    std::vector<double> v_c = {8, 9, 1   };
    Vector<double> c(4);
    c.build(i_c, v_c);

    IndexArrayType i_a = {1, 2, 3};
    std::vector<double> v_a = {   98, 99, 100};
    Vector<double> a(4);
    a.build(i_a, v_a);

    assign(c,
           grb::NoMask(),
           grb::Plus<double>(),
           a,
           grb::AllIndices());

    IndexArrayType i_result      = {0, 1,  2,  3};
    std::vector<double> v_result = {8, 107, 100, 100};
    Vector<double> result(4);
    result.build(i_result, v_result);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_test_mask_no_accum)
{
    // w = { 8, 9, 1, 7, - }
    // The size is one higher so we get an empty element at the end
    IndexArrayType i_w =      {0, 1, 2, 3 };
    std::vector<double> v_w = {8, 9, 1, 7 };
    Vector<double> w(5);
    w.build(i_w, v_w);

    //std::cout << "w: " << w << std::endl;

    // Mask
    // 1 shows we can pass through empty elements
    // 2 is normal values
    // 3 is normal values to empty elements
    IndexArrayType i_m =    { 1,    2,    4 };
    std::vector<bool> v_m = { true, true, true };
    Vector<bool> m(5);
    m.build(i_m, v_m);

    //std::cout << "m: " << m << std::endl;

    // Values to assign out -- a = { -, 98, 99, 101 }
    IndexArrayType i_u =      { 1,  2,  3};
    std::vector<double> v_u = { 98, 99, 101};
    Vector<double> u(4);
    u.build(i_u, v_u);

    //std::cout << "a: " << a << std::endl;

    // Indexes within the output where to assign to
    IndexArrayType v_ind({1,2,3,4});

    //std::cout << "v_ind: " << v_ind << std::endl;

    // result = { 8, -, 98, 7, 101 }
    // Note, the second value at index 1 removed because it doesn't exist in 2.
    // The 7 (index 3) is NOT replaced by the 99 because of the mask
    IndexArrayType i_result      = {0,     2,  3, 4};
    std::vector<double> v_result = {8,    98,  7, 101};
    Vector<double> result(5);
    result.build(i_result, v_result);

    //std::cout << "result: " << result << std::endl;

    assign(w,
           m,
           grb::NoAccumulate(),
           u,
           v_ind);

    //std::cout << "w out: " << result << std::endl;

    BOOST_CHECK_EQUAL(w, result);

    // ==============
    // REPLACE check

    // Reset w
    w.build(i_w, v_w);

    // Make the new result

    // result = { -, -, 98, -, 101 }
    // Note, the second value at index 1 removed because it doesn't exist in 2.
    // The 7 (index 3) is NOT replaced by the 99 because of the mask
    IndexArrayType i_result2     =  {2,  4};
    std::vector<double> v_result2 = {98, 101};
    Vector<double> result2(5);
    result2.build(i_result2, v_result2);

    //std::cout << "result2: " << result << std::endl;

    assign(w,
           m,
           grb::NoAccumulate(),
           u,
           v_ind,
           REPLACE);

    //std::cout << "w out: " << result << std::endl;

    BOOST_CHECK_EQUAL(w, result2);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_test_mask_accum)
{
    // w = { 8, 9, 1, 7, - }
    // The size is one higher so we get an empty element at the end
    IndexArrayType i_w =      {0, 1, 2, 3 };
    std::vector<double> v_w = {8, 9, 1, 7 };
    Vector<double> w(5);
    w.build(i_w, v_w);

    //std::cout << "w: " << w << std::endl;

    // Mask
    // 1 shows we can pass through empty elements
    // 2 is normal values
    // 3 is normal values to empty elements
    IndexArrayType i_m =    { 1,    2,    4 };
    std::vector<bool> v_m = { true, true, true };
    Vector<bool> m(5);
    m.build(i_m, v_m);

    //std::cout << "m: " << m << std::endl;

    // Values to assign out -- a = { -, 98, 99, 101 }
    IndexArrayType i_u =      { 1,  2,  3};
    std::vector<double> v_u = { 98, 99, 101};
    Vector<double> u(4);
    u.build(i_u, v_u);

    //std::cout << "a: " << a << std::endl;

    // Indexes within the output where to assign to
    IndexArrayType v_ind({1,2,3,4});

    // NOTE: t indices should be: { 2, 3, 4} - not 1 because there is no value
    // for '0' (first elem) in u.

    //std::cout << "v_ind: " << v_ind << std::endl;

    // result = { 8, 9, 99, 7, 101 }
    // accum plus means add the "t" to w.
    IndexArrayType i_result      = {0, 1,  2,  3, 4};
    std::vector<double> v_result = {8, 9, 99,  7, 101};
    Vector<double> result(5);
    result.build(i_result, v_result);

    //std::cout << "result: " << result << std::endl;

    assign(w,
           m,
           grb::Plus<double>(),
           u,
           v_ind);

    //std::cout << "w out: " << result << std::endl;

    BOOST_CHECK_EQUAL(w, result);

    // ==============
    // REPLACE check

    // Reset w
    w.build(i_w, v_w);

    // Make the new result

    // result = { -, 9, 98, -, 101 }
    IndexArrayType i_result2     =  {1, 2,  4};
    std::vector<double> v_result2 = {9, 99, 101};
    Vector<double> result2(5);
    result2.build(i_result2, v_result2);

    //std::cout << "result2: " << result << std::endl;

    assign(w,
           m,
           grb::Plus<double>(),
           u,
           v_ind,
           REPLACE);

    //std::cout << "w out: " << result << std::endl;

    BOOST_CHECK_EQUAL(w, result2);
}

//****************************************************************************
// 4.3.7.2 Standard Matrix tests
//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_test_bad_dimensions)
{
    IndexArrayType      i_c = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType      j_c = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {1, 2, 3, 4, 6, 7, 8, 9, 1};
    Matrix<double, DirectedMatrixTag> C(3, 4);
    C.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 0, 1};
    IndexArrayType j_a    = {0, 1, 0};
    std::vector<double> v_a = {1, 99, 99};
    Matrix<double, DirectedMatrixTag> A(2, 2);
    A.build(i_a, j_a, v_a);

    IndexArrayType vect_I({1,0,1});
    IndexArrayType vect_J({1,2});

    BOOST_CHECK_THROW(
        (assign(C, grb::NoMask(), grb::NoAccumulate(),
                A, vect_I, vect_J)),
        DimensionException);

    BOOST_CHECK_THROW(
        (assign(C, grb::NoMask(), grb::NoAccumulate(),
                transpose(A), vect_I, vect_J)),
        DimensionException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_test_index_out_of_bounds)
{
    IndexArrayType      i_c = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType      j_c = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {1, 2, 3, 4, 6, 7, 8, 9, 1};
    Matrix<double, DirectedMatrixTag> C(3, 4);
    C.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 0, 1};
    IndexArrayType j_a    = {0, 1, 0};
    std::vector<double> v_a = {1, 99, 99};
    Matrix<double, DirectedMatrixTag> A(2, 2);
    A.build(i_a, j_a, v_a);

    IndexArrayType vect_I({1,5});
    IndexArrayType vect_J({1,2});

    BOOST_CHECK_THROW(
        (assign(C, grb::NoMask(), grb::NoAccumulate(),
                A, vect_I, vect_J)),
        IndexOutOfBoundsException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(sparse_assign_matrix_base)
{
    std::vector<std::vector<double>> matA = {{1, 6},
                                             {9, 2}};
    grb::Matrix<double, grb::DirectedMatrixTag> mA(matA, 0);

    std::vector<std::vector<double>> matAnswer = {{0, 0, 0},
                                                  {9, 0, 2},
                                                  {1, 0, 6}};
    grb::Matrix<double, grb::DirectedMatrixTag> answer(matAnswer, 0);

    // Output space
    grb::IndexType M = 3;
    grb::IndexType N = 3;

    grb::IndexArrayType row_indices = {2, 1};
    grb::IndexArrayType col_indices = {0, 2};

    grb::Matrix<double, grb::DirectedMatrixTag> result(M, N);

    grb::assign(result,
                      grb::NoMask(),
                      grb::NoAccumulate(),
                      mA,
                      row_indices,
                      col_indices);

    BOOST_CHECK_EQUAL(result, answer);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(sparse_assign_matrix_mask)
{
    std::vector<std::vector<double>> matA = {{1, 6},
                                             {9, 2}};
    grb::Matrix<double, grb::DirectedMatrixTag> mA(matA, 0);

    std::vector<std::vector<double>> matAnswer = {{0, 0, 0},
                                                  {9, 0, 0},
                                                  {1, 0, 0}};
    grb::Matrix<double, grb::DirectedMatrixTag> answer(matAnswer, 0);

    std::vector<std::vector<bool>> matMask = {{true, true, false},
                                              {true, true, false},
                                              {true, true, false}};
    grb::Matrix<bool, grb::DirectedMatrixTag> mask(matMask, 0);


    // Output space
    grb::IndexType M = 3;
    grb::IndexType N = 3;

    grb::IndexArrayType row_indices = {2, 1};
    grb::IndexArrayType col_indices = {0, 2};

    grb::Matrix<double, grb::DirectedMatrixTag> result(M, N);

    grb::assign(result,
                      mask,
                      grb::NoAccumulate(),
                      mA,
                      row_indices,
                      col_indices);

    BOOST_CHECK_EQUAL(result, answer);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(sparse_assign_matrix_accum)
{
    std::vector<std::vector<double>> matA = {{1, 6},
                                             {9, 2}};
    grb::Matrix<double, grb::DirectedMatrixTag> mA(matA, 0);

    std::vector<std::vector<double>> matC= {{1, 2, 3},
                                            {4, 5, 6},
                                            {7, 8, 9}};
    grb::Matrix<double, grb::DirectedMatrixTag> mC(matC, 0);

    std::vector<std::vector<double>> matAnswer = {{1,  2, 3},
                                                  {13, 5, 8},
                                                  {8,  8, 15}};
    grb::Matrix<double, grb::DirectedMatrixTag> answer(matAnswer, 0);

    grb::IndexArrayType row_indices = {2, 1};
    grb::IndexArrayType col_indices = {0, 2};

    grb::assign(mC,
                      grb::NoMask(),
                      grb::Plus<double>(),
                      mA,
                      row_indices,
                      col_indices);

    BOOST_CHECK_EQUAL(mC, answer);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_test_no_mask_no_accum)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> C(3, 4);
    C.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 0, 1};
    IndexArrayType j_a    = {0, 1, 0};
    std::vector<double> v_a = {1, 99,
                               99   };
    Matrix<double, DirectedMatrixTag> A(2, 2);
    A.build(i_a, j_a, v_a);

    IndexArrayType vect_I({1,2});
    IndexArrayType vect_J({1,2});

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1};
    std::vector<double> v_result = {   1, 2,  3,
                                    4, 1, 99, 7,
                                    8, 99      };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(C,
           grb::NoMask(),
           grb::NoAccumulate(),
           A,
           vect_I,
           vect_J);

    BOOST_CHECK_EQUAL(C, result);

    assign(C,
           grb::NoMask(),
           grb::NoAccumulate(),
           transpose(A),
           vect_I,
           vect_J);

    BOOST_CHECK_EQUAL(C, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_test_accum_no_mask)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 0, 1};
    IndexArrayType j_a    = {0, 1, 0};
    std::vector<double> v_a = {1, 99,
                               99   };
    Matrix<double, DirectedMatrixTag> a(2, 2);
    a.build(i_a, j_a, v_a);

    IndexArrayType vect_I({1,2});
    IndexArrayType vect_J({1,2});

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {   1,     2, 3,
                                    4, 1,   105, 7,
                                    8, 108,   1   };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c,
           grb::NoMask(),
           grb::Plus<double>(),
           a,
           vect_I,
           vect_J);

    BOOST_CHECK_EQUAL(c, result);

    c.clear();
    c.build(i_c, j_c, v_c);
    assign(c,
           grb::NoMask(),
           grb::Plus<double>(),
           transpose(a),
           vect_I,
           vect_J);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_test_allrows_no_accum)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 0,
                             1,
                             2};
    IndexArrayType j_a    = {0, 1,
                             0,
                             0};
    std::vector<double> v_a = {1,  99,
                               1,
                               99};
    Matrix<double, DirectedMatrixTag> a(3, 2);
    a.build(i_a, j_a, v_a);

    //IndexArrayType vect_I({1,2});
    IndexArrayType vect_J({0,1});

    IndexArrayType i_result    = {0, 0, 0, 0, 1, 1, 1, 2, 2};
    IndexArrayType j_result    = {0, 1, 2, 3, 0, 2, 3, 0, 2};
    std::vector<double> v_result = {1, 99, 2,  3,
                                    1,     6,  7,
                                    99,    1   };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           a, grb::AllIndices(), vect_J);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_test_allrows_no_accum_transpose)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType j_a    = {0, 0,
                             1,
                             2};
    IndexArrayType i_a    = {0, 1,
                             0,
                             0};
    std::vector<double> v_a = {1,  99,
                               1,
                               99};
    Matrix<double, DirectedMatrixTag> a(2, 3);
    a.build(i_a, j_a, v_a);

    //IndexArrayType vect_I({1,2});
    IndexArrayType vect_J({0,1});

    IndexArrayType i_result    = {0, 0, 0, 0, 1, 1, 1, 2, 2};
    IndexArrayType j_result    = {0, 1, 2, 3, 0, 2, 3, 0, 2};
    std::vector<double> v_result = {1, 99, 2,  3,
                                    1,     6,  7,
                                    99,    1   };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           transpose(a), grb::AllIndices(), vect_J);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_test_allcols_no_accum_no_mask)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {    0,      0,
                             1,  1,  1    };
    IndexArrayType j_a    = {    1,      3,
                             0,  1,  2    };
    std::vector<double> v_a = {   99,     99,
                               1, 99, 99    };
    Matrix<double, DirectedMatrixTag> a(2, 4);
    a.build(i_a, j_a, v_a);

    IndexArrayType vect_I({1,2});
    //IndexArrayType vect_J({0,1});

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 1, 3, 0, 1, 2};
    std::vector<double> v_result = {    1,  2,  3,
                                       99,     99,
                                    1, 99, 99    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           a, vect_I, grb::AllIndices());

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_test_allcols_no_accum_no_mask_transpose)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType j_a    = {    0,      0,
                             1,  1,  1    };
    IndexArrayType i_a    = {    1,      3,
                             0,  1,  2    };
    std::vector<double> v_a = {   99,     99,
                               1, 99, 99    };
    Matrix<double, DirectedMatrixTag> a(4, 2);
    a.build(i_a, j_a, v_a);

    IndexArrayType vect_I({1,2});
    //IndexArrayType vect_J({0,1});

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 1, 3, 0, 1, 2};
    std::vector<double> v_result = {    1,  2,  3,
                                       99,     99,
                                    1, 99, 99    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           transpose(a), vect_I, grb::AllIndices());

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_test_allrowscols_no_accum_no_mask)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {    0,      0,
                             1,  1,  1,
                             2,  2};
    IndexArrayType j_a    = {    1,      3,
                             0,  1,  2,
                             0,  1    };
    std::vector<double> v_a = {    99,     99,
                                1, 99, 99,
                               97, 96    };
    Matrix<double, DirectedMatrixTag> a(3, 4);
    a.build(i_a, j_a, v_a);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           a, grb::AllIndices(), grb::AllIndices());

    BOOST_CHECK_EQUAL(c, a);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_test_allrowscols_no_accum_no_mask_transpose)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType j_a    = {    0,      0,
                             1,  1,  1,
                             2,  2};
    IndexArrayType i_a    = {    1,      3,
                             0,  1,  2,
                             0,  1    };
    std::vector<double> v_a = {    99,     99,
                                1, 99, 99,
                               97, 96    };
    Matrix<double, DirectedMatrixTag> a(4, 3);
    a.build(i_a, j_a, v_a);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           transpose(a), grb::AllIndices(), grb::AllIndices());

    Matrix<double, DirectedMatrixTag> answer(3,4);
    answer.build(j_a, i_a, v_a);
    BOOST_CHECK_EQUAL(c, answer);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_test_allrowscols_accum_no_mask)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {    0,      0,
                             1,  1,  1,
                             2,  2};
    IndexArrayType j_a    = {    1,      3,
                             0,  1,  2,
                             0,  1    };
    std::vector<double> v_a = {    99,     99,
                                1, 99, 99,
                               97, 96    };
    Matrix<double, DirectedMatrixTag> a(3, 4);
    a.build(i_a, j_a, v_a);

    assign(c, grb::NoMask(), grb::Plus<double>(),
           a, grb::AllIndices(), grb::AllIndices());

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {     100,   2, 102,
                                      5,  99, 105,   7,
                                    105, 105,   1     };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_test_allrowscols_accum_no_mask_transpose)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType j_a    = {    0,      0,
                             1,  1,  1,
                             2,  2};
    IndexArrayType i_a    = {    1,      3,
                             0,  1,  2,
                             0,  1    };
    std::vector<double> v_a = {    99,     99,
                                1, 99, 99,
                               97, 96    };
    Matrix<double, DirectedMatrixTag> a(4, 3);
    a.build(i_a, j_a, v_a);

    assign(c, grb::NoMask(), grb::Plus<double>(),
           transpose(a), grb::AllIndices(), grb::AllIndices());

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {     100,   2, 102,
                                      5,  99, 105,   7,
                                    105, 105,   1     };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_mask_no_accum)
{
    // C - Target Matrix
    IndexArrayType i_c      = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c      = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_C = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> C(3, 4);
    C.build(i_c, j_c, v_C);

    // M - Mask
    IndexArrayType i_M    = {0, 0, 1, 1, 1, 2, 2, 1};
    IndexArrayType j_M    = {1, 2, 0, 2, 3, 0, 1, 2};
    std::vector<bool> v_M = {       true,  true,
                             true,         true, true,
                             true,  true,  true};
    Matrix<bool, DirectedMatrixTag> M(3, 4);
    M.build(i_M, j_M, v_M);

    // A - Source Matrix
    IndexArrayType i_A    = {0, 0, 1};
    IndexArrayType j_A    = {0, 1, 0};
    std::vector<double> v_A = {1, 99,
                               99    };
    Matrix<double, DirectedMatrixTag> A(2, 2);
    A.build(i_A, j_A, v_A);

    // Indices
    IndexArrayType vec_row_idx({1,2});
    IndexArrayType vec_col_idx({1,2});

    // This looks like the source except we have the new matrix inserted in
    // the box of 1,1 to 2,2.  Note, that we still get the 1 at 2,2 because
    // the indices do NOT contain an entry for 2,3 so it is untouched.
    IndexArrayType i_result      = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result      = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {    1,  2,  3,
                                     4,     99, 7,
                                     8, 99, 1     };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(C,
           M,
           grb::NoAccumulate(),
           A,
           vec_row_idx,
           vec_col_idx);

    BOOST_CHECK_EQUAL(C, result);

    // ==== REPLACE ====

    // Reset C
    C.build(i_c, j_c, v_C);

    // Set up other result
    IndexArrayType i_result2      = {0, 0, 1, 1, 1, 2, 2};
    IndexArrayType j_result2      = {1, 2, 0, 2, 3, 0, 1};
    std::vector<double> v_result2 = {    1,  2,
                                     4,      99, 7,
                                     8, 99,       };
    Matrix<double, DirectedMatrixTag> result2(3, 4);
    result2.build(i_result2, j_result2, v_result2);

    assign(C,
           M,
           grb::NoAccumulate(),
           A,
           vec_row_idx,
           vec_col_idx,
           REPLACE);

    BOOST_CHECK_EQUAL(C, result2);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_mask_no_accum_transpose)
{
    // C - Target Matrix
    IndexArrayType i_c      = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c      = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_C = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> C(3, 4);
    C.build(i_c, j_c, v_C);

    // M - Mask
    IndexArrayType i_M    = {0, 0, 1, 1, 1, 2, 2, 1};
    IndexArrayType j_M    = {1, 2, 0, 2, 3, 0, 1, 2};
    std::vector<bool> v_M = {       true,  true,
                             true,         true, true,
                             true,  true,  true};
    Matrix<bool, DirectedMatrixTag> M(3, 4);
    M.build(i_M, j_M, v_M);

    // A - Source Matrix
    IndexArrayType j_A    = {0, 0, 1};
    IndexArrayType i_A    = {0, 1, 0};
    std::vector<double> v_A = {1, 99,
                               99    };
    Matrix<double, DirectedMatrixTag> A(2, 2);
    A.build(i_A, j_A, v_A);

    // Indices
    IndexArrayType vec_row_idx({1,2});
    IndexArrayType vec_col_idx({1,2});

    // This looks like the source except we have the new matrix inserted in
    // the box of 1,1 to 2,2.  Note, that we still get the 1 at 2,2 because
    // the indices do NOT contain an entry for 2,3 so it is untouched.
    IndexArrayType i_result      = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result      = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {    1,  2,  3,
                                     4,     99, 7,
                                     8, 99, 1     };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(C,
           M,
           grb::NoAccumulate(),
           transpose(A),
           vec_row_idx,
           vec_col_idx);

    BOOST_CHECK_EQUAL(C, result);

    // ==== REPLACE ====

    // Reset C
    C.build(i_c, j_c, v_C);

    // Set up other result
    IndexArrayType i_result2      = {0, 0, 1, 1, 1, 2, 2};
    IndexArrayType j_result2      = {1, 2, 0, 2, 3, 0, 1};
    std::vector<double> v_result2 = {    1,  2,
                                     4,      99, 7,
                                     8, 99,       };
    Matrix<double, DirectedMatrixTag> result2(3, 4);
    result2.build(i_result2, j_result2, v_result2);

    assign(C,
           M,
           grb::NoAccumulate(),
           transpose(A),
           vec_row_idx,
           vec_col_idx,
           REPLACE);

    BOOST_CHECK_EQUAL(C, result2);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_mask_accum)
{
    // C - Target Matrix
    IndexArrayType i_c      = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c      = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_C = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> C(3, 4);
    C.build(i_c, j_c, v_C);

    // M - Mask
    IndexArrayType i_M    = {0, 0, 1, 1, 1, 2, 2, 1};
    IndexArrayType j_M    = {1, 2, 0, 2, 3, 0, 1, 2};
    std::vector<bool> v_M = {       true,  true,
                             true,         true, true,
                             true,  true,  true};
    Matrix<bool, DirectedMatrixTag> M(3, 4);
    M.build(i_M, j_M, v_M);

    // A - Source Matrix
    IndexArrayType i_A    = {0, 0, 1};
    IndexArrayType j_A    = {0, 1, 0};
    std::vector<double> v_A = {1, 99,
                               99    };
    Matrix<double, DirectedMatrixTag> A(2, 2);
    A.build(i_A, j_A, v_A);

    // Indices
    IndexArrayType vec_row_idx({1,2});
    IndexArrayType vec_col_idx({1,2});

    // This looks like the source except we have the new matrix inserted in
    // the box of 1,1 to 2,2.  Note, that we still get the 1 at 2,2 because
    // the indices do NOT contain an entry for 2,3 so it is untouched.
    IndexArrayType i_result      = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result      = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {    1,   2,   3,
                                    4,     105,   7,
                                    8, 108,  1     };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(C,
           M,
           grb::Plus<double>(),
           A,
           vec_row_idx,
           vec_col_idx);

    BOOST_CHECK_EQUAL(C, result);

    // ==== REPLACE ====

    // Reset C
    C.build(i_c, j_c, v_C);

    // Set up other result
    IndexArrayType i_result2      = {0, 0, 1, 1, 1, 2, 2};
    IndexArrayType j_result2      = {1, 2, 0, 2, 3, 0, 1};
    std::vector<double> v_result2 = {    1,  2,
                                     4,      105, 7,
                                     8, 108,        };
    Matrix<double, DirectedMatrixTag> result2(3, 4);
    result2.build(i_result2, j_result2, v_result2);

    assign(C,
           M,
           grb::Plus<double>(),
           A,
           vec_row_idx,
           vec_col_idx,
           REPLACE);

    BOOST_CHECK_EQUAL(C, result2);

}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_mask_accum_transpose)
{
    // C - Target Matrix
    IndexArrayType i_c      = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c      = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_C = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> C(3, 4);
    C.build(i_c, j_c, v_C);

    // M - Mask
    IndexArrayType i_M    = {0, 0, 1, 1, 1, 2, 2, 1};
    IndexArrayType j_M    = {1, 2, 0, 2, 3, 0, 1, 2};
    std::vector<bool> v_M = {       true,  true,
                             true,         true, true,
                             true,  true,  true};
    Matrix<bool, DirectedMatrixTag> M(3, 4);
    M.build(i_M, j_M, v_M);

    // A - Source Matrix
    IndexArrayType j_A    = {0, 0, 1};
    IndexArrayType i_A    = {0, 1, 0};
    std::vector<double> v_A = {1, 99,
                               99    };
    Matrix<double, DirectedMatrixTag> A(2, 2);
    A.build(i_A, j_A, v_A);

    // Indices
    IndexArrayType vec_row_idx({1,2});
    IndexArrayType vec_col_idx({1,2});

    // This looks like the source except we have the new matrix inserted in
    // the box of 1,1 to 2,2.  Note, that we still get the 1 at 2,2 because
    // the indices do NOT contain an entry for 2,3 so it is untouched.
    IndexArrayType i_result      = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result      = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {    1,   2,   3,
                                    4,     105,   7,
                                    8, 108,  1     };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(C,
           M,
           grb::Plus<double>(),
           transpose(A),
           vec_row_idx,
           vec_col_idx);

    BOOST_CHECK_EQUAL(C, result);

    // ==== REPLACE ====

    // Reset C
    C.build(i_c, j_c, v_C);

    // Set up other result
    IndexArrayType i_result2      = {0, 0, 1, 1, 1, 2, 2};
    IndexArrayType j_result2      = {1, 2, 0, 2, 3, 0, 1};
    std::vector<double> v_result2 = {    1,  2,
                                     4,      105, 7,
                                     8, 108,        };
    Matrix<double, DirectedMatrixTag> result2(3, 4);
    result2.build(i_result2, j_result2, v_result2);

    assign(C,
           M,
           grb::Plus<double>(),
           transpose(A),
           vec_row_idx,
           vec_col_idx,
           REPLACE);

    BOOST_CHECK_EQUAL(C, result2);

}

//****************************************************************************
// 4.3.7.3 Assign Column tests
//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_col_test_bad_dimensions)
{
    IndexArrayType      i_c = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType      j_c = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 1};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(2);
    a.build(i_a, v_a);

    IndexArrayType vect_I({0,1,2});
    //IndexArrayType vect_J({1,2});

    BOOST_CHECK_THROW(
        (assign(c, grb::NoMask(), grb::NoAccumulate(),
                a, vect_I, 2)),
        DimensionException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_col_test_invalid_index)
{
    IndexArrayType      i_c = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType      j_c = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 1};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(2);
    a.build(i_a, v_a);

    IndexArrayType vect_I({0,2});
    //IndexArrayType vect_J({1,2});

    BOOST_CHECK_THROW(
        (assign(c, grb::NoMask(), grb::NoAccumulate(),
                a, vect_I, 5)),
        InvalidIndexException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_col_test_index_out_of_bounds)
{
    IndexArrayType      i_c = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType      j_c = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 1};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(2);
    a.build(i_a, v_a);

    IndexArrayType vect_I2({1,5});
    BOOST_CHECK_THROW(
        (assign(c, grb::NoMask(), grb::NoAccumulate(),
                a, vect_I2, 2)),
        IndexOutOfBoundsException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_col_test_no_accum_no_mask)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 1};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(2);
    a.build(i_a, v_a);

    IndexArrayType vect_I({1,2});

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {    1, 2, 3,
                                    4,  1, 6, 7,
                                    8, 99, 1    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           a, vect_I, 1);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_col_test_allindices_no_accum_no_mask)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                                  4,    6, 7,
                                  8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {1, 2};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(3);
    a.build(i_a, v_a);

    IndexArrayType i_result    = {0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {       2, 3,
                                    4,  1, 6, 7,
                                    8, 99, 1    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           a, grb::AllIndices(), 1);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_col_test_accum_no_mask)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 1};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(2);
    a.build(i_a, v_a);

    IndexArrayType vect_I({1,2});

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {    1, 2, 3,
                                    4,  1, 6, 7,
                                    8,108, 1    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::Plus<double>(),
           a, vect_I, 1);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_col_test_allindices_accum_no_mask)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {1, 2};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(3);
    a.build(i_a, v_a);

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {     1, 2, 3,
                                     4,  1, 6, 7,
                                     8,108, 1    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::Plus<double>(),
           a, grb::AllIndices(), 1);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_col_test_mask_no_accum)
{
    IndexArrayType i_C    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_C    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_C = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> C(3, 4);
    C.build(i_C, j_C, v_C);

    // m - Mask - Vector - matches single column of C
    IndexArrayType i_m    = {0,    2};
    std::vector<bool> v_m = {true, true};
    Vector<bool> m(3);
    m.build(i_m, v_m);

    // u - must be a vector
    IndexArrayType i_u    = {0, 1};
    std::vector<double> v_u = {75, 99};
    Vector<double> u(2);
    u.build(i_u, v_u);

    IndexArrayType vect_I({1,2});

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {    1, 2, 3,
                                    4,     6, 7,
                                    8, 99, 1    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(C,
           m,
           grb::NoAccumulate(),
           u,
           vect_I,
           1);

    BOOST_CHECK_EQUAL(C, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_col_test_mask_no_accum_replace)
{
    IndexArrayType i_C    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_C    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_C = {   1,  2, 3,
                               4, 21, 6, 7,
                               8, 9,  1   };
    Matrix<double, DirectedMatrixTag> C(3, 4);
    C.build(i_C, j_C, v_C);

    // m - Mask - Vector - matches single column of C
    IndexArrayType i_m    = {0,    2};
    std::vector<bool> v_m = {true, true};
    Vector<bool> m(3);
    m.build(i_m, v_m);

    // u - must be a vector
    IndexArrayType i_u    = {0, 1};
    std::vector<double> v_u = {75, 99};
    Vector<double> u(2);
    u.build(i_u, v_u);

    IndexArrayType vect_I({1,2});

    // We should ONLY see value provided cleared by the new values AND the mask
    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {    1,  2, 3,
                                    4,      6, 7,
                                    8, 99,  1    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(C,
           m,
           grb::NoAccumulate(),
           u,
           vect_I,
           1,
           REPLACE);

    BOOST_CHECK_EQUAL(C, result);
}

//****************************************************************************
// 4.3.7.4 Assign Row tests
//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_row_test_bad_dimensions)
{
    IndexArrayType      i_c = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType      j_c = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 1};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(2);
    a.build(i_a, v_a);

    //IndexArrayType vect_I({0,1,2});
    IndexArrayType vect_J({1});

    BOOST_CHECK_THROW(
        (assign(c, grb::NoMask(), grb::NoAccumulate(),
                a, 1, vect_J)),
        DimensionException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_row_test_invalid_index)
{
    IndexArrayType      i_c = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType      j_c = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 1};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(2);
    a.build(i_a, v_a);

    //IndexArrayType vect_I({0,2});
    IndexArrayType vect_J({1,2});

    BOOST_CHECK_THROW(
        (assign(c, grb::NoMask(), grb::NoAccumulate(),
                a, 5, vect_J)),
        InvalidIndexException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_row_test_index_out_of_bounds)
{
    IndexArrayType      i_c = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType      j_c = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 1};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(2);
    a.build(i_a, v_a);

    IndexArrayType vect_J2({1,5});

    BOOST_CHECK_THROW(
        (assign(c, grb::NoMask(), grb::NoAccumulate(),
                a, 2, vect_J2)),
        IndexOutOfBoundsException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_row_test_no_accum)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 1};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(2);
    a.build(i_a, v_a);

    IndexArrayType vect_J({1,2});

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {    1,  2, 3,
                                    4,  1, 99, 7,
                                    8,  9,  1    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           a, 1, vect_J);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_row_test_allindices_no_accum)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {1, 2};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(4);
    a.build(i_a, v_a);

    IndexArrayType i_result    = {0, 0, 0,  1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3,  1, 2, 0, 1, 2};
    std::vector<double> v_result = {   1,  2, 3,
                                       1, 99,
                                    8, 9,  1    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           a, 1, grb::AllIndices());

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_row_test_accum)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {0, 1};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(2);
    a.build(i_a, v_a);

    IndexArrayType vect_J({1,2});

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {    1,  2, 3,
                                    4,  1,105, 7,
                                    8,  9,  1    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::Plus<double>(),
           a, 1, vect_J);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_row_test_allindices_accum)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_a    = {1, 2};
    std::vector<double> v_a = {1, 99};
    Vector<double> a(4);
    a.build(i_a, v_a);

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {   1,  2, 3,
                                    4, 1,105, 7,
                                    8, 9,  1    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::Plus<double>(),
           a, 1, grb::AllIndices());

    BOOST_CHECK_EQUAL(c, result);
}

/// @todo add tests with masks

//****************************************************************************
// 4.3.7.5 Vector constant tests
//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_constant_test_bad_dimensions)
{
    IndexArrayType i_w = {1, 2, 3};
    std::vector<double> v_w  = {1, 2, 3};
    Vector<double> w(4);
    w.build(i_w, v_w);

    IndexArrayType i_m    = {1};  // Dimension mismatch only possible with mask
    std::vector<double> v_m = {99};
    Vector<double> m(2);
    m.build(i_m, v_m);

    IndexArrayType vect_I({1,3,2});

    BOOST_CHECK_THROW(
        (assign(w, m, grb::NoAccumulate(), 3, vect_I)),
        DimensionException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_constant_test_index_out_of_bounds)
{
    IndexArrayType i_w = {1, 2, 3};
    std::vector<double> v_w  = {1, 2, 3};
    Vector<double> w(4);
    w.build(i_w, v_w);

    IndexArrayType i_a    = {1};
    std::vector<double> v_a = {99};
    Vector<double> u(2);
    u.build(i_a, v_a);

    IndexArrayType vect_I({0,4});

    BOOST_CHECK_THROW(
        (assign(w, grb::NoMask(), grb::NoAccumulate(),
                u, vect_I)),
        IndexOutOfBoundsException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_constant_test_no_accum)
{
    IndexArrayType i_w = {0, 1, 2};
    std::vector<double> v_w = {8, 9, 1   };
    Vector<double> c(4);
    c.build(i_w, v_w);

    IndexArrayType vect_I({1,3});

    IndexArrayType i_result      = {0, 1, 2, 3};
    std::vector<double> v_result = {8,99, 1,99};
    Vector<double> result(4);
    result.build(i_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(), 99, vect_I);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_constant_test_accum)
{
    IndexArrayType i_w = {0, 1, 2};
    std::vector<double> v_w = {8, 9, 1   };
    Vector<double> c(4);
    c.build(i_w, v_w);

    IndexArrayType vect_I({1,3});

    IndexArrayType i_result      = {0,  1,  2,  3};
    std::vector<double> v_result = {8,108,  1, 99};
    Vector<double> result(4);
    result.build(i_result, v_result);

    assign(c, grb::NoMask(), grb::Plus<double>(), 99, vect_I);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_constant_test_allindices_no_accum)
{
    IndexArrayType i_w = {0, 1, 2};
    std::vector<double> v_w = {8, 9, 1   };
    Vector<double> c(4);
    c.build(i_w, v_w);

    IndexArrayType i_result      = {0,  1,  2,  3};
    std::vector<double> v_result = {99, 99, 99, 99};
    Vector<double> result(4);
    result.build(i_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           99, grb::AllIndices());

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_vec_constant_test_allindices_accum)
{
    IndexArrayType i_w = {0, 1, 2};
    std::vector<double> v_w = {8, 9, 1   };
    Vector<double> c(4);
    c.build(i_w, v_w);

    assign(c, grb::NoMask(), grb::Plus<double>(),
           99, grb::AllIndices());

    IndexArrayType i_result      = {0, 1,  2,  3};
    std::vector<double> v_result = {107, 108, 100, 99};
    Vector<double> result(4);
    result.build(i_result, v_result);

    BOOST_CHECK_EQUAL(c, result);
}

/// @todo add tests with masks

//****************************************************************************
// 4.3.7.6 Matrix constant tests
//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_constant_test_bad_dimensions)
{
    IndexArrayType      i_c = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType      j_c = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {1, 2, 3, 4, 6, 7, 8, 9, 1};
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_m    = {0, 0, 1};
    IndexArrayType j_m    = {0, 1, 0};
    std::vector<double> v_m = {1, 99, 99};
    Matrix<double, DirectedMatrixTag> m(2, 2);
    m.build(i_m, j_m, v_m);

    IndexArrayType vect_I({1,0,1});
    IndexArrayType vect_J({1,2});

    BOOST_CHECK_THROW(
        (assign(c, m, grb::NoAccumulate(),
                99, vect_I, vect_J)),
        DimensionException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_constant_test_index_out_of_bounds)
{
    IndexArrayType      i_c = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType      j_c = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {1, 2, 3, 4, 6, 7, 8, 9, 1};
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType vect_I({1,5});
    IndexArrayType vect_J({1,2});

    BOOST_CHECK_THROW(
        (assign(c, grb::NoMask(), grb::NoAccumulate(),
                99, vect_I, vect_J)),
        IndexOutOfBoundsException);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(sparse_assign_const_base)
{
    grb::Matrix<double, grb::DirectedMatrixTag> mC(3, 3);

    std::vector<std::vector<double>> matAnswer = {{0, 0, 0},
                                                  {1, 0, 1},
                                                  {1, 0, 1}};
    grb::Matrix<double, grb::DirectedMatrixTag> answer(matAnswer, 0);

    grb::IndexArrayType row_indices = {2, 1};
    grb::IndexArrayType col_indices = {0, 2};

//    grb::assign_constant(mC,
    grb::assign(mC,
                      grb::NoMask(),
                      grb::NoAccumulate(),
                      1,
                      row_indices,
                      col_indices);

    BOOST_CHECK_EQUAL(mC, answer);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_constant_test_no_accum)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType vect_I({1,2});
    IndexArrayType vect_J({1,2});

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {    1,  2,  3,
                                    4, 99, 99,  7,
                                    8, 99, 99    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           99, vect_I, vect_J);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_constant_test_accum)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType vect_I({1,2});
    IndexArrayType vect_J({1,2});

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {     1,   2, 3,
                                    4,  99, 105, 7,
                                    8, 108, 100   };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::Plus<double>(),
           99, vect_I, vect_J);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_constant_test_allrows_no_accum)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    //IndexArrayType vect_I({1,2});
    IndexArrayType vect_J({0,1});

    IndexArrayType i_result    = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_result    = {0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2};
    std::vector<double> v_result = {99, 99,  2,  3,
                                    99, 99,  6,  7,
                                    99, 99,  1    };
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           99, grb::AllIndices(), vect_J);

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_constant_test_allcols_no_accum)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType vect_I({1,2});
    //IndexArrayType vect_J({0,1});

    IndexArrayType i_result    = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2};
    IndexArrayType j_result    = {1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};
    std::vector<double> v_result = {     1,  2,  3,
                                    99, 99, 99, 99,
                                    99, 99, 99, 99};
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           99, vect_I, grb::AllIndices());

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_constant_test_allrowscols_no_accum)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    IndexArrayType i_result    = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2};
    IndexArrayType j_result    = {0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};
    std::vector<double> v_result = {99, 99, 99, 99,
                                    99, 99, 99, 99,
                                    99, 99, 99, 99};
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    assign(c, grb::NoMask(), grb::NoAccumulate(),
           99, grb::AllIndices(), grb::AllIndices());

    BOOST_CHECK_EQUAL(c, result);
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(assign_mat_constant_test_allrowscols_accum)
{
    IndexArrayType i_c    = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    IndexArrayType j_c    = {1, 2, 3, 0, 2, 3, 0, 1, 2};
    std::vector<double> v_c = {   1, 2, 3,
                               4,    6, 7,
                               8, 9, 1   };
    Matrix<double, DirectedMatrixTag> c(3, 4);
    c.build(i_c, j_c, v_c);

    assign(c, grb::NoMask(), grb::Plus<double>(),
           99, grb::AllIndices(), grb::AllIndices());

    IndexArrayType i_result    = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2};
    IndexArrayType j_result    = {0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};
    std::vector<double> v_result = {99, 100, 101, 102,
                                    103, 99, 105, 106,
                                    107, 108,100,  99};
    Matrix<double, DirectedMatrixTag> result(3, 4);
    result.build(i_result, j_result, v_result);

    BOOST_CHECK_EQUAL(c, result);
}





/// @todo add tests with masks

BOOST_AUTO_TEST_SUITE_END()
