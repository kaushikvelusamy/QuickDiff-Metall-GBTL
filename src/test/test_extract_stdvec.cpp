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

#define GRAPHBLAS_LOGGING_LEVEL 0

#include <graphblas/graphblas.hpp>

using namespace grb;

#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE extract_stdvec_test_suite

#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

//****************************************************************************
// extract standard vector tests
//****************************************************************************

//****************************************************************************
BOOST_AUTO_TEST_CASE(extract_stdvec_test_bad_dimensions)
{
    std::vector<double> vecU = {1,2,3,4,5,6};
    Vector<double> vU(vecU);

    std::vector<double> vecAnswer = {1,3,5};
    Vector<double> answer(vecAnswer);

    // Output space
    IndexType M = 3;

    std::vector<bool> vecFullMask = {true, true, true};
    Vector<bool> mask(vecFullMask);

    IndexArrayType vec_indices = {0, 2, 1};
    IndexArrayType indices(vec_indices);

    Vector<double> result(M);

    // =======
    // Check w.size < mask.size
    {
        Vector<double> bad_result(M-1);

        BOOST_CHECK_THROW(
            (extract(bad_result,
                     mask,
                     NoAccumulate(),
                     vU,
                     indices)),
            DimensionException);
    }

    // Check w.size > mask.size
    {
        Vector<double> bad_result(M+1);

        BOOST_CHECK_THROW(
            (extract(bad_result,
                     mask,
                     NoAccumulate(),
                     vU,
                     indices)),
            DimensionException);
    }

    // =======

    // Also check nindices mismatch
    {
        IndexArrayType bad_vec_indices = {2, 1};
        IndexArrayType bad_indices(bad_vec_indices);

        BOOST_CHECK_THROW(
            (extract(result,
                     NoMask(),
                     NoAccumulate(),
                     vU,
                     bad_indices)),
            DimensionException);
    }
    {
        IndexArrayType bad_vec_indices = {2, 2, 1, 0};
        IndexArrayType bad_indices(bad_vec_indices);

        BOOST_CHECK_THROW(
            (extract(result,
                     NoMask(),
                     NoAccumulate(),
                     vU,
                     bad_indices)),
            DimensionException);
    }
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(extract_stdvec_test_index_out_of_bounds)
{
    std::vector<double> vecU = {1,2,3,4,5,6};
    Vector<double> vU(vecU);

    std::vector<double> vecAnswer = {1,3,5};
    Vector<double> answer(vecAnswer);

    // Output space
    IndexType M = 3;

    std::vector<bool> vecFullMask = {true, true, true};
    Vector<bool> mask(vecFullMask);

    IndexArrayType vec_indices = {0, 7, 1};
    IndexArrayType indices(vec_indices);

    Vector<double> result(M);

    // =======
    // Check w.size < mask.size
    {
        BOOST_CHECK_THROW(
            (extract(result,
                     mask,
                     NoAccumulate(),
                     vU,
                     indices)),
            IndexOutOfBoundsException);

        BOOST_CHECK_THROW(
            (extract(result,
                     NoMask(),
                     NoAccumulate(),
                     vU,
                     indices)),
            IndexOutOfBoundsException);
    }
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(extract_stdvec_test_allindices_too_small)
{
    std::vector<double> vecU = {4, 0, 2, 0};
    Vector<double> vU(vecU, 0);

    // result is too small when I = AllIndices will pass
    {
        std::vector<double> vecAnswer = {4, 0, 2};
        Vector<double> answer(vecAnswer, 0);

        // Output rank
        IndexType M = 3;
        Vector<double> result(M);

        extract(result,
                NoMask(),
                NoAccumulate(),
                vU,
                AllIndices());

        BOOST_CHECK_EQUAL(result, answer);
    }

    // result is too large when I = AllIndices should also pass
    {
        std::vector<double> vecAnswer = {4, 0, 2, 0, 0, 0};
        Vector<double> answer(vecAnswer, 0);

        // Output rank
        IndexType M = 6;
        Vector<double> result(M);

        extract(result,
                NoMask(),
                NoAccumulate(),
                vU,
                AllIndices());

        BOOST_CHECK_EQUAL(result, answer);
    }
}


//****************************************************************************
// Standard vector passing test cases:
//
// Simplified test structure (12 test cases total):
//
// Mask cases (x3):  nomask,  noscmp, scmp
// Accum cases (x2): noaccum, accum
// Trans cases (x2): notrans, trans
//
// Within each test case: 5 checks will be run:
//
// indices: AllIndices, nodup/ordered, nodup/permute, dup/ordered, dup/permute
//****************************************************************************

//****************************************************************************
BOOST_AUTO_TEST_CASE(extract_stdvec_test_nomask_noaccum)
{
    std::vector<double> vecU = {4, 0, 2, 0};
    Vector<double> vU(vecU, 0);

    // I - AllIndices
    {
        std::vector<double> vecAnswer = {4, 0, 2, 0};
        Vector<double> answer(vecAnswer, 0);

        // Output rank
        IndexType M = 4;
        Vector<double> result(M);

        extract(result, NoMask(), NoAccumulate(),
                vU, AllIndices());

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - no dup, ordered
    {
        IndexArrayType row_indices = {0, 2};

        std::vector<double> vecAnswer = {4, 2};
        Vector<double> answer(vecAnswer, 0);

        // Output rank
        IndexType M = 2;
        Vector<double> result(M);

        extract(result, NoMask(), NoAccumulate(),
                vU, row_indices);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - no dup, permuted
    {
        IndexArrayType row_indices = {2, 0};

        std::vector<double> vecAnswer = {2, 4};
        Vector<double> answer(vecAnswer, 0);

        // Output rank
        IndexType M = 2;
        Vector<double> result(M);

        extract(result, NoMask(), NoAccumulate(),
                vU, row_indices);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - dup, ordered
    {
        IndexArrayType row_indices = {0, 1, 1, 2};

        std::vector<double> vecAnswer = {4, 0, 0, 2};
        Vector<double> answer(vecAnswer, 0);

        // Output rank
        IndexType M = 4;
        Vector<double> result(M);

        extract(result, NoMask(), NoAccumulate(),
                vU, row_indices);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - dup, permuted
    {
        IndexArrayType row_indices = {0, 2, 2, 1};

        std::vector<double> vecAnswer = {4, 2, 2, 0};
        Vector<double> answer(vecAnswer, 0);

        // Output rank
        IndexType M = 4;
        Vector<double> result(M);

        extract(result, NoMask(), NoAccumulate(),
                vU, row_indices);

        BOOST_CHECK_EQUAL(result, answer);
    }
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(extract_stdvec_test_nomask_accum)
{
    std::vector<double> vecU = {4, 0, 2, 0};
    Vector<double> vU(vecU);

    // I - AllIndices
    {
        std::vector<double> vecAnswer = {13, 9, 11, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, NoMask(), Plus<double>(),
                vU, AllIndices());

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - no dup, ordered
    {
        IndexArrayType row_indices = {0, 3};

        std::vector<double> vecAnswer = {13, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9};
        Vector<double> result(res, 0.);

        extract(result, NoMask(), Plus<double>(),
                vU, row_indices);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - no dup, permuted
    {
        IndexArrayType row_indices = {3, 0};

        std::vector<double> vecAnswer = {9, 13};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9};
        Vector<double> result(res, 0.);

        extract(result, NoMask(), Plus<double>(),
                vU, row_indices);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - dup, ordered
    {
        IndexArrayType row_indices = {0, 1, 1, 2};

        std::vector<double> vecAnswer = {13, 9, 9, 11};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, NoMask(), Plus<double>(),
                vU, row_indices);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - dup, permuted
    {
        IndexArrayType row_indices = {0, 2, 2, 1};

        std::vector<double> vecAnswer = {13, 11, 11, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, NoMask(), Plus<double>(),
                vU, row_indices);

        BOOST_CHECK_EQUAL(result, answer);
    }
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(extract_stdvec_test_noscmp_noaccum)
{
    std::vector<double> vecU = {4, 0, 2, 0};
    Vector<double> vU(vecU, 0);

    std::vector<uint8_t> vecm3 = {1, 0, 99};    // one stored zero:
    Vector<uint8_t> mask3(vecm3, 99);           // [1, 0, -]
    std::vector<uint8_t> vecm4 = {1, 0, 99, 1}; // one stored zero:
    Vector<uint8_t> mask4(vecm4, 99);           // [1, 0, -, 1]

    // I - AllIndices
    {
        std::vector<double> vecAnswer = {4, 0, 0, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask4, NoAccumulate(),
                vU, AllIndices(), REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    // I - AllIndices
    {
        std::vector<double> vecAnswer = {4, 9, 9, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask4, NoAccumulate(),
                vU, AllIndices(), MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - no dup, ordered
    {
        IndexArrayType row_indices = {0, 2, 3};

        std::vector<double> vecAnswer = {4, 0, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask3, NoAccumulate(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    // I - no dup, ordered
    {
        IndexArrayType row_indices = {0, 2, 3};

        std::vector<double> vecAnswer = {4, 9, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask3, NoAccumulate(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - no dup, permuted
    {
        IndexArrayType row_indices = {2, 3, 0};

        std::vector<double> vecAnswer = {2, 0, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask3, NoAccumulate(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {2, 3, 0};

        std::vector<double> vecAnswer = {2, 9, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask3, NoAccumulate(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - dup, ordered
    {
        IndexArrayType row_indices = {0, 0, 2, 3};

        std::vector<double> vecAnswer = {4, 0, 0, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask4, NoAccumulate(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {0, 0, 2, 3};

        std::vector<double> vecAnswer = {4, 9, 9, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask4, NoAccumulate(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - dup, permuted
    {
        IndexArrayType row_indices = {2, 3, 0, 0};

        std::vector<double> vecAnswer = {2, 0, 0, 4};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask4, NoAccumulate(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {2, 3, 0, 0};

        std::vector<double> vecAnswer = {2, 9, 9, 4};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask4, NoAccumulate(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(extract_stdvec_test_noscmp_accum)
{
    std::vector<double> vecU = {4, 0, 2, 0};
    Vector<double> vU(vecU, 0);

    std::vector<uint8_t> vecm3 = {1, 0, 99};    // one stored zero:
    Vector<uint8_t> mask3(vecm3, 99);           // [1, 0, -]
    std::vector<uint8_t> vecm4 = {1, 0, 99, 1}; // one stored zero:
    Vector<uint8_t> mask4(vecm4, 99);           // [1, 0, -, 1]

    // I - AllIndices
    {
        std::vector<double> vecAnswer = {13, 0, 0, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask4, Plus<double>(),
                vU, AllIndices(), REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        std::vector<double> vecAnswer = {13, 9, 9, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask4, Plus<double>(),
                vU, AllIndices(), MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - no dup, ordered
    {
        IndexArrayType row_indices = {0, 2, 3};

        std::vector<double> vecAnswer = {13, 0, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask3, Plus<double>(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {0, 2, 3};

        std::vector<double> vecAnswer = {13, 9, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask3, Plus<double>(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - no dup, permuted
    {
        IndexArrayType row_indices = {2, 3, 0};

        std::vector<double> vecAnswer = {11, 0, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask3, Plus<double>(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {2, 3, 0};

        std::vector<double> vecAnswer = {11, 9, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask3, Plus<double>(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - dup, ordered
    {
        IndexArrayType row_indices = {0, 0, 2, 3};

        std::vector<double> vecAnswer = {13, 0, 0, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask4, Plus<double>(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {0, 0, 2, 3};

        std::vector<double> vecAnswer = {13, 9, 9, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask4, Plus<double>(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - dup, permuted
    {
        IndexArrayType row_indices = {2, 3, 0, 0};

        std::vector<double> vecAnswer = {11, 0, 0, 13};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask4, Plus<double>(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {2, 3, 0, 0};

        std::vector<double> vecAnswer = {11, 9, 9, 13};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, mask4, Plus<double>(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(extract_stdvec_test_scmp_noaccum)
{
    std::vector<double> vecU = {4, 0, 2, 0};
    Vector<double> vU(vecU, 0);

    std::vector<uint8_t> vecm3 = {1, 0, 99};    // one stored zero:
    Vector<uint8_t> mask3(vecm3, 99);           // [1, 0, -]
    std::vector<uint8_t> vecm4 = {1, 0, 99, 1}; // one stored zero:
    Vector<uint8_t> mask4(vecm4, 99);           // [1, 0, -, 1]

    // I - AllIndices
    {
        std::vector<double> vecAnswer = {0, 0, 2, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);
        extract(result, complement(mask4), NoAccumulate(),
                vU, AllIndices(), REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        std::vector<double> vecAnswer = {9, 0, 2, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);
        extract(result, complement(mask4), NoAccumulate(),
                vU, AllIndices(), MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - no dup, ordered
    {
        IndexArrayType row_indices = {0, 2, 3};

        std::vector<double> vecAnswer = {0, 2, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);
        extract(result, complement(mask3), NoAccumulate(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {0, 2, 3};

        std::vector<double> vecAnswer = {9, 2, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);
        extract(result, complement(mask3), NoAccumulate(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - no dup, permuted
    {
        IndexArrayType row_indices = {2, 3, 0};

        std::vector<double> vecAnswer = {0, 0, 4};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);
        extract(result, complement(mask3), NoAccumulate(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {2, 3, 0};

        std::vector<double> vecAnswer = {9, 0, 4};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);
        extract(result, complement(mask3), NoAccumulate(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - dup, ordered
    {
        IndexArrayType row_indices = {0, 0, 2, 3};

        std::vector<double> vecAnswer = {0, 4, 2, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);
        extract(result, complement(mask4), NoAccumulate(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {0, 0, 2, 3};

        std::vector<double> vecAnswer = {9, 4, 2, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);
        extract(result, complement(mask4), NoAccumulate(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    // I - dup, permuted
    {
        IndexArrayType row_indices = {2, 3, 0, 0};

        std::vector<double> vecAnswer = {0, 0, 4, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);
        extract(result, complement(mask4), NoAccumulate(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {2, 3, 0, 0};

        std::vector<double> vecAnswer = {9, 0, 4, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);
        extract(result, complement(mask4), NoAccumulate(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }
}

//****************************************************************************
BOOST_AUTO_TEST_CASE(extract_stdvec_test_scmp_accum)
{
    std::vector<double> vecU = {4, 0, 2, 0};
    Vector<double> vU(vecU, 0);

    std::vector<uint8_t> vecm3 = {1, 0, 99};    // one stored zero:
    Vector<uint8_t> mask3(vecm3, 99);           // [1, 0, -]
    std::vector<uint8_t> vecm4 = {1, 0, 99, 1}; // one stored zero:
    Vector<uint8_t> mask4(vecm4, 99);           // [1, 0, -, 1]

    // I - AllIndices
    {
        std::vector<double> vecAnswer = {0, 9, 11, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, complement(mask4), Plus<double>(),
                vU, AllIndices(), REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        std::vector<double> vecAnswer = {9, 9, 11, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, complement(mask4), Plus<double>(),
                vU, AllIndices(), MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - no dup, ordered
    {
        IndexArrayType row_indices = {0, 2, 3};

        std::vector<double> vecAnswer = {0, 11, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, complement(mask3), Plus<double>(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {0, 2, 3};

        std::vector<double> vecAnswer = {9, 11, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, complement(mask3), Plus<double>(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - no dup, permuted
    {
        IndexArrayType row_indices = {2, 3, 0};

        std::vector<double> vecAnswer = {0, 9, 13};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, complement(mask3), Plus<double>(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {2, 3, 0};

        std::vector<double> vecAnswer = {9, 9, 13};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, complement(mask3), Plus<double>(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - dup, ordered
    {
        IndexArrayType row_indices = {0, 0, 2, 3};

        std::vector<double> vecAnswer = {0, 13, 11, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, complement(mask4), Plus<double>(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {0, 0, 2, 3};

        std::vector<double> vecAnswer = {9, 13, 11, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, complement(mask4), Plus<double>(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }

    // I - dup, permuted
    {
        IndexArrayType row_indices = {2, 3, 0, 0};

        std::vector<double> vecAnswer = {0, 9, 13, 0};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, complement(mask4), Plus<double>(),
                vU, row_indices, REPLACE);

        BOOST_CHECK_EQUAL(result, answer);
    }
    {
        IndexArrayType row_indices = {2, 3, 0, 0};

        std::vector<double> vecAnswer = {9, 9, 13, 9};
        Vector<double> answer(vecAnswer, 0);

        std::vector<double> res = {9, 9, 9, 9};
        Vector<double> result(res, 0.);

        extract(result, complement(mask4), Plus<double>(),
                vU, row_indices, MERGE);

        BOOST_CHECK_EQUAL(result, answer);
    }
}


BOOST_AUTO_TEST_SUITE_END()
