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
 * DM20-0442
 */

#pragma once

#include <type_traits>
#include <graphblas/types.hpp>

namespace grb
{
    //**************************************************************************
    /// @todo: How do we get this into comments?
    // // Sequence::iterator concept
    // class SequenceIteratorConcept
    // {
    // public:
    //     using difference_type = signed IndexType; ???
    //
    //     IndexType operator*() const { return 0; }
    //
    //     const SequenceIteratorConcept &operator++() { return *this; }
    //     SequenceIteratorConcept operator++(int)     { return *this; }
    //
    //     const SequenceIteratorConcept &operator--() { return *this; }
    //     SequenceIteratorConcept operator--(int)     { return *this; }
    //
    //     bool operator==(const SequenceIteratorConcept &other) const { return false; }
    //     bool operator!=(const SequenceIteratorConcept &other) const { return false; }
    //
    //     SequenceIteratorConcept &operator=(const SequenceIteratorConcept &other) { return * this; }
    //
    //     // If we support some of these, we can get slices like
    //     // begin() + size()/4
    //     IndexType operator[](difference_type n) {};
    // };
    //
    // // Sequence concept
    // class SequenceConcept
    // {
    // public:
    //     using difference_type = signed IndexType;???
    //
    //     bool empty() const { return false; }
    //
    //     SequenceIteratorConcept begin() const { return SequenceIteratorConcept(); }
    //     SequenceIteratorConcept end() const { return SequenceIteratorConcept(); }
    //
    //     IndexType size() const { return 0; }
    //
    //     IndexType operator[](difference_type n) const { return 0; }
    //
    //     const IndexType m_begin;
    //     const IndexType m_end;
    // };

    //**************************************************************************
    //**************************************************************************

    // WE ARE NOT A RANDOM_ACCESS_ITERATOR
    // This is an iterator that returns numbers a range without fully realizing
    // that range in memory. It is not mutable.
    class IndexGenerator
    {
        using difference_type = ssize_t;

    public:
        IndexGenerator(IndexType value) : m_value(value)
        {

        }

        IndexGenerator(const IndexGenerator &other)
                : m_value(other.m_value)
        {
        }

        IndexGenerator()
                : m_value(0)
        {
        }

        // reference operator*() const
        // const IndexType &operator*() const
        IndexType operator*() const
        {
            return m_value;
        }

        const IndexGenerator &operator++()
        {
            ++m_value;
            return *this;
        }

        IndexGenerator operator++(int)
        {
            IndexGenerator copy(*this);
            ++(*this);
            return copy;
        }

        const IndexGenerator &operator--()
        {
            --m_value;
            return *this;
        }

        IndexGenerator operator--(int)
        {
            IndexGenerator copy(*this);
            --(*this);
            return copy;
        }

        bool operator==(const IndexGenerator &other) const
        {
            return m_value == other.m_value;
        }

        bool operator!=(const IndexGenerator &other) const
        {
            return m_value != other.m_value;
        }

        IndexGenerator &operator=(const IndexGenerator &other)
        {
            m_value = other.m_value;
            return *this;
        }

        IndexType operator[](difference_type n)
        {
            return m_value + n;
        }

    private:
        IndexType m_value;

    };

    //**************************************************************************

    /**
     * This class is used to provide a "container" for iterators that work
     * over a range without realizing the backing store.
     */
    class IndexSequenceRange
    {
    public:
        using iterator = IndexGenerator;

        IndexSequenceRange(IndexType begin, IndexType end)
                : m_begin(begin), m_end(end)
        {
            // @TODO: Check begin < end
        }

        bool empty() const                  { return m_begin == m_end; }

        IndexGenerator begin() const        { return IndexGenerator(m_begin); }
        IndexGenerator end() const          { return IndexGenerator(m_end); }

        IndexType size() const              { return m_end - m_begin; }

        IndexType operator[](IndexType n) const     { return m_begin + n; }

        const IndexType m_begin;
        const IndexType m_end;
    };

    //**************************************************************************
    //**************************************************************************

    // Special marker classes to support AllIndices iteration.
    // NOTE: This should never be used, as we the AllIndicies never gets invoked.
    class AllIndicesIterator
    {
    public:
        IndexType operator*() const                            { return 0; }

        const AllIndicesIterator &operator++()                 { return *this; }
        AllIndicesIterator operator++(int)                     { return *this; }

        const AllIndicesIterator &operator--()                 { return *this; }
        AllIndicesIterator operator--(int)                     { return *this; }

        bool operator==(const AllIndicesIterator &other) const { return false; }
        bool operator!=(const AllIndicesIterator &other) const { return false; }

        AllIndicesIterator &operator=(const AllIndicesIterator &other)
        {
            return *this;
        }
    };

    //**************************************************************************

    /**
     * Special marker class that can be used to trigger the implementation
     * to use all the indices in source or dest (as is appropriate for the
     * API) when iterating.  For example, when doing an extract, AllIndicies()
     * can be provided instead of explicitly providing a range from 0..N of
     * the respective component.
     */
    class AllIndices
    {
    public:
        using iterator = AllIndicesIterator;

        AllIndices() {}

        bool empty() const                      { return false; }

        AllIndicesIterator begin() const        { return AllIndicesIterator(); }
        AllIndicesIterator end() const          { return AllIndicesIterator(); }

        IndexType size() const                  { return 0; }

        IndexType operator[](IndexType n) const { return 0; }
    };

    //**************************************************************************

    // For logging
    std::ostream &operator<<(std::ostream &os, const AllIndices &allIndices)
    {
        os << "AllIndices";
        return os;
    }

    //**************************************************************************
    //**************************************************************************

    // We can sse these as type traits if we need to switch off all sequence
    template<typename>
    struct is_all_sequence
        : public std::false_type {};

    template<>
    struct is_all_sequence<AllIndices>
        : public std::true_type {};

    // Functions to dynamically see if this is a all sequence.

    template <typename T>
    bool IsAllSequence(T seq)
    {
        return false;
    };

    template <>
    bool IsAllSequence(AllIndices seq)
    {
        return true;
    };


} // namespace grb
