/**
 * This file is part of Simox.
 *
 * Simox is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Simox is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    VirtualRobot
 * @author     Stefan Ulbrich
 * @copyright  Stefan Ulbrich
 *             GNU Lesser General Public License
 * @brief Defines iterators for the COLLADA container classes in order to use boost.foreach, boost::copy etc.
 */

#ifndef __NONINVASIVE_RANDOM_ACCESS_ITERATORS_H__
#define __NONINVASIVE_RANDOM_ACCESS_ITERATORS_H__

#include <boost/iterator/iterator_facade.hpp>

//namespace VirtualRobot {

template <class T>
class daeTArray_iterator : public boost::iterator_facade< daeTArray_iterator<T>, T, boost::forward_traversal_tag >
{
public:
    daeTArray_iterator()
        : array(0), index(-1)
    {
        assert(0);
    }

    explicit daeTArray_iterator(daeTArray<T>* _array, unsigned int _index)
        : array(_array), index(_index) {}

private:
    friend class boost::iterator_core_access;

    void increment()
    {
        this->index++;
    }

    bool equal(daeTArray_iterator const& other) const
    {
        if (this->array->getCount() == 0)
        {
            return true;
        }

        return this->index == other.index;
    }

    T& dereference() const
    {
        return this->array->get(this->index);
    }

    daeTArray<T>* array;
    unsigned int index;
};

template <class T>
class daeTArray_const_iterator : public boost::iterator_facade< daeTArray_const_iterator<T>, T const, boost::forward_traversal_tag >
{
public:
    daeTArray_const_iterator()
        : array(0), index(-1)
    {
        assert(0);
    }

    explicit daeTArray_const_iterator(const daeTArray<T>* _array, unsigned int _index)
        : array(_array), index(_index) {}

private:
    friend class boost::iterator_core_access;

    void increment()
    {
        this->index++;
    }

    bool equal(daeTArray_const_iterator const& other) const
    {
        if (this->array->getCount() == 0)
        {
            return true;
        }

        return this->index == other.index;
    }

    T const& dereference() const
    {
        return this->array->get(this->index);
    }

    daeTArray<T> const* array;
    unsigned int index;
};

template <class T>
inline daeTArray_iterator<T> range_begin(daeTArray<T>& x)
{
    return daeTArray_iterator<T>(&x, 0) ;
}

template <class T>
inline daeTArray_iterator<T>  range_end(daeTArray<T>& x)
{
    return daeTArray_iterator<T>(&x, x.getCount()) ;
}

template <class T>
inline daeTArray_const_iterator<T> range_begin(daeTArray<T> const& x)
{
    return daeTArray_const_iterator<T>(&x, 0) ;
}

template <class T>
inline daeTArray_const_iterator<T>  range_end(daeTArray<T> const& x)
{
    return daeTArray_const_iterator<T>(&x, x.getCount()) ;
}

//}

namespace boost
{
    // specialize range_mutable_iterator and range_const_iterator in namespace boost
    template <class T>
    struct range_mutable_iterator< daeTArray<T> >
    {
        typedef /*VirtualRobot::*/daeTArray_iterator<T> type;
    };

    template<class T>
    struct range_const_iterator< daeTArray<T> >
    {
        typedef /*VirtualRobot::*/daeTArray_const_iterator<T> type;
    };

}

#endif
