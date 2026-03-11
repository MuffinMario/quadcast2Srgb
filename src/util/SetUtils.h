#pragma once

#include "../Common.h"
#include <algorithm>

template <typename TKey>
Set<TKey> SetDifference(const Set<TKey> &p_setA, const Set<TKey> &p_setB)
{
    Set<TKey> result;
    // A - B
    std::set_difference(p_setA.begin(), p_setA.end(),
                        p_setB.begin(), p_setB.end(),
                        std::inserter(result, result.end()));
    return result;
}
