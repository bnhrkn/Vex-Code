// The MIT License (MIT)
//
// Copyright (c) 2018 Mateusz Pusz
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <type_traits>
#include "mp-units/bits/quantity_concepts.h"
#include "mp-units/reference.h"

namespace mp_units {

/**
 * @brief Explicit cast of a quantity type
 *
 * This cast converts only a quantity type. It might be used to force some
 * quantity type conversions that are not implicitly allowed but are allowed
 * explicitly.
 *
 * For example:
 *
 * @code{.cpp}
 * auto length = isq::length(42 * m);
 * auto distance = quantity_cast<isq::distance>(length);
 * @endcode
 *
 * @note This cast does not affect the underlying value of a number stored in a
 * quantity.
 *
 * @tparam ToQS a quantity specification to use for a target quantity
 */
template <QuantitySpec auto ToQS, typename Q>
  requires Quantity<std::remove_cvref_t<Q>> &&
           (castable(Q::quantity_spec, ToQS))
[[nodiscard]] constexpr Quantity auto quantity_cast(Q&& q) {
  if constexpr (detail::QuantityKindSpec<std::remove_const_t<decltype(ToQS)>> &&
                AssociatedUnit<std::remove_const_t<decltype(Q::unit)>>)
    return quantity{
        std::forward<Q>(q).numerical_value_is_an_implementation_detail_,
        Q::unit};
  else
    return quantity{
        std::forward<Q>(q).numerical_value_is_an_implementation_detail_,
        reference<ToQS, Q::unit>{}};
}

}  // namespace mp_units
