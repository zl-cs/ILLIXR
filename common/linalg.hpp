// linalg.h - v2.0 - Single-header public domain linear algebra library
//
// The intent of this library is to provide the bulk of the functionality
// you need to write programs that frequently use small, fixed-size vectors
// and matrices, in domains such as computational geometry or computer
// graphics. It strives for terse, readable source code.
//
// The original author of this software is Sterling Orsten, and its permanent
// home is <http://github.com/sgorsten/linalg/>. If you find this software
// useful, an acknowledgement in your source text and/or product documentation
// is appreciated, but not required.
//
// The author acknowledges significant insights and contributions by:
//     Stan Melax <http://github.com/melax/>
//     Dimitri Diakopoulos <http://github.com/ddiakopoulos/>


// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.
//
// In jurisdictions that recognize copyright laws, the author or authors
// of this software dedicate any and all copyright interest in the
// software to the public domain. We make this dedication for the benefit
// of the public at large and to the detriment of our heirs and
// successors. We intend this dedication to be an overt act of
// relinquishment in perpetuity of all present and future rights to this
// software under copyright law.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
// OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// For more information, please refer to <http://unlicense.org/>


#pragma once
#ifndef LINALG_H
#define LINALG_H

#include <array> // For std::array
#include <cmath> // For various unary math functions, such as std::sqrt
#include <cstdint> // For implementing namespace linalg::aliases
#include <cstdlib> // To resolve std::abs ambiguity on clang
#include <iosfwd> // For forward definitions of std::ostream
#include <type_traits> // For std::enable_if, std::is_same, std::declval

// In Visual Studio 2015, `constexpr` applied to a member function implies `const`, which causes ambiguous overload resolution
#if _MSC_VER <= 1900
#define LINALG_CONSTEXPR14
#else
#define LINALG_CONSTEXPR14 constexpr
#endif

namespace linalg
{
// Small, fixed-length vector type, consisting of exactly M elements of type T, and presumed to be a column-vector unless
// otherwise noted.
template <class T, int M> struct vec;

// Small, fixed-size matrix type, consisting of exactly M rows and N columns of type T, stored in column-major order.
template <class T, int M, int N> struct mat;

// Specialize converter<T,U> with a function application operator that converts type U to type T to enable implicit conversions
template <class T, class U> struct converter
{
};
namespace detail
{
    template <class T, class U>
    using conv_t = typename std::enable_if<!std::is_same<T, U>::value, decltype(converter<T, U> {}(std::declval<U>()))>::type;

    // Trait for retrieving scalar type of any linear algebra object
    template <class A> struct scalar_type
    {
    };
    template <class T, int M> struct scalar_type<vec<T, M>>
    {
        using type = T;
    };
    template <class T, int M, int N> struct scalar_type<mat<T, M, N>>
    {
        using type = T;
    };

    // Type returned by the compare(...) function which supports all six comparison operators against 0
    template <class T> struct ord
    {
        T a, b;
    };
    template <class T> constexpr auto operator==(const ord<T>& o, std::nullptr_t) -> bool { return o.a == o.b; }
    template <class T> constexpr auto operator!=(const ord<T>& o, std::nullptr_t) -> bool { return !(o.a == o.b); }
    template <class T> constexpr auto operator<(const ord<T>& o, std::nullptr_t) -> bool { return o.a < o.b; }
    template <class T> constexpr auto operator>(const ord<T>& o, std::nullptr_t) -> bool { return o.b < o.a; }
    template <class T> constexpr auto operator<=(const ord<T>& o, std::nullptr_t) -> bool { return !(o.b < o.a); }
    template <class T> constexpr auto operator>=(const ord<T>& o, std::nullptr_t) -> bool { return !(o.a < o.b); }

    // Patterns which can be used with the compare(...) function
    template <class A, class B> struct any_compare
    {
    };
    template <class T> struct any_compare<vec<T, 1>, vec<T, 1>>
    {
        using type = ord<T>;
        constexpr auto operator()(const vec<T, 1>& a, const vec<T, 1>& b) const -> ord<T> { return ord<T> { a.x, b.x }; }
    };
    template <class T> struct any_compare<vec<T, 2>, vec<T, 2>>
    {
        using type = ord<T>;
        constexpr auto operator()(const vec<T, 2>& a, const vec<T, 2>& b) const -> ord<T>
        {
            return !(a.x == b.x) ? ord<T> { a.x, b.x } : ord<T> { a.y, b.y };
        }
    };
    template <class T> struct any_compare<vec<T, 3>, vec<T, 3>>
    {
        using type = ord<T>;
        constexpr auto operator()(const vec<T, 3>& a, const vec<T, 3>& b) const -> ord<T>
        {
            return !(a.x == b.x) ? ord<T> { a.x, b.x } : !(a.y == b.y) ? ord<T> { a.y, b.y } : ord<T> { a.z, b.z };
        }
    };
    template <class T> struct any_compare<vec<T, 4>, vec<T, 4>>
    {
        using type = ord<T>;
        constexpr auto operator()(const vec<T, 4>& a, const vec<T, 4>& b) const -> ord<T>
        {
            return !(a.x == b.x)
                ? ord<T> { a.x, b.x }
                : !(a.y == b.y) ? ord<T> { a.y, b.y } : !(a.z == b.z) ? ord<T> { a.z, b.z } : ord<T> { a.w, b.w };
        }
    };
    template <class T, int M> struct any_compare<mat<T, M, 1>, mat<T, M, 1>>
    {
        using type = ord<T>;
        constexpr auto operator()(const mat<T, M, 1>& a, const mat<T, M, 1>& b) const -> ord<T> { return compare(a.x, b.x); }
    };
    template <class T, int M> struct any_compare<mat<T, M, 2>, mat<T, M, 2>>
    {
        using type = ord<T>;
        constexpr auto operator()(const mat<T, M, 2>& a, const mat<T, M, 2>& b) const -> ord<T>
        {
            return a.x != b.x ? compare(a.x, b.x) : compare(a.y, b.y);
        }
    };
    template <class T, int M> struct any_compare<mat<T, M, 3>, mat<T, M, 3>>
    {
        using type = ord<T>;
        constexpr auto operator()(const mat<T, M, 3>& a, const mat<T, M, 3>& b) const -> ord<T>
        {
            return a.x != b.x ? compare(a.x, b.x) : a.y != b.y ? compare(a.y, b.y) : compare(a.z, b.z);
        }
    };
    template <class T, int M> struct any_compare<mat<T, M, 4>, mat<T, M, 4>>
    {
        using type = ord<T>;
        constexpr auto operator()(const mat<T, M, 4>& a, const mat<T, M, 4>& b) const -> ord<T>
        {
            return a.x != b.x ? compare(a.x, b.x)
                              : a.y != b.y ? compare(a.y, b.y) : a.z != b.z ? compare(a.z, b.z) : compare(a.w, b.w);
        }
    };

    // Helper for compile-time index-based access to members of vector and matrix types
    template <int I> struct getter;
    template <> struct getter<0>
    {
        template <class A> constexpr auto operator()(A& a) const -> decltype(a.x) { return a.x; }
    };
    template <> struct getter<1>
    {
        template <class A> constexpr auto operator()(A& a) const -> decltype(a.y) { return a.y; }
    };
    template <> struct getter<2>
    {
        template <class A> constexpr auto operator()(A& a) const -> decltype(a.z) { return a.z; }
    };
    template <> struct getter<3>
    {
        template <class A> constexpr auto operator()(A& a) const -> decltype(a.w) { return a.w; }
    };

    // Stand-in for std::integer_sequence/std::make_integer_sequence
    template <int... I> struct seq
    {
    };
    template <int A, int N> struct make_seq_impl;
    template <int A> struct make_seq_impl<A, 0>
    {
        using type = seq<>;
    };
    template <int A> struct make_seq_impl<A, 1>
    {
        using type = seq<A + 0>;
    };
    template <int A> struct make_seq_impl<A, 2>
    {
        using type = seq<A + 0, A + 1>;
    };
    template <int A> struct make_seq_impl<A, 3>
    {
        using type = seq<A + 0, A + 1, A + 2>;
    };
    template <int A> struct make_seq_impl<A, 4>
    {
        using type = seq<A + 0, A + 1, A + 2, A + 3>;
    };
    template <int A, int B> using make_seq = typename make_seq_impl<A, B - A>::type;
    template <class T, int M, int... I> auto constexpr swizzle(const vec<T, M>& v, seq<I...> /*unused*/) -> vec<T, sizeof...(I)>
    {
        return { getter<I> {}(v)... };
    }
    template <class T, int M, int N, int... I, int... J>
    auto constexpr swizzle(const mat<T, M, N>& m, seq<I...> i, seq<J...> /*unused*/) -> mat<T, sizeof...(I), sizeof...(J)>
    {
        return { swizzle(getter<J> {}(m), i)... };
    }

    // SFINAE helpers to determine result of function application
    template <class F, class... T> using ret_t = decltype(std::declval<F>()(std::declval<T>()...));

    // SFINAE helper which is defined if all provided types are scalars
    struct empty
    {
    };
    template <class... T> struct scalars;
    template <> struct scalars<>
    {
        using type = void;
    };
    template <class T, class... U>
    struct scalars<T, U...> : std::conditional<std::is_arithmetic<T>::value, scalars<U...>, empty>::type
    {
    };
    template <class... T> using scalars_t = typename scalars<T...>::type;

    // Helpers which indicate how apply(F, ...) should be called for various arguments
    template <class F, class Void, class... T> struct apply
    {
    }; // Patterns which contain only vectors or scalars
    template <class F, int M, class A> struct apply<F, scalars_t<>, vec<A, M>>
    {
        using type = vec<ret_t<F, A>, M>;
        enum { size = M };
        template <int... I> static constexpr auto impl(seq<I...> /*unused*/, F f, const vec<A, M>& a) -> type
        {
            return { f(getter<I> {}(a))... };
        }
    };
    template <class F, int M, class A, class B> struct apply<F, scalars_t<>, vec<A, M>, vec<B, M>>
    {
        using type = vec<ret_t<F, A, B>, M>;
        enum { size = M };
        template <int... I>
        static constexpr auto impl(seq<I...> /*unused*/, F f, const vec<A, M>& a, const vec<B, M>& b) -> type
        {
            return { f(getter<I> {}(a), getter<I> {}(b))... };
        }
    };
    template <class F, int M, class A, class B> struct apply<F, scalars_t<B>, vec<A, M>, B>
    {
        using type = vec<ret_t<F, A, B>, M>;
        enum { size = M };
        template <int... I> static constexpr auto impl(seq<I...> /*unused*/, F f, const vec<A, M>& a, B b) -> type
        {
            return { f(getter<I> {}(a), b)... };
        }
    };
    template <class F, int M, class A, class B> struct apply<F, scalars_t<A>, A, vec<B, M>>
    {
        using type = vec<ret_t<F, A, B>, M>;
        enum { size = M };
        template <int... I> static constexpr auto impl(seq<I...> /*unused*/, F f, A a, const vec<B, M>& b) -> type
        {
            return { f(a, getter<I> {}(b))... };
        }
    };
    template <class F, int M, class A, class B, class C> struct apply<F, scalars_t<>, vec<A, M>, vec<B, M>, vec<C, M>>
    {
        using type = vec<ret_t<F, A, B, C>, M>;
        enum { size = M };
        template <int... I>
        static constexpr auto impl(seq<I...> /*unused*/, F f, const vec<A, M>& a, const vec<B, M>& b, const vec<C, M>& c)
            -> type
        {
            return { f(getter<I> {}(a), getter<I> {}(b), getter<I> {}(c))... };
        }
    };
    template <class F, int M, class A, class B, class C> struct apply<F, scalars_t<C>, vec<A, M>, vec<B, M>, C>
    {
        using type = vec<ret_t<F, A, B, C>, M>;
        enum { size = M };
        template <int... I>
        static constexpr auto impl(seq<I...> /*unused*/, F f, const vec<A, M>& a, const vec<B, M>& b, C c) -> type
        {
            return { f(getter<I> {}(a), getter<I> {}(b), c)... };
        }
    };
    template <class F, int M, class A, class B, class C> struct apply<F, scalars_t<B>, vec<A, M>, B, vec<C, M>>
    {
        using type = vec<ret_t<F, A, B, C>, M>;
        enum { size = M };
        template <int... I>
        static constexpr auto impl(seq<I...> /*unused*/, F f, const vec<A, M>& a, B b, const vec<C, M>& c) -> type
        {
            return { f(getter<I> {}(a), b, getter<I> {}(c))... };
        }
    };
    template <class F, int M, class A, class B, class C> struct apply<F, scalars_t<B, C>, vec<A, M>, B, C>
    {
        using type = vec<ret_t<F, A, B, C>, M>;
        enum { size = M };
        template <int... I> static constexpr auto impl(seq<I...> /*unused*/, F f, const vec<A, M>& a, B b, C c) -> type
        {
            return { f(getter<I> {}(a), b, c)... };
        }
    };
    template <class F, int M, class A, class B, class C> struct apply<F, scalars_t<A>, A, vec<B, M>, vec<C, M>>
    {
        using type = vec<ret_t<F, A, B, C>, M>;
        enum { size = M };
        template <int... I>
        static constexpr auto impl(seq<I...> /*unused*/, F f, A a, const vec<B, M>& b, const vec<C, M>& c) -> type
        {
            return { f(a, getter<I> {}(b), getter<I> {}(c))... };
        }
    };
    template <class F, int M, class A, class B, class C> struct apply<F, scalars_t<A, C>, A, vec<B, M>, C>
    {
        using type = vec<ret_t<F, A, B, C>, M>;
        enum { size = M };
        template <int... I> static constexpr auto impl(seq<I...> /*unused*/, F f, A a, const vec<B, M>& b, C c) -> type
        {
            return { f(a, getter<I> {}(b), c)... };
        }
    };
    template <class F, int M, class A, class B, class C> struct apply<F, scalars_t<A, B>, A, B, vec<C, M>>
    {
        using type = vec<ret_t<F, A, B, C>, M>;
        enum { size = M };
        template <int... I> static constexpr auto impl(seq<I...> /*unused*/, F f, A a, B b, const vec<C, M>& c) -> type
        {
            return { f(a, b, getter<I> {}(c))... };
        }
    };
    template <class F, int M, int N, class A> struct apply<F, scalars_t<>, mat<A, M, N>>
    {
        using type = mat<ret_t<F, A>, M, N>;
        enum { size = N };
        template <int... J> static constexpr auto impl(seq<J...> /*unused*/, F f, const mat<A, M, N>& a) -> type
        {
            return { apply<F, void, vec<A, M>>::impl(make_seq<0, M> {}, f, getter<J> {}(a))... };
        }
    };
    template <class F, int M, int N, class A, class B> struct apply<F, scalars_t<>, mat<A, M, N>, mat<B, M, N>>
    {
        using type = mat<ret_t<F, A, B>, M, N>;
        enum { size = N };
        template <int... J>
        static constexpr auto impl(seq<J...> /*unused*/, F f, const mat<A, M, N>& a, const mat<B, M, N>& b) -> type
        {
            return { apply<F, void, vec<A, M>, vec<B, M>>::impl(make_seq<0, M> {}, f, getter<J> {}(a), getter<J> {}(b))... };
        }
    };
    template <class F, int M, int N, class A, class B> struct apply<F, scalars_t<B>, mat<A, M, N>, B>
    {
        using type = mat<ret_t<F, A, B>, M, N>;
        enum { size = N };
        template <int... J> static constexpr auto impl(seq<J...> /*unused*/, F f, const mat<A, M, N>& a, B b) -> type
        {
            return { apply<F, void, vec<A, M>, B>::impl(make_seq<0, M> {}, f, getter<J> {}(a), b)... };
        }
    };
    template <class F, int M, int N, class A, class B> struct apply<F, scalars_t<A>, A, mat<B, M, N>>
    {
        using type = mat<ret_t<F, A, B>, M, N>;
        enum { size = N };
        template <int... J> static constexpr auto impl(seq<J...> /*unused*/, F f, A a, const mat<B, M, N>& b) -> type
        {
            return { apply<F, void, A, vec<B, M>>::impl(make_seq<0, M> {}, f, a, getter<J> {}(b))... };
        }
    };
    template <class F, class... A> struct apply<F, scalars_t<A...>, A...>
    {
        using type = ret_t<F, A...>;
        enum { size = 0 };
        static constexpr auto impl(seq<> /*unused*/, F f, A... a) -> type { return f(a...); }
    };

    // Function objects for selecting between alternatives
    struct min
    {
        template <class A, class B>
        constexpr auto operator()(A a, B b) const -> typename std::remove_reference<decltype(a < b ? a : b)>::type
        {
            return a < b ? a : b;
        }
    };
    struct max
    {
        template <class A, class B>
        constexpr auto operator()(A a, B b) const -> typename std::remove_reference<decltype(a < b ? b : a)>::type
        {
            return a < b ? b : a;
        }
    };
    struct clamp
    {
        template <class A, class B, class C>
        constexpr auto operator()(A a, B b, C c) const ->
            typename std::remove_reference<decltype(a < b ? b : a < c ? a : c)>::type
        {
            return a < b ? b : a < c ? a : c;
        }
    };
    struct select
    {
        template <class A, class B, class C>
        constexpr auto operator()(A a, B b, C c) const -> typename std::remove_reference<decltype(a ? b : c)>::type
        {
            return a ? b : c;
        }
    };
    struct lerp
    {
        template <class A, class B, class C> constexpr auto operator()(A a, B b, C c) const -> decltype(a * (1 - c) + b * c)
        {
            return a * (1 - c) + b * c;
        }
    };

    // Function objects for applying operators
    struct op_pos
    {
        template <class A> constexpr auto operator()(A a) const -> decltype(+a) { return +a; }
    };
    struct op_neg
    {
        template <class A> constexpr auto operator()(A a) const -> decltype(-a) { return -a; }
    };
    struct op_not
    {
        template <class A> constexpr auto operator()(A a) const -> decltype(!a) { return !a; }
    };
    struct op_cmp
    {
        template <class A> constexpr auto operator()(A a) const -> decltype(~(a)) { return ~a; }
    };
    struct op_mul
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a * b) { return a * b; }
    };
    struct op_div
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a / b) { return a / b; }
    };
    struct op_mod
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a % b) { return a % b; }
    };
    struct op_add
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a + b) { return a + b; }
    };
    struct op_sub
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a - b) { return a - b; }
    };
    struct op_lsh
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a << b) { return a << b; }
    };
    struct op_rsh
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a >> b) { return a >> b; }
    };
    struct op_lt
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a < b) { return a < b; }
    };
    struct op_gt
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a > b) { return a > b; }
    };
    struct op_le
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a <= b) { return a <= b; }
    };
    struct op_ge
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a >= b) { return a >= b; }
    };
    struct op_eq
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a == b) { return a == b; }
    };
    struct op_ne
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a != b) { return a != b; }
    };
    struct op_int
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a & b) { return a & b; }
    };
    struct op_xor
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a ^ b) { return a ^ b; }
    };
    struct op_un
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a | b) { return a | b; }
    };
    struct op_and
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a && b) { return a && b; }
    };
    struct op_or
    {
        template <class A, class B> constexpr auto operator()(A a, B b) const -> decltype(a || b) { return a || b; }
    };

    // Function objects for applying standard library math functions
    struct std_abs
    {
        template <class A> auto operator()(A a) const -> decltype(std::abs(a)) { return std::abs(a); }
    };
    struct std_floor
    {
        template <class A> auto operator()(A a) const -> decltype(std::floor(a)) { return std::floor(a); }
    };
    struct std_ceil
    {
        template <class A> auto operator()(A a) const -> decltype(std::ceil(a)) { return std::ceil(a); }
    };
    struct std_exp
    {
        template <class A> auto operator()(A a) const -> decltype(std::exp(a)) { return std::exp(a); }
    };
    struct std_log
    {
        template <class A> auto operator()(A a) const -> decltype(std::log(a)) { return std::log(a); }
    };
    struct std_log10
    {
        template <class A> auto operator()(A a) const -> decltype(std::log10(a)) { return std::log10(a); }
    };
    struct std_sqrt
    {
        template <class A> auto operator()(A a) const -> decltype(std::sqrt(a)) { return std::sqrt(a); }
    };
    struct std_sin
    {
        template <class A> auto operator()(A a) const -> decltype(std::sin(a)) { return std::sin(a); }
    };
    struct std_cos
    {
        template <class A> auto operator()(A a) const -> decltype(std::cos(a)) { return std::cos(a); }
    };
    struct std_tan
    {
        template <class A> auto operator()(A a) const -> decltype(std::tan(a)) { return std::tan(a); }
    };
    struct std_asin
    {
        template <class A> auto operator()(A a) const -> decltype(std::asin(a)) { return std::asin(a); }
    };
    struct std_acos
    {
        template <class A> auto operator()(A a) const -> decltype(std::acos(a)) { return std::acos(a); }
    };
    struct std_atan
    {
        template <class A> auto operator()(A a) const -> decltype(std::atan(a)) { return std::atan(a); }
    };
    struct std_sinh
    {
        template <class A> auto operator()(A a) const -> decltype(std::sinh(a)) { return std::sinh(a); }
    };
    struct std_cosh
    {
        template <class A> auto operator()(A a) const -> decltype(std::cosh(a)) { return std::cosh(a); }
    };
    struct std_tanh
    {
        template <class A> auto operator()(A a) const -> decltype(std::tanh(a)) { return std::tanh(a); }
    };
    struct std_round
    {
        template <class A> auto operator()(A a) const -> decltype(std::round(a)) { return std::round(a); }
    };
    struct std_fmod
    {
        template <class A, class B> auto operator()(A a, B b) const -> decltype(std::fmod(a, b)) { return std::fmod(a, b); }
    };
    struct std_pow
    {
        template <class A, class B> auto operator()(A a, B b) const -> decltype(std::pow(a, b)) { return std::pow(a, b); }
    };
    struct std_atan2
    {
        template <class A, class B> auto operator()(A a, B b) const -> decltype(std::atan2(a, b)) { return std::atan2(a, b); }
    };
    struct std_copysign
    {
        template <class A, class B> auto operator()(A a, B b) const -> decltype(std::copysign(a, b))
        {
            return std::copysign(a, b);
        }
    };
}

// Small, fixed-length vector type, consisting of exactly M elements of type T, and presumed to be a column-vector unless
// otherwise noted
template <class T> struct vec<T, 1>
{
    T x;
    constexpr vec()
        : x()
    { }
    constexpr vec(const T& x_)
        : x(x_)
    { }
    // NOTE: vec<T,1> does NOT have a constructor from pointer, this can conflict with initializing its single element from zero
    template <class U>
    constexpr explicit vec(const vec<U, 1>& v)
        : vec(static_cast<T>(v.x))
    { }
    constexpr auto          operator[](int /*unused*/) const -> const T& { return x; }
    LINALG_CONSTEXPR14 auto operator[](int /*unused*/) -> T& { return x; }

    template <class U, class = detail::conv_t<vec, U>>
    constexpr vec(const U& u)
        : vec(converter<vec, U> {}(u))
    { }
    template <class U, class = detail::conv_t<U, vec>> constexpr operator U() const { return converter<U, vec> {}(*this); }
};
template <class T> struct vec<T, 2>
{
    T x, y;
    constexpr vec()
        : x()
        , y()
    { }
    constexpr vec(const T& x_, const T& y_)
        : x(x_)
        , y(y_)
    { }
    constexpr explicit vec(const T& s)
        : vec(s, s)
    { }
    constexpr explicit vec(const T* p)
        : vec(p[0], p[1])
    { }
    template <class U>
    constexpr explicit vec(const vec<U, 2>& v)
        : vec(static_cast<T>(v.x), static_cast<T>(v.y))
    { }
    constexpr auto          operator[](int i) const -> const T& { return i == 0 ? x : y; }
    LINALG_CONSTEXPR14 auto operator[](int i) -> T& { return i == 0 ? x : y; }

    template <class U, class = detail::conv_t<vec, U>>
    constexpr vec(const U& u)
        : vec(converter<vec, U> {}(u))
    { }
    template <class U, class = detail::conv_t<U, vec>> constexpr operator U() const { return converter<U, vec> {}(*this); }
};
template <class T> struct vec<T, 3>
{
    T x, y, z;
    constexpr vec()
        : x()
        , y()
        , z()
    { }
    constexpr vec(const T& x_, const T& y_, const T& z_)
        : x(x_)
        , y(y_)
        , z(z_)
    { }
    constexpr vec(const vec<T, 2>& xy, const T& z_)
        : vec(xy.x, xy.y, z_)
    { }
    constexpr explicit vec(const T& s)
        : vec(s, s, s)
    { }
    constexpr explicit vec(const T* p)
        : vec(p[0], p[1], p[2])
    { }
    template <class U>
    constexpr explicit vec(const vec<U, 3>& v)
        : vec(static_cast<T>(v.x), static_cast<T>(v.y), static_cast<T>(v.z))
    { }
    constexpr auto          operator[](int i) const -> const T& { return i == 0 ? x : i == 1 ? y : z; }
    LINALG_CONSTEXPR14 auto operator[](int i) -> T& { return i == 0 ? x : i == 1 ? y : z; }
    constexpr auto          xy() const -> const vec<T, 2>& { return *reinterpret_cast<const vec<T, 2>*>(this); }
    auto                    xy() -> vec<T, 2>& { return *reinterpret_cast<vec<T, 2>*>(this); }

    template <class U, class = detail::conv_t<vec, U>>
    constexpr vec(const U& u)
        : vec(converter<vec, U> {}(u))
    { }
    template <class U, class = detail::conv_t<U, vec>> constexpr operator U() const { return converter<U, vec> {}(*this); }
};
template <class T> struct vec<T, 4>
{
    T x, y, z, w;
    constexpr vec()
        : x()
        , y()
        , z()
        , w()
    { }
    constexpr vec(const T& x_, const T& y_, const T& z_, const T& w_)
        : x(x_)
        , y(y_)
        , z(z_)
        , w(w_)
    { }
    constexpr vec(const vec<T, 2>& xy, const T& z_, const T& w_)
        : vec(xy.x, xy.y, z_, w_)
    { }
    constexpr vec(const vec<T, 3>& xyz, const T& w_)
        : vec(xyz.x, xyz.y, xyz.z, w_)
    { }
    constexpr explicit vec(const T& s)
        : vec(s, s, s, s)
    { }
    constexpr explicit vec(const T* p)
        : vec(p[0], p[1], p[2], p[3])
    { }
    template <class U>
    constexpr explicit vec(const vec<U, 4>& v)
        : vec(static_cast<T>(v.x), static_cast<T>(v.y), static_cast<T>(v.z), static_cast<T>(v.w))
    { }
    constexpr auto          operator[](int i) const -> const T& { return i == 0 ? x : i == 1 ? y : i == 2 ? z : w; }
    LINALG_CONSTEXPR14 auto operator[](int i) -> T& { return i == 0 ? x : i == 1 ? y : i == 2 ? z : w; }
    constexpr auto          xy() const -> const vec<T, 2>& { return *reinterpret_cast<const vec<T, 2>*>(this); }
    constexpr auto          xyz() const -> const vec<T, 3>& { return *reinterpret_cast<const vec<T, 3>*>(this); }
    auto                    xy() -> vec<T, 2>& { return *reinterpret_cast<vec<T, 2>*>(this); }
    auto                    xyz() -> vec<T, 3>& { return *reinterpret_cast<vec<T, 3>*>(this); }

    template <class U, class = detail::conv_t<vec, U>>
    constexpr vec(const U& u)
        : vec(converter<vec, U> {}(u))
    { }
    template <class U, class = detail::conv_t<U, vec>> constexpr operator U() const { return converter<U, vec> {}(*this); }
};

// Small, fixed-size matrix type, consisting of exactly M rows and N columns of type T, stored in column-major order.
template <class T, int M> struct mat<T, M, 1>
{
    typedef vec<T, M> V;
    V                 x;
    constexpr mat()
        : x()
    { }
    constexpr mat(const V& x_)
        : x(x_)
    { }
    constexpr explicit mat(const T& s)
        : x(s)
    { }
    constexpr explicit mat(const T* p)
        : x(p + M * 0)
    { }
    template <class U>
    constexpr explicit mat(const mat<U, M, 1>& m)
        : mat(V(m.x))
    { }
    constexpr auto          row(int i) const -> vec<T, 1> { return { x[i] }; }
    constexpr auto          operator[](int /*unused*/) const -> const V& { return x; }
    LINALG_CONSTEXPR14 auto operator[](int /*unused*/) -> V& { return x; }

    template <class U, class = detail::conv_t<mat, U>>
    constexpr mat(const U& u)
        : mat(converter<mat, U> {}(u))
    { }
    template <class U, class = detail::conv_t<U, mat>> constexpr operator U() const { return converter<U, mat> {}(*this); }
};
template <class T, int M> struct mat<T, M, 2>
{
    using V = vec<T, M>;
    V x, y;
    constexpr mat()
        : x()
        , y()
    { }
    constexpr mat(const V& x_, const V& y_)
        : x(x_)
        , y(y_)
    { }
    constexpr explicit mat(const T& s)
        : x(s)
        , y(s)
    { }
    constexpr explicit mat(const T* p)
        : x(p + M * 0)
        , y(p + M * 1)
    { }
    template <class U>
    constexpr explicit mat(const mat<U, M, 2>& m)
        : mat(V(m.x), V(m.y))
    { }
    constexpr auto          row(int i) const -> vec<T, 2> { return { x[i], y[i] }; }
    constexpr auto          operator[](int j) const -> const V& { return j == 0 ? x : y; }
    LINALG_CONSTEXPR14 auto operator[](int j) -> V& { return j == 0 ? x : y; }

    template <class U, class = detail::conv_t<mat, U>>
    constexpr mat(const U& u)
        : mat(converter<mat, U> {}(u))
    { }
    template <class U, class = detail::conv_t<U, mat>> constexpr operator U() const { return converter<U, mat> {}(*this); }
};
template <class T, int M> struct mat<T, M, 3>
{
    using V = vec<T, M>;
    V x, y, z;
    constexpr mat()
        : x()
        , y()
        , z()
    { }
    constexpr mat(const V& x_, const V& y_, const V& z_)
        : x(x_)
        , y(y_)
        , z(z_)
    { }
    constexpr explicit mat(const T& s)
        : x(s)
        , y(s)
        , z(s)
    { }
    constexpr explicit mat(const T* p)
        : x(p + M * 0)
        , y(p + M * 1)
        , z(p + M * 2)
    { }
    template <class U>
    constexpr explicit mat(const mat<U, M, 3>& m)
        : mat(V(m.x), V(m.y), V(m.z))
    { }
    constexpr auto          row(int i) const -> vec<T, 3> { return { x[i], y[i], z[i] }; }
    constexpr auto          operator[](int j) const -> const V& { return j == 0 ? x : j == 1 ? y : z; }
    LINALG_CONSTEXPR14 auto operator[](int j) -> V& { return j == 0 ? x : j == 1 ? y : z; }

    template <class U, class = detail::conv_t<mat, U>>
    constexpr mat(const U& u)
        : mat(converter<mat, U> {}(u))
    { }
    template <class U, class = detail::conv_t<U, mat>> constexpr operator U() const { return converter<U, mat> {}(*this); }
};
template <class T, int M> struct mat<T, M, 4>
{
    using V = vec<T, M>;
    V x, y, z, w;
    constexpr mat()
        : x()
        , y()
        , z()
        , w()
    { }
    constexpr mat(const V& x_, const V& y_, const V& z_, const V& w_)
        : x(x_)
        , y(y_)
        , z(z_)
        , w(w_)
    { }
    constexpr explicit mat(const T& s)
        : x(s)
        , y(s)
        , z(s)
        , w(s)
    { }
    constexpr explicit mat(const T* p)
        : x(p + M * 0)
        , y(p + M * 1)
        , z(p + M * 2)
        , w(p + M * 3)
    { }
    template <class U>
    constexpr explicit mat(const mat<U, M, 4>& m)
        : mat(V(m.x), V(m.y), V(m.z), V(m.w))
    { }
    constexpr auto          row(int i) const -> vec<T, 4> { return { x[i], y[i], z[i], w[i] }; }
    constexpr auto          operator[](int j) const -> const V& { return j == 0 ? x : j == 1 ? y : j == 2 ? z : w; }
    LINALG_CONSTEXPR14 auto operator[](int j) -> V& { return j == 0 ? x : j == 1 ? y : j == 2 ? z : w; }

    template <class U, class = detail::conv_t<mat, U>>
    constexpr mat(const U& u)
        : mat(converter<mat, U> {}(u))
    { }
    template <class U, class = detail::conv_t<U, mat>> constexpr operator U() const { return converter<U, mat> {}(*this); }
};

// Define a type which will convert to the multiplicative identity of any square matrix
struct identity_t
{
    constexpr explicit identity_t(int /*unused*/) { }
};
template <class T> struct converter<mat<T, 1, 1>, identity_t>
{
    auto operator()(identity_t /*unused*/) const -> mat<T, 1, 1> { return { vec<T, 1> { 1 } }; }
};
template <class T> struct converter<mat<T, 2, 2>, identity_t>
{
    auto operator()(identity_t /*unused*/) const -> mat<T, 2, 2> { return { { 1, 0 }, { 0, 1 } }; }
};
template <class T> struct converter<mat<T, 3, 3>, identity_t>
{
    auto operator()(identity_t /*unused*/) const -> mat<T, 3, 3> { return { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } }; }
};
template <class T> struct converter<mat<T, 4, 4>, identity_t>
{
    auto operator()(identity_t /*unused*/) const -> mat<T, 4, 4>
    {
        return { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
    }
};
constexpr identity_t identity { 1 };

// Produce a scalar by applying f(A,B) -> A to adjacent pairs of elements from a vec/mat in left-to-right/column-major order
// (matching the associativity of arithmetic and logical operators)
template <class F, class A, class B> constexpr auto fold(F f, A a, const vec<B, 1>& b) -> A { return f(a, b.x); }
template <class F, class A, class B> constexpr auto fold(F f, A a, const vec<B, 2>& b) -> A { return f(f(a, b.x), b.y); }
template <class F, class A, class B> constexpr auto fold(F f, A a, const vec<B, 3>& b) -> A
{
    return f(f(f(a, b.x), b.y), b.z);
}
template <class F, class A, class B> constexpr auto fold(F f, A a, const vec<B, 4>& b) -> A
{
    return f(f(f(f(a, b.x), b.y), b.z), b.w);
}
template <class F, class A, class B, int M> constexpr auto fold(F f, A a, const mat<B, M, 1>& b) -> A
{
    return fold(f, a, b.x);
}
template <class F, class A, class B, int M> constexpr auto fold(F f, A a, const mat<B, M, 2>& b) -> A
{
    return fold(f, fold(f, a, b.x), b.y);
}
template <class F, class A, class B, int M> constexpr auto fold(F f, A a, const mat<B, M, 3>& b) -> A
{
    return fold(f, fold(f, fold(f, a, b.x), b.y), b.z);
}
template <class F, class A, class B, int M> constexpr auto fold(F f, A a, const mat<B, M, 4>& b) -> A
{
    return fold(f, fold(f, fold(f, fold(f, a, b.x), b.y), b.z), b.w);
}

// Type aliases for the result of calling apply(...) with various arguments, can be used with return type SFINAE to constrian
// overload sets
template <class F, class... A> using apply_t = typename detail::apply<F, void, A...>::type;
template <class A>
using scalar_t = typename detail::scalar_type<A>::type; // Underlying scalar type when performing elementwise operations

// apply(f,...) applies the provided function in an elementwise fashion to its arguments, producing an object of the same
// dimensions
template <class F, class... A> constexpr auto apply(F func, const A&... args) -> apply_t<F, A...>
{
    return detail::apply<F, void, A...>::impl(detail::make_seq<0, detail::apply<F, void, A...>::size> {}, func, args...);
}

// map(a,f) is equivalent to apply(f,a)
template <class A, class F> constexpr auto map(const A& a, F func) -> apply_t<F, A> { return apply(func, a); }

// zip(a,b,f) is equivalent to apply(f,a,b)
template <class A, class B, class F> constexpr auto zip(const A& a, const B& b, F func) -> apply_t<F, A, B>
{
    return apply(func, a, b);
}

// Relational operators are defined to compare the elements of two vectors or matrices lexicographically, in column-major order
template <class A, class B> constexpr auto compare(const A& a, const B& b) -> typename detail::any_compare<A, B>::type
{
    return detail::any_compare<A, B>()(a, b);
}
template <class A, class B> constexpr auto operator==(const A& a, const B& b) -> decltype(compare(a, b) == 0)
{
    return compare(a, b) == 0;
}
template <class A, class B> constexpr auto operator!=(const A& a, const B& b) -> decltype(compare(a, b) != 0)
{
    return compare(a, b) != 0;
}
template <class A, class B> constexpr auto operator<(const A& a, const B& b) -> decltype(compare(a, b) < 0)
{
    return compare(a, b) < 0;
}
template <class A, class B> constexpr auto operator>(const A& a, const B& b) -> decltype(compare(a, b) > 0)
{
    return compare(a, b) > 0;
}
template <class A, class B> constexpr auto operator<=(const A& a, const B& b) -> decltype(compare(a, b) <= 0)
{
    return compare(a, b) <= 0;
}
template <class A, class B> constexpr auto operator>=(const A& a, const B& b) -> decltype(compare(a, b) >= 0)
{
    return compare(a, b) >= 0;
}

// Functions for coalescing scalar values
template <class A> constexpr auto any(const A& a) -> bool { return fold(detail::op_or {}, false, a); }
template <class A> constexpr auto all(const A& a) -> bool { return fold(detail::op_and {}, true, a); }
template <class A> constexpr auto sum(const A& a) -> scalar_t<A> { return fold(detail::op_add {}, scalar_t<A>(0), a); }
template <class A> constexpr auto product(const A& a) -> scalar_t<A> { return fold(detail::op_mul {}, scalar_t<A>(1), a); }
template <class A> constexpr auto minelem(const A& a) -> scalar_t<A> { return fold(detail::min {}, a.x, a); }
template <class A> constexpr auto maxelem(const A& a) -> scalar_t<A> { return fold(detail::max {}, a.x, a); }
template <class T, int M> auto    argmin(const vec<T, M>& a) -> int
{
    int j = 0;
    for (int i = 1; i < M; ++i) {
        if (a[i] < a[j]) {
            j = i;
        }
    }
    return j;
}
template <class T, int M> auto argmax(const vec<T, M>& a) -> int
{
    int j = 0;
    for (int i = 1; i < M; ++i) {
        if (a[i] > a[j]) {
            j = i;
        }
    }
    return j;
}

// Unary operators are defined component-wise for linalg types
template <class A> constexpr auto operator+(const A& a) -> apply_t<detail::op_pos, A> { return apply(detail::op_pos {}, a); }
template <class A> constexpr auto operator-(const A& a) -> apply_t<detail::op_neg, A> { return apply(detail::op_neg {}, a); }
template <class A> constexpr auto operator~(const A& a) -> apply_t<detail::op_cmp, A> { return apply(detail::op_cmp {}, a); }
template <class A> constexpr auto operator!(const A& a) -> apply_t<detail::op_not, A> { return apply(detail::op_not {}, a); }

// Binary operators are defined component-wise for linalg types
template <class A, class B> constexpr auto operator+(const A& a, const B& b) -> apply_t<detail::op_add, A, B>
{
    return apply(detail::op_add {}, a, b);
}
template <class A, class B> constexpr auto operator-(const A& a, const B& b) -> apply_t<detail::op_sub, A, B>
{
    return apply(detail::op_sub {}, a, b);
}
template <class A, class B> constexpr auto operator*(const A& a, const B& b) -> apply_t<detail::op_mul, A, B>
{
    return apply(detail::op_mul {}, a, b);
}
template <class A, class B> constexpr auto operator/(const A& a, const B& b) -> apply_t<detail::op_div, A, B>
{
    return apply(detail::op_div {}, a, b);
}
template <class A, class B> constexpr auto operator%(const A& a, const B& b) -> apply_t<detail::op_mod, A, B>
{
    return apply(detail::op_mod {}, a, b);
}
template <class A, class B> constexpr auto operator|(const A& a, const B& b) -> apply_t<detail::op_un, A, B>
{
    return apply(detail::op_un {}, a, b);
}
template <class A, class B> constexpr auto operator^(const A& a, const B& b) -> apply_t<detail::op_xor, A, B>
{
    return apply(detail::op_xor {}, a, b);
}
template <class A, class B> constexpr auto operator&(const A& a, const B& b) -> apply_t<detail::op_int, A, B>
{
    return apply(detail::op_int {}, a, b);
}
template <class A, class B> constexpr auto operator<<(const A& a, const B& b) -> apply_t<detail::op_lsh, A, B>
{
    return apply(detail::op_lsh {}, a, b);
}
template <class A, class B> constexpr auto operator>>(const A& a, const B& b) -> apply_t<detail::op_rsh, A, B>
{
    return apply(detail::op_rsh {}, a, b);
}

// Binary assignment operators a $= b is always defined as though it were explicitly written a = a $ b
template <class A, class B> constexpr auto operator+=(A& a, const B& b) -> decltype(a = a + b) { return a = a + b; }
template <class A, class B> constexpr auto operator-=(A& a, const B& b) -> decltype(a = a - b) { return a = a - b; }
template <class A, class B> constexpr auto operator*=(A& a, const B& b) -> decltype(a = a * b) { return a = a * b; }
template <class A, class B> constexpr auto operator/=(A& a, const B& b) -> decltype(a = a / b) { return a = a / b; }
template <class A, class B> constexpr auto operator%=(A& a, const B& b) -> decltype(a = a % b) { return a = a % b; }
template <class A, class B> constexpr auto operator|=(A& a, const B& b) -> decltype(a = a | b) { return a = a | b; }
template <class A, class B> constexpr auto operator^=(A& a, const B& b) -> decltype(a = a ^ b) { return a = a ^ b; }
template <class A, class B> constexpr auto operator&=(A& a, const B& b) -> decltype(a = a & b) { return a = a & b; }
template <class A, class B> constexpr auto operator<<=(A& a, const B& b) -> decltype(a = a << b) { return a = a << b; }
template <class A, class B> constexpr auto operator>>=(A& a, const B& b) -> decltype(a = a >> b) { return a = a >> b; }

// Swizzles and subobjects
template <int... I, class T, int M> constexpr auto swizzle(const vec<T, M>& a) -> vec<T, sizeof...(I)>
{
    return { detail::getter<I>(a)... };
}
template <int I0, int I1, class T, int M> constexpr auto subvec(const vec<T, M>& a) -> vec<T, I1 - I0>
{
    return detail::swizzle(a, detail::make_seq<I0, I1> {});
}
template <int I0, int J0, int I1, int J1, class T, int M, int N>
constexpr auto submat(const mat<T, M, N>& a) -> mat<T, I1 - I0, J1 - J0>
{
    return detail::swizzle(a, detail::make_seq<I0, I1> {}, detail::make_seq<J0, J1> {});
}

// Component-wise standard library math functions
template <class A> auto abs(const A& a) -> apply_t<detail::std_abs, A> { return apply(detail::std_abs {}, a); }
template <class A> auto floor(const A& a) -> apply_t<detail::std_floor, A> { return apply(detail::std_floor {}, a); }
template <class A> auto ceil(const A& a) -> apply_t<detail::std_ceil, A> { return apply(detail::std_ceil {}, a); }
template <class A> auto exp(const A& a) -> apply_t<detail::std_exp, A> { return apply(detail::std_exp {}, a); }
template <class A> auto log(const A& a) -> apply_t<detail::std_log, A> { return apply(detail::std_log {}, a); }
template <class A> auto log10(const A& a) -> apply_t<detail::std_log10, A> { return apply(detail::std_log10 {}, a); }
template <class A> auto sqrt(const A& a) -> apply_t<detail::std_sqrt, A> { return apply(detail::std_sqrt {}, a); }
template <class A> auto sin(const A& a) -> apply_t<detail::std_sin, A> { return apply(detail::std_sin {}, a); }
template <class A> auto cos(const A& a) -> apply_t<detail::std_cos, A> { return apply(detail::std_cos {}, a); }
template <class A> auto tan(const A& a) -> apply_t<detail::std_tan, A> { return apply(detail::std_tan {}, a); }
template <class A> auto asin(const A& a) -> apply_t<detail::std_asin, A> { return apply(detail::std_asin {}, a); }
template <class A> auto acos(const A& a) -> apply_t<detail::std_acos, A> { return apply(detail::std_acos {}, a); }
template <class A> auto atan(const A& a) -> apply_t<detail::std_atan, A> { return apply(detail::std_atan {}, a); }
template <class A> auto sinh(const A& a) -> apply_t<detail::std_sinh, A> { return apply(detail::std_sinh {}, a); }
template <class A> auto cosh(const A& a) -> apply_t<detail::std_cosh, A> { return apply(detail::std_cosh {}, a); }
template <class A> auto tanh(const A& a) -> apply_t<detail::std_tanh, A> { return apply(detail::std_tanh {}, a); }
template <class A> auto round(const A& a) -> apply_t<detail::std_round, A> { return apply(detail::std_round {}, a); }

template <class A, class B> auto fmod(const A& a, const B& b) -> apply_t<detail::std_fmod, A, B>
{
    return apply(detail::std_fmod {}, a, b);
}
template <class A, class B> auto pow(const A& a, const B& b) -> apply_t<detail::std_pow, A, B>
{
    return apply(detail::std_pow {}, a, b);
}
template <class A, class B> auto atan2(const A& a, const B& b) -> apply_t<detail::std_atan2, A, B>
{
    return apply(detail::std_atan2 {}, a, b);
}
template <class A, class B> auto copysign(const A& a, const B& b) -> apply_t<detail::std_copysign, A, B>
{
    return apply(detail::std_copysign {}, a, b);
}

// Component-wise relational functions on vectors
template <class A, class B> constexpr auto equal(const A& a, const B& b) -> apply_t<detail::op_eq, A, B>
{
    return apply(detail::op_eq {}, a, b);
}
template <class A, class B> constexpr auto nequal(const A& a, const B& b) -> apply_t<detail::op_ne, A, B>
{
    return apply(detail::op_ne {}, a, b);
}
template <class A, class B> constexpr auto less(const A& a, const B& b) -> apply_t<detail::op_lt, A, B>
{
    return apply(detail::op_lt {}, a, b);
}
template <class A, class B> constexpr auto greater(const A& a, const B& b) -> apply_t<detail::op_gt, A, B>
{
    return apply(detail::op_gt {}, a, b);
}
template <class A, class B> constexpr auto lequal(const A& a, const B& b) -> apply_t<detail::op_le, A, B>
{
    return apply(detail::op_le {}, a, b);
}
template <class A, class B> constexpr auto gequal(const A& a, const B& b) -> apply_t<detail::op_ge, A, B>
{
    return apply(detail::op_ge {}, a, b);
}

// Component-wise selection functions on vectors
template <class A, class B> constexpr auto min(const A& a, const B& b) -> apply_t<detail::min, A, B>
{
    return apply(detail::min {}, a, b);
}
template <class A, class B> constexpr auto max(const A& a, const B& b) -> apply_t<detail::max, A, B>
{
    return apply(detail::max {}, a, b);
}
template <class X, class L, class H> constexpr auto clamp(const X& x, const L& l, const H& h) -> apply_t<detail::clamp, X, L, H>
{
    return apply(detail::clamp {}, x, l, h);
}
template <class P, class A, class B>
constexpr auto select(const P& p, const A& a, const B& b) -> apply_t<detail::select, P, A, B>
{
    return apply(detail::select {}, p, a, b);
}
template <class A, class B, class T> constexpr auto lerp(const A& a, const B& b, const T& t) -> apply_t<detail::lerp, A, B, T>
{
    return apply(detail::lerp {}, a, b, t);
}

// Support for vector algebra
template <class T> constexpr auto cross(const vec<T, 2>& a, const vec<T, 2>& b) -> T { return a.x * b.y - a.y * b.x; }
template <class T> constexpr auto cross(T a, const vec<T, 2>& b) -> vec<T, 2> { return { -a * b.y, a * b.x }; }
template <class T> constexpr auto cross(const vec<T, 2>& a, T b) -> vec<T, 2> { return { a.y * b, -a.x * b }; }
template <class T> constexpr auto cross(const vec<T, 3>& a, const vec<T, 3>& b) -> vec<T, 3>
{
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
template <class T, int M> constexpr auto dot(const vec<T, M>& a, const vec<T, M>& b) -> T { return sum(a * b); }
template <class T, int M> constexpr auto length2(const vec<T, M>& a) -> T { return dot(a, a); }
template <class T, int M> auto           length(const vec<T, M>& a) -> T { return std::sqrt(length2(a)); }
template <class T, int M> auto           normalize(const vec<T, M>& a) -> vec<T, M> { return a / length(a); }
template <class T, int M> constexpr auto distance2(const vec<T, M>& a, const vec<T, M>& b) -> T { return length2(b - a); }
template <class T, int M> auto           distance(const vec<T, M>& a, const vec<T, M>& b) -> T { return length(b - a); }
template <class T, int M> auto           uangle(const vec<T, M>& a, const vec<T, M>& b) -> T
{
    T d = dot(a, b);
    return d > 1 ? 0 : std::acos(d < -1 ? -1 : d);
}
template <class T, int M> auto angle(const vec<T, M>& a, const vec<T, M>& b) -> T { return uangle(normalize(a), normalize(b)); }
template <class T> auto        rot(T a, const vec<T, 2>& v) -> vec<T, 2>
{
    const T s = std::sin(a);
    const T c = std::cos(a);
    return { v.x * c - v.y * s, v.x * s + v.y * c };
}
template <class T, int M> auto nlerp(const vec<T, M>& a, const vec<T, M>& b, T t) -> vec<T, M>
{
    return normalize(lerp(a, b, t));
}
template <class T, int M> auto slerp(const vec<T, M>& a, const vec<T, M>& b, T t) -> vec<T, M>
{
    T th = uangle(a, b);
    return th == 0 ? a : a * (std::sin(th * (1 - t)) / std::sin(th)) + b * (std::sin(th * t) / std::sin(th));
}

// Support for quaternion algebra using 4D vectors, representing xi + yj + zk + w
template <class T> constexpr auto qconj(const vec<T, 4>& q) -> vec<T, 4> { return { -q.x, -q.y, -q.z, q.w }; }
template <class T> auto           qinv(const vec<T, 4>& q) -> vec<T, 4> { return qconj(q) / length2(q); }
template <class T> auto           qexp(const vec<T, 4>& q) -> vec<T, 4>
{
    const auto v  = q.xyz();
    const auto vv = length(v);
    return std::exp(q.w) * vec<T, 4> { v * (vv > 0 ? std::sin(vv) / vv : 0), std::cos(vv) };
}
template <class T> auto qlog(const vec<T, 4>& q) -> vec<T, 4>
{
    const auto v  = q.xyz();
    const auto vv = length(v);
    const auto qq = length(q);
    return { v * (vv > 0 ? std::acos(q.w / qq) / vv : 0), std::log(qq) };
}
template <class T> auto qpow(const vec<T, 4>& q, const T& p) -> vec<T, 4>
{
    const auto v  = q.xyz();
    const auto vv = length(v);
    const auto qq = length(q);
    const auto th = std::acos(q.w / qq);
    return std::pow(qq, p) * vec<T, 4> { v * (vv > 0 ? std::sin(p * th) / vv : 0), std::cos(p * th) };
}
template <class T> constexpr auto qmul(const vec<T, 4>& a, const vec<T, 4>& b) -> vec<T, 4>
{
    return { a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y, a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z,
             a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z };
}
template <class T, class... R> constexpr auto qmul(const vec<T, 4>& a, R... r) -> vec<T, 4> { return qmul(a, qmul(r...)); }

// Support for 3D spatial rotations using quaternions, via qmul(qmul(q, v), qconj(q))
template <class T> constexpr auto qxdir(const vec<T, 4>& q) -> vec<T, 3>
{
    return { q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z, (q.x * q.y + q.z * q.w) * 2, (q.z * q.x - q.y * q.w) * 2 };
}
template <class T> constexpr auto qydir(const vec<T, 4>& q) -> vec<T, 3>
{
    return { (q.x * q.y - q.z * q.w) * 2, q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z, (q.y * q.z + q.x * q.w) * 2 };
}
template <class T> constexpr auto qzdir(const vec<T, 4>& q) -> vec<T, 3>
{
    return { (q.z * q.x + q.y * q.w) * 2, (q.y * q.z - q.x * q.w) * 2, q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z };
}
template <class T> constexpr auto qmat(const vec<T, 4>& q) -> mat<T, 3, 3> { return { qxdir(q), qydir(q), qzdir(q) }; }
template <class T> constexpr auto qrot(const vec<T, 4>& q, const vec<T, 3>& v) -> vec<T, 3>
{
    return qxdir(q) * v.x + qydir(q) * v.y + qzdir(q) * v.z;
}
template <class T> auto qangle(const vec<T, 4>& q) -> T { return std::atan2(length(q.xyz()), q.w) * 2; }
template <class T> auto qaxis(const vec<T, 4>& q) -> vec<T, 3> { return normalize(q.xyz()); }
template <class T> auto qnlerp(const vec<T, 4>& a, const vec<T, 4>& b, T t) -> vec<T, 4>
{
    return nlerp(a, dot(a, b) < 0 ? -b : b, t);
}
template <class T> auto qslerp(const vec<T, 4>& a, const vec<T, 4>& b, T t) -> vec<T, 4>
{
    return slerp(a, dot(a, b) < 0 ? -b : b, t);
}

// Support for matrix algebra
template <class T, int M> constexpr auto mul(const mat<T, M, 1>& a, const vec<T, 1>& b) -> vec<T, M> { return a.x * b.x; }
template <class T, int M> constexpr auto mul(const mat<T, M, 2>& a, const vec<T, 2>& b) -> vec<T, M>
{
    return a.x * b.x + a.y * b.y;
}
template <class T, int M> constexpr auto mul(const mat<T, M, 3>& a, const vec<T, 3>& b) -> vec<T, M>
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
template <class T, int M> constexpr auto mul(const mat<T, M, 4>& a, const vec<T, 4>& b) -> vec<T, M>
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
template <class T, int M, int N> constexpr auto mul(const mat<T, M, N>& a, const mat<T, N, 1>& b) -> mat<T, M, 1>
{
    return { mul(a, b.x) };
}
template <class T, int M, int N> constexpr auto mul(const mat<T, M, N>& a, const mat<T, N, 2>& b) -> mat<T, M, 2>
{
    return { mul(a, b.x), mul(a, b.y) };
}
template <class T, int M, int N> constexpr auto mul(const mat<T, M, N>& a, const mat<T, N, 3>& b) -> mat<T, M, 3>
{
    return { mul(a, b.x), mul(a, b.y), mul(a, b.z) };
}
template <class T, int M, int N> constexpr auto mul(const mat<T, M, N>& a, const mat<T, N, 4>& b) -> mat<T, M, 4>
{
    return { mul(a, b.x), mul(a, b.y), mul(a, b.z), mul(a, b.w) };
}
template <class T, int M, int N, int P>
constexpr auto mul(const mat<T, M, N>& a, const mat<T, N, P>& b, const vec<T, P>& c) -> vec<T, M>
{
    return mul(mul(a, b), c);
}
template <class T, int M, int N, int P, int Q>
constexpr auto mul(const mat<T, M, N>& a, const mat<T, N, P>& b, const mat<T, P, Q>& c) -> mat<T, M, Q>
{
    return mul(mul(a, b), c);
}
template <class T, int M, int N, int P, int Q>
constexpr auto mul(const mat<T, M, N>& a, const mat<T, N, P>& b, const mat<T, P, Q>& c, const vec<T, Q>& d) -> vec<T, M>
{
    return mul(mul(a, b, c), d);
}
template <class T, int M, int N, int P, int Q, int R>
constexpr auto mul(const mat<T, M, N>& a, const mat<T, N, P>& b, const mat<T, P, Q>& c, const mat<T, Q, R>& d) -> mat<T, M, R>
{
    return mul(mul(a, b, c), d);
}
// TODO: Variadic version of mul(...) that works on all compilers
template <class T, int M> constexpr auto outerprod(const vec<T, M>& a, const vec<T, 1>& b) -> mat<T, M, 1>
{
    return { a * b.x };
}
template <class T, int M> constexpr auto outerprod(const vec<T, M>& a, const vec<T, 2>& b) -> mat<T, M, 2>
{
    return { a * b.x, a * b.y };
}
template <class T, int M> constexpr auto outerprod(const vec<T, M>& a, const vec<T, 3>& b) -> mat<T, M, 3>
{
    return { a * b.x, a * b.y, a * b.z };
}
template <class T, int M> constexpr auto outerprod(const vec<T, M>& a, const vec<T, 4>& b) -> mat<T, M, 4>
{
    return { a * b.x, a * b.y, a * b.z, a * b.w };
}
template <class T> constexpr auto        diagonal(const mat<T, 1, 1>& a) -> vec<T, 1> { return { a.x.x }; }
template <class T> constexpr auto        diagonal(const mat<T, 2, 2>& a) -> vec<T, 2> { return { a.x.x, a.y.y }; }
template <class T> constexpr auto        diagonal(const mat<T, 3, 3>& a) -> vec<T, 3> { return { a.x.x, a.y.y, a.z.z }; }
template <class T> constexpr auto        diagonal(const mat<T, 4, 4>& a) -> vec<T, 4> { return { a.x.x, a.y.y, a.z.z, a.w.w }; }
template <class T, int N> constexpr auto trace(const mat<T, N, N>& a) -> T { return sum(diagonal(a)); }
template <class T, int M> constexpr auto transpose(const mat<T, 1, M>& m) -> mat<T, M, 1> { return { m.row(0) }; }
template <class T, int M> constexpr auto transpose(const mat<T, 2, M>& m) -> mat<T, M, 2> { return { m.row(0), m.row(1) }; }
template <class T, int M> constexpr auto transpose(const mat<T, 3, M>& m) -> mat<T, M, 3>
{
    return { m.row(0), m.row(1), m.row(2) };
}
template <class T, int M> constexpr auto transpose(const mat<T, 4, M>& m) -> mat<T, M, 4>
{
    return { m.row(0), m.row(1), m.row(2), m.row(3) };
}
template <class T> constexpr auto adjugate(const mat<T, 1, 1> & /*unused*/) -> mat<T, 1, 1> { return { vec<T, 1> { 1 } }; }
template <class T> constexpr auto adjugate(const mat<T, 2, 2>& a) -> mat<T, 2, 2>
{
    return { { a.y.y, -a.x.y }, { -a.y.x, a.x.x } };
}
template <class T> constexpr auto        adjugate(const mat<T, 3, 3>& a) -> mat<T, 3, 3>;
template <class T> constexpr auto        adjugate(const mat<T, 4, 4>& a) -> mat<T, 4, 4>;
template <class T, int N> constexpr auto comatrix(const mat<T, N, N>& a) -> mat<T, N, N> { return transpose(adjugate(a)); }
template <class T> constexpr auto        determinant(const mat<T, 1, 1>& a) -> T { return a.x.x; }
template <class T> constexpr auto        determinant(const mat<T, 2, 2>& a) -> T { return a.x.x * a.y.y - a.x.y * a.y.x; }
template <class T> constexpr auto        determinant(const mat<T, 3, 3>& a) -> T
{
    return a.x.x * (a.y.y * a.z.z - a.z.y * a.y.z) + a.x.y * (a.y.z * a.z.x - a.z.z * a.y.x)
        + a.x.z * (a.y.x * a.z.y - a.z.x * a.y.y);
}
template <class T> constexpr auto        determinant(const mat<T, 4, 4>& a) -> T;
template <class T, int N> constexpr auto inverse(const mat<T, N, N>& a) -> mat<T, N, N> { return adjugate(a) / determinant(a); }

// Vectors and matrices can be used as ranges
template <class T, int M> auto        begin(vec<T, M>& a) -> T* { return &a.x; }
template <class T, int M> auto        begin(const vec<T, M>& a) -> const T* { return &a.x; }
template <class T, int M> auto        end(vec<T, M>& a) -> T* { return begin(a) + M; }
template <class T, int M> auto        end(const vec<T, M>& a) -> const T* { return begin(a) + M; }
template <class T, int M, int N> auto begin(mat<T, M, N>& a) -> vec<T, M>* { return &a.x; }
template <class T, int M, int N> auto begin(const mat<T, M, N>& a) -> const vec<T, M>* { return &a.x; }
template <class T, int M, int N> auto end(mat<T, M, N>& a) -> vec<T, M>* { return begin(a) + N; }
template <class T, int M, int N> auto end(const mat<T, M, N>& a) -> const vec<T, M>* { return begin(a) + N; }

// Factory functions for 3D spatial transformations (will possibly be removed or changed in a future version)
enum fwd_axis { neg_z, pos_z }; // Should projection matrices be generated assuming forward is {0,0,-1} or {0,0,1}
enum z_range { neg_one_to_one, zero_to_one }; // Should projection matrices map z into the range of [-1,1] or [0,1]?
template <class T> auto rotation_quat(const vec<T, 3>& axis, T angle) -> vec<T, 4>
{
    return { axis * std::sin(angle / 2), std::cos(angle / 2) };
}
template <class T> auto rotation_quat(const mat<T, 3, 3>& m) -> vec<T, 4>;
template <class T> auto translation_matrix(const vec<T, 3>& translation) -> mat<T, 4, 4>
{
    return { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { translation, 1 } };
}
template <class T> auto rotation_matrix(const vec<T, 4>& rotation) -> mat<T, 4, 4>
{
    return { { qxdir(rotation), 0 }, { qydir(rotation), 0 }, { qzdir(rotation), 0 }, { 0, 0, 0, 1 } };
}
template <class T> auto scaling_matrix(const vec<T, 3>& scaling) -> mat<T, 4, 4>
{
    return { { scaling.x, 0, 0, 0 }, { 0, scaling.y, 0, 0 }, { 0, 0, scaling.z, 0 }, { 0, 0, 0, 1 } };
}
template <class T> auto pose_matrix(const vec<T, 4>& q, const vec<T, 3>& p) -> mat<T, 4, 4>
{
    return { { qxdir(q), 0 }, { qydir(q), 0 }, { qzdir(q), 0 }, { p, 1 } };
}
template <class T>
auto frustum_matrix(T x0, T x1, T y0, T y1, T n, T f, fwd_axis a = neg_z, z_range z = neg_one_to_one) -> mat<T, 4, 4>;
template <class T>
auto perspective_matrix(T fovy, T aspect, T n, T f, fwd_axis a = neg_z, z_range z = neg_one_to_one) -> mat<T, 4, 4>
{
    T y = n * std::tan(fovy / 2);
    T x = y * aspect;
    return frustum_matrix(-x, x, -y, y, n, f, a, z);
}

// Provide implicit conversion between linalg::vec<T,M> and std::array<T,M>
template <class T> struct converter<vec<T, 1>, std::array<T, 1>>
{
    auto operator()(const std::array<T, 1>& a) const -> vec<T, 1> { return { a[0] }; }
};
template <class T> struct converter<vec<T, 2>, std::array<T, 2>>
{
    auto operator()(const std::array<T, 2>& a) const -> vec<T, 2> { return { a[0], a[1] }; }
};
template <class T> struct converter<vec<T, 3>, std::array<T, 3>>
{
    auto operator()(const std::array<T, 3>& a) const -> vec<T, 3> { return { a[0], a[1], a[2] }; }
};
template <class T> struct converter<vec<T, 4>, std::array<T, 4>>
{
    auto operator()(const std::array<T, 4>& a) const -> vec<T, 4> { return { a[0], a[1], a[2], a[3] }; }
};

template <class T> struct converter<std::array<T, 1>, vec<T, 1>>
{
    auto operator()(const vec<T, 1>& a) const -> std::array<T, 1> { return { a[0] }; }
};
template <class T> struct converter<std::array<T, 2>, vec<T, 2>>
{
    auto operator()(const vec<T, 2>& a) const -> std::array<T, 2> { return { a[0], a[1] }; }
};
template <class T> struct converter<std::array<T, 3>, vec<T, 3>>
{
    auto operator()(const vec<T, 3>& a) const -> std::array<T, 3> { return { a[0], a[1], a[2] }; }
};
template <class T> struct converter<std::array<T, 4>, vec<T, 4>>
{
    auto operator()(const vec<T, 4>& a) const -> std::array<T, 4> { return { a[0], a[1], a[2], a[3] }; }
};

// Provide typedefs for common element types and vector/matrix sizes
namespace aliases
{
    using bool1     = vec<bool, 1>;
    using byte1     = vec<uint8_t, 1>;
    using short1    = vec<int16_t, 1>;
    using ushort1   = vec<uint16_t, 1>;
    using bool2     = vec<bool, 2>;
    using byte2     = vec<uint8_t, 2>;
    using short2    = vec<int16_t, 2>;
    using ushort2   = vec<uint16_t, 2>;
    using bool3     = vec<bool, 3>;
    using byte3     = vec<uint8_t, 3>;
    using short3    = vec<int16_t, 3>;
    using ushort3   = vec<uint16_t, 3>;
    using bool4     = vec<bool, 4>;
    using byte4     = vec<uint8_t, 4>;
    using short4    = vec<int16_t, 4>;
    using ushort4   = vec<uint16_t, 4>;
    using int1      = vec<int, 1>;
    using uint1     = vec<unsigned int, 1>;
    using float1    = vec<float, 1>;
    using double1   = vec<double, 1>;
    using int2      = vec<int, 2>;
    using uint2     = vec<unsigned int, 2>;
    using float2    = vec<float, 2>;
    using double2   = vec<double, 2>;
    using int3      = vec<int, 3>;
    using uint3     = vec<unsigned int, 3>;
    using float3    = vec<float, 3>;
    using double3   = vec<double, 3>;
    using int4      = vec<int, 4>;
    using uint4     = vec<unsigned int, 4>;
    using float4    = vec<float, 4>;
    using double4   = vec<double, 4>;
    using bool1x1   = mat<bool, 1, 1>;
    using int1x1    = mat<int, 1, 1>;
    using float1x1  = mat<float, 1, 1>;
    using double1x1 = mat<double, 1, 1>;
    using bool1x2   = mat<bool, 1, 2>;
    using int1x2    = mat<int, 1, 2>;
    using float1x2  = mat<float, 1, 2>;
    using double1x2 = mat<double, 1, 2>;
    using bool1x3   = mat<bool, 1, 3>;
    using int1x3    = mat<int, 1, 3>;
    using float1x3  = mat<float, 1, 3>;
    using double1x3 = mat<double, 1, 3>;
    using bool1x4   = mat<bool, 1, 4>;
    using int1x4    = mat<int, 1, 4>;
    using float1x4  = mat<float, 1, 4>;
    using double1x4 = mat<double, 1, 4>;
    using bool2x1   = mat<bool, 2, 1>;
    using int2x1    = mat<int, 2, 1>;
    using float2x1  = mat<float, 2, 1>;
    using double2x1 = mat<double, 2, 1>;
    using bool2x2   = mat<bool, 2, 2>;
    using int2x2    = mat<int, 2, 2>;
    using float2x2  = mat<float, 2, 2>;
    using double2x2 = mat<double, 2, 2>;
    using bool2x3   = mat<bool, 2, 3>;
    using int2x3    = mat<int, 2, 3>;
    using float2x3  = mat<float, 2, 3>;
    using double2x3 = mat<double, 2, 3>;
    using bool2x4   = mat<bool, 2, 4>;
    using int2x4    = mat<int, 2, 4>;
    using float2x4  = mat<float, 2, 4>;
    using double2x4 = mat<double, 2, 4>;
    using bool3x1   = mat<bool, 3, 1>;
    using int3x1    = mat<int, 3, 1>;
    using float3x1  = mat<float, 3, 1>;
    using double3x1 = mat<double, 3, 1>;
    using bool3x2   = mat<bool, 3, 2>;
    using int3x2    = mat<int, 3, 2>;
    using float3x2  = mat<float, 3, 2>;
    using double3x2 = mat<double, 3, 2>;
    using bool3x3   = mat<bool, 3, 3>;
    using int3x3    = mat<int, 3, 3>;
    using float3x3  = mat<float, 3, 3>;
    using double3x3 = mat<double, 3, 3>;
    using bool3x4   = mat<bool, 3, 4>;
    using int3x4    = mat<int, 3, 4>;
    using float3x4  = mat<float, 3, 4>;
    using double3x4 = mat<double, 3, 4>;
    using bool4x1   = mat<bool, 4, 1>;
    using int4x1    = mat<int, 4, 1>;
    using float4x1  = mat<float, 4, 1>;
    using double4x1 = mat<double, 4, 1>;
    using bool4x2   = mat<bool, 4, 2>;
    using int4x2    = mat<int, 4, 2>;
    using float4x2  = mat<float, 4, 2>;
    using double4x2 = mat<double, 4, 2>;
    using bool4x3   = mat<bool, 4, 3>;
    using int4x3    = mat<int, 4, 3>;
    using float4x3  = mat<float, 4, 3>;
    using double4x3 = mat<double, 4, 3>;
    using bool4x4   = mat<bool, 4, 4>;
    using int4x4    = mat<int, 4, 4>;
    using float4x4  = mat<float, 4, 4>;
    using double4x4 = mat<double, 4, 4>;
}

// Provide output streaming operators, writing something that resembles an aggregate literal that could be used to construct the
// specified value
namespace ostream_overloads
{
    template <class C, class T> auto operator<<(std::basic_ostream<C>& out, const vec<T, 1>& v) -> std::basic_ostream<C>&
    {
        return out << '{' << v[0] << '}';
    }
    template <class C, class T> auto operator<<(std::basic_ostream<C>& out, const vec<T, 2>& v) -> std::basic_ostream<C>&
    {
        return out << '{' << v[0] << ',' << v[1] << '}';
    }
    template <class C, class T> auto operator<<(std::basic_ostream<C>& out, const vec<T, 3>& v) -> std::basic_ostream<C>&
    {
        return out << '{' << v[0] << ',' << v[1] << ',' << v[2] << '}';
    }
    template <class C, class T> auto operator<<(std::basic_ostream<C>& out, const vec<T, 4>& v) -> std::basic_ostream<C>&
    {
        return out << '{' << v[0] << ',' << v[1] << ',' << v[2] << ',' << v[3] << '}';
    }

    template <class C, class T, int M>
    auto operator<<(std::basic_ostream<C>& out, const mat<T, M, 1>& m) -> std::basic_ostream<C>&
    {
        return out << '{' << m[0] << '}';
    }
    template <class C, class T, int M>
    auto operator<<(std::basic_ostream<C>& out, const mat<T, M, 2>& m) -> std::basic_ostream<C>&
    {
        return out << '{' << m[0] << ',' << m[1] << '}';
    }
    template <class C, class T, int M>
    auto operator<<(std::basic_ostream<C>& out, const mat<T, M, 3>& m) -> std::basic_ostream<C>&
    {
        return out << '{' << m[0] << ',' << m[1] << ',' << m[2] << '}';
    }
    template <class C, class T, int M>
    auto operator<<(std::basic_ostream<C>& out, const mat<T, M, 4>& m) -> std::basic_ostream<C>&
    {
        return out << '{' << m[0] << ',' << m[1] << ',' << m[2] << ',' << m[3] << '}';
    }
}
}

namespace std
{
// Provide specializations for std::hash<...> with linalg types
template <class T> struct hash<linalg::vec<T, 1>>
{
    auto operator()(const linalg::vec<T, 1>& v) const -> std::size_t
    {
        std::hash<T> h;
        return h(v.x);
    }
};
template <class T> struct hash<linalg::vec<T, 2>>
{
    auto operator()(const linalg::vec<T, 2>& v) const -> std::size_t
    {
        std::hash<T> h;
        return h(v.x) ^ (h(v.y) << 1);
    }
};
template <class T> struct hash<linalg::vec<T, 3>>
{
    auto operator()(const linalg::vec<T, 3>& v) const -> std::size_t
    {
        std::hash<T> h;
        return h(v.x) ^ (h(v.y) << 1) ^ (h(v.z) << 2);
    }
};
template <class T> struct hash<linalg::vec<T, 4>>
{
    auto operator()(const linalg::vec<T, 4>& v) const -> std::size_t
    {
        std::hash<T> h;
        return h(v.x) ^ (h(v.y) << 1) ^ (h(v.z) << 2) ^ (h(v.w) << 3);
    }
};

template <class T, int M> struct hash<linalg::mat<T, M, 1>>
{
    auto operator()(const linalg::mat<T, M, 1>& v) const -> std::size_t
    {
        std::hash<linalg::vec<T, M>> h;
        return h(v.x);
    }
};
template <class T, int M> struct hash<linalg::mat<T, M, 2>>
{
    auto operator()(const linalg::mat<T, M, 2>& v) const -> std::size_t
    {
        std::hash<linalg::vec<T, M>> h;
        return h(v.x) ^ (h(v.y) << M);
    }
};
template <class T, int M> struct hash<linalg::mat<T, M, 3>>
{
    auto operator()(const linalg::mat<T, M, 3>& v) const -> std::size_t
    {
        std::hash<linalg::vec<T, M>> h;
        return h(v.x) ^ (h(v.y) << M) ^ (h(v.z) << (M * 2));
    }
};
template <class T, int M> struct hash<linalg::mat<T, M, 4>>
{
    auto operator()(const linalg::mat<T, M, 4>& v) const -> std::size_t
    {
        std::hash<linalg::vec<T, M>> h;
        return h(v.x) ^ (h(v.y) << M) ^ (h(v.z) << (M * 2)) ^ (h(v.w) << (M * 3));
    }
};
}

// Definitions of functions too long to be defined inline
template <class T> constexpr auto linalg::adjugate(const mat<T, 3, 3>& a) -> linalg::mat<T, 3, 3>
{
    return { { a.y.y * a.z.z - a.z.y * a.y.z, a.z.y * a.x.z - a.x.y * a.z.z, a.x.y * a.y.z - a.y.y * a.x.z },
             { a.y.z * a.z.x - a.z.z * a.y.x, a.z.z * a.x.x - a.x.z * a.z.x, a.x.z * a.y.x - a.y.z * a.x.x },
             { a.y.x * a.z.y - a.z.x * a.y.y, a.z.x * a.x.y - a.x.x * a.z.y, a.x.x * a.y.y - a.y.x * a.x.y } };
}

template <class T> constexpr auto linalg::adjugate(const mat<T, 4, 4>& a) -> linalg::mat<T, 4, 4>
{
    return { { a.y.y * a.z.z * a.w.w + a.w.y * a.y.z * a.z.w + a.z.y * a.w.z * a.y.w - a.y.y * a.w.z * a.z.w
                   - a.z.y * a.y.z * a.w.w - a.w.y * a.z.z * a.y.w,
               a.x.y * a.w.z * a.z.w + a.z.y * a.x.z * a.w.w + a.w.y * a.z.z * a.x.w - a.w.y * a.x.z * a.z.w
                   - a.z.y * a.w.z * a.x.w - a.x.y * a.z.z * a.w.w,
               a.x.y * a.y.z * a.w.w + a.w.y * a.x.z * a.y.w + a.y.y * a.w.z * a.x.w - a.x.y * a.w.z * a.y.w
                   - a.y.y * a.x.z * a.w.w - a.w.y * a.y.z * a.x.w,
               a.x.y * a.z.z * a.y.w + a.y.y * a.x.z * a.z.w + a.z.y * a.y.z * a.x.w - a.x.y * a.y.z * a.z.w
                   - a.z.y * a.x.z * a.y.w - a.y.y * a.z.z * a.x.w },
             { a.y.z * a.w.w * a.z.x + a.z.z * a.y.w * a.w.x + a.w.z * a.z.w * a.y.x - a.y.z * a.z.w * a.w.x
                   - a.w.z * a.y.w * a.z.x - a.z.z * a.w.w * a.y.x,
               a.x.z * a.z.w * a.w.x + a.w.z * a.x.w * a.z.x + a.z.z * a.w.w * a.x.x - a.x.z * a.w.w * a.z.x
                   - a.z.z * a.x.w * a.w.x - a.w.z * a.z.w * a.x.x,
               a.x.z * a.w.w * a.y.x + a.y.z * a.x.w * a.w.x + a.w.z * a.y.w * a.x.x - a.x.z * a.y.w * a.w.x
                   - a.w.z * a.x.w * a.y.x - a.y.z * a.w.w * a.x.x,
               a.x.z * a.y.w * a.z.x + a.z.z * a.x.w * a.y.x + a.y.z * a.z.w * a.x.x - a.x.z * a.z.w * a.y.x
                   - a.y.z * a.x.w * a.z.x - a.z.z * a.y.w * a.x.x },
             { a.y.w * a.z.x * a.w.y + a.w.w * a.y.x * a.z.y + a.z.w * a.w.x * a.y.y - a.y.w * a.w.x * a.z.y
                   - a.z.w * a.y.x * a.w.y - a.w.w * a.z.x * a.y.y,
               a.x.w * a.w.x * a.z.y + a.z.w * a.x.x * a.w.y + a.w.w * a.z.x * a.x.y - a.x.w * a.z.x * a.w.y
                   - a.w.w * a.x.x * a.z.y - a.z.w * a.w.x * a.x.y,
               a.x.w * a.y.x * a.w.y + a.w.w * a.x.x * a.y.y + a.y.w * a.w.x * a.x.y - a.x.w * a.w.x * a.y.y
                   - a.y.w * a.x.x * a.w.y - a.w.w * a.y.x * a.x.y,
               a.x.w * a.z.x * a.y.y + a.y.w * a.x.x * a.z.y + a.z.w * a.y.x * a.x.y - a.x.w * a.y.x * a.z.y
                   - a.z.w * a.x.x * a.y.y - a.y.w * a.z.x * a.x.y },
             { a.y.x * a.w.y * a.z.z + a.z.x * a.y.y * a.w.z + a.w.x * a.z.y * a.y.z - a.y.x * a.z.y * a.w.z
                   - a.w.x * a.y.y * a.z.z - a.z.x * a.w.y * a.y.z,
               a.x.x * a.z.y * a.w.z + a.w.x * a.x.y * a.z.z + a.z.x * a.w.y * a.x.z - a.x.x * a.w.y * a.z.z
                   - a.z.x * a.x.y * a.w.z - a.w.x * a.z.y * a.x.z,
               a.x.x * a.w.y * a.y.z + a.y.x * a.x.y * a.w.z + a.w.x * a.y.y * a.x.z - a.x.x * a.y.y * a.w.z
                   - a.w.x * a.x.y * a.y.z - a.y.x * a.w.y * a.x.z,
               a.x.x * a.y.y * a.z.z + a.z.x * a.x.y * a.y.z + a.y.x * a.z.y * a.x.z - a.x.x * a.z.y * a.y.z
                   - a.y.x * a.x.y * a.z.z - a.z.x * a.y.y * a.x.z } };
}

template <class T> constexpr auto linalg::determinant(const mat<T, 4, 4>& a) -> T
{
    return a.x.x
        * (a.y.y * a.z.z * a.w.w + a.w.y * a.y.z * a.z.w + a.z.y * a.w.z * a.y.w - a.y.y * a.w.z * a.z.w - a.z.y * a.y.z * a.w.w
           - a.w.y * a.z.z * a.y.w)
        + a.x.y
        * (a.y.z * a.w.w * a.z.x + a.z.z * a.y.w * a.w.x + a.w.z * a.z.w * a.y.x - a.y.z * a.z.w * a.w.x - a.w.z * a.y.w * a.z.x
           - a.z.z * a.w.w * a.y.x)
        + a.x.z
        * (a.y.w * a.z.x * a.w.y + a.w.w * a.y.x * a.z.y + a.z.w * a.w.x * a.y.y - a.y.w * a.w.x * a.z.y - a.z.w * a.y.x * a.w.y
           - a.w.w * a.z.x * a.y.y)
        + a.x.w
        * (a.y.x * a.w.y * a.z.z + a.z.x * a.y.y * a.w.z + a.w.x * a.z.y * a.y.z - a.y.x * a.z.y * a.w.z - a.w.x * a.y.y * a.z.z
           - a.z.x * a.w.y * a.y.z);
}

template <class T> auto linalg::rotation_quat(const mat<T, 3, 3>& m) -> linalg::vec<T, 4>
{
    const vec<T, 4> q { m.x.x - m.y.y - m.z.z, m.y.y - m.x.x - m.z.z, m.z.z - m.x.x - m.y.y, m.x.x + m.y.y + m.z.z };
    const vec<T, 4> s[] { { 1, m.x.y + m.y.x, m.z.x + m.x.z, m.y.z - m.z.y },
                          { m.x.y + m.y.x, 1, m.y.z + m.z.y, m.z.x - m.x.z },
                          { m.x.z + m.z.x, m.y.z + m.z.y, 1, m.x.y - m.y.x },
                          { m.y.z - m.z.y, m.z.x - m.x.z, m.x.y - m.y.x, 1 } };
    return copysign(normalize(sqrt(max(T(0), T(1) + q))), s[argmax(q)]);
}

template <class T> auto linalg::frustum_matrix(T x0, T x1, T y0, T y1, T n, T f, fwd_axis a, z_range z) -> linalg::mat<T, 4, 4>
{
    const T s = a == pos_z ? T(1) : T(-1);
    const T o = z == neg_one_to_one ? n : 0;
    return { { 2 * n / (x1 - x0), 0, 0, 0 },
             { 0, 2 * n / (y1 - y0), 0, 0 },
             { -s * (x0 + x1) / (x1 - x0), -s * (y0 + y1) / (y1 - y0), s * (f + o) / (f - n), s },
             { 0, 0, -(n + o) * f / (f - n), 0 } };
}

#endif
