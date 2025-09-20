//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2024 Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
//
//   Permission to use, copy, modify, and/or distribute this software for any purpose with or
//   without fee is hereby granted, provided that the above copyright notice and this permission
//   notice appear in all copies. THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
//   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
//   AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
//   CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
//   WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
//   CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#pragma once

#include <utility>
#include <iterator>
#include <type_traits>

namespace JPL
{
    /// Zip iterator for the FlatMap

    // ----- pair_ref & arrow proxy -----
    template<class First, class Second>
    struct pair_ref
    {
        First& first;
        Second& second;

        // Allows structured bindings and easy copying to a real pair if needed
        constexpr operator std::pair<First, Second>() const { return { first, second }; }
    };

    template<class First, class Second>
    struct arrow_proxy
    {
        pair_ref<First, Second> ref;
        constexpr pair_ref<First, Second>* operator->() noexcept { return &ref; }
    };

    // ----- zip_iter -----
    template<class First, class Second>
    class zip_iter
    {
        template<class, class> friend class zip_iter; // allow access to p1_/p2_
    public:
        using difference_type = std::ptrdiff_t;
        using value_type = std::pair<std::remove_cv_t<First>, std::remove_cv_t<Second>>;
        using reference = pair_ref<First, Second>;
        using pointer = arrow_proxy<First, Second>;
        using iterator_category = std::random_access_iterator_tag;

        constexpr zip_iter() noexcept : p1_(nullptr), p2_(nullptr) {}
        constexpr zip_iter(First* p1, Second* p2) noexcept : p1_(p1), p2_(p2) {}

        // dereference
        constexpr reference operator*()  const noexcept { return { *p1_, *p2_ }; }
        constexpr pointer   operator->() const noexcept { return pointer{ **this }; }
        constexpr reference operator[](difference_type n) const noexcept { return *(*this + n); }

        // ++/--
        constexpr zip_iter& operator++()    noexcept { ++p1_; ++p2_; return *this; }
        constexpr zip_iter  operator++(int) noexcept { auto tmp = *this; ++(*this); return tmp; }
        constexpr zip_iter& operator--()    noexcept { --p1_; --p2_; return *this; }
        constexpr zip_iter  operator--(int) noexcept { auto tmp = *this; --(*this); return tmp; }

        // +/- n
        constexpr zip_iter& operator+=(difference_type n) noexcept { p1_ += n; p2_ += n; return *this; }
        constexpr zip_iter& operator-=(difference_type n) noexcept { return (*this += -n); }

        friend zip_iter operator+(zip_iter it, difference_type n) noexcept { it += n; return it; }
        friend zip_iter operator+(difference_type n, zip_iter it) noexcept { it += n; return it; }
        friend zip_iter operator-(zip_iter it, difference_type n) noexcept { it -= n; return it; }

        // distance (supports const/non-const mixes)
        template<class F2, class S2>
            requires (std::is_same_v<std::remove_cv_t<F2>, std::remove_cv_t<First>>
        && std::is_same_v<std::remove_cv_t<S2>, std::remove_cv_t<Second>>)
            friend difference_type operator-(const zip_iter& a, const zip_iter<F2, S2>& b) noexcept
        {
            return a.p1_ - b.p1_;
        }

        // comparisons (compare by the first pointer; both advance together)
        template<class F2, class S2>
            requires (std::is_same_v<std::remove_cv_t<F2>, std::remove_cv_t<First>>
        && std::is_same_v<std::remove_cv_t<S2>, std::remove_cv_t<Second>>)
            friend bool operator==(const zip_iter& a, const zip_iter<F2, S2>& b) noexcept
        {
            return a.p1_ == b.p1_;
        }

        template<class F2, class S2>
            requires (std::is_same_v<std::remove_cv_t<F2>, std::remove_cv_t<First>>
        && std::is_same_v<std::remove_cv_t<S2>, std::remove_cv_t<Second>>)
            friend bool operator<(const zip_iter& a, const zip_iter<F2, S2>& b) noexcept
        {
            return a.p1_ < b.p1_;
        }

        template<class F2, class S2>
            requires (std::is_same_v<std::remove_cv_t<F2>, std::remove_cv_t<First>>
        && std::is_same_v<std::remove_cv_t<S2>, std::remove_cv_t<Second>>)
            friend bool operator!=(const zip_iter& a, const zip_iter<F2, S2>& b) noexcept { return !(a == b); }

        template<class F2, class S2>
            requires (std::is_same_v<std::remove_cv_t<F2>, std::remove_cv_t<First>>
        && std::is_same_v<std::remove_cv_t<S2>, std::remove_cv_t<Second>>)
            friend bool operator<=(const zip_iter& a, const zip_iter<F2, S2>& b) noexcept { return !(b < a); }

        template<class F2, class S2>
            requires (std::is_same_v<std::remove_cv_t<F2>, std::remove_cv_t<First>>
        && std::is_same_v<std::remove_cv_t<S2>, std::remove_cv_t<Second>>)
            friend bool operator>(const zip_iter& a, const zip_iter<F2, S2>& b) noexcept { return (b < a); }

        template<class F2, class S2>
            requires (std::is_same_v<std::remove_cv_t<F2>, std::remove_cv_t<First>>
        && std::is_same_v<std::remove_cv_t<S2>, std::remove_cv_t<Second>>)
            friend bool operator>=(const zip_iter& a, const zip_iter<F2, S2>& b) noexcept { return !(a < b); }

        // enabling conversion when pointer types are convertible (e.g., T* -> const T*)
        template<class F2, class S2> requires (
            std::is_same_v<std::remove_cv_t<F2>, std::remove_cv_t<First>> &&
            std::is_same_v<std::remove_cv_t<S2>, std::remove_cv_t<Second>> &&
            std::is_convertible_v<F2*, First*>&&
            std::is_convertible_v<S2*, Second*>)
            zip_iter(const zip_iter<F2, S2>& other) noexcept
            : p1_(other.p1_), p2_(other.p2_)
        {
        }

    private:
        First* p1_;
        Second* p2_;
    };


    // ----- tiny range helper -----
    template<class First, class Second>
    struct zip_range
    {
        using iterator = zip_iter<First, Second>;
        using const_iterator = zip_iter<const std::remove_cv_t<First>, const std::remove_cv_t<Second>>;

        First* a{};
        Second* b{};
        std::size_t n{};

        constexpr iterator begin() noexcept { return { a, b }; }
        constexpr iterator end()   noexcept { return { a + n, b + n }; }

        constexpr const_iterator begin() const noexcept { return { a, b }; }
        constexpr const_iterator end()   const noexcept { return { a + n, b + n }; }
    };

} // namespace JPL