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

#include "JPLSpatial/Containers/ZipIterator.h"

#if __cplusplus >= 202302L
#include <flat_map>
#else
#include <vector>
#include <utility>
#include <functional>
#include <stdexcept>
#include <initializer_list>
#endif

namespace JPL
{
#if __cplusplus >= 202302L
	template<class T, class ...Args>
	using FlatMap = std::flat_map<T, Args...>;
#else

	// std::flat_map is available since C++23
	// This is a simple implementation of the same concept.
	// This should be faster than a regular hash map for small number of elements.
	template<class Key,
		class T,
		class Equals = std::equal_to<Key>,
		class KeyContainer = std::vector<Key>,
		class MappedContainer = std::vector<T>>
	class FlatMap
	{
	public:
		using allocator_type = KeyContainer::allocator_type;

		using key_type = Key;
		using mapped_type = T;
		using value_type = std::pair<key_type, mapped_type>;
		using key_compare = Equals;

		using key_container_type = KeyContainer;
		using mapped_container_type = MappedContainer;
		
		using size_type = std::size_t;
		using difference_type = std::ptrdiff_t;
		
		using reference = std::pair<const key_type&, mapped_type&>;
		using const_reference = std::pair<const key_type&, const mapped_type&>;

		 // iterator forbids modifying keys (like std::map / std::flat_map)
		using iterator = zip_iter<const Key, T>;
		using const_iterator = zip_iter<const Key, const T>;

		//using reverse_iterator = std::reverse_iterator<iterator>;
		//using const_reverse_iterator = std::reverse_iterator<const_iterator>;
	public:

		FlatMap(const FlatMap&) = default;
		FlatMap(FlatMap&&) noexcept = default;
		FlatMap& operator=(const FlatMap&) = default;
		FlatMap& operator=(FlatMap&&) noexcept = default;
		~FlatMap() = default;

		FlatMap() = default;

		explicit FlatMap(const allocator_type& allocator) : cont(allocator) {}
		FlatMap(Equals eq, const allocator_type& allocator = {}) : eq(std::move(eq)), cont(allocator) {}

		template<class InputIter>
		FlatMap(InputIter first, InputIter last, Equals eq = {}, const allocator_type& allocator = {})
			: eq(std::move(eq))
			, cont(allocator)
		{
			const auto n = static_cast<size_type>(std::distance(first, last));
			cont.keys.reserve(n);
			cont.values.reserve(n);
			for (; first != last; ++first)
			{
				cont.keys.push_back(first->first);
				cont.values.push_back(first->second);
			}
		}

		explicit FlatMap(std::initializer_list<value_type> init, const allocator_type& allocator = {})
			: FlatMap(init.begin(), init.end(), {}, allocator)
		{
		}

		// Storage
		[[nodiscard]] JPL_INLINE const key_container_type& keys() const noexcept { return cont.keys; }
		[[nodiscard]] JPL_INLINE const mapped_container_type& values() const noexcept { return cont.values; }

		// Capacity
		[[nodiscard]] JPL_INLINE void       reserve(std::size_t n) { cont.keys.reserve(n); cont.values.reserve(n); }
		[[nodiscard]] JPL_INLINE size_type  size() const { return cont.keys.size(); }
		[[nodiscard]] JPL_INLINE bool       empty() const { return size() == 0; }

		// iteration
		[[nodiscard]] JPL_INLINE iterator       begin()        noexcept { return { cont.keys.data(), cont.values.data() }; }
		[[nodiscard]] JPL_INLINE iterator       end()          noexcept { return { cont.keys.data() + cont.keys.size(), cont.values.data() + cont.values.size() }; }
		[[nodiscard]] JPL_INLINE const_iterator begin()  const noexcept { return { cont.keys.data(), cont.values.data() }; }
		[[nodiscard]] JPL_INLINE const_iterator end()    const noexcept { return { cont.keys.data() + cont.keys.size(), cont.values.data() + cont.values.size() }; }
		[[nodiscard]] JPL_INLINE const_iterator cbegin() const noexcept { return begin(); }
		[[nodiscard]] JPL_INLINE const_iterator cend()   const noexcept { return end(); }

		// (keeps relative order; no uniqueness check)
		JPL_INLINE void push_back(const Key& k, const T& v) { cont.keys.push_back(k); cont.values.push_back(v); }

	private:
		// Heterogeneous support bits
		template<class L, class R>
		using EqResult = std::invoke_result_t<const Equals&, const L&, const R&>;

		template<class L, class R>
		static constexpr bool EqComparable =
			requires (const Equals & e, const L & l, const R & r)
		{
			{ std::invoke(e, l, r) } -> std::convertible_to<bool>;
		};

	public:

		// ----- lookup -----
		[[nodiscard]] JPL_INLINE iterator find(const Key& key)
		{
			auto it_key = std::ranges::find_if(cont.keys, [&](const Key& k) { return eq(k, key); });
			if (it_key == cont.keys.end())
				return end();
			return begin() + (it_key - cont.keys.begin());
		}

		[[nodiscard]] JPL_INLINE const_iterator find(const Key& key) const
		{
			auto it_key = std::ranges::find_if(cont.keys, [&](const Key& k) { return eq(k, key); });
			if (it_key == cont.keys.end())
				return end();
			return begin() + (it_key - cont.keys.begin());
		}

		// ----- heterogeneous find
		template<class K2> requires (EqComparable<Key, K2> || EqComparable<K2, Key>)
		[[nodiscard]] JPL_INLINE iterator find(const K2& key_like)
		{
			auto it_key = std::ranges::find_if(cont.keys, [&](const Key& k)
			{
				if constexpr (EqComparable<Key, K2>)
				{
					return eq(k, key_like);
				}
				else
				{
					return eq(key_like, k);
				}
			});
			if (it_key == cont.keys.end())
				return end();
			return begin() + (it_key - cont.keys.begin());
		}

		template<class K2> requires (EqComparable<Key, K2> || EqComparable<K2, Key>)
		[[nodiscard]] JPL_INLINE const_iterator find(const K2& key_like) const
		{
			auto it_key = std::ranges::find_if(cont.keys, [&](const Key& k)
			{
				if constexpr (EqComparable<Key, K2>)
				{
					return eq(k, key_like);
				}
				else
				{
					return eq(key_like, k);
				}
			});
			if (it_key == cont.keys.end())
				return end();
			return begin() + (it_key - cont.keys.begin());
		}

		// ----- bounds-checked access -----
		[[nodiscard]] JPL_INLINE mapped_type& at(const Key& key)
		{
			auto it = find(key);
			if (it == end())
				throw std::out_of_range("FlatMap::at: key not found");
			return it->second;
		}

		[[nodiscard]] JPL_INLINE const mapped_type& at(const Key& key) const
		{
			auto it = find(key);
			if (it == end())
				throw std::out_of_range("FlatMap::at: key not found");
			return it->second;
		}


		// ----- operator[] inserts default if absent (unsorted; appends) -----
		[[nodiscard]] JPL_INLINE mapped_type& operator[](const Key& key)
		{
			if (auto it = find(key); it != end())
				return it->second;
			cont.keys.push_back(key);
			cont.values.emplace_back();
			return cont.values.back();
		}

		[[nodiscard]] JPL_INLINE mapped_type& operator[](Key&& key)
		{
			if (auto it = find(key); it != end())
				return it->second;
			cont.keys.push_back(std::move(key));
			cont.values.emplace_back();
			return cont.values.back();
		}


		// ----- insertion (no sorting; enforces uniqueness) -----
		template<class... Args>
		JPL_INLINE std::pair<iterator, bool> emplace(Args&&... args)
		{
			std::pair<Key, T> t(std::forward<Args>(args)...);
			
			if (auto it = find(t.first); it != end())
				return { it, false }; // already exists

			// Append (unsorted policy)
			cont.keys.emplace_back(std::move(t.first));
			cont.values.emplace_back(std::move(t.second));

			return { end() - 1, true };
		}

		// ----- erase by iterator -----
		JPL_INLINE size_t erase(const_iterator iter)
		{
			if (iter == end())
				return 0;
			const auto idx = static_cast<size_type>(iter - cbegin());
			cont.keys.erase(cont.keys.begin() + static_cast<difference_type>(idx));
			cont.values.erase(cont.values.begin() + static_cast<difference_type>(idx));
			return 1;
		}

		JPL_INLINE size_t erase(iterator it)
		{
			if (it == end())
				return 0;
			const auto idx = static_cast<size_type>(it - begin());
			cont.keys.erase(cont.keys.begin() + static_cast<difference_type>(idx));
			cont.values.erase(cont.values.begin() + static_cast<difference_type>(idx));
			return 1;
		}

		// ----- erase by key -----
		JPL_INLINE size_t erase(const Key& key)
		{
			if (auto it = find(key); it != end())
				return erase(it);
			return 0;
		}

		// heterogeneous erase by key-like
		template<class K2> requires (EqComparable<Key, K2> || EqComparable<K2, Key>)
			JPL_INLINE size_t erase(const K2& key_like)
		{
			if (auto it = find(key_like); it != end())
				return erase(it);
			return 0;
		}

		// ----- erase_if (predicate sees a pair-like proxy) -----
		template<class Predicate>
		JPL_INLINE size_t erase_if(Predicate&& pred)
		{
			// Stable single-pass compaction over both arrays.
			const size_type n = size();
			size_type w = 0; // write index
			for (size_type i = 0; i < n; ++i)
			{
				pair_ref<const Key, T> ref{ cont.keys[i], cont.values[i] };
				if (!std::invoke(pred, ref))
				{
					// keep item i -> move it to slot w (only if gaps have opened)
					if (w != i)
					{
						cont.keys[w] = std::move(cont.keys[i]);
						cont.values[w] = std::move(cont.values[i]);
					}
					++w;
				}
			}
			const size_t removed = n - w;
			cont.keys.resize(w);
			cont.values.resize(w);
			return removed;
		}

		[[nodiscard]] JPL_INLINE bool contains(const Key& key) const
		{
			return find(key) != end();
		}

		template<class K2> requires (EqComparable<Key, K2> || EqComparable<K2, Key>)
		[[nodiscard]] JPL_INLINE bool contains(const K2& key_like) const
		{
			return find(key_like) != end();
		}

		friend void swap(FlatMap& a, FlatMap& b) noexcept(
			noexcept(std::swap(a.cont.keys, b.cont.keys)) &&
			noexcept(std::swap(a.cont.values, b.cont.values)) &&
			noexcept(std::swap(a.eq, b.eq))
			)
		{
			using std::swap;
			swap(a.cont.keys, b.cont.keys);
			swap(a.cont.values, b.cont.values);
			swap(a.eq, b.eq);
		}

		private:
			struct containers
			{
				key_container_type    keys;
				mapped_container_type values;

				containers() = default;
				containers(const allocator_type& allocator)
					: keys(allocator)
					, values(allocator)
				{
				}
			} cont;

			[[no_unique_address]] Equals eq;
	};
#endif

	// Alias to override just the allocator for the FlatMap
	template<class KeyType, class T, template<class> class AllocatorType>
	using FlatMapWithAllocator =
		FlatMap<
			KeyType,
			T,
			std::equal_to<KeyType>,
			std::vector<KeyType, AllocatorType<KeyType>>,
			std::vector<T, AllocatorType<T>>
		>;
} // namespace JPL