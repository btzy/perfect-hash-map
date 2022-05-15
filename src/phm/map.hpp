#pragma once

#include <array>
#include <concepts>
#include <functional>
#include <utility>
#include <type_traits>

inline int f() {
    return 42;
}

namespace phm {

namespace detail {
/*template <typename Key, size_t I, Key... key>
struct get_elem;
template <typename Key, size_t I, Key key, Key... okey>
struct get_elem<Key, I, key, okey...> {
        constexpr static Key value = typename get_elem<Key, I - 1, okey...>::value;
};
template <typename Key, Key key, Key... okey>
struct get_elem<Key, 0, key, okey...> {
        constexpr static Key value = key;
};
template <typename Key, size_t I>
struct get_elem<Key, I> {
        static_assert(I != I, "Index out of bounds");
};
template <typename Key, size_t I, Key... key>
constexpr inline Key get_elem_v = get_elem<Key, I, key>::value;*/
}

template <typename Key, Key... lkey>
struct keys {
    using key_type = Key;
    constexpr static std::size_t size = sizeof...(lkey);
    constexpr static Key get(std::size_t I) noexcept { return std::array<Key, size>{lkey...}[I]; }
    template <std::size_t I>
    constexpr static Key key = get(I);

    // Sanity check to ensure that there is at least one key; this is required because we want to
    // fill in the unused indices of the key table with the first valid key.
    static_assert(size > 0, "There should be at least one key");
};

namespace detail {
template <typename T, typename MakeSentinel, std::size_t... I>
constexpr std::array<T, sizeof...(I)> make_initialized_values(
    MakeSentinel make_sentinel,
    std::index_sequence<I...>) noexcept(noexcept(make_sentinel())) {
    return {(I, make_sentinel())...};
}

template <auto Value>
struct return_value {
    constexpr auto operator()() const noexcept { return Value; }
};

// Makes the table of keys.  Unused indices are set to some other valid key; this ensures that a
// checked operation that hashes to this index will always say that it is an incorrect key.
template <typename Keys, typename PerfectHash, std::size_t NumBuckets>
constexpr std::array<typename Keys::key_type, NumBuckets> make_keys_arr() noexcept {
    std::array<typename Keys::key_type, NumBuckets> res =
        make_initialized_values<typename Keys::key_type, return_value<Keys::get(0)>>(
            {}, std::make_index_sequence<NumBuckets>{});
    for (std::size_t i = 0; i != Keys::size; ++i) {
        typename Keys::key_type key = Keys::get(i);
        std::size_t idx = PerfectHash{}(key);
        res[idx] = key;
    }
    return res;
}
}  // namespace detail

template <typename MakeSentinel>
struct is_equal_to_sentinel {
    [[no_unique_address]] MakeSentinel make_sentinel;
    template <typename U>
    bool operator()(const U& u) const noexcept(noexcept(u == make_sentinel())) {
        return u == make_sentinel();
    }
};

// The hash map consists of two parts: the keys array and the values array.  The keys array is
// constexpr and there is one copy per template instantiation.  There is one values array per
// hashmap.  Importantly, PerfectHash does not satisfy Hash!  PerfectHash is required to return
// values between 0 and NumBuckets-1 inclusive. Keys::key_type should be trivial.  PerfectHash
// should be constexpr and stateless, because we need to perform hashing at compile time sometimes.
// If using heterogenous lookup, then PerfectHash and KeyEqual must also support the lookup type.
//
// Member functions whose parallel in std::unordered_map return iterators only return a pointer to
// the mapped type here.
//
// Unless the function ends with "_checked", they assume that the given key is in the set of valid
// Keys.  If it isn't then it might cause collisions.  Checked operations check against the key
// table, and bails out if they key doesn't match.  This works even for unused indices because they
// are set to some other key (from a different index), and hence never compare equal to something
// that hashes to this index.
template <typename Keys,
          typename T,
          typename PerfectHash,
          std::size_t NumBuckets,
          typename MakeSentinel,
          typename KeyEqual = std::equal_to<typename Keys::key_type>,
          typename IsSentinel = is_equal_to_sentinel<MakeSentinel>>
class perfect_hash_map_with_sentinel_value {
   private:
    // Keys array (used with checked operations only).
    constexpr static std::array<typename Keys::key_type, NumBuckets> keys_ =
        detail::make_keys_arr<Keys, PerfectHash, NumBuckets>();

    [[no_unique_address]] MakeSentinel make_sentinel_;
    [[no_unique_address]] KeyEqual key_equal_;
    [[no_unique_address]] IsSentinel is_sentinel_;
    [[no_unique_address]] std::array<T, NumBuckets> values_;

   public:
    using key_type = typename Keys::key_type;
    using mapped_type = T;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using perfect_hasher = PerfectHash;
    using key_equal = KeyEqual;
    using mapped_iterator_type = T*;
    using mapped_const_iterator_type = const T*;

    //=== Some sanity checks ===//
    // MakeSentinel returns something convertible to T
    static_assert(std::is_constructible_v<T, std::invoke_result_t<MakeSentinel>>,
                  "MakeSentinel must return T when called with no arguments");
    static_assert(std::is_convertible_v<std::invoke_result_t<IsSentinel, T>, bool>,
                  "IsSentinel must return bool when called with a T argument");

    template <typename MS = MakeSentinel, typename KE = KeyEqual, typename IS = IsSentinel>
    requires std::constructible_from<KeyEqual, KE&&> && std::constructible_from<IsSentinel, IS&&>
    constexpr perfect_hash_map_with_sentinel_value(
        MS&& make_sentinel = MS{},
        KE&& key_equal = KE{},
        IS&& is_sentinel = IS{MS{}}) noexcept(noexcept(std::declval<MS>()) &&
                                              std::is_nothrow_constructible_v<MakeSentinel, MS&&> &&
                                              std::is_nothrow_constructible_v<KeyEqual, KE&&> &&
                                              std::is_nothrow_constructible_v<IsSentinel, IS&&>)
        : make_sentinel_(std::forward<MS>(make_sentinel)),
          key_equal_(std::forward<KE>(key_equal)),
          is_sentinel_(std::forward<IS>(is_sentinel)),
          values_{detail::make_initialized_values<T, MakeSentinel>(
              make_sentinel_,
              std::make_index_sequence<NumBuckets>{})} {}

    // Emplaces a value if it doesn't exist yet, like the try_emplace for std::unordered_map, but
    // does an additional move assignment since the underlying storage is already T.
    // Returns pair<mapped_iterator_type, bool>.
    template <typename K, typename... Args>
    requires std::constructible_from<T, Args&&...>
    constexpr std::pair<T*, bool> try_emplace(const K& k, Args&&... args) noexcept(
        noexcept(PerfectHash{}(k)) && noexcept(is_sentinel_(std::declval<T&>())) &&
        std::is_nothrow_constructible_v<T, Args&&...> && std::is_nothrow_move_assignable_v<T>) {
        const std::size_t idx = PerfectHash{}(k);
        T& val = values_[idx];
        if (is_sentinel_(val)) {
            return {&(val = T{std::forward<Args>(args)...}), true};
        } else {
            return {&val, false};
        }
    }

    // Emplaces or replaces a value, but does an additional move assignment since the underlying
    // storage is already T. Returns pair<mapped_iterator_type, bool>.
    template <typename K, typename... Args>
    requires std::constructible_from<T, Args&&...>
    constexpr std::pair<T*, bool> try_emplace_or_assign(const K& k, Args&&... args) noexcept(
        noexcept(PerfectHash{}(k)) && noexcept(is_sentinel(std::declval<T&>())) &&
        std::is_nothrow_constructible_v<T, Args&&...> && std::is_nothrow_move_assignable_v<T>) {
        const std::size_t idx = PerfectHash{}(k);
        T& val = values_[idx];
        if (is_sentinel_(val)) {
            return {&(val = T{std::forward<Args>(args)...}), true};
        } else {
            return {&(val = T{std::forward<Args>(args)...}), false};
        }
    }

    // Gets the value associated with this key.  If they key is not associated with a value, returns
    // a references to the value created by make_sentinel().
    template <auto k>
    constexpr T& lookup() noexcept {
        constexpr std::size_t idx = PerfectHash{}(k);
        return values_[idx];
    }
    template <auto k>
    constexpr const T& lookup() const noexcept {
        constexpr std::size_t idx = PerfectHash{}(k);
        return values_[idx];
    }

    // Gets the value associated with this key.  If they key is not associated with a value, returns
    // a references to the value created by make_sentinel().
    template <typename K>
    constexpr T& lookup(const K& k) noexcept(noexcept(PerfectHash{}(k))) {
        const std::size_t idx = PerfectHash{}(k);
        return values_[idx];
    }
    template <typename K>
    constexpr T& operator[](const K& k) noexcept(noexcept(PerfectHash{}(k))) {
        return lookup(k);
    }
    template <typename K>
    constexpr const T& lookup(const K& k) const noexcept(noexcept(PerfectHash{}(k))) {
        const std::size_t idx = PerfectHash{}(k);
        return values_[idx];
    }
    template <typename K>
    constexpr const T& operator[](const K& k) const noexcept(noexcept(PerfectHash{}(k))) {
        return lookup(k);
    }

    // Returns an iterator to the element matching the specified key.  If the key cannot be found,
    // returns end().  This function is for compatibility with std::unordered_map, but might be a
    // bit slower than lookup_unchecked().  For optimal performance, you should use
    // lookup_unchecked() and then use is_sentinel() to test the return value if necessary.
    template <typename K>
    constexpr T* find(const K& k) noexcept(
        noexcept(PerfectHash{}(k)) && noexcept(is_sentinel(std::declval<T&>()))) {
        T& val = lookup(k);
        if (is_sentinel_(val)) {
            return end();
        }
        return &val;
    }
    template <typename K>
    constexpr const T* find(const K& k) const
        noexcept(noexcept(PerfectHash{}(k)) && noexcept(is_sentinel(std::declval<T&>()))) {
        T& val = lookup(k);
        if (is_sentinel(val)) {
            return end();
        }
        return &val;
    }

    ////////////// Iterators //////////////
    T* begin() noexcept { return values_.data(); }
    T* end() noexcept { return values_.data() + NumBuckets; }
};
}  // namespace phm
