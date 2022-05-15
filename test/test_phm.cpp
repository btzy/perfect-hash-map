#include <catch2/catch_all.hpp>

#include <phm/phm.hpp>

#include <functional>
#include <string>

struct return_empty_string {
    std::string operator()() const noexcept { return {}; }
};

TEST_CASE("easy test", "[map]") {
    phm::perfect_hash_map_with_sentinel_value<phm::keys<std::size_t, 1, 2, 3>,
                                              std::string,
                                              std::identity,
                                              4,
                                              return_empty_string>
        map;
    REQUIRE(map.try_emplace(1, "one").second);
    REQUIRE(!map.try_emplace(1, "one2").second);
    REQUIRE(map.try_emplace(3, "three").second);
    REQUIRE(!map.try_emplace(3, "three2").second);
    REQUIRE(map.try_emplace(2, "two").second);
    REQUIRE(!map.try_emplace(2, "two2").second);
    REQUIRE(map.lookup<2>() == "two");
    REQUIRE(map.lookup(2) == "two");
    REQUIRE(map.lookup<1>() == "one");
    REQUIRE(map.lookup(1) == "one");
    REQUIRE(map.lookup<3>() == "three");
    REQUIRE(map.lookup(3) == "three");
    REQUIRE(map.lookup(0) == return_empty_string{}());
}
