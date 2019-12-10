//
// Created by john on 12/10/19.
//

#ifndef RTT_RTT_TRAITS_H
#define RTT_RTT_TRAITS_H

#include <list>
#include <type_traits>

#include "roboteam_utils/Vector2.h"

namespace rtt::ai::position::type_traits {

    /**
     * Type trait that checks whether type T has a specific member function
     * Member function is:
     *  computePath which takes two const Vector2&'s.
     *  computePath returns std::list<rtt::Vector2>
     * @tparam T Type to check against
     *
     * Has constexpr static boolean ::value for actual value
     *
     * This is the base false case
     */
    template<typename T, typename = void>
    struct has_path_computation : std::false_type {
    };

    /**
     * True case for has_path_computation
     * @tparam T type to check
     *
     * std::enable_if_t disables a certain function if the value after it is not true
     * std::is_same_v checks whether 2 types are the same
     *  std::is_same_v<bool, bool> == true
     *  std::is_same_v<bool, std::vector<bool>> == false
     * std::declval<T>() default constructs the T type even if this is not allowed, compiletime, so no harm
     *
     * TODO: C++20 -> refactor to Concept
     */
    template<typename T>
    struct has_path_computation<T, std::enable_if_t<std::is_same_v<decltype(
    std::declval<T>().computePath(Vector2(), Vector2())), std::list<rtt::Vector2>>>>
            : std::true_type {
    };

#if __cplusplus > 201703L
    /**
     * Concept that essentially ensures the same as has_path_computation
     * @tparam T type to check
     */
    template <typename T>
    concept PathComputer = requires(T a) {
        { a.computePath(Vector2(), Vector2()) } -> std::list<rtt::Vector2>;
    }
#endif

    /**
     * Base false case for type trait has_path_tracking
     * @tparam T type to check conditions for
     *
     * ::value is True if T has a member function called trackPath
     * trackPath can have any return type
     * trackPath must have the following arguments, in order:
     *      const Vector2&
     *      const Vector2&
     *      std::list<rtt::Vector2>&
     *      Vector2&
     *      double&
     *
     * Otherwise false
     */
    template <typename T, typename = void>
    struct  has_path_tracking : std::false_type {};

    /**
     * True case for typetrait has_path_tracking
     * @tparam T type to check against
     *
     * std::declval<T&> constructs a reference that's default constructed.
     * Keep in mind std::declval can only be used in decltype, meaning it won't actually get evaluated
     * https://en.cppreference.com/w/cpp/utility/declval
     *
     * TODO: C++20 -> translate to Concept (Not sure how to do non-const lvalue references in concepts yet, hence
     * for now not done yet)
     */
    template <typename T>
    struct has_path_tracking<T,
            decltype(std::declval<T>().trackPath(Vector2(), Vector2(), std::declval<std::list<Vector2>&>(), std::declval<Vector2&>(), std::declval<double&>()))>
            : std::true_type { };


} // rtt::ai::position::type_traits

#endif //RTT_RTT_TRAITS_H
