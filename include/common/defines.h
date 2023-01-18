#ifndef DEFINE_HAS_PARAM
#define DEFINE_HAS_PARAM(x) \
namespace hippo {\
    template<typename T, typename = void>\
    struct has_param_##x {\
        static constexpr bool value = false;\
    };\
    \
    template<typename T>\
    struct has_param_##x<T, decltype((void) T::x)> {\
        typedef T type;\
        static constexpr bool value = true;\
    };\
    \
    template<typename T>\
    using has_param_##x##_t = typename has_param_##x<T>::type;\
    \
    template<typename T>\
    constexpr bool has_param_##x##_v() {return has_param_##x<T>::value;}\
    \
    template<typename T, typename = void>\
    struct has_no_param_##x {\
        typedef T type;\
        static constexpr bool value = true;\
    };\
    \
    template<typename T>\
    struct has_no_param_##x<T, decltype((void) T::x)> {\
        static constexpr bool value = false;\
    };\
    \
    template<typename T>\
    using has_no_param_##x##_t = typename has_no_param_##x<T>::type;\
    \
    template<typename T>\
    constexpr bool has_no_param_##x##_v() {return has_no_param_##x<T>::value;}\
}
#endif


#define DEFAULT_RATE 200.0