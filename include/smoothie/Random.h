#pragma once

#include <random>

namespace smoothie
{

template <typename T>
class Distribution
{
public:
    Distribution(T d, std::default_random_engine &e)
        : d_{ std::move(d) }
        , e_{ e }
    {
    }

    auto operator()() { return d_(e_); }

private:
    T d_;
    std::default_random_engine &e_;
};

class RandomEngine
{
public:
#ifdef __clang__
#pragma clang diagnostic push
#pragma ide diagnostic ignored "google-explicit-constructor"
#endif
    operator std::default_random_engine &() { return engine_; }
#ifdef __clang__
#pragma clang diagnostic pop
#endif

private:
    std::random_device rd_;
    std::default_random_engine engine_{ rd_() };
};

struct Random
{
    static std::default_random_engine &randomEngine()
    {
        static RandomEngine instance;
        return instance;
    }

    template <typename T, typename std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
    static auto randomDistribution(T min, T max, std::default_random_engine &e = randomEngine())
    {
        using D = std::uniform_real_distribution<T>;
        D d{ min, max };
        return Distribution<D>(d, e);
    }

    static auto random01()
    {
        static auto distribution = randomDistribution<float>(0.0f, 1.0f);
        return distribution();
    }
};

}
