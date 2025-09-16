#pragma once
#include <cmath>
#include <iomanip>
#include <sstream>

#ifdef DOUBLE_PRECISION
using real = double;
#else
using real = float;
#endif

namespace OxygenMathLite
{
    namespace Constants
    {
        constexpr real PI = 3.14159265358979323846f;
        constexpr real TWO_PI = 2.0f * PI;
        constexpr real HALF_PI = 0.5f * PI;
        constexpr real DEG_TO_RAD = PI / 180.0f;
        constexpr real RAD_TO_DEG = 180.0f / PI;

        constexpr real Epsilon = 1e-6f;
    }
    namespace MathTools
    {
        inline real Clamp(real value, real min, real max) noexcept
        {
            return (value < min ? min : (value > max ? max : value));
        }
        inline real Lerp(real a, real b, real t) noexcept
        {
            return a + t * (b - a);
        }
        inline real ToRadians(real degrees)
        {
            return degrees * Constants::DEG_TO_RAD;
        }
        inline real ToDegrees(real radians)
        {
            return radians * Constants::RAD_TO_DEG;
        }

        template <typename T>
        void Swap(T &a, T &b) noexcept
        {
            T temp = a;
            a = b;
            b = temp;
        }
    }
    struct Vec2
    {
        real x = 0.0f;
        real y = 0.0f;

        // ========== 构造函数 ==========
        constexpr Vec2() : x(0.0f), y(0.0f) {}
        constexpr Vec2(real x, real y) : x(x), y(y) {}
        constexpr Vec2(const Vec2 &other) : x(other.x), y(other.y) {}
        constexpr Vec2(Vec2 &&other) : x(other.x), y(other.y) {}

        Vec2 &operator=(const Vec2 &other)
        {
            x = other.x;
            y = other.y;
            return *this;
        }
        Vec2 &operator=(Vec2 &&other)
        {
            x = other.x;
            y = other.y;
            return *this;
        }
        ~Vec2() = default;

        // ========== 静态工厂方法 ==========
        static Vec2 Zero() { return {0.0f, 0.0f}; }
        static Vec2 One() { return {1.0f, 1.0f}; }
        static Vec2 Up() { return {0.0f, 1.0f}; }
        static Vec2 Down() { return {0.0f, -1.0f}; }
        static Vec2 Left() { return {-1.0f, 0.0f}; }
        static Vec2 Right() { return {1.0f, 0.0f}; }

        // ========== 运算符重载 ==========
        Vec2 operator+(const Vec2 &other) const { return {x + other.x, y + other.y}; }
        Vec2 operator-(const Vec2 &other) const { return {x - other.x, y - other.y}; }
        Vec2 operator*(real scalar) const { return {x * scalar, y * scalar}; }
        Vec2 operator/(real scalar) const { return {x / scalar, y / scalar}; }
        Vec2 operator-() const { return {-x, -y}; }
        inline bool operator==(const Vec2 &other) const { return x == other.x && y == other.y; }
        inline bool operator!=(const Vec2 &other) const { return !(*this == other); }

        Vec2 &operator+=(const Vec2 &other)
        {
            x += other.x;
            y += other.y;
            return *this;
        }
        Vec2 &operator-=(const Vec2 &other)
        {
            x -= other.x;
            y -= other.y;
            return *this;
        }
        Vec2 &operator*=(real scalar)
        {
            x *= scalar;
            y *= scalar;
            return *this;
        }
        Vec2 &operator/=(real scalar)
        {
            x /= scalar;
            y /= scalar;
            return *this;
        }

        // ========== 数学函数 ==========
        inline real dot(const Vec2 &other) const { return x * other.x + y * other.y; }
        inline real cross(const Vec2 &other) const { return x * other.y - y * other.x; }
        inline real length() const { return std::sqrt(x * x + y * y); }
        inline real lengthSquared() const { return x * x + y * y; }
        Vec2 normalize() const
        {
            real len = length();
            if (len < Constants::Epsilon)
                return {0, 0};
            return {x / len, y / len};
        }

        void normalizeSelf()
        {
            real len = length();
            if (len < Constants::Epsilon)
            {
                x = y = 0;
                return;
            }
            x /= len;
            y /= len;
        }
        inline Vec2 perpendicular() const { return {-y, x}; }
        inline Vec2 rotate(real radians) const { return {x * std::cos(radians) - y * std::sin(radians), x * std::sin(radians) + y * std::cos(radians)}; }

        // 投影到另一个向量上
        Vec2 project(const Vec2 &other) const
        {
            real len = other.length();
            if (len < Constants::Epsilon)
                return {0, 0};
            return {other.x / len, other.y / len};
        }

        // 获得关于 normal 的反射向量
        Vec2 reflect(const Vec2 &normal) const
        {
            Vec2 n = normal.normalize();
            return *this - 2 * this->dot(n) * n;
        }

        void clear()
        {
            x = 0;
            y = 0;
        }

        // ========== 判断函数 ==========
        // 判断向量是否为零向量
        inline bool isZero() const { return x == 0 && y == 0; }
        // 判断向量是否为单位向量
        inline bool isUnit() const { return lengthSquared() - 1 < Constants::Epsilon; }

        // ========== 输出支持 ==========

        // 默认宽度为最长的数字长度 + 3
        friend std::ostream &operator<<(std::ostream &os, const Vec2 &vec)
        {
            std::ostringstream x_stream, y_stream;
            x_stream << vec.x;
            y_stream << vec.y;

            size_t x_length = x_stream.str().length();
            size_t y_length = y_stream.str().length();
            size_t max_length = std::max(x_length, y_length);

            size_t width = max_length + 3;

            os << "[" << std::setw(width) << vec.x << "," << std::setw(width) << vec.y << "]";
            return os;
        }
    };

    // ========== 全局运算符 ==========
    inline Vec2 operator*(real scalar, const Vec2 &vec)
    {
        return vec * scalar;
    }

}