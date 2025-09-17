#pragma once
#include <cmath>
#include <iomanip>
#include <sstream>
#include <algorithm>

#ifdef DOUBLE_PRECISION
using real = double;
#else
using real = float;
#endif

namespace OxygenMathLite
{
    // ========= 前置声明 =========
    struct Vec2;
    struct Vec3;

    // ========= 全局运算符声明 =========
    inline Vec2 operator*(real scalar, const Vec2 &vec);
    inline Vec3 operator*(real scalar, const Vec3 &vec);

    namespace Constants
    {
#ifdef DOUBLE_PRECISION
        constexpr real PI = 3.141592653589793238462643383279502884;
        constexpr real Epsilon = 1e-12;
#else
        constexpr real PI = 3.14159265358979323846f;
        constexpr real Epsilon = 1e-6f;
#endif
        constexpr real TWO_PI = 2.0f * PI;
        constexpr real HALF_PI = 0.5f * PI;
        constexpr real DEG_TO_RAD = PI / 180.0f;
        constexpr real RAD_TO_DEG = 180.0f / PI;
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

    // ====================== Vec2 ======================
    struct Vec2
    {
        real x = 0.0f;
        real y = 0.0f;

        constexpr Vec2() : x(0.0f), y(0.0f) {}
        constexpr Vec2(real x, real y) : x(x), y(y) {}
        constexpr Vec2(const Vec2 &other) : x(other.x), y(other.y) {}
        constexpr Vec2(Vec2 &&other) : x(other.x), y(other.y) {}

        Vec2 &operator=(const Vec2 &other)
        {
            x = other.x; y = other.y; return *this;
        }
        Vec2 &operator=(Vec2 &&other)
        {
            x = other.x; y = other.y; return *this;
        }
        ~Vec2() = default;

        static Vec2 Zero() { return {0.0f, 0.0f}; }
        static Vec2 One() { return {1.0f, 1.0f}; }
        static Vec2 Up() { return {0.0f, 1.0f}; }
        static Vec2 Down() { return {0.0f, -1.0f}; }
        static Vec2 Left() { return {-1.0f, 0.0f}; }
        static Vec2 Right() { return {1.0f, 0.0f}; }

        Vec2 operator+(const Vec2 &other) const { return {x + other.x, y + other.y}; }
        Vec2 operator-(const Vec2 &other) const { return {x - other.x, y - other.y}; }
        Vec2 operator*(real scalar) const { return {x * scalar, y * scalar}; }
        Vec2 operator/(real scalar) const { return {x / scalar, y / scalar}; }
        Vec2 operator-() const { return {-x, -y}; }
        inline bool operator==(const Vec2 &other) const { return x == other.x && y == other.y; }
        inline bool operator!=(const Vec2 &other) const { return !(*this == other); }

        Vec2 &operator+=(const Vec2 &other) { x += other.x; y += other.y; return *this; }
        Vec2 &operator-=(const Vec2 &other) { x -= other.x; y -= other.y; return *this; }
        Vec2 &operator*=(real scalar) { x *= scalar; y *= scalar; return *this; }
        Vec2 &operator/=(real scalar) { x /= scalar; y /= scalar; return *this; }

        inline real dot(const Vec2 &other) const { return x * other.x + y * other.y; }
        inline real cross(const Vec2 &other) const { return x * other.y - y * other.x; }
        inline real length() const { return std::sqrt(x * x + y * y); }
        inline real lengthSquared() const { return x * x + y * y; }

        Vec2 normalize() const
        {
            real len = length();
            if (len < Constants::Epsilon) return {0, 0};
            return {x / len, y / len};
        }
        void normalizeSelf()
        {
            real len = length();
            if (len < Constants::Epsilon) { x = y = 0; return; }
            x /= len; y /= len;
        }

        inline Vec2 perpendicular() const { return {-y, x}; }
        inline Vec2 rotate(real radians) const
        {
            return {x * std::cos(radians) - y * std::sin(radians),
                    x * std::sin(radians) + y * std::cos(radians)};
        }

        Vec2 project(const Vec2 &other) const
        {
            real lenSq = other.lengthSquared();
            if (lenSq < Constants::Epsilon) return {0, 0};
            return (this->dot(other) / lenSq) * other;
        }

        Vec2 reflect(const Vec2 &normal) const
        {
            Vec2 n = normal.normalize();
            return *this - 2 * this->dot(n) * n;
        }

        void clear() { x = y = 0; }

        inline bool isZero() const { return x == 0 && y == 0; }
        inline bool isUnit() const { return std::fabs(lengthSquared() - 1) < Constants::Epsilon; }

        friend std::ostream &operator<<(std::ostream &os, const Vec2 &vec)
        {
            std::ostringstream xs, ys;
            xs << vec.x; ys << vec.y;
            size_t width = std::max(xs.str().length(), ys.str().length()) + 3;
            os << "[" << std::setw(width) << vec.x << "," << std::setw(width) << vec.y << "]";
            return os;
        }
    };

    // ====================== Vec3 ======================
    struct Vec3
    {
        real x = 0.0f, y = 0.0f, z = 0.0f;

        constexpr Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
        constexpr Vec3(real x, real y, real z) : x(x), y(y), z(z) {}
        constexpr Vec3(const Vec3 &o) : x(o.x), y(o.y), z(o.z) {}
        constexpr Vec3(Vec3 &&o) : x(o.x), y(o.y), z(o.z) {}

        Vec3 &operator=(const Vec3 &o) { x=o.x; y=o.y; z=o.z; return *this; }
        Vec3 &operator=(Vec3 &&o) { x=o.x; y=o.y; z=o.z; return *this; }
        ~Vec3() = default;

        static Vec3 Zero() { return {0,0,0}; }
        static Vec3 One() { return {1,1,1}; }
        static Vec3 Up() { return {0,1,0}; }
        static Vec3 Down() { return {0,-1,0}; }
        static Vec3 Left() { return {-1,0,0}; }
        static Vec3 Right() { return {1,0,0}; }
        static Vec3 Forward() { return {0,0,1}; }
        static Vec3 Backward() { return {0,0,-1}; }

        Vec3 operator+(const Vec3 &o) const { return {x+o.x,y+o.y,z+o.z}; }
        Vec3 operator-(const Vec3 &o) const { return {x-o.x,y-o.y,z-o.z}; }
        Vec3 operator*(real s) const { return {x*s,y*s,z*s}; }
        Vec3 operator/(real s) const { return {x/s,y/s,z/s}; }
        Vec3 operator-() const { return {-x,-y,-z}; }
        inline bool operator==(const Vec3 &o) const { return x==o.x && y==o.y && z==o.z; }
        inline bool operator!=(const Vec3 &o) const { return !(*this==o); }

        Vec3 &operator+=(const Vec3 &o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
        Vec3 &operator-=(const Vec3 &o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
        Vec3 &operator*=(real s) { x*=s; y*=s; z*=s; return *this; }
        Vec3 &operator/=(real s) { x/=s; y/=s; z/=s; return *this; }

        real length() const { return std::sqrt(x*x+y*y+z*z); }
        real lengthSquared() const { return x*x+y*y+z*z; }

        Vec3 normalize() const
        {
            real len=length();
            if(len<Constants::Epsilon) return {0,0,0};
            return {x/len,y/len,z/len};
        }
        void normalizeSelf()
        {
            real len=length();
            if(len<Constants::Epsilon){x=y=z=0;return;}
            x/=len; y/=len; z/=len;
        }

        inline real dot(const Vec3 &o) const { return x*o.x+y*o.y+z*o.z; }
        Vec3 cross(const Vec3 &o) const { return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x}; }

        Vec3 reflect(const Vec3 &normal) const
        {
            Vec3 n=normal.normalize();
            return *this - 2 * this->dot(n) * n;
        }

        Vec3 project(const Vec3 &other) const
        {
            real lenSq=other.lengthSquared();
            if(lenSq<Constants::Epsilon) return {0,0,0};
            return (this->dot(other)/lenSq) * other;
        }

        void clear(){x=y=z=0;}

        inline bool isZero() const { return x==0&&y==0&&z==0; }
        inline bool isUnit() const { return std::fabs(lengthSquared()-1)<Constants::Epsilon; }

        friend std::ostream &operator<<(std::ostream &os,const Vec3 &v)
        {
            std::ostringstream xs,ys,zs;
            xs<<v.x; ys<<v.y; zs<<v.z;
            size_t width=std::max({xs.str().length(),ys.str().length(),zs.str().length()})+3;
            os<<"["<<std::setw(width)<<v.x
              <<","<<std::setw(width)<<v.y
              <<","<<std::setw(width)<<v.z<<"]";
            return os;
        }
    };

    // ========= 全局运算符定义 =========
    inline Vec2 operator*(real scalar, const Vec2 &vec) { return vec * scalar; }
    inline Vec3 operator*(real scalar, const Vec3 &vec) { return vec * scalar; }

} // namespace OxygenMathLite
