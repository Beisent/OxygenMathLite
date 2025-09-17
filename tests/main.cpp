#include "OxygenMathLite.h"
#include <cassert>
#include <iostream>

void testConstants()
{
    using namespace OxygenMathLite::Constants;

    // 测试 PI 值
    assert(std::abs(PI - 3.14159265358979323846f) < Epsilon);

    // 测试 TWO_PI 值
    assert(std::abs(TWO_PI - 2.0f * PI) < Epsilon);

    // 测试 HALF_PI 值
    assert(std::abs(HALF_PI - 0.5f * PI) < Epsilon);

    // 测试角度转换常量
    assert(std::abs(DEG_TO_RAD - PI / 180.0f) < Epsilon);
    assert(std::abs(RAD_TO_DEG - 180.0f / PI) < Epsilon);

    // 测试 Epsilon 值
    assert(Epsilon == 1e-6f);

    std::cout << "Constants tests passed.\n";
}
void testMathTools()
{
    using namespace OxygenMathLite::MathTools;
    using namespace OxygenMathLite::Constants;

    // 测试 Clamp 函数
    assert(Clamp(5.0f, 0.0f, 10.0f) == 5.0f);
    assert(Clamp(-5.0f, 0.0f, 10.0f) == 0.0f);
    assert(Clamp(15.0f, 0.0f, 10.0f) == 10.0f);

    // 测试 Lerp 函数
    assert(Lerp(0.0f, 10.0f, 0.5f) == 5.0f);
    assert(Lerp(0.0f, 10.0f, 0.0f) == 0.0f);
    assert(Lerp(0.0f, 10.0f, 1.0f) == 10.0f);

    // 测试角度转换函数
    assert(std::abs(ToRadians(180.0f) - PI) < Epsilon);
    assert(std::abs(ToDegrees(PI) - 180.0f) < Epsilon);

    // 测试 Swap 函数
    int a = 5, b = 10;
    Swap(a, b);
    assert(a == 10 && b == 5);

    std::cout << "MathTools tests passed.\n";
}
void testVec2()
{
    using namespace OxygenMathLite;
    using namespace OxygenMathLite::Constants;

    // 测试构造函数
    Vec2 v1;
    assert(v1.x == 0.0f && v1.y == 0.0f);

    Vec2 v2(3.0f, 4.0f);
    assert(v2.x == 3.0f && v2.y == 4.0f);

    Vec2 v3(v2);
    assert(v3.x == 3.0f && v3.y == 4.0f);

    // 测试静态工厂方法
    Vec2 zero = Vec2::Zero();
    assert(zero.x == 0.0f && zero.y == 0.0f);

    Vec2 one = Vec2::One();
    assert(one.x == 1.0f && one.y == 1.0f);

    Vec2 up = Vec2::Up();
    assert(up.x == 0.0f && up.y == 1.0f);

    Vec2 down = Vec2::Down();
    assert(down.x == 0.0f && down.y == -1.0f);

    Vec2 left = Vec2::Left();
    assert(left.x == -1.0f && left.y == 0.0f);

    Vec2 right = Vec2::Right();
    assert(right.x == 1.0f && right.y == 0.0f);

    // 测试运算符重载
    Vec2 a(1.0f, 2.0f);
    Vec2 b(3.0f, 4.0f);

    Vec2 sum = a + b;
    assert(sum.x == 4.0f && sum.y == 6.0f);

    Vec2 diff = a - b;
    assert(diff.x == -2.0f && diff.y == -2.0f);

    Vec2 scaled = a * 2.0f;
    assert(scaled.x == 2.0f && scaled.y == 4.0f);

    Vec2 divided = a / 2.0f;
    assert(divided.x == 0.5f && divided.y == 1.0f);

    Vec2 neg = -a;
    assert(neg.x == -1.0f && neg.y == -2.0f);

    // 测试比较运算符
    Vec2 c(1.0f, 2.0f);
    assert(a == c);
    assert(a != b);

    // 测试复合赋值运算符
    Vec2 d(1.0f, 1.0f);
    d += a;
    assert(d.x == 2.0f && d.y == 3.0f);

    d -= a;
    assert(d.x == 1.0f && d.y == 1.0f);

    d *= 2.0f;
    assert(d.x == 2.0f && d.y == 2.0f);

    d /= 2.0f;
    assert(d.x == 1.0f && d.y == 1.0f);

    // 测试数学函数
    Vec2 e(3.0f, 4.0f);
    assert(std::abs(e.length() - 5.0f) < Epsilon);
    assert(e.lengthSquared() == 25.0f);

    Vec2 f(3.0f, 4.0f);
    Vec2 normalized = f.normalize();
    assert(std::abs(normalized.length() - 1.0f) < Epsilon);

    Vec2 g(3.0f, 4.0f);
    g.normalizeSelf();
    assert(std::abs(g.length() - 1.0f) < Epsilon);

    Vec2 h(3.0f, 4.0f);
    Vec2 zeroVec;
    Vec2 normalizedZero = zeroVec.normalize();
    assert(normalizedZero.x == 0.0f && normalizedZero.y == 0.0f);

    Vec2 i(1.0f, 0.0f);
    Vec2 j(0.0f, 1.0f);
    assert(std::abs(i.dot(j) - 0.0f) < Epsilon);
    assert(std::abs(i.dot(i) - 1.0f) < Epsilon);

    Vec2 k(2.0f, 1.0f);
    Vec2 l(1.0f, 3.0f);
    assert(std::abs(k.cross(l) - 5.0f) < Epsilon);

    Vec2 m(1.0f, 0.0f);
    Vec2 perp = m.perpendicular();
    assert(perp.x == 0.0f && perp.y == 1.0f);

    Vec2 n(1.0f, 0.0f);
    Vec2 rotated = n.rotate(MathTools::ToRadians(90.0f));
    assert(std::abs(rotated.x - 0.0f) < Epsilon && std::abs(rotated.y - 1.0f) < Epsilon);

    Vec2 o(1.0f, 1.0f);
    Vec2 p(1.0f, 0.0f);
    Vec2 proj = o.project(p);
    assert(std::abs(proj.x - 1.0f) < Epsilon && proj.y == 0.0f);

    Vec2 q(1.0f, 1.0f);
    Vec2 r(1.0f, 0.0f);
    Vec2 refl = q.reflect(r);
    assert(std::abs(refl.x - (-1.0f)) < Epsilon && std::abs(refl.y - 1.0f) < Epsilon);

    Vec2 s;
    s.clear();
    assert(s.x == 0.0f && s.y == 0.0f);

    // 测试判断函数
    Vec2 t(0.0f, 0.0f);
    assert(t.isZero());

    Vec2 u(1.0f, 0.0f);
    assert(u.isUnit());

    // 测试全局运算符
    Vec2 v(2.0f, 3.0f);
    Vec2 w = 2.0f * v;
    assert(w.x == 4.0f && w.y == 6.0f);

    std::cout << "Vec2 tests passed.\n";
}
void testVec3()
{
    using namespace OxygenMathLite;
    using namespace OxygenMathLite::Constants;

    // 测试构造函数
    Vec3 v1;
    assert(v1.x == 0.0f && v1.y == 0.0f && v1.z == 0.0f);

    Vec3 v2(1.0f, 2.0f, 3.0f);
    assert(v2.x == 1.0f && v2.y == 2.0f && v2.z == 3.0f);

    Vec3 v3(v2);
    assert(v3.x == 1.0f && v3.y == 2.0f && v3.z == 3.0f);

    // 测试静态工厂方法
    Vec3 zero = Vec3::Zero();
    assert(zero.x == 0.0f && zero.y == 0.0f && zero.z == 0.0f);

    Vec3 one = Vec3::One();
    assert(one.x == 1.0f && one.y == 1.0f && one.z == 1.0f);

    Vec3 up = Vec3::Up();
    assert(up.x == 0.0f && up.y == 1.0f && up.z == 0.0f);

    Vec3 down = Vec3::Down();
    assert(down.x == 0.0f && down.y == -1.0f && down.z == 0.0f);

    Vec3 left = Vec3::Left();
    assert(left.x == -1.0f && left.y == 0.0f && left.z == 0.0f);

    Vec3 right = Vec3::Right();
    assert(right.x == 1.0f && right.y == 0.0f && right.z == 0.0f);

    Vec3 forward = Vec3::Forward();
    assert(forward.x == 0.0f && forward.y == 0.0f && forward.z == 1.0f);

    Vec3 backward = Vec3::Backward();
    assert(backward.x == 0.0f && backward.y == 0.0f && backward.z == -1.0f);

    // 测试运算符重载
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);

    Vec3 sum = a + b;
    assert(sum.x == 5.0f && sum.y == 7.0f && sum.z == 9.0f);

    Vec3 diff = a - b;
    assert(diff.x == -3.0f && diff.y == -3.0f && diff.z == -3.0f);

    Vec3 scaled = a * 2.0f;
    assert(scaled.x == 2.0f && scaled.y == 4.0f && scaled.z == 6.0f);

    Vec3 divided = a / 2.0f;
    assert(divided.x == 0.5f && divided.y == 1.0f && divided.z == 1.5f);

    Vec3 neg = -a;
    assert(neg.x == -1.0f && neg.y == -2.0f && neg.z == -3.0f);

    // 测试比较运算符
    Vec3 c(1.0f, 2.0f, 3.0f);
    assert(a == c);
    assert(a != b);

    // 测试复合赋值运算符
    Vec3 d(1.0f, 1.0f, 1.0f);
    d += a;
    assert(d.x == 2.0f && d.y == 3.0f && d.z == 4.0f);

    d -= a;
    assert(d.x == 1.0f && d.y == 1.0f && d.z == 1.0f);

    d *= 2.0f;
    assert(d.x == 2.0f && d.y == 2.0f && d.z == 2.0f);

    d /= 2.0f;
    assert(d.x == 1.0f && d.y == 1.0f && d.z == 1.0f);

    // 测试数学函数
    Vec3 e(1.0f, 2.0f, 2.0f);
    assert(std::abs(e.length() - 3.0f) < Epsilon);
    assert(e.lengthSquared() == 9.0f);

    Vec3 f(1.0f, 2.0f, 2.0f);
    Vec3 normalized = f.normalize();
    assert(std::abs(normalized.length() - 1.0f) < Epsilon);

    Vec3 g(1.0f, 2.0f, 2.0f);
    g.normalizeSelf();
    assert(std::abs(g.length() - 1.0f) < Epsilon);

    Vec3 h(1.0f, 2.0f, 2.0f);
    Vec3 zeroVec;
    Vec3 normalizedZero = zeroVec.normalize();
    assert(normalizedZero.x == 0.0f && normalizedZero.y == 0.0f && normalizedZero.z == 0.0f);

    Vec3 i(1.0f, 2.0f, 3.0f);
    Vec3 j(4.0f, 5.0f, 6.0f);
    assert(std::abs(i.dot(j) - 32.0f) < Epsilon);

    Vec3 k(1.0f, 2.0f, 3.0f);
    Vec3 l(4.0f, 5.0f, 6.0f);
    Vec3 crossProd = k.cross(l);
    assert(crossProd.x == -3.0f && crossProd.y == 6.0f && crossProd.z == -3.0f);

    Vec3 m(1.0f, 1.0f, 1.0f);
    Vec3 n(1.0f, 0.0f, 0.0f);
    Vec3 refl = m.reflect(n);
    assert(std::abs(refl.x - (-1.0f)) < Epsilon && refl.y == 1.0f && refl.z == 1.0f);

    Vec3 o(1.0f, 1.0f, 1.0f);
    Vec3 p(1.0f, 0.0f, 0.0f);
    Vec3 proj = o.project(p);
    assert(std::abs(proj.x - 1.0f) < Epsilon && proj.y == 0.0f && proj.z == 0.0f);

    Vec3 q;
    q.clear();
    assert(q.x == 0.0f && q.y == 0.0f && q.z == 0.0f);

    // 测试判断函数
    Vec3 r(0.0f, 0.0f, 0.0f);
    assert(r.isZero());

    Vec3 s(1.0f, 0.0f, 0.0f);
    assert(s.isUnit());

    // 测试全局运算符
    Vec3 t(1.0f, 2.0f, 3.0f);
    Vec3 u = 2.0f * t;
    assert(u.x == 2.0f && u.y == 4.0f && u.z == 6.0f);

    std::cout << "Vec3 tests passed.\n";
}
int main()
{
    testConstants();
    testMathTools();
    testVec2();
    testVec3();

    std::cout << "All tests passed successfully!\n";
    return 0;
}
