// test_oxygen.cpp
#include <iostream>
#include <cassert>
#include <cmath>
#include "OxygenMathLite.h"

using namespace OxygenMathLite;

int main()
{
    std::cout << "===== OxygenMathLite 测试开始 =====\n";

    // ---------- Vec2 测试 ----------
    Vec2 a2(3.0f, 4.0f);
    Vec2 b2(-1.0f, 2.0f);
    std::cout << "a2 = " << a2 << "\n";
    std::cout << "b2 = " << b2 << "\n";

    // length / normalize / dot / cross / rotate / reflect
    assert(std::fabs(a2.length() - 5.0f) < 1e-4f);
    Vec2 an = a2.normalize();
    assert(std::fabs(an.length() - 1.0f) < 1e-3f || an.isUnit());

    real dot2 = a2.dot(b2);
    real cross2 = a2.cross(b2);
    std::cout << "Vec2 dot = " << dot2 << ", cross = " << cross2 << "\n";

    Vec2 rotated = a2.rotate(Constants::DEG_TO_RAD * 90.0f);
    std::cout << "a2 rotated 90deg = " << rotated << "\n";

    Vec2 incoming(1.0f, -1.0f);
    Vec2 n(0.0f, 1.0f);
    Vec2 refl = incoming.reflect(n);
    std::cout << "reflect(1,-1) on normal (0,1) = " << refl << "\n";
    // reflection across horizontal should invert y
    assert(std::fabs(refl.x - 1.0f) < 1e-4f && std::fabs(refl.y - 1.0f) < 1e-4f);

    // ---------- Vec3 / Vec4 测试 ----------
    Vec3 va(1, 2, 3), vb(4, 5, 6);
    std::cout << "va = " << va << "\n"
              << "vb = " << vb << "\n";
    Vec3 vc = va + vb;
    std::cout << "va+vb = " << vc << "\n";
    real dot3 = va.dot(vb);
    Vec3 cross3 = va.cross(vb);
    std::cout << "Vec3 dot = " << dot3 << ", cross = " << cross3 << "\n";
    assert(std::fabs(dot3 - (1 * 4 + 2 * 5 + 3 * 6)) < 1e-4f);

    Vec4 v4(1, 2, 3, 1);
    std::cout << "v4 length = " << v4.length() << "\n";

    // ---------- Mat2 测试 ----------
    Mat2 R = Mat2::Rotation(Constants::DEG_TO_RAD * 90.0f);
    Vec2 v2(1, 0);
    Vec2 v2r = R * v2;
    std::cout << "Mat2 rotation(90) * (1,0) = " << v2r << "\n";
    // (1,0) rotated 90deg -> (0,1) approximately
    assert(std::fabs(v2r.x) < 1e-3f && std::fabs(v2r.y - 1.0f) < 1e-3f);

    Mat2 A(2, 3, 4, 5);
    Mat2 B(1, 0, 0, 1);
    Mat2 C = A * B;
    std::cout << "A*I = \n"
              << C << "\n";

    // ---------- Mat3 测试 ----------
    Mat3 M = Mat3::Identity();
    Vec3 vv(1, 2, 3);
    Vec3 mv = M * vv;
    std::cout << "Mat3 Identity * vv = " << mv << "\n";
    Mat3 A3(1, 2, 3, 4, 5, 6, 7, 8, 10);
    real detA3 = A3.Det();
    std::cout << "det(A3) = " << detA3 << "\n";
    Mat3 invA3;
    try
    {
        invA3 = A3.Inv();
        Mat3 I3 = A3 * invA3;
        std::cout << "A3 * invA3 (approx) =\n"
                  << I3 << "\n";
    }
    catch (...)
    {
        std::cout << "A3 is singular (unexpected)\n";
    }

    // ---------- MathTools ----------
    real clamped = MathTools::Clamp(5.0f, 0.0f, 3.0f);
    assert(clamped == 3.0f);
    real lerped = MathTools::Lerp(0.0f, 10.0f, 0.25f);
    assert(std::fabs(lerped - 2.5f) < 1e-6f);
    std::cout << "Clamp(5,0,3)=" << clamped << ", Lerp(0,10,0.25)=" << lerped << "\n";

    // Random functions (just call to ensure no crash)
    real r = MathTools::RandomRange(-1.0f, 1.0f);
    Vec2 ru = MathTools::RandomUnitVector2();
    Vec2 ric = MathTools::RandomInsideUnitCircle();
    std::cout << "random range = " << r << ", random unit = " << ru << ", inside circle = " << ric << "\n";

    // ---------- Geometry2D 测试 ----------
    // point to line
    Vec2 p(1, 1);
    Vec2 l1(0, 0), l2(2, 0);
    real dpl = Geometry2D::DistancePointToLine(l1, l2, p);
    std::cout << "Distance point to line = " << dpl << "\n";
    assert(std::fabs(dpl - 1.0f) < 1e-4f);

    // line segment to line segment: intersecting -> distance 0
    Vec2 a(0, 0), b(2, 2), c(0, 2), d(2, 0);
    real dd = Geometry2D::DistanceLineToLine(a, b, c, d);
    std::cout << "Distance between intersecting segments = " << dd << "\n";
    assert(std::fabs(dd - 0.0f) < 1e-6f);

    // parallel non-overlapping segments
    Vec2 p1(0, 0), p2(2, 0), q1(0, 1), q2(2, 1);
    real dp = Geometry2D::DistanceLineToLine(p1, p2, q1, q2);
    std::cout << "Distance between parallel segments (expected 1) = " << dp << "\n";
    assert(std::fabs(dp - 1.0f) < 1e-3f);

    // skew non-intersecting segments: nearest endpoint distance
    Vec2 s1(0, 0), s2(1, 0), t1(2, 1), t2(3, 1);
    real ds = Geometry2D::DistanceLineToLine(s1, s2, t1, t2);
    std::cout << "Distance between skew segments = " << ds << "\n";
    // expected ~ sqrt(2) since (1,0) to (2,1) distance
    assert(ds > 0.0f);

    // Closest point on segment
    Vec2 cp = Geometry2D::ClosestPointOnLineSegment(p1, p2, Vec2(3, 0.5f));
    std::cout << "Closest point on [0,0]-[2,0] to (3,0.5) = " << cp << "\n";
    assert(std::fabs(cp.x - 2.0f) < 1e-4f);

    // ---------- Integration2D 测试 ----------
    Vec2 pos(0, 0), vel(1, 0), acc(0, -9.8f);
    Integration2D::Euler(pos, vel, acc, 0.1f);
    std::cout << "After Euler step pos=" << pos << ", vel=" << vel << "\n";

    pos = Vec2(0, 0);
    vel = Vec2(1, 0);
    Integration2D::RK2(pos, vel, acc, 0.1f);
    std::cout << "After RK2 step pos=" << pos << ", vel=" << vel << "\n";

    std::cout << "===== 所有测试完成=====\n";
    return 0;
}
