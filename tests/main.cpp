#include <iostream>
#include "OxygenMathLite.h"

using namespace OxygenMathLite;

int main()
{
    Vec2 a(1.0f, 2.0f);
    Vec2 b(3.0f, 4.0f);
    Vec2 c = a + b;
    std::cout << c << std::endl;
    return 0;
}
