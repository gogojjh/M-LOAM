#ifndef COLOR_HPP_
#define COLOR_HPP_

namespace common {
    //----------------------------------- color utils
    /**
     * 黑色   Black    0   0    0    0
       白色   White    255    255    255    16777215
       灰色   Gray    192    192    192    12632256
       深灰色    Dark    Grey    128    128    128    8421504
       红色    Red    255    0    0    255
       深红色    Dark    Red    128    0    0    128
       绿色    Green    0    255    0    65280
       深绿色    Dark    Green    0    128    0    32768
       蓝色    Blue    0    0    255    16711680
       深蓝色    Dark    Blue    0    0    128    8388608
       紫红色    Magenta    255    0    255    16711935
       深紫红    Dark    Magenta    128    0    128    8388736
       紫色    Cyan    0    255    255    16776960
       深紫    Dark    Cyan    0    128    128    8421376
       黄色    Yellow    255    255    0    65535
       棕色    Brown    128    128    0    32896
     */
    // struct Color {
    //     Color(float r, float g, float b)
    //     {
    //         rgbA.r = r;
    //         rgbA.g = g;
    //         rgbA.b = b;
    //         rgbA.a = 1.0;
    //     }

    //     std_msgs::ColorRGBA rgbA;
    // };
    // const struct Color BLACK(0.0, 0.0, 0.0);
    // const struct Color WHITE(1.0, 1.0, 1.0);
    // const struct Color RED(1.0, 0.0, 0.0);
    // const struct Color DARKRED(0.5, 0.0, 0.0);
    // const struct Color GREEN(0.0, 1.0, 0.0);
    // const struct Color DARKGREEN(0.0, 0.5, 0.0);
    // const struct Color BLUE(0.0, 0.0, 1.0);
    // const struct Color DARKBLUE(0.0, 0.0, 0.5);
    // const struct Color MAGENTA(1.0, 0.0, 1.0);
    // const struct Color DARKMAGENTA(0.5, 0.0, 0.5);
    // const struct Color CYAN(0.0, 1.0, 1.0);
    // const struct Color DARKCYAN(0.0, 0.5, 0.5);
    // const struct Color YELLOW(1.0, 1.0, 0.0);
    // const struct Color BROWN(0.5, 0.5, 0.0);

    const std::string RED("\033[0;31m");
    const std::string GREEN("\033[1;32m");
    const std::string YELLOW("\033[1;33m");
    const std::string CYAN("\033[0;36m");
    const std::string MAGENTA("\033[0;35m");
    const std::string RESET("\033[0m");
}

#endif /* COLOR_HPP_ */