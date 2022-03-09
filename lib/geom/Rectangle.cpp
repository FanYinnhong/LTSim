#include <iostream>
#include "Rectangle.h"
namespace shapes {
    // 构造函数
    Rectangle::Rectangle () {}
    Rectangle::Rectangle (int x0, int y0, int x1, int y1) {
        this->x0 = x0;
        this->y0 = y0;
        this->x1 = x1;
        this->y1 = y1;
    }
    // 析构函数
    Rectangle::~Rectangle () {}
    // 获取矩形面积
    int Rectangle::getArea () {
        return (this->x1 - this->x0) * (this->y1 - this->y0);
    }
    // 获取矩形的边长
    void Rectangle::getSize (int *width, int *height) {
        (*width) = x1 - x0;
        (*height) = y1 - y0;
    }
    // 移动矩形
    void Rectangle::move (int dx, int dy) {
        this->x0 += dx;
        this->y0 += dy;
        this->x1 += dx;
        this->y1 += dy;
    }
}