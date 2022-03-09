# distutils: language = c++
from Rectangle cimport Rectangle

# 接口类
# Python可以直接访问接口类，接口类可以直接访问C++类
cdef class PyRectangle:
    cdef Rectangle c_rect    # 存储C++对象
    def __cinit__(self, int x0, int y0, int x1, int y1):
        self.c_rect = Rectangle(x0, y0, x1, y1)
    def get_area(self):
        return self.c_rect.getArea()
    def get_size(self):
        cdef int width, height
        self.c_rect.getSize(&width, &height)
        return width, height
    def move(self, dx, dy):
        self.c_rect.move(dx, dy)