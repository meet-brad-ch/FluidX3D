#include "setup/domain/shape.hpp"
#include "shapes.hpp" // the core's cell tests (compiled with each example; the unit tests link them separately)

#include <algorithm>

Shape Shape::sphere(Position center, Length radius) {
    Shape shape;
    shape.kind_ = SPHERE;
    shape.a_ = center;
    shape.r1_ = radius;
    return shape;
}

Shape Shape::cylinder(Position center, Axis axis, Length radius, Length length) {
    Shape shape;
    shape.kind_ = CYLINDER;
    shape.a_ = center;
    shape.axis_ = axis;
    shape.r1_ = radius;
    shape.r2_ = length;
    return shape;
}

Shape Shape::box(Position from, Position to) {
    Shape shape;
    shape.kind_ = BOX;
    shape.a_ = from;
    shape.b_ = to;
    return shape;
}

Shape Shape::triangle(Position a, Position b, Position c) {
    Shape shape;
    shape.kind_ = TRIANGLE;
    shape.a_ = a;
    shape.b_ = b;
    shape.c_ = c;
    return shape;
}

Shape Shape::torus(Position center, Axis axis, Length ring_radius, Length tube_radius) {
    Shape shape;
    shape.kind_ = TORUS;
    shape.a_ = center;
    shape.axis_ = axis;
    shape.r1_ = ring_radius;
    shape.r2_ = tube_radius;
    return shape;
}

Shape Shape::half_space(Position point, float3 outward_normal) {
    Shape shape;
    shape.kind_ = HALF_SPACE;
    shape.a_ = point;
    shape.normal_ = outward_normal;
    return shape;
}

Shape operator!(const Shape& shape) { return Shape::combined(Shape::NOT, { shape }); }
Shape operator&(const Shape& a, const Shape& b) { return Shape::combined(Shape::AND, { a, b }); }
Shape operator|(const Shape& a, const Shape& b) { return Shape::combined(Shape::OR, { a, b }); }

Shape Shape::combined(Kind kind, std::vector<Shape> operands) {
    Shape shape;
    shape.kind_ = kind;
    shape.operands_ = std::move(operands);
    return shape;
}

Shape::Cells Shape::in_cells(float cell_size, const uint3& cells) const {
    // a point in metres to the core's cell coordinates, in which the center of cell i is at i
    const auto point = [cell_size](const Position& p) {
        return float3(p.x.si() / cell_size - 0.5f, p.y.si() / cell_size - 0.5f, p.z.si() / cell_size - 0.5f);
    };
    Cells result;
    result.kind_ = kind_;
    result.axis_ = axis_;
    result.p0_ = point(a_);
    result.p1_ = point(b_);
    result.p2_ = point(c_);
    result.r1_ = r1_.si() / cell_size;
    result.r2_ = r2_.si() / cell_size;
    switch(kind_) {
        case CYLINDER: { // the core's cylinder takes its axis as a vector as long as the cylinder
            const float length = r2_.si() / cell_size;
            result.p1_ = float3(axis_ == Axis::X ? length : 0.0f, axis_ == Axis::Y ? length : 0.0f, axis_ == Axis::Z ? length : 0.0f);
            break;
        }
        case BOX:
            result.x_ = CellSpan::of(a_.x.si() / cell_size, b_.x.si() / cell_size, cells.x);
            result.y_ = CellSpan::of(a_.y.si() / cell_size, b_.y.si() / cell_size, cells.y);
            result.z_ = CellSpan::of(a_.z.si() / cell_size, b_.z.si() / cell_size, cells.z);
            break;
        case HALF_SPACE: // the normal is a direction: no conversion
            result.p1_ = normal_;
            break;
        default:
            break;
    }
    for(const Shape& operand : operands_) result.operands_.push_back(operand.in_cells(cell_size, cells));
    return result;
}

bool Shape::Cells::contains(uint32_t x, uint32_t y, uint32_t z) const {
    switch(kind_) {
        case SPHERE: return ::sphere(x, y, z, p0_, r1_); // ::, not Shape's factories
        case CYLINDER: return ::cylinder(x, y, z, p0_, p1_, r1_);
        case BOX: return x_.contains(x) && y_.contains(y) && z_.contains(z);
        case TRIANGLE: return ::triangle(x, y, z, p0_, p1_, p2_);
        case HALF_SPACE: return ::plane(x, y, z, p0_, p1_); // the core's plane(): behind the normal
        case TORUS:
            switch(axis_) {
                case Axis::X: return torus_x(x, y, z, p0_, r2_, r1_);
                case Axis::Y: return torus_y(x, y, z, p0_, r2_, r1_);
                case Axis::Z: return torus_z(x, y, z, p0_, r2_, r1_);
            }
            return false;
        case NOT: return !operands_[0].contains(x, y, z);
        case AND: return std::all_of(operands_.begin(), operands_.end(), [&](const Cells& c) { return c.contains(x, y, z); });
        case OR: return std::any_of(operands_.begin(), operands_.end(), [&](const Cells& c) { return c.contains(x, y, z); });
    }
    return false;
}

float Shape::Cells::fill(uint32_t x, uint32_t y, uint32_t z) const {
    switch(kind_) {
        case SPHERE: return fmax(sphere_plic(x, y, z, p0_, r1_), 0.0f); // -1 outside
        case NOT: return 1.0f - operands_[0].fill(x, y, z);
        case AND: {
            float share = 1.0f;
            for(const Cells& operand : operands_) share = fmin(share, operand.fill(x, y, z));
            return share;
        }
        case OR: {
            float share = 0.0f;
            for(const Cells& operand : operands_) share = fmax(share, operand.fill(x, y, z));
            return share;
        }
        default: return contains(x, y, z) ? 1.0f : 0.0f;
    }
}
