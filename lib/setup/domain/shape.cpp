#include "setup/domain/shape.hpp"
#include "setup/core/setup_error.hpp"
#include "shapes.hpp" // the core's cell tests (compiled with each example; the unit tests link them separately)

#include <cmath>
#include <utility>

Shape Shape::primitive(const Node& node) {
    Shape shape{};
    shape.nodes_ = { node };
    return shape;
}

Shape Shape::sphere(si::Position center, si::Length radius) {
    const Node node{ .kind = Kind::SPHERE, .a = center, .r1 = radius };
    const Shape shape{ primitive(node) };
    return shape;
}

Shape Shape::cylinder(si::Position center, Axis axis, si::Length radius, si::Length length) {
    const Node node{ .kind = Kind::CYLINDER, .a = center, .r1 = radius, .r2 = length, .axis = axis };
    const Shape shape{ primitive(node) };
    return shape;
}

Shape Shape::box(si::Position from, si::Position to) {
    const Node node{ .kind = Kind::BOX, .a = from, .b = to };
    const Shape shape{ primitive(node) };
    return shape;
}

Shape Shape::triangle(si::Position a, si::Position b, si::Position c) {
    const Node node{ .kind = Kind::TRIANGLE, .a = a, .b = b, .c = c };
    const Shape shape{ primitive(node) };
    return shape;
}

Shape Shape::torus(si::Position center, Axis axis, si::Length ring_radius, si::Length tube_radius) {
    const Node node{ .kind = Kind::TORUS, .a = center, .r1 = ring_radius, .r2 = tube_radius, .axis = axis };
    const Shape shape{ primitive(node) };
    return shape;
}

Shape Shape::half_space(si::Position point, si::Direction outward_normal) {
    const Node node{ .kind = Kind::HALF_SPACE, .a = point, .normal = outward_normal };
    const Shape shape{ primitive(node) };
    return shape;
}

Shape operator!(const Shape& shape) {
    Shape complement{ shape };
    const Shape::Node op{ .kind = Shape::Kind::NOT };
    complement.nodes_.push_back(op);
    return complement;
}

Shape operator&(const Shape& a, const Shape& b) {
    const Shape both{ Shape::combined(Shape::Kind::AND, a, b) };
    return both;
}

Shape operator|(const Shape& a, const Shape& b) {
    const Shape either{ Shape::combined(Shape::Kind::OR, a, b) };
    return either;
}

Shape Shape::combined(Kind kind, const Shape& a, const Shape& b) {
    Shape shape{};
    shape.nodes_ = a.nodes_;
    shape.nodes_.insert(shape.nodes_.end(), b.nodes_.begin(), b.nodes_.end());
    const Node op{ .kind = kind };
    shape.nodes_.push_back(op);
    return shape;
}

float Shape::in_cell_units(si::Length x, si::Length cell) {
    const auto ratio = x / cell;
    const double cells{ au::as_raw_number(ratio) };
    const float cells_float{ static_cast<float>(cells) };
    return cells_float;
}

float3 Shape::cell_point(const si::Position& p, si::Length cell) {
    constexpr double cell_centre{ 0.5 }; // the centre of cell i is at i + 1/2 cells from the origin corner
    const double x{ au::as_raw_number(p.x / cell) - cell_centre };
    const double y{ au::as_raw_number(p.y / cell) - cell_centre };
    const double z{ au::as_raw_number(p.z / cell) - cell_centre };
    const float3 point{ static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    return point;
}

Shape::Cells::Node Shape::cell_node(const Node& node, si::Length cell, const uint3& cells) {
    Cells::Node result{};
    result.kind = node.kind;
    result.axis = node.axis;
    result.p0 = cell_point(node.a, cell);
    result.p1 = cell_point(node.b, cell);
    result.p2 = cell_point(node.c, cell);
    result.r1 = in_cell_units(node.r1, cell);
    result.r2 = in_cell_units(node.r2, cell);
    if(node.kind == Kind::CYLINDER) { // the core's cylinder takes its axis as a vector as long as the cylinder
        const float length{ in_cell_units(node.r2, cell) };
        const float along_x{ node.axis == Axis::X ? length : 0.0f };
        const float along_y{ node.axis == Axis::Y ? length : 0.0f };
        const float along_z{ node.axis == Axis::Z ? length : 0.0f };
        result.p1 = float3(along_x, along_y, along_z);
    } else if(node.kind == Kind::BOX) {
        const float from_x{ in_cell_units(node.a.x, cell) };
        const float to_x{ in_cell_units(node.b.x, cell) };
        const float from_y{ in_cell_units(node.a.y, cell) };
        const float to_y{ in_cell_units(node.b.y, cell) };
        const float from_z{ in_cell_units(node.a.z, cell) };
        const float to_z{ in_cell_units(node.b.z, cell) };
        result.x = CellSpan::of(from_x, to_x, cells.x);
        result.y = CellSpan::of(from_y, to_y, cells.y);
        result.z = CellSpan::of(from_z, to_z, cells.z);
    } else if(node.kind == Kind::HALF_SPACE) { // the normal is a direction: no conversion of its length
        const float normal_x{ static_cast<float>(node.normal.x) };
        const float normal_y{ static_cast<float>(node.normal.y) };
        const float normal_z{ static_cast<float>(node.normal.z) };
        result.p1 = float3(normal_x, normal_y, normal_z);
    }
    return result;
}

Shape::Segment Shape::joined(Segment first, Segment second, const Cells::Node& op) {
    const bool second_deeper{ second.slots > first.slots };
    const bool same{ first.slots == second.slots };
    const std::uint32_t same_slots{ first.slots + 1u }; // the first's result waits in one slot while the second needs all of its own
    Segment result{};
    if(second_deeper) {
        result = std::move(second);
        result.nodes.insert(result.nodes.end(), first.nodes.begin(), first.nodes.end());
    } else {
        result = std::move(first);
        result.nodes.insert(result.nodes.end(), second.nodes.begin(), second.nodes.end());
    }
    if(same) {
        result.slots = same_slots;
    }
    result.nodes.push_back(op);
    return result;
}

void Shape::add_node(std::vector<Segment>& parts, const Cells::Node& node) {
    const bool binary{ node.kind == Kind::AND || node.kind == Kind::OR };
    const bool unary{ node.kind == Kind::NOT };
    const std::size_t needed{ binary ? 2u : 1u };
    if((binary || unary) && parts.size() < needed) {
        throw SetupError("Shape::in_cells(): an operator without its operands (the shape's postfix list is broken)");
    }
    if(unary) {
        parts.back().nodes.push_back(node);
        return;
    }
    if(binary) {
        Segment second{ std::move(parts.back()) };
        parts.pop_back();
        Segment first{ std::move(parts.back()) };
        parts.pop_back();
        parts.push_back(joined(std::move(first), std::move(second), node));
        return;
    }
    const Segment leaf{ .nodes = { node }, .slots = 1u };
    parts.push_back(leaf);
}

Shape::Cells Shape::in_cells(si::Length cell, const uint3& cells) const {
    // one pass over the postfix list with a stack of parts; nodes_.size() bounds the loop and the stack
    std::vector<Segment> parts{};
    parts.reserve(nodes_.size());
    for(const Node& node : nodes_) {
        const Cells::Node converted{ cell_node(node, cell, cells) };
        add_node(parts, converted);
    }
    if(parts.size() != 1u) {
        throw SetupError("Shape::in_cells(): the shape's postfix list does not end in one shape");
    }
    if(parts.front().slots > Cells::stack_capacity) {
        throw SetupError("Shape::in_cells(): the shape needs more stack slots than Shape::Cells::stack_capacity");
    }
    Cells result{};
    result.nodes_ = std::move(parts.front().nodes);
    return result;
}

bool Shape::Cells::contains(uint32_t x, uint32_t y, uint32_t z) const {
    std::array<bool, stack_capacity> stack{};
    std::size_t top{ 0u }; // the number of values on the stack; in_cells() proved that it stays within the capacity
    for(const Node& node : nodes_) {
        switch(node.kind) {
            case Kind::NOT:
                stack[top - 1u] = !stack[top - 1u];
                break;
            case Kind::AND:
                stack[top - 2u] = stack[top - 2u] && stack[top - 1u];
                top--;
                break;
            case Kind::OR:
                stack[top - 2u] = stack[top - 2u] || stack[top - 1u];
                top--;
                break;
            default:
                stack[top] = primitive_contains(node, x, y, z);
                top++;
                break;
        }
    }
    const bool inside{ stack[0] };
    return inside;
}

float Shape::Cells::fill(uint32_t x, uint32_t y, uint32_t z) const {
    std::array<float, stack_capacity> stack{};
    std::size_t top{ 0u }; // the number of values on the stack; in_cells() proved that it stays within the capacity
    for(const Node& node : nodes_) {
        switch(node.kind) {
            case Kind::NOT:
                stack[top - 1u] = 1.0f - stack[top - 1u];
                break;
            case Kind::AND: // the smaller part (both are 0 to 1)
                stack[top - 2u] = std::fmin(stack[top - 2u], stack[top - 1u]);
                top--;
                break;
            case Kind::OR: // the larger part
                stack[top - 2u] = std::fmax(stack[top - 2u], stack[top - 1u]);
                top--;
                break;
            default:
                stack[top] = primitive_fill(node, x, y, z);
                top++;
                break;
        }
    }
    const float share{ stack[0] };
    return share;
}

bool Shape::Cells::primitive_contains(const Node& node, uint32_t x, uint32_t y, uint32_t z) {
    bool inside{ false };
    switch(node.kind) {
        case Kind::SPHERE:
            inside = ::sphere(x, y, z, node.p0, node.r1); // ::, not Shape's factories
            break;
        case Kind::CYLINDER:
            inside = ::cylinder(x, y, z, node.p0, node.p1, node.r1);
            break;
        case Kind::BOX:
            inside = node.x.contains(x) && node.y.contains(y) && node.z.contains(z);
            break;
        case Kind::TRIANGLE:
            inside = ::triangle(x, y, z, node.p0, node.p1, node.p2);
            break;
        case Kind::HALF_SPACE:
            inside = ::plane(x, y, z, node.p0, node.p1); // the core's plane(): behind the normal
            break;
        case Kind::TORUS:
            inside = torus_contains(node, x, y, z);
            break;
        default: // an operator: no primitive test
            break;
    }
    return inside;
}

bool Shape::Cells::torus_contains(const Node& node, uint32_t x, uint32_t y, uint32_t z) {
    bool inside{ false };
    switch(node.axis) {
        case Axis::X:
            inside = torus_x(x, y, z, node.p0, node.r2, node.r1);
            break;
        case Axis::Y:
            inside = torus_y(x, y, z, node.p0, node.r2, node.r1);
            break;
        case Axis::Z:
            inside = torus_z(x, y, z, node.p0, node.r2, node.r1);
            break;
    }
    return inside;
}

float Shape::Cells::primitive_fill(const Node& node, uint32_t x, uint32_t y, uint32_t z) {
    if(node.kind == Kind::SPHERE) {
        const float plic{ sphere_plic(x, y, z, node.p0, node.r1) }; // -1 outside
        const float share{ std::fmax(plic, 0.0f) };
        return share;
    }
    const bool inside{ primitive_contains(node, x, y, z) };
    const float share{ inside ? 1.0f : 0.0f };
    return share;
}
