#pragma once

#include "setup/core/quantity.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/domain/lattice.hpp"
#include "utilities.hpp"
#include <cstdint>
#include <vector>

/// @brief A region of the domain in metres from its origin corner: a solid object, water or gas.
///
/// Shapes combine with ! (outside), & (in both) and | (in either):
/// @code
/// !Shape::cylinder({ 5_cm, 0_m, 5_cm }, Axis::Y, 4_cm, 1_m)           // the wall around a pipe
/// Shape::torus(center, Axis::X, 2_cm, 2_cm) & Shape::box(from, to) // part of a ring
/// @endcode
/// A cell belongs to a shape when its center does (cell i spans i to i+1 cell sizes from the origin), as the core's
/// shapes.hpp tests cells; a box covers whole cells as CellSpan.
class Shape {
public:
    static Shape sphere(Position center, Length radius);
    static Shape cylinder(Position center, Axis axis, Length radius, Length length); ///< its axis through center
    static Shape box(Position from, Position to);                                    ///< between two corners
    static Shape triangle(Position a, Position b, Position c);                       ///< a plate about one cell thick
    static Shape torus(Position center, Axis axis, Length ring_radius, Length tube_radius); ///< a ring about the axis

    friend Shape operator!(const Shape& shape);             ///< everywhere outside the shape
    friend Shape operator&(const Shape& a, const Shape& b); ///< in both
    friend Shape operator|(const Shape& a, const Shape& b); ///< in either

    /// A shape on one grid, in lattice units.
    class Cells {
    public:
        bool contains(uint32_t x, uint32_t y, uint32_t z) const;
        /// The share of the cell inside the shape, 0 to 1: smooth (PLIC) at a sphere's surface, else 0 or 1.
        float fill(uint32_t x, uint32_t y, uint32_t z) const;

    private:
        friend class Shape;
        int kind_ = 0;
        float3 p0_, p1_, p2_; ///< points in the core's cell coordinates (cell i is at i)
        float r1_ = 0.0f, r2_ = 0.0f;
        Axis axis_ = Axis::X;
        CellSpan x_, y_, z_;
        std::vector<Cells> operands_;
    };

    /// @param cell_size the grid's cell size in metres
    /// @param cells     the grid's cells along X, Y and Z
    Cells in_cells(float cell_size, const uint3& cells) const;

private:
    enum Kind { SPHERE, CYLINDER, BOX, TRIANGLE, TORUS, NOT, AND, OR };

    Kind kind_ = SPHERE;
    Position a_{}, b_{}, c_{};
    Length r1_{}, r2_{};
    Axis axis_ = Axis::X;
    std::vector<Shape> operands_;

    static Shape combined(Kind kind, std::vector<Shape> operands);
};
