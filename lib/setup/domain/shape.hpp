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
    /// @param center the sphere's center
    /// @param radius its radius
    /// @return the sphere
    static Shape sphere(Position center, Length radius);

    /// @brief A cylinder with its axis through a point.
    /// @param center the center of the cylinder
    /// @param axis the axis it lies along
    /// @param radius its radius
    /// @param length its length along the axis
    /// @return the cylinder
    static Shape cylinder(Position center, Axis axis, Length radius, Length length);

    /// @brief An axis-aligned box between two corners, covering whole cells.
    /// @param from the corner with the smaller coordinates
    /// @param to the opposite corner
    /// @return the box
    static Shape box(Position from, Position to);

    /// @brief A triangular plate about one cell thick.
    /// @param a, b, c its corners
    /// @return the plate
    static Shape triangle(Position a, Position b, Position c);

    /// @brief A ring about an axis.
    /// @param center the ring's center
    /// @param axis the axis through the center
    /// @param ring_radius the radius of the ring's centerline
    /// @param tube_radius the radius of the tube
    /// @return the torus
    static Shape torus(Position center, Axis axis, Length ring_radius, Length tube_radius);

    /// @brief Everything on one side of the plane through a point: the side the normal points away from (a sloped
    /// beach, a wall).
    /// @param point a point on the plane
    /// @param outward_normal the direction pointing out of the shape
    /// @return the half-space
    static Shape half_space(Position point, float3 outward_normal);

    /// @param shape a shape
    /// @return everywhere outside the shape
    friend Shape operator!(const Shape& shape);

    /// @param a a shape
    /// @param b a shape
    /// @return the cells in both
    friend Shape operator&(const Shape& a, const Shape& b);

    /// @param a a shape
    /// @param b a shape
    /// @return the cells in either
    friend Shape operator|(const Shape& a, const Shape& b);

    /// A shape on one grid, in lattice units.
    class Cells {
    public:
        /// @param x, y, z a cell
        /// @return whether the cell's center is inside the shape
        bool contains(uint32_t x, uint32_t y, uint32_t z) const;

        /// @brief The share of a cell inside the shape, 0 to 1: smooth (PLIC) at a sphere's surface, else 0 or 1.
        /// @param x, y, z the cell
        /// @return the share
        float fill(uint32_t x, uint32_t y, uint32_t z) const;

    private:
        friend class Shape;
        int kind_ = 0;                ///< the Shape::Kind
        float3 p0_;                   ///< a center, a corner or a point on a plane, in the core's cell coordinates (cell i is at i)
        float3 p1_;                   ///< a corner, a cylinder's axis vector or a half-space's normal
        float3 p2_;                   ///< a triangle's third corner
        float r1_ = 0.0f;             ///< a radius in cells
        float r2_ = 0.0f;             ///< a cylinder's length or a torus's tube radius, in cells
        Axis axis_ = Axis::X;         ///< a torus's axis
        CellSpan x_;                  ///< a box's cells along X
        CellSpan y_;                  ///< a box's cells along Y
        CellSpan z_;                  ///< a box's cells along Z
        std::vector<Cells> operands_; ///< the operands of a combination
    };

    /// @brief The shape on a grid.
    /// @param cell_size the grid's cell size in metres
    /// @param cells     the grid's cells along X, Y and Z
    /// @return the shape in lattice units
    Cells in_cells(float cell_size, const uint3& cells) const;

private:
    /// What a shape is.
    enum Kind {
        SPHERE,     ///< sphere()
        CYLINDER,   ///< cylinder()
        BOX,        ///< box()
        TRIANGLE,   ///< triangle()
        TORUS,      ///< torus()
        HALF_SPACE, ///< half_space()
        NOT,        ///< operator!
        AND,        ///< operator&
        OR          ///< operator|
    };

    Kind kind_ = SPHERE;          ///< what the shape is
    Position a_{};                ///< a center, a corner or a point on a plane
    Position b_{};                ///< a box's or triangle's second corner
    Position c_{};                ///< a triangle's third corner
    Length r1_{};                 ///< a radius
    Length r2_{};                 ///< a cylinder's length or a torus's tube radius
    Axis axis_ = Axis::X;         ///< a cylinder's or torus's axis
    float3 normal_{};             ///< HALF_SPACE: the outward normal
    std::vector<Shape> operands_; ///< the operands of a combination

    /// @brief A combination of shapes.
    /// @param kind NOT, AND or OR
    /// @param operands the shapes combined
    /// @return the combination
    static Shape combined(Kind kind, std::vector<Shape> operands);
};
