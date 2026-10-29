#pragma once

#include "setup/boundaries/boundary_flags.hpp"
#include "setup/domain/lattice.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"
#include "utilities.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

/// @file shape.hpp
/// @brief Regions of the domain in metres (Shape) and their cells on a grid (Shape::Cells).

/// @brief A region of the domain in metres from its origin corner: a solid object, water or gas.
///
/// Shapes combine with ! (outside), & (in both) and | (in either):
/// @code
/// using namespace si::symbols;
/// !Shape::cylinder({ 5.0 * cm, 0.0 * m, 5.0 * cm }, Axis::Y, 4.0 * cm, 1.0 * m)   // the wall around a pipe
/// Shape::torus(center, Axis::X, 2.0 * cm, 2.0 * cm) & Shape::box(from, to)         // part of a ring
/// @endcode
/// A cell belongs to a shape when its centre does (cell i spans i to i+1 cell sizes from the origin), as the core's
/// shapes.hpp tests cells; a box covers whole cells as CellSpan.
///
/// A shape is a list of nodes in postfix order: the primitives and the operators NOT, AND and OR, each operator after
/// its operands. A combination appends its operands' lists and its operator, so no part of a shape holds another
/// shape, and nothing is recursive. A default shape is a sphere of radius 0 at the origin corner (it has no cells).
class Shape {
private:
    /// What a node of a shape is: a primitive or an operator.
    enum class Kind {
        SPHERE,     ///< sphere()
        CYLINDER,   ///< cylinder()
        BOX,        ///< box()
        TRIANGLE,   ///< triangle()
        TORUS,      ///< torus()
        HALF_SPACE, ///< half_space()
        NOT,        ///< operator!: the outside of the node before it
        AND,        ///< operator&: in both of the two operands before it
        OR          ///< operator|: in either of the two operands before it
    };

    /// One node of a shape in metres: a primitive with its values, or an operator (no values).
    struct Node {
        Kind kind{ Kind::SPHERE };        ///< what the node is
        si::Position a{};                 ///< a centre, a corner or a point on a plane
        si::Position b{};                 ///< a box's or a triangle's second corner
        si::Position c{};                 ///< a triangle's third corner
        si::Length r1{ au::ZERO };        ///< a radius
        si::Length r2{ au::ZERO };        ///< a cylinder's length or a torus's tube radius
        Axis axis{ Axis::X };             ///< a cylinder's or a torus's axis
        si::Direction normal{};           ///< HALF_SPACE: the outward normal
    };

public:
    /// @brief Makes a sphere.
    /// @param center the sphere's centre
    /// @param radius its radius, more than 0
    /// @return the sphere
    static Shape sphere(si::Position center, si::Length radius);

    /// @brief Makes a cylinder whose axis goes through a point, along a coordinate axis.
    /// @param center the centre of the cylinder
    /// @param axis the coordinate axis it lies along
    /// @param radius the cylinder's radius
    /// @param length its length along the axis
    /// @return the cylinder
    static Shape cylinder(si::Position center, Axis axis, si::Length radius, si::Length length);

    /// @brief Makes an axis-aligned box between two corners, covering whole cells.
    /// @param from the corner with the smaller coordinates
    /// @param to the opposite corner
    /// @return the box
    static Shape box(si::Position from, si::Position to);

    /// @brief Makes a triangular plate about one cell thick.
    /// @param a the first corner
    /// @param b the second corner
    /// @param c the third corner
    /// @return the plate
    static Shape triangle(si::Position a, si::Position b, si::Position c);

    /// @brief Makes a ring about a coordinate axis.
    /// @param center the ring's centre
    /// @param axis the coordinate axis through the centre
    /// @param ring_radius the radius of the ring's centre line
    /// @param tube_radius the radius of the ring's round cross-section
    /// @return the torus
    static Shape torus(si::Position center, Axis axis, si::Length ring_radius, si::Length tube_radius);

    /// @brief Makes everything on one side of the plane through a point: the side that the normal points away from
    /// (a sloped beach, a wall).
    /// @param point a point on the plane
    /// @param outward_normal the direction that points out of the shape (any length but 0)
    /// @return the half-space
    static Shape half_space(si::Position point, si::Direction outward_normal);

    /// @brief Makes everywhere outside a shape.
    /// @param shape the shape whose outside is wanted
    /// @return the complement
    friend Shape operator!(const Shape& shape);

    /// @brief Makes the intersection of two shapes.
    /// @param a a shape
    /// @param b a shape
    /// @return the cells in both
    friend Shape operator&(const Shape& a, const Shape& b);

    /// @brief Makes the union of two shapes.
    /// @param a a shape
    /// @param b a shape
    /// @return the cells in either
    friend Shape operator|(const Shape& a, const Shape& b);

    /// @brief A shape on one grid, in the core's cell coordinates: what the builders test cell by cell.
    ///
    /// Its nodes are in postfix order, with the operands of each AND and OR in the order that needs the fewest stack
    /// slots (the operand that needs more slots first; both operators are commutative, so the result is the same).
    /// Then a shape of n primitives needs at most log2(n) + 1 slots, and contains() and fill() evaluate the nodes in
    /// one loop over a fixed stack, without an allocation.
    class Cells {
    public:
        /// @brief Tells whether a cell's centre is inside the shape.
        /// @param x the cell along X, counted from 0
        /// @param y the cell along Y, counted from 0
        /// @param z the cell along Z, counted from 0
        /// @return true when the centre is inside
        bool contains(uint32_t x, uint32_t y, uint32_t z) const;

        /// @brief Gives the part of a cell inside the shape, 0 to 1: smooth (PLIC) at a sphere's surface, else 0 or
        /// 1. NOT gives 1 minus its operand, AND the smaller and OR the larger of its two operands.
        /// @param x the cell along X, counted from 0
        /// @param y the cell along Y, counted from 0
        /// @param z the cell along Z, counted from 0
        /// @return the part (a ratio)
        float fill(uint32_t x, uint32_t y, uint32_t z) const;

        /// The stack slots of contains() and fill(): enough for any shape of fewer than 2^63 primitives.
        static constexpr std::size_t stack_capacity = 64u;

    private:
        /// One node of a shape on the grid: a primitive in the core's cell coordinates, or an operator.
        struct Node {
            Kind kind{ Kind::SPHERE };  ///< what the node is
            float3 p0{};                ///< a centre, a corner or a point on a plane, in the core's cell coordinates (cell i is at i)
            float3 p1{};                ///< a corner, a cylinder's axis vector (in cells) or a half-space's normal
            float3 p2{};                ///< a triangle's third corner, in the core's cell coordinates
            float r1{};                 ///< a radius in cells
            float r2{};                 ///< a cylinder's length or a torus's tube radius, in cells
            Axis axis{ Axis::X };       ///< a torus's axis
            CellSpan x{};               ///< a box's cells along X
            CellSpan y{};               ///< a box's cells along Y
            CellSpan z{};               ///< a box's cells along Z
        };

        /// @brief Tells whether a cell's centre is inside a primitive node.
        /// @param node a primitive (not an operator)
        /// @param x the cell along X
        /// @param y the cell along Y
        /// @param z the cell along Z
        /// @return true when the centre is inside
        static bool primitive_contains(const Node& node, uint32_t x, uint32_t y, uint32_t z);

        /// @brief Tells whether a cell's centre is inside a torus, by its axis (the core's torus_x/y/z()).
        /// @param node a torus
        /// @param x the cell along X
        /// @param y the cell along Y
        /// @param z the cell along Z
        /// @return true when the centre is inside
        static bool torus_contains(const Node& node, uint32_t x, uint32_t y, uint32_t z);

        /// @brief Gives the part of a cell inside a primitive node: PLIC for a sphere, else 0 or 1.
        /// @param node a primitive (not an operator)
        /// @param x the cell along X
        /// @param y the cell along Y
        /// @param z the cell along Z
        /// @return the part (a ratio)
        static float primitive_fill(const Node& node, uint32_t x, uint32_t y, uint32_t z);

        friend class Shape;
        std::vector<Node> nodes_{}; ///< the nodes in postfix order, the deeper operand of each AND and OR first
    };

    /// @brief Gives the shape on a grid. Lengths become cells in double, and each value becomes a float once, for
    /// the core's cell tests.
    /// @param cell the grid's cell size, more than 0
    /// @param cells the grid's cells along X, Y and Z
    /// @return the shape in the core's cell coordinates
    /// @throws SetupError when the shape is not a valid postfix list or needs more than Cells::stack_capacity stack
    /// slots (neither can happen with the factories and operators)
    Cells in_cells(si::Length cell, const uint3& cells) const;

private:
    std::vector<Node> nodes_{ Node{} }; ///< the nodes in postfix order: each operator after its operands

    /// @brief Makes a shape of one primitive node.
    /// @param node the primitive
    /// @return the shape
    static Shape primitive(const Node& node);

    /// @brief Makes a combination of two shapes: the nodes of a, then those of b, then the operator.
    /// @param kind AND or OR
    /// @param a the first operand
    /// @param b the second operand
    /// @return the combination
    static Shape combined(Kind kind, const Shape& a, const Shape& b);

    /// A part of a shape on a grid while in_cells() orders it: its nodes in postfix order and the stack slots that
    /// their evaluation needs.
    struct Segment {
        std::vector<Cells::Node> nodes{}; ///< the nodes in postfix order
        std::uint32_t slots{};            ///< the stack slots that the evaluation of the nodes needs
    };

    /// @brief Joins the two operands of an AND or an OR: the one that needs more stack slots first (the first given
    /// when both need the same), then the operator.
    /// @param first the first operand
    /// @param second the second operand
    /// @param op the operator node, AND or OR
    /// @return the joined part, with its stack slots: the larger of the two, or one more when both are the same
    static Segment joined(Segment first, Segment second, const Cells::Node& op);

    /// @brief Adds one node of the postfix list to the parts of in_cells(): a primitive as a new part, NOT onto the
    /// last part, AND and OR as the join of the last two parts.
    /// @param parts the parts so far, in postfix order
    /// @param node the next node, on the grid
    /// @throws SetupError when an operator has fewer parts before it than operands
    static void add_node(std::vector<Segment>& parts, const Cells::Node& node);

    /// @brief Gives one node on a grid.
    /// @param node a node in metres
    /// @param cell the grid's cell size
    /// @param cells the grid's cells along X, Y and Z
    /// @return the node in the core's cell coordinates (an operator stays an operator)
    static Cells::Node cell_node(const Node& node, si::Length cell, const uint3& cells);

    /// @brief Gives a length in cells, as a float.
    /// @param x a length
    /// @param cell the cell size
    /// @return x / cell
    static float in_cell_units(si::Length x, si::Length cell);

    /// @brief Gives a point in the core's cell coordinates, in which the centre of cell i is at i.
    /// @param p a point in metres from the origin corner
    /// @param cell the cell size
    /// @return p / cell - 1/2 along each axis
    static float3 cell_point(const si::Position& p, si::Length cell);
};
