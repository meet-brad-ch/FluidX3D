#pragma once

#include "core/types.hpp"
#include "core/boundary_utils.hpp"
#include "boundaries/boundary_flags.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <vector>
#include <functional>

extern Units units;  // Global units object from lbm.cpp

/**
 * @file surface_builder.hpp
 * @brief Fluent free surface configuration for FluidX3D simulations
 *
 * Provides a readable API for setting up free surface (multiphase) simulations
 * including dam breaks, waves, droplets, and hydraulic flows.
 *
 * @note Requires SURFACE extension enabled in defines.hpp
 */

/**
 * @class SurfaceBuilder
 * @brief Fluent API for free surface simulation setup
 *
 * This class simplifies setting up free surface simulations including:
 * - Water level initialization
 * - Droplet and column shapes
 * - Gravity and surface tension
 * - Solid walls and open boundaries
 * - Hydrostatic pressure initialization
 *
 * @par Example (dam break):
 * @code
 * SurfaceBuilder surface(lbm);
 * surface
 *     .set_fluid_region([&](uint x, uint y, uint z) {
 *         return z < Nz*3/4 && y < Ny/8;  // Tall column at inlet
 *     })
 *     .set_gravity_lbm(0.0002f)
 *     .set_solid_walls()
 *     .initialize_hydrostatic()
 *     .apply();
 * @endcode
 *
 * @par Example (water level):
 * @code
 * SurfaceBuilder surface(lbm);
 * surface
 *     .set_water_level(0.5f)             // 50% of domain height
 *     .set_gravity_lbm(0.001f)
 *     .set_solid_walls()
 *     .apply();
 * @endcode
 */
class SurfaceBuilder {
public:
    /**
     * @brief Construct SurfaceBuilder for an LBM simulation
     * @param lbm Reference to the LBM simulation object
     */
    explicit SurfaceBuilder(LBM& lbm) : lbm_(lbm) {}

    // ========================================================================
    // Fluid Region Initialization
    // ========================================================================

    /**
     * @brief Set water level as fraction of domain height
     * @param height_fraction Water level (0.0 to 1.0, fraction of Nz)
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * surface.set_water_level(0.5f);  // Water fills bottom half
     * @endcode
     */
    SurfaceBuilder& set_water_level(float32_t height_fraction) {
        water_level_fraction_ = height_fraction;
        use_water_level_ = true;
        return *this;
    }

    /**
     * @brief Set water level in LBM cells
     * @param height_cells Water level in cells
     * @return Reference for method chaining
     */
    SurfaceBuilder& set_water_level_cells(uint32_t height_cells) {
        water_level_cells_ = height_cells;
        use_water_level_cells_ = true;
        return *this;
    }

    /**
     * @brief Define fluid region using a predicate function
     * @param predicate Function (x, y, z) -> bool, returns true for fluid cells
     * @return Reference for method chaining
     *
     * @par Example (dam break):
     * @code
     * const uint Nx = lbm.get_Nx(), Ny = lbm.get_Ny(), Nz = lbm.get_Nz();
     * surface.set_fluid_region([=](uint x, uint y, uint z) {
     *     return z < Nz*6/8 && y < Ny/8;  // Tall column at y=0
     * });
     * @endcode
     */
    SurfaceBuilder& set_fluid_region(std::function<bool(uint32_t, uint32_t, uint32_t)> predicate) {
        fluid_predicate_ = predicate;
        use_predicate_ = true;
        return *this;
    }

    /**
     * @brief Add a spherical droplet
     * @param center_x Center X position (fraction of Nx, 0-1)
     * @param center_y Center Y position (fraction of Ny, 0-1)
     * @param center_z Center Z position (fraction of Nz, 0-1)
     * @param radius Radius (fraction of smallest domain dimension)
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * surface.add_droplet(0.5f, 0.5f, 0.7f, 0.1f);  // Centered droplet at 70% height
     * @endcode
     */
    SurfaceBuilder& add_droplet(float32_t center_x, float32_t center_y,
                                 float32_t center_z, float32_t radius) {
        Droplet d;
        d.cx = center_x;
        d.cy = center_y;
        d.cz = center_z;
        d.radius = radius;
        droplets_.push_back(d);
        return *this;
    }

    /**
     * @brief Add a cylindrical fluid column
     * @param center_x Center X position (fraction of Nx)
     * @param center_y Center Y position (fraction of Ny)
     * @param radius Radius (fraction of smallest horizontal dimension)
     * @param height Height (fraction of Nz)
     * @return Reference for method chaining
     */
    SurfaceBuilder& add_column(float32_t center_x, float32_t center_y,
                                float32_t radius, float32_t height) {
        Column c;
        c.cx = center_x;
        c.cy = center_y;
        c.radius = radius;
        c.height = height;
        columns_.push_back(c);
        return *this;
    }

    // ========================================================================
    // Physics Configuration
    // ========================================================================

    /**
     * @brief Set gravity magnitude in LBM units
     * @param g Gravitational acceleration (positive value, applied in -Z direction)
     * @return Reference for method chaining
     *
     * @note Typical values: 0.0001 to 0.001 for stable simulations
     */
    SurfaceBuilder& set_gravity_lbm(float32_t g) {
        gravity_lbm_ = g;
        return *this;
    }

    /**
     * @brief Set gravity direction
     * @param axis Gravity direction axis (default: Z, meaning -Z is down)
     * @return Reference for method chaining
     */
    SurfaceBuilder& set_gravity_axis(Axis axis) {
        gravity_axis_ = axis;
        return *this;
    }

    /**
     * @brief Set surface tension coefficient in LBM units
     * @param sigma Surface tension coefficient
     * @return Reference for method chaining
     *
     * @note Surface tension is set during LBM construction, this stores the value
     *       for documentation purposes
     */
    SurfaceBuilder& set_surface_tension_lbm(float32_t sigma) {
        surface_tension_ = sigma;
        has_surface_tension_ = true;
        return *this;
    }

    // ========================================================================
    // Boundary Configuration
    // ========================================================================

    /**
     * @brief Set all domain boundaries as solid walls
     * @return Reference for method chaining
     */
    SurfaceBuilder& set_solid_walls() {
        solid_walls_ = true;
        return *this;
    }

    /**
     * @brief Set specific faces as solid
     * @param faces Vector of faces to make solid
     * @return Reference for method chaining
     */
    SurfaceBuilder& set_solid_faces(const std::vector<Face>& faces) {
        for (Face f : faces) {
            solid_faces_.push_back(f);
        }
        return *this;
    }

    /**
     * @brief Set a face as open (equilibrium) boundary
     * @param face Which face is open
     * @param velocity Optional velocity at boundary (LBM units)
     * @return Reference for method chaining
     */
    SurfaceBuilder& set_open_boundary(Face face, float32_t velocity = 0.0f) {
        OpenBoundary ob;
        ob.face = face;
        ob.velocity = velocity;
        open_boundaries_.push_back(ob);
        return *this;
    }

    /**
     * @brief Add a rectangular solid block region
     * @param min_x Minimum X (fraction of Nx or cells if use_cells=true)
     * @param max_x Maximum X
     * @param min_y Minimum Y
     * @param max_y Maximum Y
     * @param min_z Minimum Z
     * @param max_z Maximum Z
     * @param use_cells If true, coordinates are in cells; if false, fractions
     * @return Reference for method chaining
     *
     * @par Example (socket obstruction):
     * @code
     * surface.add_solid_block_cells(0, Nx, 0, socket_y, 0, socket_z);
     * @endcode
     */
    SurfaceBuilder& add_solid_block(float32_t min_x, float32_t max_x,
                                     float32_t min_y, float32_t max_y,
                                     float32_t min_z, float32_t max_z) {
        SolidBlock block;
        block.min_x = min_x; block.max_x = max_x;
        block.min_y = min_y; block.max_y = max_y;
        block.min_z = min_z; block.max_z = max_z;
        block.use_cells = false;
        solid_blocks_.push_back(block);
        return *this;
    }

    /**
     * @brief Add a rectangular solid block region in cell coordinates
     */
    SurfaceBuilder& add_solid_block_cells(uint32_t min_x, uint32_t max_x,
                                           uint32_t min_y, uint32_t max_y,
                                           uint32_t min_z, uint32_t max_z) {
        SolidBlock block;
        block.min_x = (float32_t)min_x; block.max_x = (float32_t)max_x;
        block.min_y = (float32_t)min_y; block.max_y = (float32_t)max_y;
        block.min_z = (float32_t)min_z; block.max_z = (float32_t)max_z;
        block.use_cells = true;
        solid_blocks_.push_back(block);
        return *this;
    }

    /**
     * @brief Add a velocity inlet region
     * @param predicate Function (x, y, z) -> bool, returns true for inlet cells
     * @param velocity_lbm Inlet velocity in LBM units
     * @param axis Velocity direction axis
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * surface.add_velocity_inlet(
     *     [=](uint x, uint y, uint z) { return y <= 1 && z >= socket_z && z < water_z; },
     *     u_inlet_lbm, Axis::Y);
     * @endcode
     */
    SurfaceBuilder& add_velocity_inlet(std::function<bool(uint32_t, uint32_t, uint32_t)> predicate,
                                        float32_t velocity_lbm, Axis axis) {
        VelocityInlet inlet;
        inlet.predicate = predicate;
        inlet.velocity = velocity_lbm;
        inlet.axis = axis;
        velocity_inlets_.push_back(inlet);
        return *this;
    }

    /**
     * @brief Add an equilibrium outlet region
     * @param predicate Function (x, y, z) -> bool, returns true for outlet cells
     * @param velocity_lbm Outlet velocity in LBM units (optional)
     * @param axis Velocity direction axis
     * @return Reference for method chaining
     */
    SurfaceBuilder& add_equilibrium_outlet(std::function<bool(uint32_t, uint32_t, uint32_t)> predicate,
                                            float32_t velocity_lbm = 0.0f, Axis axis = Axis::Y) {
        EquilibriumOutlet outlet;
        outlet.predicate = predicate;
        outlet.velocity = velocity_lbm;
        outlet.axis = axis;
        equilibrium_outlets_.push_back(outlet);
        return *this;
    }

    // ========================================================================
    // Initialization Options
    // ========================================================================

    /**
     * @brief Initialize with hydrostatic pressure distribution
     * @return Reference for method chaining
     *
     * Sets initial density based on water depth for stable startup.
     */
    SurfaceBuilder& initialize_hydrostatic() {
        init_hydrostatic_ = true;
        return *this;
    }

    /**
     * @brief Set initial velocity for fluid region
     * @param vx Velocity X component (LBM units)
     * @param vy Velocity Y component (LBM units)
     * @param vz Velocity Z component (LBM units)
     * @return Reference for method chaining
     */
    SurfaceBuilder& set_initial_velocity(float32_t vx, float32_t vy, float32_t vz) {
        init_velocity_ = float3(vx, vy, vz);
        has_init_velocity_ = true;
        return *this;
    }

    // ========================================================================
    // Apply
    // ========================================================================

    /**
     * @brief Apply all configured free surface settings
     *
     * This method iterates over all cells and applies the configured
     * fluid regions, boundaries, and initializations.
     */
    void apply() {
        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        // Calculate water level in cells
        uint32_t water_height = 0;
        if (use_water_level_) {
            water_height = (uint32_t)(water_level_fraction_ * (float32_t)Nz);
        } else if (use_water_level_cells_) {
            water_height = water_level_cells_;
        }

        // Pre-calculate droplet parameters in cells
        struct DropletParams {
            float32_t cx, cy, cz, r2;
        };
        std::vector<DropletParams> dp;
        for (const auto& d : droplets_) {
            DropletParams p;
            p.cx = d.cx * (float32_t)Nx;
            p.cy = d.cy * (float32_t)Ny;
            p.cz = d.cz * (float32_t)Nz;
            float32_t min_dim = (float32_t)std::min({Nx, Ny, Nz});
            float32_t r = d.radius * min_dim;
            p.r2 = r * r;
            dp.push_back(p);
        }

        // Pre-calculate column parameters
        struct ColumnParams {
            float32_t cx, cy, r2, max_z;
        };
        std::vector<ColumnParams> cp;
        for (const auto& c : columns_) {
            ColumnParams p;
            p.cx = c.cx * (float32_t)Nx;
            p.cy = c.cy * (float32_t)Ny;
            float32_t min_horiz = (float32_t)std::min(Nx, Ny);
            float32_t r = c.radius * min_horiz;
            p.r2 = r * r;
            p.max_z = c.height * (float32_t)Nz;
            cp.push_back(p);
        }

        // Get reference height for hydrostatic initialization
        uint32_t ref_height = water_height;
        if (ref_height == 0 && !columns_.empty()) {
            ref_height = (uint32_t)cp[0].max_z;
        }
        if (ref_height == 0) {
            ref_height = Nz / 2;
        }

        parallel_for(lbm_.get_N(), [&](uint64_t n) {
            uint32_t x = 0, y = 0, z = 0;
            lbm_.coordinates(n, x, y, z);

            bool is_fluid = false;

            // Check water level
            if (use_water_level_ || use_water_level_cells_) {
                if (z < water_height) {
                    is_fluid = true;
                }
            }

            // Check predicate
            if (use_predicate_ && fluid_predicate_) {
                if (fluid_predicate_(x, y, z)) {
                    is_fluid = true;
                }
            }

            // Check droplets
            for (const auto& d : dp) {
                float32_t dx = (float32_t)x - d.cx;
                float32_t dy = (float32_t)y - d.cy;
                float32_t dz = (float32_t)z - d.cz;
                if (dx*dx + dy*dy + dz*dz <= d.r2) {
                    is_fluid = true;
                }
            }

            // Check columns
            for (const auto& c : cp) {
                float32_t dx = (float32_t)x - c.cx;
                float32_t dy = (float32_t)y - c.cy;
                if (dx*dx + dy*dy <= c.r2 && (float32_t)z < c.max_z) {
                    is_fluid = true;
                }
            }

            // Set fluid flag
            if (is_fluid) {
                lbm_.flags[n] = TYPE_F;

                // Initialize hydrostatic pressure
                if (init_hydrostatic_) {
                    lbm_.rho[n] = units.rho_hydrostatic(gravity_lbm_, (float32_t)z, (float32_t)ref_height);
                }

                // Initialize velocity
                if (has_init_velocity_) {
                    lbm_.u.x[n] = init_velocity_.x;
                    lbm_.u.y[n] = init_velocity_.y;
                    lbm_.u.z[n] = init_velocity_.z;
                }
            }

            // Apply solid walls
            if (solid_walls_) {
                if (x == 0 || x == Nx - 1 || y == 0 || y == Ny - 1 || z == 0 || z == Nz - 1) {
                    lbm_.flags[n] = TYPE_S;
                }
            }

            // Apply specific solid faces
            for (Face f : solid_faces_) {
                if (boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, f)) {
                    lbm_.flags[n] = TYPE_S;
                }
            }

            // Apply open boundaries
            for (const auto& ob : open_boundaries_) {
                if (boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, ob.face)) {
                    // Don't override corners that are already solid
                    if (lbm_.flags[n] != TYPE_S) {
                        lbm_.flags[n] = TYPE_E;
                        // Set velocity based on face normal
                        if (ob.velocity != 0.0f) {
                            switch (ob.face) {
                                case Face::X_MIN: lbm_.u.x[n] = ob.velocity; break;
                                case Face::X_MAX: lbm_.u.x[n] = -ob.velocity; break;
                                case Face::Y_MIN: lbm_.u.y[n] = ob.velocity; break;
                                case Face::Y_MAX: lbm_.u.y[n] = -ob.velocity; break;
                                case Face::Z_MIN: lbm_.u.z[n] = ob.velocity; break;
                                case Face::Z_MAX: lbm_.u.z[n] = -ob.velocity; break;
                            }
                        }
                    }
                }
            }

            // Apply solid blocks
            for (const auto& block : solid_blocks_) {
                float32_t bmin_x = block.use_cells ? block.min_x : block.min_x * (float32_t)Nx;
                float32_t bmax_x = block.use_cells ? block.max_x : block.max_x * (float32_t)Nx;
                float32_t bmin_y = block.use_cells ? block.min_y : block.min_y * (float32_t)Ny;
                float32_t bmax_y = block.use_cells ? block.max_y : block.max_y * (float32_t)Ny;
                float32_t bmin_z = block.use_cells ? block.min_z : block.min_z * (float32_t)Nz;
                float32_t bmax_z = block.use_cells ? block.max_z : block.max_z * (float32_t)Nz;

                if ((float32_t)x >= bmin_x && (float32_t)x < bmax_x &&
                    (float32_t)y >= bmin_y && (float32_t)y < bmax_y &&
                    (float32_t)z >= bmin_z && (float32_t)z < bmax_z) {
                    lbm_.flags[n] = TYPE_S;
                }
            }

            // Apply velocity inlets
            for (const auto& inlet : velocity_inlets_) {
                if (inlet.predicate && inlet.predicate(x, y, z)) {
                    lbm_.flags[n] = TYPE_F;
                    switch (inlet.axis) {
                        case Axis::X: lbm_.u.x[n] = inlet.velocity; break;
                        case Axis::Y: lbm_.u.y[n] = inlet.velocity; break;
                        case Axis::Z: lbm_.u.z[n] = inlet.velocity; break;
                    }
                }
            }

            // Apply equilibrium outlets
            for (const auto& outlet : equilibrium_outlets_) {
                if (outlet.predicate && outlet.predicate(x, y, z)) {
                    lbm_.flags[n] = TYPE_E;
                    if (outlet.velocity != 0.0f) {
                        switch (outlet.axis) {
                            case Axis::X: lbm_.u.x[n] = outlet.velocity; break;
                            case Axis::Y: lbm_.u.y[n] = outlet.velocity; break;
                            case Axis::Z: lbm_.u.z[n] = outlet.velocity; break;
                        }
                    }
                }
            }
        });
    }

    /**
     * @brief Configure recommended visualization for free surface
     *
     * Sets visualization mode to PHI raytrace (single GPU) or rasterize (multi-GPU)
     */
    void configure_visualization() {
        if (lbm_.get_D() == 1u) {
            lbm_.graphics.visualization_modes = VIS_PHI_RAYTRACE;
        } else {
            lbm_.graphics.visualization_modes = VIS_PHI_RASTERIZE;
        }
    }

private:
    LBM& lbm_;

    // Water level
    bool use_water_level_ = false;
    bool use_water_level_cells_ = false;
    float32_t water_level_fraction_ = 0.5f;
    uint32_t water_level_cells_ = 0;

    // Predicate function
    bool use_predicate_ = false;
    std::function<bool(uint32_t, uint32_t, uint32_t)> fluid_predicate_;

    // Droplets
    struct Droplet {
        float32_t cx, cy, cz, radius;
    };
    std::vector<Droplet> droplets_;

    // Columns
    struct Column {
        float32_t cx, cy, radius, height;
    };
    std::vector<Column> columns_;

    // Physics
    float32_t gravity_lbm_ = 0.0002f;
    Axis gravity_axis_ = Axis::Z;
    bool has_surface_tension_ = false;
    float32_t surface_tension_ = 0.0f;

    // Boundaries
    bool solid_walls_ = false;
    std::vector<Face> solid_faces_;

    struct OpenBoundary {
        Face face;
        float32_t velocity;
    };
    std::vector<OpenBoundary> open_boundaries_;

    // Solid blocks
    struct SolidBlock {
        float32_t min_x, max_x, min_y, max_y, min_z, max_z;
        bool use_cells;
    };
    std::vector<SolidBlock> solid_blocks_;

    // Velocity inlets
    struct VelocityInlet {
        std::function<bool(uint32_t, uint32_t, uint32_t)> predicate;
        float32_t velocity;
        Axis axis;
    };
    std::vector<VelocityInlet> velocity_inlets_;

    // Equilibrium outlets
    struct EquilibriumOutlet {
        std::function<bool(uint32_t, uint32_t, uint32_t)> predicate;
        float32_t velocity;
        Axis axis;
    };
    std::vector<EquilibriumOutlet> equilibrium_outlets_;

    // Initialization
    bool init_hydrostatic_ = false;
    bool has_init_velocity_ = false;
    float3 init_velocity_{0.0f, 0.0f, 0.0f};
};
