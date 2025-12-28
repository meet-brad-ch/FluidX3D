#pragma once

#include "core/types.hpp"
#include "boundaries/boundary_flags.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <string>
#include <fstream>
#include <deque>
#include <cmath>

extern Units units;  // Global units object from lbm.cpp

// Note: Cell type flags TYPE_S, TYPE_E, TYPE_X etc. are defined in defines.hpp
// Note: Axis enum (X, Y, Z) is defined in boundary_flags.hpp

/**
 * @file force_analyzer.hpp
 * @brief Force and drag coefficient analysis for FluidX3D simulations
 *
 * Provides a simple API for calculating aerody namic forces and coefficients
 * on solid objects marked with TYPE_X flag during voxelization.
 */

/**
 * @class ForceAnalyzer
 * @brief Calculates forces and aerodynamic coefficients on solid objects
 *
 * This class wraps the LBM force calculation functions and provides
 * easy-to-use methods for computing drag and lift coefficients.
 *
 * @par Prerequisites:
 * - Object must be voxelized with TYPE_S|TYPE_X flags
 * - Units must be configured via SimulationSetup::configure_units()
 * - FORCE_FIELD extension must be enabled in defines.hpp
 *
 * @par Example:
 * @code
 * // After voxelization with force tracking enabled
 * ForceAnalyzer forces(lbm);
 * forces.set_reference_area(0.112f)      // Frontal area in m²
 *       .set_reference_velocity(60.0f)   // Flow velocity in m/s
 *       .set_flow_direction(Axis::Y);    // Flow along Y axis
 *
 * while(lbm.get_t() <= lbm_T) {
 *     lbm.run(1u);
 *     forces.print_coefficients();
 * }
 * @endcode
 */
class ForceAnalyzer {
public:
    /**
     * @brief Construct ForceAnalyzer for an LBM simulation
     * @param lbm Reference to the LBM simulation object
     * @param flag_marker Cell flag to identify tracked object (default: TYPE_S|TYPE_X)
     */
    explicit ForceAnalyzer(LBM& lbm, uchar flag_marker = TYPE_S | TYPE_X)
        : lbm_(lbm), flag_marker_(flag_marker) {}

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Set reference area for coefficient calculations (SI units)
     * @param si_area_m2 Reference area in square meters (typically frontal area)
     * @return Reference for method chaining
     */
    ForceAnalyzer& set_reference_area(float32_t si_area_m2) {
        reference_area_ = si_area_m2;
        return *this;
    }

    /**
     * @brief Set reference velocity for coefficient calculations (SI units)
     * @param si_velocity Reference velocity in m/s (typically freestream velocity)
     * @return Reference for method chaining
     */
    ForceAnalyzer& set_reference_velocity(float32_t si_velocity) {
        reference_velocity_ = si_velocity;
        return *this;
    }

    /**
     * @brief Set fluid density for coefficient calculations (SI units)
     * @param si_density Fluid density in kg/m³ (default: 1.225 for air at STP)
     * @return Reference for method chaining
     */
    ForceAnalyzer& set_fluid_density(float32_t si_density = 1.225f) {
        fluid_density_ = si_density;
        return *this;
    }

    /**
     * @brief Set flow direction axis for drag/lift decomposition
     * @param axis Principal flow direction
     * @return Reference for method chaining
     *
     * @note Drag is force component along flow direction.
     *       Lift is force component perpendicular to flow (typically Z for horizontal flow).
     */
    ForceAnalyzer& set_flow_direction(Axis axis) {
        flow_axis_ = axis;
        return *this;
    }

    /**
     * @brief Set the flag marker used to identify the tracked object
     * @param flag_marker Cell flag combination (default: TYPE_S|TYPE_X)
     * @return Reference for method chaining
     */
    ForceAnalyzer& set_flag_marker(uchar flag_marker) {
        flag_marker_ = flag_marker;
        return *this;
    }

    // ========================================================================
    // Force Calculations (LBM units)
    // ========================================================================

    /**
     * @brief Get total force on object in LBM units
     * @return Force vector (Fx, Fy, Fz) in LBM units
     */
    float3 get_force_lbm() {
        return lbm_.object_force(flag_marker_);
    }

    /**
     * @brief Get torque on object around center of mass in LBM units
     * @return Torque vector in LBM units
     */
    float3 get_torque_lbm() {
        float3 com = get_center_of_mass_lbm();
        return lbm_.object_torque(com, flag_marker_);
    }

    /**
     * @brief Get center of mass of the tracked object in LBM units
     * @return Position (x, y, z) in cell coordinates
     */
    float3 get_center_of_mass_lbm() {
        return lbm_.object_center_of_mass(flag_marker_);
    }

    // ========================================================================
    // Force Calculations (SI units)
    // ========================================================================

    /**
     * @brief Get total force on object in SI units (Newtons)
     * @return Force vector (Fx, Fy, Fz) in Newtons
     */
    float3 get_force_si() {
        float3 lbm_force = get_force_lbm();
        return float3(
            units.si_F(lbm_force.x),
            units.si_F(lbm_force.y),
            units.si_F(lbm_force.z)
        );
    }

    /**
     * @brief Get torque on object in SI units (N·m)
     * @return Torque vector in Newton-meters
     */
    float3 get_torque_si() {
        float3 lbm_torque = get_torque_lbm();
        return float3(
            units.si_M(lbm_torque.x),
            units.si_M(lbm_torque.y),
            units.si_M(lbm_torque.z)
        );
    }

    // ========================================================================
    // Aerodynamic Coefficients
    // ========================================================================

    /**
     * @brief Calculate drag coefficient (Cd)
     * @return Drag coefficient (dimensionless)
     *
     * Cd = F_drag / (0.5 * rho * u² * A)
     */
    float32_t get_drag_coefficient() {
        float3 force_si = get_force_si();
        float32_t drag_force = get_force_component(force_si, flow_axis_);
        return force_to_coefficient(drag_force);
    }

    /**
     * @brief Calculate lift coefficient (Cl)
     * @return Lift coefficient (dimensionless)
     *
     * Lift is perpendicular to flow direction (typically Z for horizontal flow).
     */
    float32_t get_lift_coefficient() {
        float3 force_si = get_force_si();
        // Lift is typically in Z direction for horizontal flow
        Axis lift_axis = (flow_axis_ == Axis::Z) ? Axis::Y : Axis::Z;
        float32_t lift_force = get_force_component(force_si, lift_axis);
        return force_to_coefficient(lift_force);
    }

    /**
     * @brief Calculate side force coefficient
     * @return Side force coefficient (dimensionless)
     */
    float32_t get_side_coefficient() {
        float3 force_si = get_force_si();
        Axis side_axis = get_side_axis();
        float32_t side_force = get_force_component(force_si, side_axis);
        return force_to_coefficient(side_force);
    }

    /**
     * @brief Get dynamic pressure used in coefficient calculations
     * @return Dynamic pressure in Pa (0.5 * rho * u²)
     */
    float32_t get_dynamic_pressure() const {
        return 0.5f * fluid_density_ * reference_velocity_ * reference_velocity_;
    }

    // ========================================================================
    // Output Helpers
    // ========================================================================

    /**
     * @brief Print current force values to console
     */
    void print_forces() {
        float3 force_si = get_force_si();
        print_info("Force [N]: Fx=" + to_string(force_si.x, 3u) +
                   ", Fy=" + to_string(force_si.y, 3u) +
                   ", Fz=" + to_string(force_si.z, 3u));
    }

    /**
     * @brief Print current coefficients to console
     */
    void print_coefficients() {
        float32_t Cd = get_drag_coefficient();
        float32_t Cl = get_lift_coefficient();
        print_info("Cd=" + to_string(Cd, 4u) + ", Cl=" + to_string(Cl, 4u));
    }

    /**
     * @brief Print comprehensive force and coefficient summary
     */
    void print_summary() {
        float3 force_si = get_force_si();
        float3 torque_si = get_torque_si();
        float32_t Cd = get_drag_coefficient();
        float32_t Cl = get_lift_coefficient();

        print_info("=== Force Analysis ===");
        print_info("Force [N]:  Fx=" + to_string(force_si.x, 3u) +
                   ", Fy=" + to_string(force_si.y, 3u) +
                   ", Fz=" + to_string(force_si.z, 3u));
        print_info("Torque [Nm]: Tx=" + to_string(torque_si.x, 3u) +
                   ", Ty=" + to_string(torque_si.y, 3u) +
                   ", Tz=" + to_string(torque_si.z, 3u));
        print_info("Cd=" + to_string(Cd, 4u) + ", Cl=" + to_string(Cl, 4u));
    }

    /**
     * @brief Initialize a log file with header
     * @param path File path for logging
     */
    void init_log_file(const std::string& path) {
        log_path_ = path;
        std::ofstream file(path);
        file << "# timestep\tFx[N]\tFy[N]\tFz[N]\tCd\tCl\n";
        file.close();
    }

    /**
     * @brief Append current values to log file
     * @note init_log_file() must be called first
     */
    void log_to_file() {
        if (log_path_.empty()) return;

        float3 force_si = get_force_si();
        float32_t Cd = get_drag_coefficient();
        float32_t Cl = get_lift_coefficient();

        std::ofstream file(log_path_, std::ios::app);
        file << lbm_.get_t() << "\t"
             << force_si.x << "\t"
             << force_si.y << "\t"
             << force_si.z << "\t"
             << Cd << "\t"
             << Cl << "\n";
        file.close();
    }

    // ========================================================================
    // Convergence Monitoring
    // ========================================================================

    /**
     * @brief Enable convergence tracking with specified history size
     * @param history_size Number of samples to use for convergence check (default: 100)
     * @return Reference for method chaining
     *
     * @note Must be called before using is_converged() or get_convergence_error()
     *
     * @par Example:
     * @code
     * forces.enable_convergence_tracking(200);  // Use 200 samples
     * while(!forces.is_converged(1e-4f)) {
     *     lbm.run(100u);
     *     forces.update();  // Sample current values
     * }
     * @endcode
     */
    ForceAnalyzer& enable_convergence_tracking(size_t history_size = 100) {
        convergence_enabled_ = true;
        max_history_size_ = history_size;
        cd_history_.clear();
        cl_history_.clear();
        return *this;
    }

    /**
     * @brief Sample current coefficient values for convergence tracking
     *
     * Call this periodically (e.g., every 100 timesteps) to update the
     * convergence history. The method calculates Cd and Cl and stores
     * them in the history buffer.
     *
     * @note enable_convergence_tracking() must be called first
     */
    void update() {
        if (!convergence_enabled_) return;

        float32_t Cd = get_drag_coefficient();
        float32_t Cl = get_lift_coefficient();

        cd_history_.push_back(Cd);
        cl_history_.push_back(Cl);

        // Keep history size bounded
        while (cd_history_.size() > max_history_size_) {
            cd_history_.pop_front();
        }
        while (cl_history_.size() > max_history_size_) {
            cl_history_.pop_front();
        }
    }

    /**
     * @brief Check if force coefficients have converged
     * @param tolerance Maximum allowed relative variation (default: 1e-4 = 0.01%)
     * @return True if both Cd and Cl have converged within tolerance
     *
     * Convergence is determined by comparing the standard deviation of
     * recent samples to the mean. Converged when: stddev/mean < tolerance
     *
     * @note enable_convergence_tracking() and update() must be called first
     * @note Returns false if history is less than half full
     *
     * @par Example:
     * @code
     * forces.enable_convergence_tracking(100);
     * while(!forces.is_converged(1e-4f)) {
     *     lbm.run(100u);
     *     forces.update();
     *     forces.print_coefficients();
     * }
     * print_info("Simulation converged!");
     * @endcode
     */
    bool is_converged(float32_t tolerance = 1e-4f) const {
        if (!convergence_enabled_) return false;
        if (cd_history_.size() < max_history_size_ / 2) return false;

        float32_t cd_error = calculate_relative_variation(cd_history_);
        float32_t cl_error = calculate_relative_variation(cl_history_);

        return (cd_error < tolerance) && (cl_error < tolerance);
    }

    /**
     * @brief Get current convergence error (relative variation)
     * @return Maximum of Cd and Cl relative variations
     *
     * @note enable_convergence_tracking() and update() must be called first
     * @note Returns 1.0 if history is empty
     */
    float32_t get_convergence_error() const {
        if (!convergence_enabled_ || cd_history_.empty()) return 1.0f;

        float32_t cd_error = calculate_relative_variation(cd_history_);
        float32_t cl_error = calculate_relative_variation(cl_history_);

        return std::max(cd_error, cl_error);
    }

    /**
     * @brief Get current Cd convergence error (relative variation)
     * @return Cd relative variation (stddev/mean)
     */
    float32_t get_cd_convergence_error() const {
        if (!convergence_enabled_ || cd_history_.empty()) return 1.0f;
        return calculate_relative_variation(cd_history_);
    }

    /**
     * @brief Get current Cl convergence error (relative variation)
     * @return Cl relative variation (stddev/mean)
     */
    float32_t get_cl_convergence_error() const {
        if (!convergence_enabled_ || cl_history_.empty()) return 1.0f;
        return calculate_relative_variation(cl_history_);
    }

    /**
     * @brief Get average Cd from history (for converged value)
     * @return Mean Cd from history buffer, or current Cd if history empty
     */
    float32_t get_average_cd() {
        if (cd_history_.empty()) return get_drag_coefficient();
        return calculate_mean(cd_history_);
    }

    /**
     * @brief Get average Cl from history (for converged value)
     * @return Mean Cl from history buffer, or current Cl if history empty
     */
    float32_t get_average_cl() {
        if (cl_history_.empty()) return get_lift_coefficient();
        return calculate_mean(cl_history_);
    }

    /**
     * @brief Print convergence status to console
     */
    void print_convergence_status() const {
        if (!convergence_enabled_) {
            print_info("Convergence tracking not enabled");
            return;
        }

        float32_t cd_err = get_cd_convergence_error();
        float32_t cl_err = get_cl_convergence_error();
        size_t samples = cd_history_.size();

        print_info("Convergence: Cd_err=" + to_string(cd_err, 6u) +
                   ", Cl_err=" + to_string(cl_err, 6u) +
                   " (" + std::to_string(samples) + "/" + std::to_string(max_history_size_) + " samples)");
    }

private:
    LBM& lbm_;
    uchar flag_marker_;

    // Reference values for coefficient calculation
    float32_t reference_area_ = 1.0f;      // m²
    float32_t reference_velocity_ = 1.0f;  // m/s
    float32_t fluid_density_ = 1.225f;     // kg/m³ (air at STP)
    Axis flow_axis_ = Axis::Y;         // Default: flow along Y

    // Logging
    std::string log_path_;

    // Convergence tracking
    bool convergence_enabled_ = false;
    size_t max_history_size_ = 100;
    std::deque<float32_t> cd_history_;
    std::deque<float32_t> cl_history_;

    // Helper: Calculate mean of a deque
    static float32_t calculate_mean(const std::deque<float32_t>& values) {
        if (values.empty()) return 0.0f;
        float32_t sum = 0.0f;
        for (float32_t v : values) sum += v;
        return sum / (float32_t)values.size();
    }

    // Helper: Calculate relative variation (stddev/mean) of a deque
    static float32_t calculate_relative_variation(const std::deque<float32_t>& values) {
        if (values.size() < 2) return 1.0f;

        float32_t mean = calculate_mean(values);
        if (std::abs(mean) < 1e-10f) return 1.0f;  // Avoid division by zero

        float32_t sum_sq = 0.0f;
        for (float32_t v : values) {
            float32_t diff = v - mean;
            sum_sq += diff * diff;
        }
        float32_t stddev = std::sqrt(sum_sq / (float32_t)values.size());
        return stddev / std::abs(mean);
    }

    // Helper: Convert force to coefficient
    float32_t force_to_coefficient(float32_t force_si) const {
        float32_t q = get_dynamic_pressure();
        if (q * reference_area_ < 1e-10f) return 0.0f;
        return force_si / (q * reference_area_);
    }

    // Helper: Extract force component for given axis
    float32_t get_force_component(const float3& force, Axis axis) const {
        switch (axis) {
            case Axis::X: return force.x;
            case Axis::Y: return force.y;
            case Axis::Z: return force.z;
            default: return force.y;
        }
    }

    // Helper: Get side force axis (perpendicular to flow and lift)
    Axis get_side_axis() const {
        switch (flow_axis_) {
            case Axis::X: return Axis::Y;
            case Axis::Y: return Axis::X;
            case Axis::Z: return Axis::X;
            default: return Axis::X;
        }
    }
};
