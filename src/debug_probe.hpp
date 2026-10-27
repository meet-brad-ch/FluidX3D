// DEBUG_PROBE (this fork): what stream_collide computes in every cell, written
// to device buffers on every step, so that a host reference can check the kernel step by step.
//
// - The cell probe: debug_probe::cell_channels floats per cell, structure of arrays (channel c of cell n at
//   c*N + n), overwritten every step. It holds the inputs of the scalar collision (the velocity the thermal and moisture
//   equilibria use, the relaxation rate) and its results (T, Q, the fluxes before the collision).
// - The cell trace: one chosen cell writes debug_probe::trace_channels floats per step into a ring of
//   debug_probe::trace_steps steps (step t at (t % trace_steps) * trace_channels): the cell probe's channels plus every
//   population of the thermal and moisture lattices before the collision, their equilibria and after the collision.
//
// The kernel reads the same numbers from defines that lbm.cpp writes from the constants below (one source).
#pragma once

#include <array>
#include <string>

/// @brief The layout of the DEBUG_PROBE buffers, shared by lbm.cpp (the OpenCL defines) and the host tools.
class DebugProbe {
public:
	/// @brief The channels of the cell probe, in buffer order.
	enum class Cell : unsigned int {
		rho = 0u,         ///< the density from the DDFs
		ux_j, uy_j, uz_j, ///< the velocity j / rho before the Guo force shift
		fx, fy, fz,       ///< the total force of the cell (volume force, level forcing, buoyancy)
		ux_eq, uy_eq, uz_eq, ///< the velocity in the thermal and moisture equilibria, j / rho + F / (2 rho)
		w_flow,           ///< the flow's relaxation rate after the subgrid model
		w_T,              ///< the thermal and moisture relaxation rate (0 for TYPE_T cells: no collision)
		T,                ///< the temperature before the collision (Sum g + 1)
		Q,                ///< the moisture before the collision (Sum h + 1); 0 without MOISTURE
		T_buoy,           ///< the buoyant temperature (T, or theta_v in lattice units with MOISTURE)
		T_ref,            ///< the buoyancy's reference temperature
		gflux_x, gflux_y, gflux_z, ///< Sum c_i g_i before the collision
		hflux_x, hflux_y, hflux_z, ///< Sum c_i h_i before the collision; 0 without MOISTURE
		theta_v,          ///< the kernel's theta_v (K) with MOISTURE and a level pressure; 0 otherwise
		flags,            ///< the cell's flags
		count             ///< the number of channels
	};
	static constexpr unsigned int cell_channels = static_cast<unsigned int>(Cell::count);
	/// @brief The first channel of each group in the trace record of one step.
	static constexpr unsigned int trace_cell = 0u;                       ///< the cell probe's channels
	static constexpr unsigned int trace_g_pre = cell_channels;           ///< g_0..g_6 before the collision
	static constexpr unsigned int trace_g_eq = trace_g_pre+7u;           ///< the thermal equilibrium
	static constexpr unsigned int trace_g_post = trace_g_eq+7u;          ///< g after the collision and the sources
	static constexpr unsigned int trace_h_pre = trace_g_post+7u;         ///< h_0..h_6 before the collision
	static constexpr unsigned int trace_h_eq = trace_h_pre+7u;           ///< the moisture equilibrium
	static constexpr unsigned int trace_h_post = trace_h_eq+7u;          ///< h after the collision and the sources
	static constexpr unsigned int trace_step = trace_h_post+7u;          ///< the step t as a float (exact below 2^24)
	static constexpr unsigned int trace_channels = trace_step+1u;
	static constexpr unsigned int trace_steps = 256u;                    ///< the ring length; read before it wraps
	/// @brief The cell probe's channel names, in buffer order (for the dump headers).
	static std::array<std::string, cell_channels> cell_names() {
		return { "rho", "ux_j", "uy_j", "uz_j", "fx", "fy", "fz", "ux_eq", "uy_eq", "uz_eq", "w_flow", "w_T", "T", "Q",
		         "T_buoy", "T_ref", "gflux_x", "gflux_y", "gflux_z", "hflux_x", "hflux_y", "hflux_z", "theta_v", "flags" };
	}
	/// @brief The index of a channel.
	static constexpr unsigned int index(const Cell c) { return static_cast<unsigned int>(c); }
	/// @brief The OpenCL defines of the layout: def_dbg_<name> per cell channel (def_dbg_gflux and def_dbg_hflux for the
	/// first flux component), def_dbg_channels, def_trace_<group>, def_trace_channels, def_trace_steps.
	/// @return the define lines, each starting with a newline, for LBM_Domain::device_defines()
	static std::string opencl_defines() {
		const std::array<std::string, cell_channels> names = cell_names();
		std::string s = "\n	#define DEBUG_PROBE\n	#define def_dbg_channels "+std::to_string(cell_channels)+"u";
		for(unsigned int c=0u; c<cell_channels; c++) s += "\n	#define def_dbg_"+names[c]+" "+std::to_string(c)+"u";
		s += "\n	#define def_dbg_gflux "+std::to_string(index(Cell::gflux_x))+"u";
		s += "\n	#define def_dbg_hflux "+std::to_string(index(Cell::hflux_x))+"u";
		s += "\n	#define def_trace_g_pre "+std::to_string(trace_g_pre)+"u";
		s += "\n	#define def_trace_g_eq "+std::to_string(trace_g_eq)+"u";
		s += "\n	#define def_trace_g_post "+std::to_string(trace_g_post)+"u";
		s += "\n	#define def_trace_h_pre "+std::to_string(trace_h_pre)+"u";
		s += "\n	#define def_trace_h_eq "+std::to_string(trace_h_eq)+"u";
		s += "\n	#define def_trace_h_post "+std::to_string(trace_h_post)+"u";
		s += "\n	#define def_trace_step "+std::to_string(trace_step)+"u";
		s += "\n	#define def_trace_channels "+std::to_string(trace_channels)+"u";
		s += "\n	#define def_trace_steps "+std::to_string(trace_steps)+"u";
		return s;
	}
};
