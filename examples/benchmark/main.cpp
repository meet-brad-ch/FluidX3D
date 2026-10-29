/// @file main.cpp
/// @brief The performance benchmark: a 256^3 box run 1000 times for 10 steps, the peak speed printed in MLUPs/s (the
/// core's own LBM, no Setup API).

#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

/// @brief Runs the benchmark and prints the peak lattice updates per second; on Windows it waits for a key.
void main_setup() { // benchmark; required extensions in defines.hpp: BENCHMARK, optionally FP16S or FP16C
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	constexpr uint box_cells = 256u;      // the default box: 256 x 256 x 256 cells (other boxes below)
	constexpr uint runs = 1000u;          // the runs of the measurement
	constexpr uint steps_per_run = 10u;   // the time steps of each run
	constexpr ulong total_steps = static_cast<ulong>(runs) * static_cast<ulong>(steps_per_run); // for the progress line
	constexpr double per_million = 1E-6;  // cell updates to million cell updates
	uint mlups = 0u;
	{
		//LBM lbm( 32u,  32u,  32u, 1.0f);
		//LBM lbm( 64u,  64u,  64u, 1.0f);
		//LBM lbm(128u, 128u, 128u, 1.0f);
		LBM lbm(box_cells, box_cells, box_cells, 1.0f); // default
		//LBM lbm(384u, 384u, 384u, 1.0f);
		//LBM lbm(512u, 512u, 512u, 1.0f);

		//const uint memory = 1488u; // memory occupation in MB (for multi-GPU benchmarks: make this close to as large as the GPU's VRAM capacity)
		//const uint3 lbm_N = (resolution(float3(1.0f, 1.0f, 1.0f), memory)/4u)*4u; // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
		//LBM lbm(1u*lbm_N.x, 1u*lbm_N.y, 1u*lbm_N.z, 1u, 1u, 1u, 1.0f); // 1 GPU
		//LBM lbm(2u*lbm_N.x, 1u*lbm_N.y, 1u*lbm_N.z, 2u, 1u, 1u, 1.0f); // 2 GPUs
		//LBM lbm(2u*lbm_N.x, 2u*lbm_N.y, 1u*lbm_N.z, 2u, 2u, 1u, 1.0f); // 4 GPUs
		//LBM lbm(2u*lbm_N.x, 2u*lbm_N.y, 2u*lbm_N.z, 2u, 2u, 2u, 1.0f); // 8 GPUs

		// #########################################################################################################################################################################################
		for(uint i = 0u; i < runs; i++) {
			lbm.run(steps_per_run, total_steps);
			const double cells{ static_cast<double>(lbm.get_N()) };
			const double updates_per_second{ cells * per_million / info.runtime_lbm_timestep_smooth };
			mlups = max(mlups, to_uint(updates_per_second));
		}
	} // make lbm object go out of scope to free its memory
	print_info("Peak MLUPs/s = " + to_string(mlups));
#if defined(_WIN32)
	wait();
#endif // Windows
} /**/
