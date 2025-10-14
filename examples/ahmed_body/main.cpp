//================================================================================
// FluidX3D Ahmed Body Example - Extensively Commented
//================================================================================
// This example simulates airflow over an Ahmed body, a standardized bluff body
// used for aerodynamic testing and CFD validation. The primary goal is to
// calculate the drag coefficient (Cd).
//
// Required extensions in defines.hpp (or passed via CMake):
// - FP16C: Uses a mix of 16-bit and 32-bit floating-point numbers for a good
//   balance of performance and precision.
// - FORCE_FIELD: Enables the calculation of forces on solid boundaries. This is
//   essential for measuring lift and drag.
// - EQUILIBRIUM_BOUNDARIES: Allows for stable inflow/outflow boundary conditions
//   to create the wind tunnel effect.
// - SUBGRID: Enables a turbulence model (Smagorinsky-Lilly) to keep the simulation
//   stable at higher Reynolds numbers, which is typical for vehicle aerodynamics.
// - INTERACTIVE_GRAPHICS (optional): To view the simulation in real-time.
//================================================================================

#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() {
	//================================================================================
	// 1. DEFINE SIMULATION PARAMETERS (SI and LBM Units)
	//================================================================================
	// Here, we define all the physical and numerical parameters for the simulation.
	// It's a common practice to define real-world (SI) parameters and then
	// convert them to the dimensionless Lattice Boltzmann (LBM) unit system.

	// --- LBM & Grid Parameters ---

	// Target GPU VRAM usage in Megabytes. The resolution() function will
	// calculate the largest possible grid that fits into this memory budget.
	const uint memory = 10000u;

	// Characteristic velocity in LBM units. This is a crucial parameter for stability.
	// It must be significantly less than the lattice speed of sound (c=1/√3 ≈ 0.577).
	// A value between 0.01 and 0.1 is typical and safe.
	const float lbm_u = 0.05f;

	// A scaling factor for the simulation domain size relative to the object's size.
	// A larger domain reduces the influence of boundaries on the flow around the body.
	const float box_scale = 6.0f;

	// --- SI (Real-World) Parameters ---

	// The desired real-world wind speed in meters per second (m/s).
	const float si_u = 60.0f;

	// Kinematic viscosity of air at sea level in m²/s.
	const float si_nu=1.48E-5f;

	// Density of air at sea level in kg/m³.
	const float si_rho=1.225f;

	// Ahmed body geometry dimensions in SI units (meters).
	const float si_width=0.389f, si_height=0.288f, si_length=1.044f;

	// Frontal area (A) of the Ahmed body for drag coefficient calculation.
	// This includes the main body and the small stilts it stands on.
	const float si_A = si_width*si_height+2.0f*0.05f*0.03f;

	// Total real-world time to simulate in seconds.
	const float si_T = 0.25f;

	// Calculate the total simulation box dimensions in SI units based on the Ahmed body size and the box_scale factor.
	const float si_Lx = units.x(box_scale*si_width);
	const float si_Ly = units.x(box_scale*si_length);
	const float si_Lz = units.x(0.5f*(box_scale-1.0f)*si_width+si_height);

	//================================================================================
	// 2. GRID RESOLUTION AND UNIT CONVERSION
	//================================================================================
	// This section determines the grid size and sets up the conversion between LBM and SI units.

	// Automatically calculate the grid resolution (lbm_N) that fits the specified 'memory'
	// while maintaining the aspect ratio of the SI-unit simulation box.
	const uint3 lbm_N = resolution(float3(si_Lx, si_Ly, si_Lz), memory);

	// This is the most critical step for unit conversion. It establishes the scaling factors
	// between LBM and SI units by pairing known quantities in both systems. We are telling
	// the `units` object that a length of `lbm_N.y` cells in the simulation corresponds
	// to `box_scale * si_length` meters in the real world, and a velocity of `lbm_u` in the
	// simulation corresponds to `si_u` m/s in the real world.
	units.set_m_kg_s((float)lbm_N.y, lbm_u, 1.0f, box_scale*si_length, si_u, si_rho);

	// Now that the conversion factors are set, we can convert other SI parameters to LBM units.
	// Convert kinematic viscosity from SI (m²/s) to LBM units.
	const float lbm_nu = units.nu(si_nu);

	// Convert total simulation time from SI (seconds) to LBM timesteps.
	const ulong lbm_T = units.t(si_T);

	// Convert the Ahmed body's characteristic length from SI (meters) to LBM units (cells).
	const float lbm_length = units.x(si_length);

	// Calculate and print the Reynolds number (Re), a key dimensionless number in fluid dynamics
	// that characterizes the flow regime (laminar vs. turbulent).
	print_info("Re = "+to_string(to_uint(units.si_Re(si_width, si_u, si_nu))));

	// Instantiate the main LBM simulation object with the calculated grid resolution and LBM viscosity.
	LBM lbm(lbm_N, lbm_nu);

	//================================================================================
	// 3. GEOMETRY DEFINITION (AHMED BODY)
	//================================================================================

	// Get the full path to the Ahmed body's STL file from the `resources/` directory.
	const string stl_path = get_resource_path("ahmed_25deg_m.stl");

	// Check if the STL file exists. If not, provide the user with instructions on how to obtain it.
	if(stl_path.empty()) {
		print_info("This example requires the Ahmed body geometry file.");
		print_info("Steps:");
		print_info("  1. Download from https://github.com/nathanrooy/ahmed-bluff-body-cfd/blob/master/geometry/ahmed_25deg_m.stl");
		print_info("  2. Convert from ASCII to binary STL format (if needed)");
		print_info("  3. Save as ahmed_25deg_m.stl in resources/");
		wait(); // Wait for user to press Enter before exiting.
		return;
	}

	// Read the STL file into a `Mesh` object. This involves several transformations:
	// - The mesh is initially scaled so its length matches `lbm_length`.
	// - It's rotated 90 degrees around the Z-axis to align with the flow direction (Y-axis).
	// - It's centered within the simulation box's cross-section.
	Mesh* mesh = read_stl(stl_path, lbm.size(), lbm.center(), float3x3(float3(0, 0, 1), radians(90.0f)), lbm_length);

	// Translate the mesh to its final position in the wind tunnel.
	// It's moved slightly forward and placed on the ground plane (z=1).
	mesh->translate(float3(0.0f, units.x(0.5f*(0.5f*box_scale*si_length-si_width))-mesh->pmin.y, 1.0f-mesh->pmin.z));

	// Voxelize the mesh on the GPU. This converts the continuous triangle mesh into a
	// discrete set of grid cells marked as solid.
	// We use a unique flag combination `TYPE_S | TYPE_X`. This allows us to specifically
	// calculate forces on the Ahmed body, distinguishing it from other solid boundaries
	// like the ground plane, which is only marked with `TYPE_S`.
	lbm.voxelize_mesh_on_device(mesh, TYPE_S|TYPE_X);

	//================================================================================
	// 4. SET BOUNDARY AND INITIAL CONDITIONS
	//================================================================================
	// This loop runs in parallel on the CPU to efficiently set the initial state
	// of every cell in the simulation grid.

	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz();
	parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u;
		// Convert 1D index 'n' to 3D coordinates (x, y, z).
		lbm.coordinates(n, x, y, z);

		// Set the ground plane (z=0) as a solid, no-slip boundary.
		if(z==0u) lbm.flags[n] = TYPE_S;

		// Initialize the entire flow field (except for already defined solid cells)
		// with the wind speed. The flow is directed along the positive Y-axis.
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;

		// Define the wind tunnel boundaries. All outer boundaries are set as
		// equilibrium boundaries (`TYPE_E`), which act as stable inflow/outflow
		// conditions and prevent pressure waves from reflecting back into the domain.
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==Nz-1u) lbm.flags[n] = TYPE_E;
	});

	//================================================================================
	// 5. RUN SIMULATION AND EXPORT DATA
	//================================================================================

	// Configure the visualization modes for interactive graphics.
	// VIS_FLAG_SURFACE will render the solid body using marching cubes for a smooth appearance.
	// VIS_FIELD will show the flow field data (e.g., velocity or pressure).
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_FIELD;
	// Set the field mode to 1, which corresponds to visualizing density/pressure.
	lbm.graphics.field_mode = 1;
	// Set the slice mode to 1, which displays a slice of the field data along the X-plane.
	lbm.graphics.slice_mode = 1;

	// This commented-out line shows how you could set a specific, fixed camera position
	// for non-interactive rendering. You can get these values by running in interactive
	// mode, positioning the camera, and pressing 'G'.
	//lbm.graphics.set_camera_centered(20.0f, 30.0f, 10.0f, 1.648722f);

	// Initialize the simulation on the GPU. This transfers all the data (rho, u, flags)
	// from CPU to GPU memory and prepares the simulation to run. `lbm.run(0u, ...)`
	// performs only initialization without advancing the time step.
	lbm.run(0u, lbm_T);

	// Set up a path for saving output files, organized by floating-point precision and memory usage.
	#if defined(FP16S)
		const string path = get_exe_path()+"FP16S/"+to_string(memory)+"MB/";
	#elif defined(FP16C)
		const string path = get_exe_path()+"FP16C/"+to_string(memory)+"MB/";
	#else // FP32
		const string path = get_exe_path()+"FP32/"+to_string(memory)+"MB/";
	#endif // FP32

	// This commented-out line would write a full status report of the simulation
	// configuration (grid size, memory, parameters, etc.) to a text file.
	//lbm.write_status(path);

	// This commented-out line would create a new data file and write the header for
	// logging the drag coefficient (Cd) at each time step.
	//write_file(path+"Cd.dat", "# t\tCd\n");

	// Pre-calculate the center of mass, needed for torque calculations.
	const float3 lbm_com = lbm.object_center_of_mass(TYPE_S|TYPE_X);
	print_info("com = "+to_string(lbm_com.x, 2u)+", "+to_string(lbm_com.y, 2u)+", "+to_string(lbm_com.z, 2u));

	// This is the main simulation loop. It will continue until the total simulated time `lbm_T` is reached.
	while(lbm.get_t()<=lbm_T) {
		// Start a clock to time how long the force calculation and data output takes per step.
		Clock clock;

		// Calculate the total force exerted by the fluid on the Ahmed body.
		// This is a highly efficient operation that performs a reduction (summation) on the GPU.
		// It sums the forces on all cells marked with the unique `TYPE_S | TYPE_X` flag.
		const float3 lbm_force = lbm.object_force(TYPE_S|TYPE_X);

		// This commented-out line would calculate the torque on the Ahmed body around its center of mass.
		//const float3 lbm_torque = lbm.object_torque(lbm_com, TYPE_S|TYPE_X);

		// This commented-out line would print the full force and torque vectors to the console.
		//print_info("F="+to_string(lbm_force.x, 2u)+","+to_string(lbm_force.y, 2u)+","+to_string(lbm_force.z, 2u)+", T="+to_string(lbm_torque.x, 2u)+","+to_string(lbm_torque.y, 2u)+","+to_string(lbm_torque.z, 2u)+", t="+to_string(clock.stop(), 3u));

		// Calculate the drag coefficient (Cd).
		// 1. `lbm_force.y` is the force component in the direction of the flow (drag).
		// 2. `units.si_F()` converts this LBM force into SI units (Newtons).
		// 3. The result is divided by the dynamic pressure (0.5 * rho * u^2) and the frontal area (A).
		// Note in code: The result is expected to be higher than experimental values because
		// the simulation resolution might not be sufficient to fully resolve the turbulent
		// boundary layer. A wall model would be needed for higher accuracy.
		const float Cd = units.si_F(lbm_force.y)/(0.5f*si_rho*sq(si_u)*si_A);

		// Print the calculated drag coefficient and the time taken for the calculation.
		print_info("Cd = "+to_string(Cd, 3u)+", t = "+to_string(clock.stop(), 3u));

		// This commented-out line would append the current time step and the calculated Cd
		// to the data file, creating a time series of the drag coefficient.
		//write_line(path+"Cd.dat", to_string(lbm.get_t())+"\t"+to_string(Cd, 3u)+"\n");

		// Advance the simulation by one time step.
		lbm.run(1u, lbm_T);
	}

	// This commented-out line would write a final status report after the simulation has finished.
	//lbm.write_status(path);
}