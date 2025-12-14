#pragma once



//#define D2Q9 // choose D2Q9 velocity set for 2D; allocates 53 (FP32) or 35 (FP16) Bytes/node
//#define D3Q15 // choose D3Q15 velocity set for 3D; allocates 77 (FP32) or 47 (FP16) Bytes/node
#define D3Q19 // choose D3Q19 velocity set for 3D; allocates 93 (FP32) or 55 (FP16) Bytes/node; (default)
//#define D3Q27 // choose D3Q27 velocity set for 3D; allocates 125 (FP32) or 71 (FP16) Bytes/node

#define SRT // choose single-relaxation-time LBM collision operator; (default)
//#define TRT // choose two-relaxation-time LBM collision operator

#define FP16S // compress LBM DDFs to range-shifted IEEE-754 FP16; number conversion is done in hardware; all arithmetic is still done in FP32
//#define FP16C // compress LBM DDFs to more accurate custom FP16C format; number conversion is emulated in software; all arithmetic is still done in FP32

//#define BENCHMARK // disable all extensions and setups and run benchmark setup instead

//#define VOLUME_FORCE // enables global force per volume in one direction, specified in the LBM class constructor; the force can be changed on-the-fly between time steps at no performance cost
//#define FORCE_FIELD // enables a force per volume for each lattice point independently; allocates an extra 12 Bytes/node; enables computing the forces from the fluid on solid boundaries with lbm.calculate_force_on_boundaries();
//#define MOVING_BOUNDARIES // enables moving solids: set solid nodes to TYPE_S and set their velocity u unequal to zero
#define EQUILIBRIUM_BOUNDARIES // enables fixing the velocity/density by marking nodes with TYPE_E; can be used for inflow/outflow; does not reflect shock waves
//#define SURFACE // enables free surface LBM: mark fluid nodes with TYPE_F; at initialization the TYPE_I interface and TYPE_G gas domains will automatically be completed; allocates an extra 12 Bytes/node
//#define TEMPERATURE // enables temperature extension; set fixed-temperature nodes with TYPE_T (similar to EQUILIBRIUM_BOUNDARIES); allocates an extra 32 (FP32) or 18 (FP16) Bytes/node
#define SUBGRID // enables Smagorinsky-Lilly subgrid turbulence model to keep simulations with very large Reynolds number stable

#define INTERACTIVE_GRAPHICS // enable interactive graphics; start/pause the simulation by pressing P
//#define INTERACTIVE_GRAPHICS_ASCII // enable interactive graphics in ASCII mode in the console
//#define GRAPHICS // run FluidX3D in the console, but still enable graphics functionality for writing rendered frames to the hard drive

#define GRAPHICS_FRAME_WIDTH 1920 // set frame width if only GRAPHICS is enabled
#define GRAPHICS_FRAME_HEIGHT 1080 // set frame height if only GRAPHICS is enabled
#define GRAPHICS_BACKGROUND_COLOR 0x000000 // set background color; black background (default) = 0x000000, white background = 0xFFFFFF
#define GRAPHICS_U_MAX 0.15f // maximum velocity for velocity coloring in units of LBM lattice speed of sound (c=1/sqrt(3)) (default: 0.15f)
#define GRAPHICS_RHO_DELTA 0.001f // coloring range for density rho will be [1.0f-GRAPHICS_RHO_DELTA, 1.0f+GRAPHICS_RHO_DELTA] (default: 0.001f)
#define GRAPHICS_T_DELTA 1.0f // coloring range for temperature T will be [1.0f-GRAPHICS_T_DELTA, 1.0f+GRAPHICS_T_DELTA] (default: 1.0f)
#define GRAPHICS_F_MAX 0.001f // maximum force in LBM units for visualization of forces on solid boundaries if VOLUME_FORCE is enabled and lbm.update_force_field(); is called (default: 0.001f)
#define GRAPHICS_Q_CRITERION 0.0001f // Q-criterion value for Q-criterion isosurface visualization (default: 0.0001f)
#define GRAPHICS_STREAMLINE_SPARSE 4 // set how many streamlines there are every x lattice points
#define GRAPHICS_STREAMLINE_LENGTH 128 // set maximum length of streamlines
#define GRAPHICS_RAYTRACING_TRANSMITTANCE 0.25f // transmitted light fraction in raytracing graphics ("0.25f" = 1/4 of light is transmitted and 3/4 is absorbed along longest box side length, "1.0f" = no absorption)
#define GRAPHICS_RAYTRACING_COLOR 0x005F7F // absorption color of fluid in raytracing graphics



// #############################################################################################################

#define VIS_FLAG_LATTICE  0b00000001 // lbm.graphics.visualization_modes = VIS_...|VIS_...|VIS_...;
#define VIS_FLAG_SURFACE  0b00000010
#define VIS_FIELD         0b00000100
#define VIS_STREAMLINES   0b00001000
#define VIS_Q_CRITERION   0b00010000
#define VIS_PHI_RASTERIZE 0b00100000
#define VIS_PHI_RAYTRACE  0b01000000
#define VIS_PARTICLES     0b10000000

#define TYPE_S 0b00000001 // (stationary or moving) solid boundary
#define TYPE_E 0b00000010 // equilibrium boundary (inflow/outflow)
#define TYPE_T 0b00000100 // temperature boundary
#define TYPE_F 0b00001000 // fluid
#define TYPE_I 0b00010000 // interface
#define TYPE_G 0b00100000 // gas
#define TYPE_X 0b01000000 // reserved type X
#define TYPE_Y 0b10000000 // reserved type Y

#if defined(FP16S) || defined(FP16C)
#define fpxx ushort
#else // FP32
#define fpxx float
#endif // FP32

#ifdef BENCHMARK
#undef UPDATE_FIELDS
#undef VOLUME_FORCE
#undef FORCE_FIELD
#undef MOVING_BOUNDARIES
#undef EQUILIBRIUM_BOUNDARIES
#undef SURFACE
#undef TEMPERATURE
#undef SUBGRID
#undef INTERACTIVE_GRAPHICS
#undef INTERACTIVE_GRAPHICS_ASCII
#undef GRAPHICS
#endif // BENCHMARK

#ifdef FORCE_FIELD
#define VOLUME_FORCE
#endif // FORCE_FIELD

#ifdef SURFACE // (rho, u) need to be updated exactly every LBM step
#define UPDATE_FIELDS // update (rho, u, T) in every LBM step
#endif // SURFACE

#ifdef TEMPERATURE
#define VOLUME_FORCE
#endif // TEMPERATURE

#ifdef INTERACTIVE_GRAPHICS
#define GRAPHICS
#endif // INTERACTIVE_GRAPHICS
#ifdef INTERACTIVE_GRAPHICS_ASCII
#define GRAPHICS
#endif // INTERACTIVE_GRAPHICS_ASCII
