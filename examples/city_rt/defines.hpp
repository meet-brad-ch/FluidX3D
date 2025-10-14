#pragma once

// Velocity set and collision operator
#define D3Q19
#define SRT

// Floating point precision - FP16S gives a good balance of speed and memory
#define FP16S

// --- Enabled Extensions ---
// Enables fixing velocity/density at boundaries (for inflow/outflow).
#define EQUILIBRIUM_BOUNDARIES
// Enables Smagorinsky-Lilly subgrid turbulence model for stability at high Reynolds numbers.
#define SUBGRID
// Enable interactive graphics with a window.
#define INTERACTIVE_GRAPHICS

// --- Disabled Extensions ---
//#define BENCHMARK
//#define VOLUME_FORCE
//#define FORCE_FIELD
//#define MOVING_BOUNDARIES
//#define SURFACE
//#define TEMPERATURE
//#define PARTICLES
//#define INTERACTIVE_GRAPHICS_ASCII
//#define GRAPHICS

// Graphics settings (can be left as default)
#define GRAPHICS_FRAME_WIDTH 1920
#define GRAPHICS_FRAME_HEIGHT 1080
#define GRAPHICS_BACKGROUND_COLOR 0x000000
#define GRAPHICS_U_MAX 0.18f
#define GRAPHICS_RHO_DELTA 0.001f
#define GRAPHICS_T_DELTA 1.0f
#define GRAPHICS_F_MAX 0.001f
#define GRAPHICS_Q_CRITERION 0.0001f
#define GRAPHICS_STREAMLINE_SPARSE 8
#define GRAPHICS_STREAMLINE_LENGTH 128
#define GRAPHICS_RAYTRACING_TRANSMITTANCE 0.25f
#define GRAPHICS_RAYTRACING_COLOR 0x005F7F

// --- Internal Defines (do not change) ---
#define TYPE_S 0b00000001
#define TYPE_E 0b00000010
#define TYPE_T 0b00000100
#define TYPE_F 0b00001000
#define TYPE_I 0b00010000
#define TYPE_G 0b00100000
#define TYPE_X 0b01000000
#define TYPE_Y 0b10000000
#define VIS_FLAG_LATTICE  0b00000001
#define VIS_FLAG_SURFACE  0b00000010
#define VIS_FIELD         0b00000100
#define VIS_STREAMLINES   0b00001000
#define VIS_Q_CRITERION   0b00010000
#define VIS_PHI_RASTERIZE 0b00100000
#define VIS_PHI_RAYTRACE  0b01000000
#define VIS_PARTICLES     0b10000000
#if defined(FP16S) || defined(FP16C)
#define fpxx ushort
#else
#define fpxx float
#endif
#if defined(INTERACTIVE_GRAPHICS) || defined(INTERACTIVE_GRAPHICS_ASCII)
#define GRAPHICS
#define UPDATE_FIELDS
#endif