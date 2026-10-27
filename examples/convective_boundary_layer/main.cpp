// Convective boundary layer: a boundary layer heated from the floor, with an optional heated patch that roots a
// thermal. The example records the flow in a box as frames (format 2: 8-bit w, u, v and a cloud bit per cell) and
// writes the horizontal-mean statistics. The default case: a dry, windless mixed layer to zi = 750 m under a 1 K
// inversion and 5 K/km above, floor flux 0.262 K m/s, a periodic domain of 1280 m (1.7 zi), a patch 160 m across
// heated 4x, the box 640 m (0.85 zi). The other cases (for example sp2011, the dry CBL of Sullivan and Patton
// 2011): run the example without arguments to see them.
//
// Lattice: Boussinesq through TEMPERATURE with T = 1 + theta' / 1 K (theta' the potential temperature above the mixed
// layer's start). The buoyancy is fz * beta * (T - 1) with a tiny fz, so the constant part of the volume force
// (fz itself) leaves the density almost uniform: beta = g dt^2/dx / (theta_0 fz). The floor (z = 0) and the lid are
// TYPE_S (adiabatic); the floor flux is a heat source in the first fluid layer (TYPE_Y, LBM::set_heat_rate()), the
// patch's 4x four such layers. SUBGRID with the eddy heat diffusivity (Pr_t = 1/3). FP32 DDFs (FP16 storage loses
// 1.4-2.2 % of a source this small against the near-floor temperature, tests/physics/heat_source.cpp).
//
//     convective_boundary_layer <output folder> [quick | <case> ...]
//
// Writes into the output folder: header.json and frames.bin (format 2: per frame w, u, v as uint8 over the box, k up,
// j north, i east, u and v less each level's mean over the domain, then the ql bits, all 0 in a dry run), w_f16.bin (w of the
// same box as IEEE half floats, without the 8-bit steps), profiles.csv (horizontal means every frame interval, also
// in the spin-up) and progress lines on stdout. "quick" runs a 128 x 128 x 100 column for 100 s, to check the outputs.
//
// Moist air (MOISTURE builds, the sounding case with --moist): the sounding's third column qt_kgkg gives the start
// total water, the floor's moisture flux follows the heat flux cell by cell (flux * cp / (Lv * Bowen), Bowen 0.5, as in
// the MicroHH reference runs), the buoyancy uses theta_v after the saturation adjustment at each level's hydrostatic pressure
// (from 101325 Pa), and every interval the host's saturation adjustment gives each cell's liquid water: the profile
// columns qt, ql and the cloud fraction, a CLOUD line (the cloud base and top, the mixed layer's condensation level),
// the frames' ql bits (1 = liquid water) and at the end the cloud base against the condensation level.

#include "setup/setup.hpp"
#include "moist_air.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

namespace {

/// The run's physical and lattice settings.
struct CblSettings {
	uint Nx = 512u, Ny = 512u, Nz = 400u; // cells; z = 0 floor and z = Nz-1 lid are solid
	uint Dx = 2u;                         // domains along x: the FP32 DDFs of 105 M cells are over one device buffer
	uint box = 256u;                      // the recorded box's cells east (and north unless box_north is set), centred on the patch
	uint box_north = 0u;                  // the recorded box's cells north (0: box); a longer box downwind when there is wind
	float box_first_north = 0.0f;         // m, the centre of the box's first (southernmost) cell north of the patch's centre (0: the box is centred)
	float patch_north = 0.0f;             // m, the patch's centre north of the domain's middle (with wind the patch sits upwind)
	float dx = 2.5f;                      // m
	float dt = 0.04f;                     // s; 8 m/s = 0.128 in lattice units
	float theta_0 = 300.0f;               // K, the mixed layer's potential temperature
	float zi = 750.0f;                    // m, the inversion's height at the start
	float jump = 1.0f;                    // K, the inversion
	float jump_depth = 0.0f;              // m, the inversion spread over this depth (0: a step)
	float lapse = 0.005f;                 // K/m above it
	float flux = 0.262f;                  // K m/s, the floor's kinematic heat flux
	float patch = 160.0f;                 // m, the heated disc's diameter
	uint patch_layers = 4u;               // the patch's flux in floor-flux units, as heated layers; 0: no patch (a homogeneous layer)
	bool patch_surface = false;           // the patch's 4x flux in the surface layer itself (TYPE_X|TYPE_Y, the kernel's rate) instead of 4 heated layers
	std::vector<float> profile_z, profile_theta; // a tabulated start profile (m, K) instead of zi/jump/lapse: theta - theta_0 interpolated in height
	bool frames = true;                   // write frames.bin and w_f16.bin (the recording); off for a statistics run
	bool w_plane = false;                 // write the w plane at 0.5 zi (float32, Nx x Ny) every interval while recording: w_plane.bin
	unsigned seed = 2u;                   // the start noise's seed
	float noise = 0.1f;                   // K, random start perturbation below noise_top
	float noise_top = 100.0f;             // m
	float spinup = 1200.0f;               // s without frames
	float record = 1800.0f;               // s of frames
	float interval = 20.0f;               // s between frames (and profile rows)
	float nu = 1.0E-5f;                   // lattice; the Smagorinsky model carries the turbulence
	float alpha = 2.0E-5f;                // lattice; tauT 0.50004 without THERMAL_DIFFUSIVITY_FIX: the unstable regime, see sullivan_patton()
	float tracer = 1.0f;                  // the passive tracer's uniform start value of Q (MOISTURE builds, --tracer)
	bool moist = false;                   // MOISTURE: total water from the sounding, the floor's moisture flux, the moist buoyancy and the clouds (--moist)
	std::vector<float> profile_qt;        // kg/kg, the tabulated sounding's total water (its third column qt_kgkg, when present)
	float bowen = 0.5f;                   // the Bowen ratio: the moisture flux is flux * cp / (Lv * bowen) (as in the MicroHH reference runs)
	float cloud_fraction_min = 1.0E-3f;   // a level is cloudy when at least this share of its cells holds liquid water (the cloud base and top)
	float inv_prandtl_t = 3.0f;           // Pr_t = 1/3
	float sponge_from = 0.0f;             // m; above it w is damped and T relaxed to the start profile, the rate rising as the square of the height into the sponge to 1/sponge_tau at the lid (0: no sponge)
	float sponge_tau = 450.0f;            // s, the sponge's time scale at the lid (the design appendix: Rayleigh damping, tau 450 s)
	float wind_speed = 0.0f;              // m/s at wind_height (0: no wind); the profile u(h) = wind_speed (h / wind_height)^wind_exponent, held by nudging
	float wind_height = 100.0f;           // m
	float wind_exponent = 0.25f;          // the power law's exponent
	float wind_from_deg = 0.0f;           // the direction the wind comes from, degrees clockwise from north (0: from the north, the wind blows toward -y)
	float nudge_tau = 600.0f;             // s, the nudging time scale (MicroHH's --nudge 600)
	float fz = -1.0E-6f;                  // lattice; the buoyancy's carrier, see the file's comment
	float w_lo = -10.105561f, w_hi = 15.158342f, uv_lo = -10.105561f, uv_hi = 10.105561f; // the quantization: w -4 to 6 w*, u and v +-4 w*, for w* = 2.53 m/s

	float speed() const { return dx/dt; }                               // m/s per lattice speed
	float beta() const { return 9.81f*dt*dt/dx/(theta_0*-fz); }         // per K
	float heat_rate() const { return flux*dt/dx; }                      // K per step and source layer
	float moisture_flux() const { return flux*(float)moist_air::Cp/((float)moist_air::Lv*bowen); } // kg/kg m/s
	float moisture_rate() const { return moisture_flux()*dt/dx/0.01f; } // Q per step and source layer (Q = 1 + qt / 0.01)
	ulong steps(float seconds) const { return (ulong)std::llround(seconds/dt); }

	/// The homogeneous convective boundary layer of Sullivan and Patton 2011 (J. Atmos. Sci. 68): floor flux 0.24 K m/s,
	/// theta 300 K to 974 m, +8 K over the next 100 m, 3 K/km above; here the lid at 2048 m (about 2 zi) and the domain
	/// cells_wide cells square at the given cell size; no patch, no frames, the statistics and the w plane.
	/// The molecular diffusivity of T and Q in lattice units. The D3Q7 lattice needs 0.014 (tauT 0.556): below it the
	/// lattice creates heat and negative temperatures next to TYPE_S walls in a buoyant flow (tests/physics/
	/// heat_conservation.cpp). 0.014 is 8.75 m^2/s at 10 m cells and 0.16 s, and it smears the capping inversion: the
	/// inversion's theta'^2 peak 3.6 against MicroHH's 107, zi 802 against 1001 m. SCALAR_FV has no such floor: at 1e-4
	/// (0.06 m^2/s) zi is 979 m, the peak 47, and the profiles over 0.1-0.8 zi agree with MicroHH within 7 %.
	static float default_alpha() {
#ifdef SCALAR_FV
		return 1.0E-4f;
#else // SCALAR_FV
		return 0.014f;
#endif // SCALAR_FV
	}

	static CblSettings sullivan_patton(float cell, uint cells_wide) {
		CblSettings s;
		s.dx = cell; s.dt = 0.04f*cell/2.5f; // the lattice speed of a 2.5 m run with dt 0.04 s
		s.Nx = cells_wide; s.Ny = cells_wide; s.Nz = (uint)std::lround(2048.0f/cell)+2u; s.Dx = 1u; s.box = cells_wide;
		s.theta_0 = 300.0f; s.zi = 974.0f; s.jump = 8.0f; s.jump_depth = 100.0f; s.lapse = 0.003f; s.flux = 0.24f;
		s.patch_layers = 0u; s.frames = false; s.w_plane = true;
		s.noise_top = 300.0f; s.spinup = 1200.0f; s.record = 1800.0f; s.interval = 20.0f;
		s.alpha = default_alpha();
		return s;
	}

	/// A CBL on a tabulated sounding: the start profile from a CSV (z_m, thl_K and optionally qt_kgkg), the surface flux
	/// 0.262 K m/s, zi (1864 m, the dry reference case's; --zi gives another one), the patch 400 m at 4x in the surface layer,
	/// the recorded box 1600 m, 900 s spin-up and 1800 s of frames; cells_wide cells square.
	static CblSettings tabulated_sounding(float cell, uint cells_wide, const std::filesystem::path& profile, float zi) {
		CblSettings s;
		s.dx = cell; s.dt = 0.04f*cell/2.5f;
		s.zi = zi; s.flux = 0.2618979f;
		s.Nx = cells_wide; s.Ny = cells_wide; s.Nz = (uint)std::lround(2.0f*s.zi/cell)+2u; s.Dx = 1u; s.box = (uint)std::lround(1600.0f/cell);
		s.patch = 400.0f; s.patch_layers = 4u; s.patch_surface = true; s.frames = true; s.w_plane = true;
		s.noise_top = 300.0f; s.spinup = 900.0f; s.record = 1800.0f; s.interval = 20.0f;
		s.alpha = default_alpha();
		s.sponge_from = 1.6f*s.zi; // the lid rule: the lid at 2 zi, the sponge from 1.6 zi
		std::ifstream in(profile);
		if(!in) print_error("profile not found: "+profile.string());
		std::string line; std::getline(in, line); // the header z_m,thl_K[,qt_kgkg]
		const bool with_qt = line.find("qt_kgkg")!=std::string::npos;
		while(std::getline(in, line)) {
			const size_t c = line.find(',');
			if(c==std::string::npos) continue;
			s.profile_z.push_back(std::stof(line.substr(0u, c)));
			s.profile_theta.push_back(std::stof(line.substr(c+1u)));
			if(with_qt) {
				const size_t c2 = line.find(',', c+1u);
				if(c2==std::string::npos) print_error("profile row without its qt_kgkg value: "+line);
				s.profile_qt.push_back(std::stof(line.substr(c2+1u)));
			}
		}
		if(s.profile_z.size()<2u) print_error("profile holds fewer than 2 levels: "+profile.string());
		{ // theta_0 = the mixed layer's value: the median of the profile between 0.2 and 0.8 zi
			std::vector<float> mixed;
			for(size_t i=0u; i<s.profile_z.size(); i++) if(s.profile_z[i]>0.2f*s.zi&&s.profile_z[i]<0.8f*s.zi) mixed.push_back(s.profile_theta[i]);
			if(mixed.empty()) print_error("profile has no level between 0.2 and 0.8 zi");
			std::sort(mixed.begin(), mixed.end()); s.theta_0 = mixed[mixed.size()/2u];
		}
		return s;
	}

	/// theta - theta_0 at height h from the tabulated profile (linear, held constant outside its range).
	float profile_at(float h) const {
		if(h<=profile_z.front()) return profile_theta.front()-theta_0;
		for(size_t i=1u; i<profile_z.size(); i++) if(h<=profile_z[i]) { const float f = (h-profile_z[i-1u])/(profile_z[i]-profile_z[i-1u]); return profile_theta[i-1u]+f*(profile_theta[i]-profile_theta[i-1u])-theta_0; }
		return profile_theta.back()-theta_0;
	}
	/// The tabulated sounding's qt (kg/kg) at height h (linear, held constant outside its range).
	float qt_at(float h) const {
		if(h<=profile_z.front()) return profile_qt.front();
		for(size_t i=1u; i<profile_z.size(); i++) if(h<=profile_z[i]) { const float f = (h-profile_z[i-1u])/(profile_z[i]-profile_z[i-1u]); return profile_qt[i-1u]+f*(profile_qt[i]-profile_qt[i-1u]); }
		return profile_qt.back();
	}

	static CblSettings quick() {
		CblSettings s;
		s.Nx = 128u; s.Ny = 128u; s.Nz = 100u; s.Dx = 1u; s.box = 64u;
		s.zi = 180.0f; s.patch = 40.0f; s.noise_top = 30.0f; s.spinup = 60.0f; s.record = 40.0f; s.interval = 10.0f;
		s.alpha = default_alpha();
		return s;
	}
};

/// The horizontal means of one layer, in SI units.
struct LayerMeans {
	double theta = 0.0;  ///< K above theta_0
	double w_var = 0.0;  ///< m^2/s^2
	double w_skew = 0.0; ///< <w'^3> / <w'^2>^(3/2)
	double theta_var = 0.0; ///< K^2
	double w_theta = 0.0;///< K m/s, resolved flux
	double u = 0.0, v = 0.0; ///< m/s
	double w_max = 0.0, w_min = 0.0; ///< m/s
};

/// The moist air of every layer (index z, moist runs): the horizontal means of qt and ql and the share of wet cells.
struct CloudLevels {
	std::vector<double> qt;       ///< kg/kg
	std::vector<double> ql;       ///< kg/kg
	std::vector<double> fraction; ///< the share of the layer's cells holding liquid water
};

/// The convective boundary layer run: grid set-up, spin-up, recording.
class CblRun {
public:
	CblRun(const CblSettings& s, const std::filesystem::path& folder)
		: s_(s), folder_(folder),
		  lbm_(s.Nx, s.Ny, s.Nz, s.Dx, 1u, 1u, s.nu, 0.0f, 0.0f, s.fz, 0.0f, s.alpha, s.beta()) {}

	void run() {
		std::filesystem::create_directories(folder_);
		set_up();
		profiles_.open(folder_/"profiles.csv");
		profiles_ << "time_s,z_m,theta_K,w_var_m2ps2,w_theta_Kmps,u_mps,v_mps,w_max_mps,w_min_mps,theta_var_K2,w_skew" << (s_.moist ? ",qt_kgkg,ql_kgkg,cloud_fraction" : "") << "\n";
		if(s_.frames) { frames_.open(folder_/"frames.bin", std::ios::binary); w16_.open(folder_/"w_f16.bin", std::ios::binary); }
		if(s_.w_plane) w_plane_.open(folder_/"w_plane.bin", std::ios::binary);
		lbm_.run(0u);
		const ulong every = s_.steps(s_.interval), spin = s_.steps(s_.spinup), total = spin+s_.steps(s_.record);
		const Clock clock;
		while(lbm_.get_t()<total) {
			lbm_.run(every);
			const float time = (float)lbm_.get_t()*s_.dt;
			const bool recording = lbm_.get_t()>spin;
			read_fields();
			const std::vector<LayerMeans> means = layer_means();
			const float zi = inversion_height(means);
#ifdef MOISTURE
			if(s_.moist) clouds_ = cloud_levels(); // before the profiles and the frame, which write it
#endif // MOISTURE
			write_profiles(time, means);
			if(s_.wind_speed>0.0f) nudge_wind(means, zi);
			if(recording) { if(s_.frames) write_frame(means); else frames_written_++; if(s_.w_plane) write_w_plane(zi); zi_sum_ += mixed_layer_top(means); }
			double w_max = 0.0, w_min = 0.0;
			for(const LayerMeans& m : means) { w_max = std::max(w_max, m.w_max); w_min = std::min(w_min, m.w_min); }
			const double wall = clock.stop();
			std::printf("PROGRESS t %.0f s (%llu/%llu steps, %.1f %%) zi %.0f m w %.2f..%.2f m/s frames %u wall %.0f s, %.0f MLUPs",
			            time, (unsigned long long)lbm_.get_t(), (unsigned long long)total, 100.0*(double)lbm_.get_t()/(double)total, zi, w_min, w_max, frames_written_, wall,
			            (double)lbm_.get_N()*(double)lbm_.get_t()/wall*1.0E-6);
			if(s_.wind_speed>0.0f) std::printf(", wind off the power law %.1f %% of %.2f m/s", 100.0*wind_error_, s_.wind_speed);
			std::puts("");
#ifdef MOISTURE
			if(s_.moist) report_cloud(time, means, clouds_, recording); else report_tracer(time);
#endif // MOISTURE
			std::fflush(stdout);
			if(!std::isfinite(w_max)||!std::isfinite(w_min)||w_max>40.0||w_min<-40.0) {
				std::printf("FAILED: the flow blew up at t %.0f s (w %.2f..%.2f m/s)\n", time, w_min, w_max);
				std::fflush(stdout);
				std::exit(1);
			}
			if(recording&&s_.frames) write_header(zi_sum_/(double)frames_written_);
		}
		if(s_.wind_speed>0.0f) std::printf("WIND the largest layer-mean deviation below zi from the power-law target at the end: %.2f %% of %.2f m/s (information, not a check: with tau %.0f s the nudged wind does not reach the power law; compare it with an LES reference profile)\n", 100.0*wind_error_, s_.wind_speed, s_.nudge_tau);
#ifdef MOISTURE
		if(s_.moist) report_cloud_check();
#endif // MOISTURE
		std::printf("RESULT recorded %u frames into %s\n", frames_written_, folder_.string().c_str());
		std::fflush(stdout);
	}

private:
	CblSettings s_;
	std::filesystem::path folder_;
	LBM lbm_;
	std::ofstream profiles_, frames_, w16_, w_plane_;
	uint frames_written_ = 0u;
	std::vector<double> mean_u_sum_, mean_v_sum_; // per box level the sum over the written frames of the domain's mean u and v (m/s), for the header's mean_wind
	double zi_sum_ = 0.0; // over the written frames the mixed layer's top (mixed_layer_top()), for the header's zi
	ulong clipped_ = 0ull;
	std::vector<float> rate_, T_ref_; // the sponge's per-level rate and reference temperature, set once in set_up()
	double wind_error_ = 0.0;        // the largest |<u> - u_target| below zi over the target's speed at wind_height, at the last interval (information: the nudging does not reach the power-law target)
	std::vector<double> pressure_;   // Pa per level, hydrostatic from 101325 Pa through the start sounding (moist runs)
	CloudLevels clouds_;             // the last interval's moist air per level (moist runs)
	double cloud_base_sum_ = 0.0, lcl_sum_ = 0.0; // over the recorded intervals with a cloud: the cloud base and the condensation level (m)
	uint cloudy_intervals_ = 0u, recorded_intervals_ = 0u;

	ulong index(uint x, uint y, uint z) const { return (ulong)x+((ulong)y+(ulong)z*(ulong)s_.Ny)*(ulong)s_.Nx; }
	float height(uint z) const { return ((float)z-0.5f)*s_.dx; } // the centre of cell z above the floor's top (z = 1 is the first fluid layer)
	/// The wind's target profile (m/s): a power law of the height, blowing away from wind_from_deg (0: from the north toward -y, 90: from the east toward -x).
	float wind_target(float h) const { return h<=0.0f ? 0.0f : s_.wind_speed*std::pow(h/s_.wind_height, s_.wind_exponent); }
	float wind_target_x(float h) const { return -wind_target(h)*std::sin(s_.wind_from_deg*3.14159265f/180.0f); }
	float wind_target_y(float h) const { return -wind_target(h)*std::cos(s_.wind_from_deg*3.14159265f/180.0f); }
	/// The nudging: every interval the per-level force is (u_target - <u>) / tau per step, from the layer means just measured; the sponge stays.
	void nudge_wind(const std::vector<LayerMeans>& means, float zi) {
		std::vector<float> fx(s_.Nz, 0.0f), fy(s_.Nz, 0.0f);
		const float per_step = s_.dt/s_.nudge_tau;
		wind_error_ = 0.0;
		for(uint z=1u; z+1u<s_.Nz; z++) {
			const float h = height(z);
			fx[z] = (wind_target_x(h)-(float)means[z].u)/s_.speed()*per_step;
			fy[z] = (wind_target_y(h)-(float)means[z].v)/s_.speed()*per_step;
			if(h<zi) wind_error_ = std::max(wind_error_, std::hypot(wind_target_x(h)-means[z].u, wind_target_y(h)-means[z].v)/(double)s_.wind_speed);
		}
		lbm_.set_level_forcing(fx, fy, rate_, T_ref_);
	}
	/// theta - theta_0 of the start profile at height h: the tabulated sounding, or the mixed layer, the inversion spread over jump_depth and the lapse above.
	float start_theta(float h) const {
		if(!s_.profile_z.empty()) return s_.profile_at(h);
		if(h<s_.zi) return 0.0f;
		if(h<s_.zi+s_.jump_depth) return s_.jump*(h-s_.zi)/s_.jump_depth;
		return s_.jump+s_.lapse*(h-s_.zi-s_.jump_depth);
	}

	/// Flags, the start temperature (mixed layer, inversion, stable air, floor noise) and the heat sources.
	void set_up() {
		std::mt19937 random(s_.seed);
		std::uniform_real_distribution<float> noise(-s_.noise, s_.noise);
		const float cx = 0.5f*(float)s_.Nx, cy = 0.5f*(float)s_.Ny+s_.patch_north/s_.dx, r = 0.5f*s_.patch/s_.dx;
		ulong patch_cells = 0ull;
		for(uint z=0u; z<s_.Nz; z++) for(uint y=0u; y<s_.Ny; y++) for(uint x=0u; x<s_.Nx; x++) {
			const ulong n = index(x, y, z);
			if(z==0u||z==s_.Nz-1u) {
				lbm_.flags[n] = TYPE_S;
#ifdef MOISTURE
				// the walls hold the air's water: the first Esoteric-Pull step streams their start populations into the fluid
				if(s_.moist) lbm_.Q[n] = 1.0f+s_.qt_at(std::clamp(height(z), 0.0f, height(s_.Nz-2u)))/0.01f;
#endif // MOISTURE
				continue;
			}
			const float h = height(z);
			float theta = start_theta(h);
			if(h<s_.noise_top) theta += noise(random);
			lbm_.T[n] = 1.0f+theta;
#ifdef MOISTURE
			// moist: the sounding's total water; dry: the passive tracer's uniform start value (--tracer; SCALAR_FV carries Q - 1, so 1 tests nothing there)
			lbm_.Q[n] = s_.moist ? 1.0f+s_.qt_at(h)/0.01f : s_.tracer;
#endif // MOISTURE
			if(s_.wind_speed>0.0f) { lbm_.u.x[n] = wind_target_x(h)/s_.speed(); lbm_.u.y[n] = wind_target_y(h)/s_.speed(); } // the start wind is the target profile
			const bool in_patch = s_.patch_layers>0u&&std::hypot((float)x+0.5f-cx, (float)y+0.5f-cy)<=r;
			if(z==1u||(in_patch&&!s_.patch_surface&&z<=s_.patch_layers)) lbm_.flags[n] = TYPE_Y;
			if(in_patch&&s_.patch_surface&&z==1u) lbm_.flags[n] = TYPE_X|TYPE_Y;
			if(in_patch&&z==1u) patch_cells++;
		}
		lbm_.set_heat_rate(s_.heat_rate());
		lbm_.set_inv_prandtl_t(s_.inv_prandtl_t);
		{ // the per-level terms: no wind nudging yet (fx, fy 0), the sponge's damping rate and reference temperature (the start profile without noise)
			std::vector<float> fx(s_.Nz, 0.0f), fy(s_.Nz, 0.0f);
			rate_.assign(s_.Nz, 0.0f); T_ref_.assign(s_.Nz, 1.0f);
			std::vector<float>& rate = rate_; std::vector<float>& T_ref = T_ref_;
			const float lid = height(s_.Nz-1u);
			for(uint z=0u; z<s_.Nz; z++) {
				const float h = height(z);
				T_ref[z] = 1.0f+start_theta(h);
				if(s_.sponge_from>0.0f&&h>=s_.sponge_from) { const float f = (h-s_.sponge_from)/(lid-s_.sponge_from); rate[z] = s_.dt/s_.sponge_tau*f*f; }
			}
			lbm_.set_level_forcing(fx, fy, rate, T_ref);
		}
#ifdef MOISTURE
		if(s_.moist) set_up_moist();
#endif // MOISTURE
		std::printf("SETUP %u x %u x %u cells of %.2f m, dt %.3f s, beta %.4f, heat rate %.4e per step, patch %llu cells, %u domains, alpha %.4g (%.3g m^2/s)\n",
		            s_.Nx, s_.Ny, s_.Nz, s_.dx, s_.dt, s_.beta(), s_.heat_rate(), (unsigned long long)patch_cells, s_.Dx, s_.alpha, (double)s_.alpha*(double)s_.dx*(double)s_.dx/(double)s_.dt);
		std::fflush(stdout);
	}

	void read_fields() {
		lbm_.update_fields();
		lbm_.u.read_from_device();
		lbm_.T.read_from_device();
#ifdef MOISTURE
		if(s_.moist) lbm_.Q.read_from_device();
#endif // MOISTURE
	}

#ifdef MOISTURE
	/// @brief The uniform-scalar drift of the moisture transport: Q starts at the tracer value (--tracer, default 1)
	/// in every cell and has no source in a dry run, so it is a passive tracer that must stay uniform; prints the
	/// largest relative change, its level and the rms over the fluid cells. With SCALAR_FV use a value other than 1:
	/// the finite volumes carry Q - 1, and a zero field stays zero whatever the transport does.
	/// @param time the model time (s)
	void report_tracer(const float time) {
		lbm_.Q.read_from_device();
		double largest = 0.0, sum2 = 0.0, cells = 0.0;
		uint largest_z = 0u;
		for(uint z=1u; z+1u<s_.Nz; z++) for(uint y=0u; y<s_.Ny; y++) for(uint x=0u; x<s_.Nx; x++) {
			const double d = (double)lbm_.Q[index(x, y, z)]/(double)s_.tracer-1.0;
			if(std::fabs(d)>largest) { largest = std::fabs(d); largest_z = z; }
			sum2 += d*d; cells += 1.0;
		}
		std::printf("TRACER t %.0f s: the uniform Q (start %g) max |dQ/Q| %.3e at level %u (%.0f m), rms %.3e\n", time, (double)s_.tracer, largest, largest_z, height(largest_z), std::sqrt(sum2/cells));
	}

	/// @brief The moist air's per-level terms: the hydrostatic pressure from 101325 Pa through the start sounding (as
	/// the MicroHH reference runs integrate it: T = thl (p / 1e5)^(Rd/cp) (1 + 0.61 qt) per step), the reference theta_v
	/// of the start sounding at each level (the buoyancy's level reference), theta_0 (the thl that T = 1 stands for)
	/// and the floor's moisture source.
	void set_up_moist() {
		pressure_.assign(s_.Nz, 0.0);
		std::vector<float> pressure(s_.Nz, 0.0f), theta_v(s_.Nz, 0.0f);
		double p = moist_air::GroundPressure, h_prev = 0.0;
		for(uint z=0u; z<s_.Nz; z++) {
			const double h = std::max((double)height(z), 0.0);
			const double thl = (double)s_.theta_0+(double)start_theta((float)h), qt = (double)s_.qt_at((float)h);
			if(h>h_prev) {
				const double t_k = thl*std::pow(p/1.0E5, moist_air::Rd/moist_air::Cp)*(1.0+0.61*qt);
				p *= std::exp(-moist_air::G*(h-h_prev)/(moist_air::Rd*t_k));
				h_prev = h;
			}
			pressure_[z] = p;
			pressure[z] = (float)p;
			theta_v[z] = (float)moist_air::virtual_theta(thl, qt, p);
		}
		lbm_.set_level_moist(pressure, theta_v);
		lbm_.set_theta_0(s_.theta_0);
		lbm_.set_moisture_rate(s_.moisture_rate());
		std::printf("MOIST qt %.2f g/kg at the floor, the moisture flux %.3e kg/kg m/s (Bowen %.2f), rate %.4e Q per step; %.0f Pa at the floor, %.0f Pa at the lid\n",
		            1000.0f*s_.qt_at(height(1u)), s_.moisture_flux(), s_.bowen, s_.moisture_rate(), pressure_[1], pressure_[s_.Nz-1u]);
	}

	/// The liquid water of the cell at n (kg/kg) by the host's saturation adjustment at its level's pressure.
	double liquid(ulong n, uint z) {
		const double thl = (double)s_.theta_0+(double)lbm_.T[n]-1.0, qt = ((double)lbm_.Q[n]-1.0)*0.01;
		return moist_air::saturation_adjust(thl, qt, pressure_[z]).liquid;
	}

	/// Per level: the mean qt and ql (kg/kg) and the share of cells holding liquid water.
	CloudLevels cloud_levels() {
		CloudLevels c;
		c.qt.assign(s_.Nz, 0.0); c.ql.assign(s_.Nz, 0.0); c.fraction.assign(s_.Nz, 0.0);
		const double cells = (double)s_.Nx*(double)s_.Ny;
		parallel_for(s_.Nz-2u, [&](uint k) {
			const uint z = k+1u;
			double qt = 0.0, ql = 0.0, wet = 0.0;
			for(uint y=0u; y<s_.Ny; y++) for(uint x=0u; x<s_.Nx; x++) {
				const ulong n = index(x, y, z);
				const double l = liquid(n, z);
				qt += ((double)lbm_.Q[n]-1.0)*0.01; ql += l; if(l>0.0) wet += 1.0;
			}
			c.qt[z] = qt/cells; c.ql[z] = ql/cells; c.fraction[z] = wet/cells;
		});
		return c;
	}

	/// The mixed layer's condensation level (m): its mean thl and qt over 0.25 to 0.75 zi lifted at constant thl and qt
	/// to the lowest level where the saturation adjustment gives liquid water; -1 when none below the lid. zi is the
	/// case's (--zi), not the heat-flux readout: under deep clouds the readout moved into the cloud layer
	/// (2600-3300 m against 1704 m in a humid case), the window took in cloudy air and the level came out 128 m high.
	double condensation_level(const std::vector<LayerMeans>& means, const CloudLevels& c) const {
		const float zi = s_.zi;
		double thl = 0.0, qt = 0.0, n = 0.0;
		for(uint z=1u; z+1u<s_.Nz; z++) if(height(z)>=0.25f*zi&&height(z)<=0.75f*zi) { thl += (double)s_.theta_0+means[z].theta; qt += c.qt[z]; n += 1.0; }
		if(n==0.0) return -1.0;
		thl /= n; qt /= n;
		for(uint z=1u; z+1u<s_.Nz; z++) if(moist_air::saturation_adjust(thl, qt, pressure_[z]).liquid>0.0) return (double)height(z);
		return -1.0;
	}

	/// The CLOUD line of an interval: the cloud base and top (the lowest and highest levels with at least
	/// cloud_fraction_min of their cells wet), the largest cloud fraction, the largest mean ql, the condensation level.
	/// While recording, sums the base and the condensation level for the end's check.
	void report_cloud(float time, const std::vector<LayerMeans>& means, const CloudLevels& c, bool recording) {
		int base = -1, top = -1;
		double cover = 0.0, ql_max = 0.0;
		for(uint z=1u; z+1u<s_.Nz; z++) {
			if(c.fraction[z]>=(double)s_.cloud_fraction_min) { if(base<0) base = (int)z; top = (int)z; }
			cover = std::max(cover, c.fraction[z]); ql_max = std::max(ql_max, c.ql[z]);
		}
		const double lcl = condensation_level(means, c);
		const double base_m = base>0 ? (double)height((uint)base) : -1.0, top_m = top>0 ? (double)height((uint)top) : -1.0;
		std::printf("CLOUD t %.0f s: base %.0f m, top %.0f m, largest cloud fraction %.4f, largest mean ql %.3e kg/kg; the mixed layer's condensation level %.0f m\n",
		            time, base_m, top_m, cover, ql_max, lcl);
		if(recording) {
			recorded_intervals_++;
			if(base_m>0.0&&lcl>0.0) { cloud_base_sum_ += base_m; lcl_sum_ += lcl; cloudy_intervals_++; }
		}
	}

	/// The end's check: the time-mean cloud base over the recorded intervals within 10 % of the time-mean condensation
	/// level; FAILED when no recorded interval had a cloud.
	void report_cloud_check() const {
		if(cloudy_intervals_==0u) { std::printf("RESULT CLOUD FAILED: no cloud in the %u recorded intervals\n", recorded_intervals_); return; }
		const double base = cloud_base_sum_/(double)cloudy_intervals_, lcl = lcl_sum_/(double)cloudy_intervals_, error = std::fabs(base/lcl-1.0);
		std::printf("RESULT CLOUD the mean cloud base %.0f m against the mean condensation level %.0f m: %.1f %% (limit 10 %%) %s; a cloud in %u of %u recorded intervals\n",
		            base, lcl, 100.0*error, error<=0.10 ? "PASSED" : "FAILED", cloudy_intervals_, recorded_intervals_);
	}
#endif // MOISTURE

	/// The horizontal means of every fluid layer (index z).
	std::vector<LayerMeans> layer_means() {
		std::vector<LayerMeans> means(s_.Nz);
		const double c = (double)s_.speed(), cells = (double)s_.Nx*(double)s_.Ny;
		parallel_for(s_.Nz-2u, [&](uint k) {
			const uint z = k+1u;
			double t = 0.0, tt = 0.0, w = 0.0, ww = 0.0, www = 0.0, wt = 0.0, u = 0.0, v = 0.0, w_max = -1E9, w_min = 1E9;
			for(uint y=0u; y<s_.Ny; y++) for(uint x=0u; x<s_.Nx; x++) {
				const ulong n = index(x, y, z);
				const double tn = (double)lbm_.T[n]-1.0, wn = (double)lbm_.u.z[n]*c;
				t += tn; tt += tn*tn; w += wn; ww += wn*wn; www += wn*wn*wn; wt += wn*tn; u += (double)lbm_.u.x[n]*c; v += (double)lbm_.u.y[n]*c;
				w_max = std::max(w_max, wn); w_min = std::min(w_min, wn);
			}
			LayerMeans& m = means[z];
			m.theta = t/cells; m.u = u/cells; m.v = v/cells;
			const double wm = w/cells, tm = t/cells;
			m.w_var = ww/cells-wm*wm;
			m.theta_var = tt/cells-tm*tm;
			const double w3 = www/cells-3.0*wm*ww/cells+2.0*wm*wm*wm; // the central third moment
			m.w_skew = m.w_var>0.0 ? w3/std::pow(m.w_var, 1.5) : 0.0;
			m.w_theta = wt/cells-wm*tm;
			m.w_max = w_max; m.w_min = w_min;
		});
		return means;
	}

	/// zi: the height of the most negative resolved heat flux w'theta' (the entrainment flux), above a tenth of the
	/// column. (The largest temperature gradient moved to the lid once entrainment smeared the inversion: 984 m of 997.)
	/// Before the convection reaches the inversion there is no negative flux: then the start's zi.
	float inversion_height(const std::vector<LayerMeans>& means) const {
		double lowest = 0.0;
		float zi = s_.zi;
		for(uint z=2u; z+2u<s_.Nz; z++) {
			if(height(z)<0.1f*height(s_.Nz-1u)) continue;
			if(s_.sponge_from>0.0f&&height(z)>=s_.sponge_from) break; // the flux minimum is searched below the sponge
			if(means[z].w_theta<lowest) { lowest = means[z].w_theta; zi = height(z); }
		}
		return zi;
	}

	/// The mixed layer's top by the rule of the MicroHH reference runs: the lowest
	/// height above 100 m where the layer-mean theta exceeds its mean between 100 and 600 m by 0.5 K. The header's zi:
	/// the heat-flux minimum moves into the cloud layer under deep clouds (humid 2493 m against 1704 m), and the
	/// tools that read the header take zi as the mixed layer's top (the wind below it, w*). Fails without such a level.
	float mixed_layer_top(const std::vector<LayerMeans>& means) const {
		double sum = 0.0, n = 0.0;
		for(uint z=1u; z+1u<s_.Nz; z++) if(height(z)>=100.0f&&height(z)<=600.0f) { sum += means[z].theta; n += 1.0; }
		if(n==0.0) print_error("mixed_layer_top(): no level between 100 and 600 m");
		const double mixed = sum/n;
		for(uint z=1u; z+1u<s_.Nz; z++) if(height(z)>100.0f&&means[z].theta>mixed+0.5) return height(z);
		print_error("mixed_layer_top(): theta nowhere 0.5 K above the mixed layer below the lid");
		return 0.0f;
	}

	void write_profiles(float time, const std::vector<LayerMeans>& means) {
		for(uint z=1u; z+1u<s_.Nz; z++) {
			const LayerMeans& m = means[z];
			char line[256];
			std::snprintf(line, sizeof(line), "%.1f,%.2f,%.5f,%.5f,%.6f,%.5f,%.5f,%.4f,%.4f,%.6f,%.4f", time, height(z), m.theta, m.w_var, m.w_theta, m.u, m.v, m.w_max, m.w_min, m.theta_var, m.w_skew);
			profiles_ << line;
			if(s_.moist) { std::snprintf(line, sizeof(line), ",%.7f,%.7f,%.6f", clouds_.qt[z], clouds_.ql[z], clouds_.fraction[z]); profiles_ << line; }
			profiles_ << "\n";
		}
		profiles_.flush();
	}

	uint8_t quantize(float value, float lo, float hi) {
		const float q = std::round((value-lo)*255.0f/(hi-lo));
		if(q<0.0f||q>255.0f) clipped_++;
		return (uint8_t)std::clamp(q, 0.0f, 255.0f);
	}

	/// One frame of the box: w, u, v as uint8 (u and v less the layer's mean), the ql bits, and w as half floats.
	void write_frame(const std::vector<LayerMeans>& means) {
		const uint b = s_.box, bn = box_north(), x0 = (s_.Nx-b)/2u, y0 = box_y0(), levels = s_.Nz-2u;
		const ulong cells = (ulong)b*(ulong)bn*(ulong)levels;
		std::vector<uint8_t> w(cells), u(cells), v(cells), ql((cells+7ull)/8ull, 0u);
		std::vector<ushort> w16(cells);
		const float c = s_.speed();
		parallel_for(levels, [&](uint k) {
			const uint z = k+1u;
			for(uint j=0u; j<bn; j++) for(uint i=0u; i<b; i++) {
				const ulong n = index(x0+i, y0+j, z), o = ((ulong)k*(ulong)bn+(ulong)j)*(ulong)b+(ulong)i;
				const float wn = lbm_.u.z[n]*c;
				w[o] = quantize(wn, s_.w_lo, s_.w_hi);
				u[o] = quantize(lbm_.u.x[n]*c-(float)means[z].u, s_.uv_lo, s_.uv_hi);
				v[o] = quantize(lbm_.u.y[n]*c-(float)means[z].v, s_.uv_lo, s_.uv_hi);
				w16[o] = float_to_half(wn);
			}
		});
#ifdef MOISTURE
		if(s_.moist) { // the ql bits: 1 where the cell holds liquid water (ql > 0), big-endian bit order
			if(((ulong)b*(ulong)bn)%8ull!=0ull) print_error("the ql bits need a box level of a multiple of 8 cells, so that no byte spans two levels");
			parallel_for(levels, [&](uint k) {
				const uint z = k+1u;
				for(uint j=0u; j<bn; j++) for(uint i=0u; i<b; i++) {
					const ulong o = ((ulong)k*(ulong)bn+(ulong)j)*(ulong)b+(ulong)i;
					if(liquid(index(x0+i, y0+j, z), z)>0.0) ql[o>>3] |= (uint8_t)(0x80u>>(uint)(o&7ull));
				}
			});
		}
#endif // MOISTURE
		frames_.write((const char*)w.data(), (std::streamsize)cells);
		frames_.write((const char*)u.data(), (std::streamsize)cells);
		frames_.write((const char*)v.data(), (std::streamsize)cells);
		frames_.write((const char*)ql.data(), (std::streamsize)ql.size());
		frames_.flush();
		w16_.write((const char*)w16.data(), (std::streamsize)(cells*2ull));
		w16_.flush();
		if(mean_u_sum_.empty()) { mean_u_sum_.assign(levels, 0.0); mean_v_sum_.assign(levels, 0.0); }
		for(uint k=0u; k<levels; k++) { mean_u_sum_[k] += means[k+1u].u; mean_v_sum_[k] += means[k+1u].v; } // the means the frame left out of u and v
		frames_written_++;
	}

	uint box_north() const { return s_.box_north>0u ? s_.box_north : s_.box; }
	/// The box's first cell north (index): centred on the patch, or box_first_north m from the patch's centre.
	uint box_y0() const {
		const float cy = 0.5f*(float)s_.Ny+s_.patch_north/s_.dx; // the patch's centre in cell units (a cell's centre is at i + 0.5)
		const float first = s_.box_first_north!=0.0f ? s_.box_first_north : -0.5f*(float)box_north()*s_.dx+0.5f*s_.dx;
		const long y0 = std::lround(cy+first/s_.dx-0.5f);
		if(y0<0||(ulong)y0+box_north()>s_.Ny) print_error("the recorded box reaches outside the domain north");
		return (uint)y0;
	}
	/// The w plane (float32, Nx x Ny, m/s) at the level nearest 0.5 zi, for the spectra.
	void write_w_plane(float zi) {
		const uint z = std::clamp((uint)std::lround(0.5f*zi/s_.dx)+1u, 1u, s_.Nz-2u);
		std::vector<float> plane((size_t)s_.Nx*s_.Ny);
		const float c = s_.speed();
		for(uint y=0u; y<s_.Ny; y++) for(uint x=0u; x<s_.Nx; x++) plane[(size_t)x+(size_t)y*s_.Nx] = lbm_.u.z[index(x, y, z)]*c;
		w_plane_.write((const char*)plane.data(), (std::streamsize)(plane.size()*sizeof(float)));
		w_plane_.flush();
	}

	/// header.json for the frames written so far (the format-2 fields that analysis tools read).
	void write_header(double zi) const {
		const uint b = s_.box, bn = box_north(), levels = s_.Nz-2u;
		const double first_north = s_.box_first_north!=0.0f ? (double)s_.box_first_north : -0.5*(double)bn*(double)s_.dx+0.5*(double)s_.dx;
		const double w_star = std::cbrt(9.81/(double)s_.theta_0*(double)s_.flux*zi);
		const double first = -0.5*(double)b*(double)s_.dx+0.5*(double)s_.dx;
		std::ofstream h(folder_/"header.json");
		h << "{\n";
		h << "  \"format\": 2,\n";
		// the case's fields, in the form the analysis tools read
		// the class: a placeholder for the tools that file the recording under a weather class; a moist run's humidity is "unset", so an index of the raw recording fails instead of calling it dry
		h << "  \"case\": {\"source\": \"FluidX3D convective_boundary_layer\", \"quality\": \"" << (s_.moist ? "moist_cbl" : "dry_cbl") << "\", \"humidity\": \"" << (s_.moist ? "unset" : "dry") << "\", \"hours\": 13.0, \"ground\": 0.0"
		  << ", \"flux\": " << s_.flux << ", \"theta_0\": " << s_.theta_0 << ", \"patch\": " << s_.patch << ", \"heat\": " << s_.patch_layers << ", \"patch_mode\": \"" << (s_.patch_surface ? "surface" : "layers") << "\"" << ", \"background\": 1.0, \"seed\": " << s_.seed
		  << ", \"spinup\": " << (int)s_.spinup << ", \"record\": " << (int)s_.record << ", \"cells\": " << s_.Nx << ", \"dx\": " << s_.dx << ", \"dz\": " << s_.dx
		  << ", \"top\": " << (double)(s_.Nz-1u)*(double)s_.dx << ", \"dump\": " << (int)s_.interval << ", \"dt\": " << s_.dt
		  << ", \"air\": \"mixed layer to " << s_.zi << " m, inversion " << s_.jump << " K, " << s_.lapse*1000.0f << " K/km above\""
		  << ", \"cells_north\": " << s_.Ny << ", \"patch_north\": " << s_.patch_north/s_.dx << ", \"wind\": ";
		if(s_.wind_speed>0.0f) { // the header's wind: the nudging target per box level
			h << "{\"field\": \"power-law\", \"note\": \"power law: " << s_.wind_speed << " m/s at " << s_.wind_height << " m, exponent " << s_.wind_exponent << ", from " << s_.wind_from_deg << " deg\", \"nudge\": " << s_.nudge_tau << ", \"scale\": 1.0, \"u\": [";
			for(uint k=0u; k<levels; k++) h << (k ? ", " : "") << wind_target_x(height(k+1u));
			h << "], \"v\": [";
			for(uint k=0u; k<levels; k++) h << (k ? ", " : "") << wind_target_y(height(k+1u));
			h << "]}";
		} else h << "null";
		h << ", \"zi\": " << zi << ", \"w_star\": " << w_star << "},\n";
		h << "  \"box\": {\"cells\": " << b << ", \"cells_north\": " << bn << ", \"levels\": " << levels << ", \"dx\": " << s_.dx << ", \"dz\": " << s_.dx
		  << ", \"first_east\": " << first << ", \"first_north\": " << first_north << ", \"first_up\": " << 0.5*(double)s_.dx << "},\n";
		// per box level the domain's mean u and v over the written frames (m/s): what the frames' u and v leave out
		h << "  \"mean_wind\": {\"u\": [";
		for(uint k=0u; k<levels; k++) h << (k ? ", " : "") << mean_u_sum_[k]/(double)frames_written_;
		h << "], \"v\": [";
		for(uint k=0u; k<levels; k++) h << (k ? ", " : "") << mean_v_sum_[k]/(double)frames_written_;
		h << "]},\n";
		h << "  \"frames\": {\"count\": " << frames_written_ << ", \"interval\": " << s_.interval << ", \"first_model_time\": " << s_.spinup+s_.interval << "},\n";
		h << "  \"fields\": [\"w\", \"u\", \"v\", \"ql\"],\n";
		h << "  \"layout\": \"frames.bin: per frame w, u, v as uint8 (k up, j north, i east), value = lo + byte * (hi - lo) / 255 with quant's lo and hi in m/s, then ql as one bit per cell (" << (s_.moist ? "1 = liquid water" : "all 0, dry") << "), big-endian bit order; w, u, v at the cells' centres, u and v less each level's mean over the domain; w_f16.bin: w of the same cells as IEEE half floats\",\n";
		h << "  \"quant\": {\"w\": {\"lo\": " << s_.w_lo << ", \"hi\": " << s_.w_hi << "}, \"u\": {\"lo\": " << s_.uv_lo << ", \"hi\": " << s_.uv_hi
		  << "}, \"v\": {\"lo\": " << s_.uv_lo << ", \"hi\": " << s_.uv_hi << "}},\n";
		h << "  \"clipped\": " << clipped_ << "\n";
		h << "}\n";
	}
};

} // namespace

void main_setup() { // extensions: VOLUME_FORCE, TEMPERATURE, SUBGRID
	const auto take_option = [](const string& name) { // removes "<name> <value>" from the arguments; returns the value (> 0), or 0 when the option is absent
		for(size_t k=0u; k<main_arguments.size(); k++) {
			if(main_arguments[k]!=name) continue;
			if(k+1u>=main_arguments.size()) print_error(name+" needs a value.");
			const float value = std::stof(main_arguments[k+1u]);
			if(!(value>0.0f)) print_error(name+" must be greater than 0.");
			main_arguments.erase(main_arguments.begin()+(long long)k, main_arguments.begin()+(long long)k+2);
			return value;
		}
		return 0.0f;
	};
	const auto take_flag = [](const string& name) { // removes "<name>" from the arguments; returns whether it was there
		for(size_t k=0u; k<main_arguments.size(); k++) if(main_arguments[k]==name) { main_arguments.erase(main_arguments.begin()+(long long)k); return true; }
		return false;
	};
	const float alpha_option = take_option("--alpha"); // the molecular diffusivity of T and Q in lattice units (0: the case's own value)
	const float tracer_option = take_option("--tracer"); // the passive tracer's uniform start value of Q (0: 1)
	const float zi_option = take_option("--zi"); // the sounding case's zi in m (0: 1864, the dry reference case's); the lid and the sponge count in it
	const bool moist_option = take_flag("--moist"); // the sounding's qt, the moisture flux, the moist buoyancy and the clouds
	if(main_arguments.empty()) print_error("convective_boundary_layer <output folder> [quick | sp2011 <cell m> <cells wide> [seed [domains [patch m [surface]]]] | sounding <cell m> <cells wide> <profile.csv> [seed [domains [lid zi [sponge from zi [wind m/s at 100 m [from deg]]]]]]] [--alpha <lattice units>] [--tracer <Q>] [--zi <m>] [--moist]");
	CblSettings s;
	if(main_arguments.size()>1u&&main_arguments[1]=="quick") { s = CblSettings::quick(); if(main_arguments.size()>2u&&main_arguments[2]=="surface") s.patch_surface = true; }
	else if(main_arguments.size()>3u&&main_arguments[1]=="sp2011") {
		s = CblSettings::sullivan_patton(std::stof(main_arguments[2]), (uint)std::stoul(main_arguments[3]));
		if(main_arguments.size()>4u) s.seed = (unsigned)std::stoul(main_arguments[4]);
		if(main_arguments.size()>5u) s.Dx = (uint)std::stoul(main_arguments[5]); // domains along x: one FP32 buffer holds at most 6140 MB on the 4090, 80 M cells of fi
		if(main_arguments.size()>6u) { s.patch = std::stof(main_arguments[6]); s.patch_layers = 4u; s.frames = true; s.box = 256u; } // the 4x patch, the frames of a 256-cell box
		if(main_arguments.size()>7u&&main_arguments[7]=="surface") s.patch_surface = true; // the 4x flux in the surface layer, not 4 layers
	}
	else if(main_arguments.size()>4u&&main_arguments[1]=="sounding") {
		s = CblSettings::tabulated_sounding(std::stof(main_arguments[2]), (uint)std::stoul(main_arguments[3]), std::filesystem::path(main_arguments[4]), zi_option>0.0f ? zi_option : 1864.0f);
		if(main_arguments.size()>5u) s.seed = (unsigned)std::stoul(main_arguments[5]);
		if(main_arguments.size()>6u) s.Dx = (uint)std::stoul(main_arguments[6]);
		if(main_arguments.size()>7u) s.Nz = (uint)std::lround(std::stof(main_arguments[7])*s.zi/s.dx)+2u; // the lid in zi (2.0 with the sponge, 3.0 without)
		if(main_arguments.size()>8u) s.sponge_from = std::stof(main_arguments[8])*s.zi; // the sponge's start in zi (0: no sponge)
		if(main_arguments.size()>9u) s.wind_speed = std::stof(main_arguments[9]); // m/s at 100 m, from the north
		if(main_arguments.size()>10u) s.wind_from_deg = std::stof(main_arguments[10]);
		if(s.wind_speed>0.0f) { // the geometry with wind: the domain twice as long north, the patch 2000 m upwind of the middle, the box 3 times longer and 4 km downwind of the patch
			s.Ny = 2u*s.Nx; s.patch_north = 200.0f*s.dx; s.box_north = 3u*s.box; s.box_first_north = -(2.5f*(float)s.box*s.dx-0.5f*s.dx); s.Dx = 2u;
		}
	}
	if(alpha_option>0.0f) {
#ifndef SCALAR_FV
		if(alpha_option<0.014f) print_error("--alpha below 0.014 (tauT 0.556) needs SCALAR_FV: the D3Q7 thermal lattice creates heat next to TYPE_S walls there (tests/physics/heat_conservation).");
#endif // SCALAR_FV
		s.alpha = alpha_option;
	}
	if(tracer_option>0.0f) {
#ifndef MOISTURE
		print_error("--tracer needs MOISTURE: the tracer is the moisture field Q.");
#endif // MOISTURE
		s.tracer = tracer_option;
	}
	if(zi_option>0.0f&&!(main_arguments.size()>1u&&main_arguments[1]=="sounding")) print_error("--zi applies to the sounding case only.");
	if(moist_option) {
#ifndef MOISTURE
		print_error("--moist needs MOISTURE: the total water is the moisture field Q.");
#endif // MOISTURE
		if(s.profile_qt.empty()) print_error("--moist needs the sounding case with a qt_kgkg column in its profile.");
		if(tracer_option>0.0f) print_error("--moist and --tracer both set Q: give one of them.");
		s.moist = true;
	}
	CblRun run(s, std::filesystem::path(main_arguments[0]));
	run.run();
}
