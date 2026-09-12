// Compares a baseline output with its expectation: the same keys in the same order, and numbers equal within a
// tolerance. Mesh voxelization may differ by a few cells between builds (/fp:fast code generation of the inline
// STL transform), so exact text comparison would fail for no real change.
// Usage: fluidx3d_baseline_compare <expected> <actual>; exit code 0 when they match, 1 otherwise.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double relative_tolerance = 1e-4; // a shift of the geometry by one cell changes counts by far more
constexpr double absolute_tolerance = 1e-12; // for values that are zero up to rounding

struct Line {
	std::string text, key;
	std::vector<double> values;
};

std::vector<Line> read_lines(const std::string& path) {
	std::vector<Line> lines;
	std::ifstream file(path);
	std::string text;
	while(std::getline(file, text)) {
		if(!text.empty() && text.back()=='\r') text.pop_back();
		if(text.empty()) continue;
		Line line{text, {}, {}};
		std::istringstream stream(text);
		std::string prefix;
		stream >> prefix >> line.key;
		double value = 0.0;
		while(stream >> value) line.values.push_back(value);
		lines.push_back(line);
	}
	return lines;
}

bool nearly_equal(const double a, const double b) {
	return std::fabs(a-b) <= std::max(absolute_tolerance, relative_tolerance*std::max(std::fabs(a), std::fabs(b)));
}

bool matches(const Line& expected, const Line& actual) {
	if(expected.key!=actual.key || expected.values.size()!=actual.values.size()) return false;
	for(size_t i=0; i<expected.values.size(); i++) {
		if(!nearly_equal(expected.values[i], actual.values[i])) return false;
	}
	return true;
}

} // namespace

int main(int argc, char* argv[]) {
	if(argc!=3) {
		std::cerr << "usage: fluidx3d_baseline_compare <expected> <actual>\n";
		return 2;
	}
	const std::vector<Line> expected = read_lines(argv[1]), actual = read_lines(argv[2]);
	bool match = expected.size()==actual.size();
	for(size_t i=0; i<std::max(expected.size(), actual.size()); i++) {
		const bool has_expected = i<expected.size(), has_actual = i<actual.size();
		if(has_expected && has_actual && matches(expected[i], actual[i])) continue;
		match = false;
		if(has_expected) std::cout << "  expected: " << expected[i].text << "\n";
		if(has_actual) std::cout << "  actual:   " << actual[i].text << "\n";
	}
	return match ? 0 : 1;
}
