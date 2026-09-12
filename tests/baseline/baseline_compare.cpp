// Compares a baseline output with its expectation: the same keys in the same order, and numbers equal within a
// tolerance. Mesh voxelization may differ by a few cells between builds (/fp:fast code generation of the inline
// STL transform), so exact text comparison would fail for no real change.
// Usage: fluidx3d_baseline_compare <expected> <actual>; exit code 0 when they match, 1 otherwise.
//        fluidx3d_baseline_compare --report <a> <b>: every key of both side by side, "!=" where they differ (for an
//        original example against its port, see tests/originals); exit code 0.
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

std::string values_text(const Line* line) {
	if(line==nullptr) return "-";
	std::ostringstream text;
	for(size_t i=0; i<line->values.size(); i++) text << (i>0 ? " " : "") << line->values[i];
	return text.str();
}

// matched by key: the two files may come from different programs, with lines missing on either side
void report(const std::vector<Line>& a, const std::vector<Line>& b) {
	std::vector<std::string> keys;
	for(const std::vector<Line>* lines : { &a, &b }) {
		for(const Line& line : *lines) if(std::find(keys.begin(), keys.end(), line.key)==keys.end()) keys.push_back(line.key);
	}
	const auto find = [](const std::vector<Line>& lines, const std::string& key) -> const Line* {
		const auto found = std::find_if(lines.begin(), lines.end(), [&](const Line& line) { return line.key==key; });
		return found!=lines.end() ? &*found : nullptr;
	};
	for(const std::string& key : keys) {
		const Line* line_a = find(a, key);
		const Line* line_b = find(b, key);
		const bool same = line_a!=nullptr && line_b!=nullptr && matches(*line_a, *line_b);
		std::cout << (same ? "   " : "!= ") << key << std::string(key.size()<16 ? 16-key.size() : 1, ' ')
			<< values_text(line_a) << "  |  " << values_text(line_b) << "\n";
	}
}

} // namespace

int main(int argc, char* argv[]) {
	if(argc==4 && std::string(argv[1])=="--report") {
		report(read_lines(argv[2]), read_lines(argv[3]));
		return 0;
	}
	if(argc!=3) {
		std::cerr << "usage: fluidx3d_baseline_compare [--report] <expected> <actual>\n";
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
