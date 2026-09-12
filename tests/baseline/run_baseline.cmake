# Runs one example's baseline build and compares its "BASELINE ..." lines with the stored expectation.
# Called by CTest (see cmake/FluidX3DExample.cmake):
#   cmake -DEXE=<exe> -DWORKDIR=<bin> -DEXPECTED=<tests/baselines/x.txt> -DACTUAL=<build file> -DCOMPARE=<comparator> -P run_baseline.cmake
# The comparator (baseline_compare.cpp) accepts numbers within a small tolerance.
# To accept new output as the expectation (after an intended change), run CTest with FLUIDX3D_BLESS=1 set.

# Examples wait for Enter on missing resources; an empty stdin makes that return immediately.
file(WRITE "${ACTUAL}.stdin" "")
execute_process(
	COMMAND "${EXE}"
	WORKING_DIRECTORY "${WORKDIR}"
	INPUT_FILE "${ACTUAL}.stdin"
	OUTPUT_VARIABLE output
	ERROR_VARIABLE output
	RESULT_VARIABLE exit_code
)

string(REGEX MATCHALL "BASELINE [^\r\n]*" lines "${output}")
if(NOT lines)
	string(LENGTH "${output}" output_length)
	math(EXPR tail_start "${output_length} - 1500")
	if(tail_start LESS 0)
		set(tail_start 0)
	endif()
	string(SUBSTRING "${output}" ${tail_start} -1 tail)
	message("${tail}")
	message("BASELINE_SKIPPED: no baseline output (exit code ${exit_code}); usually a missing resource file")
	return()
endif()

list(JOIN lines "\n" actual)
file(WRITE "${ACTUAL}" "${actual}\n")

if(DEFINED ENV{FLUIDX3D_BLESS})
	file(WRITE "${EXPECTED}" "${actual}\n")
	message("Baseline written: ${EXPECTED}")
	return()
endif()
if(NOT EXISTS "${EXPECTED}")
	message(FATAL_ERROR "No expected baseline ${EXPECTED}; run CTest with FLUIDX3D_BLESS=1 to create it")
endif()

execute_process(
	COMMAND "${COMPARE}" "${EXPECTED}" "${ACTUAL}"
	OUTPUT_VARIABLE differences
	RESULT_VARIABLE compare_result
)
if(NOT compare_result EQUAL 0)
	message("${differences}")
	message(FATAL_ERROR "Baseline mismatch: ${EXPECTED} (actual output: ${ACTUAL})")
endif()
message("Baseline matches: ${EXPECTED}")
