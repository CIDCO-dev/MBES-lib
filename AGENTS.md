# AGENTS.md

## Purpose
This document summarizes the technical analysis of the `MBES-lib` repository, the verified code/documentation inconsistencies, and the recommended actions.

## Maintenance Rule
- The `version.md` file must be kept up to date for every change made in the repository.
- Any planned or completed modification (code, CI, build, documentation) must be added to `version.md`.

## Analysis Scope
- Documentation: `README.md`, `Doxyfile`
- Build/CI: `CMakeLists.txt`, `Makefile`, `MakefileWindows`, `Jenkinsfile`, `Scripts/*`
- Source code: `src/datagrams/*`, `src/examples/*`, `src/hydroblock/*`, `src/examples/overlap/*`
- Tests: `test/main.cpp`, `test/*`

## Repository Snapshot
- Main language: C++
- Domain: MBES datagram parsing, georeferencing, ray tracing, and conversion tooling
- Formats supported in code:
- `.all`, `.xtf`, `.s7k`, `.kmall`
- Hydroblock directory mode (detected as a directory)

## Dependencies
### Build and test (required)
- `gcc`, `g++`, `make` (Ubuntu package: `build-essential`)
- `cmake` and `ctest`
- `libeigen3-dev` (Eigen)
- `pkg-config`

### Documentation (required for `make doc`)
- `doxygen`
- `graphviz` (`dot`)

### Quality and coverage (required for `make coverage`)
- `cppcheck`
- `gcovr`
- `python3` (if `gcovr` is installed via `pip`)

### Optional (specific targets)
- `libpcl-dev` and `qtbase5-dev` for visualization/overlap targets (not required for the main build)

## Verified Inconsistencies

### 1) Incomplete format documentation
- Observation: README lists 4 formats (`.all`, `.kmall`, `.s7k`, `.xtf`) but does not mention Hydroblock directory mode.
- Evidence:
- `README.md:5`
- `src/datagrams/DatagramParserFactory.cpp:17`
- Impact: users may assume Hydroblock directories are not supported.

### 2) Incomplete executable documentation
- Observation: README documents 5 programs, but CMake builds more (`raytrace`, `datagram-raytracer`, `wgs2lgf`, `lgf2wgs`).
- Evidence:
- `README.md:11`
- `CMakeLists.txt:31`
- `CMakeLists.txt:35`
- `CMakeLists.txt:55`
- `CMakeLists.txt:59`
- Impact: available features are not fully documented.

### 3) Versioning split across files
- Observation: version values are defined differently across tools.
- Evidence:
- `Makefile:4` (`0.1.0`)
- `MakefileWindows:4` (`0.1.0`)
- `Jenkinsfile:4` and `Jenkinsfile:6` (`0.1.$BUILD_ID`)
- `Doxyfile:41` (`CURRENT_VERSION_OF_MBES_LIB`, placeholder)
- `CMakeLists.txt:3` (no `project(... VERSION ...)`)
- Impact: artifacts and documentation can drift out of sync.

### 4) Obsolete/broken overlap/viewer targets
- Observation: overlap/viewer references files that are missing in the repository.
- Evidence:
- `src/examples/overlap/overlap.cpp:43` includes `../viewer/smallUtilityFunctions.hpp`
- `src/examples/viewer` is missing in the repository
- `MakefileWindows:47` references `src/examples/viewer/`
- Impact: `overlap` / `pcl-viewer` build reliability is poor.

### 5) C++ standard mismatch between root and overlap subproject
- Observation: root project uses C++17, overlap subproject uses C++11.
- Evidence:
- `CMakeLists.txt:8`
- `src/examples/overlap/CMakeLists.txt:2`
- Impact: inconsistent build behavior across targets.

### 6) Incorrect error message in `georeference`
- Observation: filename is not interpolated in one exception string.
- Evidence:
- `src/examples/georeference.cpp:222`
- Impact: weaker runtime diagnostics.

### 7) Incomplete CMake runtime output mapping
- Observation: `wgs2lgf` and `lgf2wgs` are built but not included in `RUNTIME_OUTPUT_DIRECTORY` properties.
- Evidence:
- `CMakeLists.txt:56`
- `CMakeLists.txt:60`
- `CMakeLists.txt:65`
- Impact: output paths may differ from the rest of the binaries.

### 8) Partial aggregated test coverage
- Observation: `GeorefPCLviewerTest.hpp` exists but is not included by `test/main.cpp`.
- Evidence:
- `test/GeorefPCLviewerTest.hpp:1`
- `test/main.cpp:5`
- Impact: some existing tests are not executed by the main test binary.

### 9) CTest discovers no tests
- Observation: CMake builds `tests`, but no tests are registered in CTest.
- Evidence:
- `CMakeLists.txt:70`
- `Scripts/linuxBuildAndTest.bash:8`
- Observed output: `ctest --test-dir build` returns `No tests were found!!!`.
- Impact: `ctest` provides no validation, while project scripts run `build/test/tests` directly.

### 10) Doxygen configuration partially outdated
- Observation: several Doxygen keys are obsolete for Doxygen 1.9.8.
- Evidence:
- `Doxyfile:247`
- `Doxyfile:1108`
- `Doxyfile:1244`
- `Doxyfile:1525`
- `Doxyfile:1813`
- `Doxyfile:1829`
- `Doxyfile:1895`
- `Doxyfile:1993`
- `Doxyfile:2173`
- `Doxyfile:2186`
- `Doxyfile:2195`
- `Doxyfile:2237`
- `Doxyfile:2244`
- `Doxyfile:2470`
- Impact: warning noise in documentation generation and harder maintenance.

### 11) Deprecated gcovr option in Makefile
- Observation: coverage target uses deprecated `gcovr --branches`.
- Evidence:
- `Makefile:94`
- `Makefile:95`
- Impact: warning noise and potential breakage in future gcovr releases.

### 12) Memory-safety warnings identified
- Observation: non-virtual polymorphic deletion risk and `new[]`/`delete` mismatch in tests.
- Evidence:
- `src/svp/SvpSelectionStrategy.hpp:18` (no virtual destructor)
- `test/RayTracerAppTest.hpp:120` (`delete svpStrategy`)
- `test/KongsbergParserTest.hpp:224` (`new[]`)
- `test/KongsbergParserTest.hpp:232` (`delete` instead of `delete[]`)
- Impact: possible undefined behavior (mainly in tests) and reduced code quality.

## Execution Validation
- Validation timestamp: `2026-02-19 17:08:43 UTC`
- CMake build: OK (`cmake -S . -B build && cmake --build build`)
- Test binary run: OK (`./build/test/tests -r compact`) => 120 cases, 687 assertions, all passed
- CTest: not configured for discovery (`No tests were found!!!`)
- Documentation (`make doc`): OK, with Doxygen warnings (obsolete tags + one documented parameter mismatch)
- Coverage (`make coverage`): OK, with warnings (`gcovr --branches` deprecated, compiler warnings)
- Confirmed binary outputs:
- `build/bin/*` for most tools
- `build/wgs2lgf` and `build/lgf2wgs` (outside `build/bin`, consistent with inconsistency #7)

## Recommended Actions (priority order)
1. Define a single version source in CMake (`project(MBES-lib VERSION x.y.z)`) and propagate it to CI/documentation.
2. Update `README.md` (actual supported formats, full executable list, build prerequisites).
3. Fix or remove obsolete targets (`viewer`/`overlap`) until missing sources are restored.
4. Normalize runtime output paths across all CMake targets, including `wgs2lgf` and `lgf2wgs`.
5. Register tests in CTest (`enable_testing()` + `add_test(...)`) so `ctest` becomes reliable.
6. Refresh `Doxyfile` with `doxygen -u`, then fix remaining documentation warnings.
7. Fix memory-safety issues and highest-priority compiler warnings.
