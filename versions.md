# versions.md

## Capture Context
- Repository: `MBES-lib`
- Branch: `master`
- Commit: `b44f136`
- Capture timestamp: `2026-02-19 17:27:08 UTC`

## Versions Declared in Repository Files
| Item | Version / Value | Source |
|---|---|---|
| Minimum CMake | `3.10` | `CMakeLists.txt:1` |
| Root C++ standard | `C++17` | `CMakeLists.txt:8` |
| Eigen requirement | `required` (no pinned version) | `CMakeLists.txt:5` |
| Linux Makefile version | `0.1.0` | `Makefile:4` |
| Windows Makefile version | `0.1.0` | `MakefileWindows:4` |
| Jenkins version scheme | `0.1.$BUILD_ID` | `Jenkinsfile:4`, `Jenkinsfile:6`, `Jenkinsfile:8` |
| Doxygen project version | `CURRENT_VERSION_OF_MBES_LIB` (placeholder) | `Doxyfile:41` |
| Embedded test framework | Catch2 `2.13.10` | `test/catch.hpp:2`, `test/catch.hpp:16` |
| Overlap subproject C++ standard | `C++11` | `src/examples/overlap/CMakeLists.txt:2` |

## Versions Detected in the Environment
| Tool | Detected Version |
|---|---|
| `cmake` | `3.28.3` |
| `ctest` | `3.28.3` |
| `g++` | `13.3.0` |
| `make` | `4.3` |
| `doxygen` | `1.9.8` |
| `cppcheck` | `2.13.0` |
| `gcovr` | `8.6` |
| `python3` | `3.12.3` |
| `pkg-config` | `1.8.1` |

## Build/Test/Doc Validation
- CMake build: OK (`cmake -S . -B build && cmake --build build`)
- Test executable: OK (`./build/test/tests -r compact`)  
  Result: 120 test cases, 687 assertions, all passed.
- CTest discovery: not configured (`No tests were found!!!`)  
  Structural reference: test binary exists (`CMakeLists.txt:70`) but no registered `add_test`.
- Documentation (`make doc`): OK, with Doxygen warnings (obsolete tags).
- Coverage (`make coverage`): OK, with deprecation warning for `gcovr --branches`.

## Versioning Inconsistencies
1. No single canonical project version source.
2. CMake does not define a formal project version (`project(... VERSION ...)` is missing).
3. Jenkins uses dynamic versioning (`0.1.$BUILD_ID`) while Makefiles use static `0.1.0`.
4. Doxygen still uses an unresolved placeholder (`CURRENT_VERSION_OF_MBES_LIB`).
5. C++ standard is inconsistent between root (`C++17`) and `overlap` (`C++11`).

## Normalization Recommendations
1. Define canonical version in CMake: `project(MBES-lib VERSION X.Y.Z)`.
2. Reuse that version in CI, documentation, and packaging.
3. Avoid hardcoded versions spread across multiple files (`Makefile`, `MakefileWindows`, `Jenkinsfile`, `Doxyfile`).
4. Align C++ standard across active subprojects (prefer `C++17`).
