# version.md

## Purpose
Track all repository changes in one place.
This file must be updated whenever a change is planned or completed.

## Change Log

| Date (UTC) | Status | Area | Change |
|---|---|---|---|
| 2026-02-19 | Done | Docs | Converted `AGENTS.md` and `versions.md` content to English to enforce English-only repository documentation. |
| 2026-02-19 | Done | CI | Migrated CI from Jenkins to GitHub Actions in `.github/workflows/ci.yml` with Linux/Windows build-test jobs, documentation job, and coverage job. |
| 2026-02-19 | Done | Build | Updated `CMakeLists.txt` to `cmake_minimum_required(VERSION 3.10)`. |
| 2026-02-19 | Done | Build | Updated Eigen discovery in `CMakeLists.txt` to `find_package(Eigen3 REQUIRED NO_MODULE)` for vcpkg compatibility. |
| 2026-02-19 | Done | Docs | Added dependencies section to `AGENTS.md`. |
| 2026-02-19 | Done | Docs | Added repository analysis and version audit files: `AGENTS.md` and `versions.md`. |
| 2026-02-19 | Planned | Testing | Register tests in CTest (`enable_testing()` + `add_test(...)`) so `ctest` discovers and runs tests. |
| 2026-02-19 | Planned | Build | Normalize binary output paths for all CMake targets, including `wgs2lgf` and `lgf2wgs`. |
| 2026-02-19 | Planned | Versioning | Define a single project version in CMake and propagate it to CI and documentation. |
| 2026-02-19 | Planned | Docs | Update `README.md` to cover all supported formats and all built executables. |
| 2026-02-19 | Planned | Docs | Refresh `Doxyfile` (`doxygen -u`) and remove deprecated tags. |
| 2026-02-19 | Done | Build | Fixed MSVC build error in `src/datagrams/kmall/KmallParser.cpp` by replacing variable-length array usage with `std::vector<float>`. |
| 2026-02-19 | Done | Build | Replaced non-portable `strcasecmp` usage with a cross-platform case-insensitive comparison helper in `src/examples/wgs2lgf.cpp`, `src/examples/lgf2wgs.cpp`, and `src/examples/wgs2lgf_from_lat_lon_eh.cpp`. |

## Update Rule
When a change starts, add a `Planned` entry.
When it is completed, update that entry to `Done` (or add a new `Done` line with details).
