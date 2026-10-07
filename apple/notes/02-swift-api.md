# 2. Swift API

**Status:** superseded by `apple/notes/05-swift-r-api.md` (R-shaped Swift API, matching tests, macOS memory range). Constraints below still apply.

Goal: call the C++ engine from Swift without going through R. The C++ to wrap is the non-`USING_R` build (`PointCloudDefault` + `src/api/arbor.h`).

## Constraints already visible

- **GPL-3.** A Swift app that links this library is a combined work under GPL-3. Fine for a personal tool. It blocks a closed-source app store binary unless the copyright holder relicenses.
- **Not a C API.** Headers expose C++ classes, templates (`visit`, CRTP `BasePointCloud`), `std::vector`, `std::function`, and exceptions (`std::runtime_error`). Swift cannot call `visit()` in a useful way.
- **`USING_R` must be off.** Otherwise the point type becomes an Rcpp data frame.
- **C++20** for `PointCloudDefault` (`std::span`). `QSF` also needs `std::filesystem` (C++17).
- **OpenMP** is how the hot loops parallelize. A Swift process that links the library has to link the same OpenMP runtime, or the library must be built single-threaded. See `apple/notes/03-macos.md`.
- **Points stay in C++.** Stages mutate large clouds. Copying every cloud into Swift arrays between stages will dominate runtime. Swift should pass buffers in and pull attributes out at the end.
- **Parameters** should be a plain Swift struct that fills `arbor::settings::ArborParameters`. Do not expose `visit()`.
- **Errors** today are C++ exceptions plus the logger. A Swift API needs a chosen policy: let Swift/C++ interop catch them, or catch at the boundary and return a Swift `Error`.
- **Services** are global (`ServiceLocator`). A Swift logger has to be registered once, and it must be safe to call from OpenMP threads.

## Options (pick one before writing a package)

| Approach | What it is | Why it might fit |
| --- | --- | --- |
| A. C ABI shim | `extern "C"` functions plus a Swift module map | Stable. Hides templates and exceptions. Extra C++ file to maintain |
| B. Swift/C++ interop | Import `arbor.h` with Xcode's C++ interop | Less glue. Fights templates, exceptions, and `std::function` |
| C. Objective-C++ | `.mm` wrapper types | Familiar on Apple. Another language in the boundary |

Default to **A** unless a spike shows B can see `PointCloudDefault` and the `arbor::` functions without a wrapper. Do not start with C.

## Smallest Swift surface worth a spike

Match the R pipeline, not every helper:

1. Create a cloud from x, y, z buffers
2. `segment_ground` → `wood_likelihood` / semantic → `find_seeds` → `segment_instance` → `qsf`
3. Read back tree id, wood flag, and a QSM (cylinders or a mesh path)

`arbor::utils::*` and the fitting headers stay internal until something in that pipeline needs them.

## Still to decide

- [ ] Static library vs dynamic library vs xcframework
- [ ] Who owns the cloud object across the Swift boundary (opaque pointer vs Swift class with `deinit`)
- [ ] Coordinate and attribute layout (separate `Float` buffers vs one struct-of-arrays matching `PointCloudDefault`)
- [ ] Whether QSM results are returned in memory or only written to `.qsm` / `.qsf`
- [ ] Minimum Swift / Xcode version
- [ ] Where the spike lives (`swift/` in this repo vs a separate package) so it does not get pulled into `R CMD check`
