# 3. macOS

**Status:** the book’s macOS warning is an OpenMP performance limit on the R binary toolchain. CPU OpenMP via `libomp` is the path that fits Arbor with a build-flag change. See `apple/notes/04-openmp-apple.md`. No build has been run on this machine yet.

## What the book says

[Arbor book, Minimum Requirements](https://r-lidar.github.io/arbor_book/): Linux or Windows. R packages built on macOS do not parallelize with OpenMP, so Arbor is much slower, and they do not recommend a Mac.

That matches the R package, not a crash in the C++.

- `src/Makevars` turns OpenMP on only through R’s `$(SHLIB_OPENMP_CXXFLAGS)`. CRAN’s macOS R is Apple Clang, and those flags are empty. r-universe binaries are built the same way. `#pragma omp` is then ignored and the code runs on one thread.
- `src/myomp.h` defines `omp_get_thread_num()` and `omp_get_max_threads()` as 1 when `_OPENMP` is unset. `vendor/ptd/PTD.cpp` has the same stub.
- The hot loops use `parallel for`, `critical`, and `reduction` (graph kNN in `GraphBuilder.cpp`, semantic and instance segmentation, ground height, and one QSM per tree in `qsf_api.cpp`). Those constructs still run the full loop on one thread. The serial path is intentional.
- The book’s timings (about 1–15 minutes) assume that parallelism. The README’s “hundreds of QSMs per minute” does too. One thread keeps the same RAM demand and drops the speed claim.

A source build that links Homebrew `libomp` can define `_OPENMP`. The book is about the package you get from `install.packages`, which does not.

`arbor.pro` is a separate problem: its `unix` block passes GCC’s `-fopenmp`, which Apple Clang rejects. That is the standalone library, not the R package the book is describing.

This Mac (from the session): macOS 27, Darwin 27.0.0, Apple Silicon. Confirm compiler, Homebrew, and `libomp` when the diagnosis starts.

There are two different programs people might mean by "the C++ does not run on macOS":

| Build | How | Defines | OpenMP flags in tree |
| --- | --- | --- | --- |
| R package | `src/Makevars` | `USING_R` | `$(SHLIB_OPENMP_CXXFLAGS)` — whatever R's Makeconf sets |
| Shared library | `qmake arbor.pro && make` | no `USING_R`, `NOGDAL` | `unix { -fopenmp }` with no Apple exception |

GitHub Actions runs `R CMD check` on `macos-latest` for R release and devel (`.github/workflows/R-CMD-check.yaml`). That says the **R package** is expected to build on GitHub's macOS image. It says nothing about macOS 27, and nothing about `arbor.pro`.

## Facts in the source

- `#pragma omp` appears in segment, seed-adjacent utils, qsf, fitting-adjacent code, and `vendor/ptd`. The pragmas are not wrapped in `#ifdef _OPENMP`.
- `src/myomp.h` stubs `omp_get_*` to 1 thread when `_OPENMP` is unset. That header does not remove the pragmas.
- Apple Clang does not accept GCC's `-fopenmp` by itself. The usual workaround is Homebrew `libomp` and `-Xpreprocessor -fopenmp` plus `-lomp`. `arbor.pro` does not do that. Its `unix` block treats macOS like Linux.
- `DESCRIPTION` does not set `CXX_STD`. The R objects include `src/qsf/QSF.cpp`, which includes `<filesystem>` (C++17). The standalone point type includes `<span>` (C++20), and `arbor.pro` passes `-std=c++20`. The R object list does not compile `PointCloudDefault.cpp`.
- `arbor.pro` does not compile `src/vendor/libqsf`, while `QSF.cpp` calls `libqsf`. If qmake fails, check undefined `libqsf` symbols before blaming OpenMP.
- `src/misc/read_adtree_skeleton.cpp` includes Rcpp and is only in the R object list.

## What to run, in order

Record the command, the compiler, and the first error. Stop at the first hard failure instead of stacking fixes.

- [ ] `R CMD INSTALL .` (or the install line in `AGENTS.md`) and note whether OpenMP flags are empty
- [ ] `qmake arbor.pro && make` and save the first compiler or linker error
- [ ] If the failure is `-fopenmp`: retry that one build with Homebrew `libomp` flags, as a test, and write down whether the rest of the file then links
- [ ] If the failure is a missing symbol: check it against the `arbor.pro` source list in `apple/notes/01-cpp.md`
- [ ] If both builds succeed: the bug is runtime. Capture the crash or wrong result and the pipeline step, then look at that `.cpp` file

## Not assumed yet

- Empty OpenMP flags are not automatically a failed R build. Clang ignores unknown `#pragma omp` with a warning, and `myomp.h` can compile the runtime calls single-threaded.
- GDAL is not a macOS requirement for this tree. `arbor.pro` defines `NOGDAL`, and the R package does not link GDAL.
- A failure on this OS is not the same fact as a failure on `macos-latest` in CI.
