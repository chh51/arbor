# Xcode project, then Swift API, then Swift Testing

**Status:** step 1 is in place. Step 2's Swift API calls ArborCore through `apple/bridge` for the pipeline and QSM/QSF surface the static library already compiles. `resolveOversegmentation`, `extractTreeContext`, and `qsfSegmentSemantic` still throw `ArborError.engineNotConnected` because that C++ lives in Rcpp translation units ArborCore does not compile. Swift tests are not started. The processing surface is `apple/notes/05-swift-r-api.md`. The macOS compiler and memory notes are `apple/notes/04-openmp-apple.md`.

Work in this order. Each step is done when the one before it builds.

1. An Xcode project that compiles and links the C++ engine on macOS 27.
2. A Swift API in that project for the processing functions. GUI and plot helpers stay out.
3. Swift Testing tests that call that API on the same sample files the R tests use.

`arbor.pro` and `src/Makevars` stay as they are. The Xcode project is an additional macOS build, not a replacement for the R package.

## 1. Xcode project for the C++

Put it at `apple/Arbor.xcodeproj` so it sits beside the R tree. When the project exists, add `^apple$` to `.Rbuildignore`.

First target: a static library, `ArborCore`. No Swift yet. A one-file C++ command-line target that links `ArborCore` is enough to prove the link. It does not need to process a cloud.

### What the library compiles

Apple Clang, C++20, and these defines unset or off: `USING_R`. `NOGDAL` can stay, matching `arbor.pro`. Header search paths are the `-I` list in `src/Makevars`.

Compile the stage sources `arbor.pro` already lists, plus `src/vendor/libqsf/*.cpp`. `QSF.cpp` calls `libqsf`, and `arbor.pro` currently omits it, so a straight copy of that file will fail at link.

Leave out anything that includes Rcpp:

- `src/R/`
- `src/RcppExports.cpp`, `src/RcppAutoExport.cpp`
- `src/misc/read_adtree_skeleton.cpp`
- `src/experimental/qsm_distance.cpp`
- `src/experimental/extrat_context.cpp` (the bottom of the file calls `Rcpp::wrap`)

`src/pointcloud/PointCloudDataFrame.cpp` is wrapped in `#ifdef USING_R`, so compiling it into this library produces an empty file. `PointCloud` is then `PointCloudDefault`.

### OpenMP and RAM

Link Homebrew `libomp` (`-Xpreprocessor -fopenmp`, the libomp include path, `-lomp`). That is the macOS branch `arbor.pro` does not have. Details are in `apple/notes/04-openmp-apple.md`.

One binary covers 16 GB through 256 GB. Do not add a build setting per RAM size. Thread count stays a run-time `OMP_NUM_THREADS`. The 16 GB machine is for the sample scenes below. The book’s 32 GB note is for a multi-thousand-square-metre batch, which is outside these tests.

### Done when

`ArborCore` builds with Apple Clang and the small C++ driver links. Swift is not in the project yet.

That build is `apple/Arbor.xcodeproj`, scheme `arbor-link`. OpenMP is Homebrew `libomp` at `/opt/homebrew/opt/libomp`. The language standard in the project is GNU++20 so Apple headers still provide `M_PI`. `apple/driver/main.cpp` only takes the address of the public entry points.

```bash
xcodebuild -project apple/Arbor.xcodeproj -scheme arbor-link -configuration Debug build
```

## 2. Swift processing API

Add a Swift target in the same project that links `ArborCore`. This is the R processing API from `apple/notes/05-swift-r-api.md`, not the plots.

Swift names follow the R names (`segmentGround` for `segment_ground`). Defaults and result fields stay aligned. `arborParametersDefault` mirrors `arbor::settings::ArborParameters`. Constants stay `ARBORTREE` 0, `ARBORLOW` 1, `ARBORUNDERSTORY` 2, `ARBORBUFFER` 3.

In scope: the pipeline (`hybridHomogeneization`, `segmentGround`, `woodLikelihood`, `segmentSemantic`, `findSeeds`, `segmentInstance`, `resolveOversegmentation`, `flagBuffer`, `flagSmallTrees`), QSM/QSF build, read/write, metrics (`qsmDbh`, `qsmStats`, and the other `qsm_*` measures), `qsfLog`, and `qsfFilter` plus its wrappers. CRS is a string on the cloud and on each model.

Out of scope: `plot`, `plot_semantic`, `plot_instance`, `plot_qsm`, `plot_likelihood`, `plot_passage`, `plot_semantic_instance`, `qsf_treemap`, `add_dbh3d`, `foliage.colors`, `install_cmd_tools`.

### Not every R function is a C++ call

Some exported R functions are data-frame logic the Swift target has to reimplement if the tests are going to match:

| R function | Where the work is |
| --- | --- |
| `wood_likelihood` | Thin wrapper over C++ anisotropy |
| `segment_ground`, `segment_semantic`, `segment_instance`, `find_seeds`, `qsm`, `qsf` | C++ |
| `flag_small_trees` | R only (`data.table` on `hag` / `treeID`) |
| `flag_buffer` | R only, and it calls `sf` (`st_convex_hull`, `st_buffer`, `st_intersects`) |
| `qsf_log`, `qsf_filter*` | R only, parsing the `message` attribute |
| `qsm_stats` and several `qsm_*` summaries | R only, on the cylinder table |
| `resolve_oversegmentation` | R driver over C++ detect/merge helpers |

`flag_buffer` is processing, not a plot. The pipeline test expects `ARBORBUFFER` in `UserData`, so the Swift port needs a convex hull and an inward buffer. That geometry is not in the C++ engine today. It is the one processing function that will not fall out of linking `ArborCore`.

Call the C++ through a small C++ wrapper compiled into `ArborCore` or a sibling target, with types Swift interop can see. Importing `src/api/arbor.h` directly means templates, `std::function`, and exceptions. The wrapper can catch `std::runtime_error` and surface a Swift `Error`.

The public Swift cloud type owns the columns the stages read and write (X, Y, Z, Classification, hag, pwood, foliage, passage, UserData, treeID). Stages mutate it in place, as the R functions mutate the `LAS`.

## 3. Swift Testing on the R sample files

Use Swift Testing (`import Testing`, `@Test`, `#expect`), in a test target in the same Xcode project. The tests call the Swift API, not the C++ headers.

The R fixtures are already on disk under `inst/extdata/`:

| File | R test |
| --- | --- |
| `9x9.laz` (9.4 MB) | `test-pipeline.R` |
| `tree_qsm.laz` (1.8 MB) | `test-qsm.R`, `test-qsf.R`, `test-crs.R` |
| `tree_6307.laz` | `test-qsm.R` placeholder warning |
| `oak-plantation.qsf` plus `inst/extdata/qsm/` | `test-qsf_log.R` |
| `complex_slice*.las`, `slice_buttress.las`, `double_ring_slice*.las` | `test-fitting.R` |

Arbor’s C++ does not read LAS or LAZ. The R tests use lidR for that, then pass a data frame in. Keep the same split: the Swift processing API takes points already in memory, and the test target is what opens the files.

`oak-plantation.qsf` can go through Arbor’s own QSF reader. The `.las` / `.laz` files need a reader in the test target (or a one-time export of the points). The engine library should not gain a LAS reader just for the tests.

Two gaps between “the same file” and “the same input the R test ran”:

- `test-pipeline.R` does not use all of `9x9.laz`. It reads with lidR `readTLS` and `-keep_random_fraction 0.6`. That subset is not stable across runs. The assertions are mostly structural (`pwood` exists, `UserData` codes, `qsf` length `22` with tolerance `1`). A Swift test on the full file, or on a different 60%, is not the same run. Before writing that test, freeze the points the R side actually kept, or accept that only the structural checks are comparable.
- CRS checks (`NAD83 / MTM zone 7` on `tree_qsm.laz`) come from the LAS header via lidR and `sf`. A reader that drops the header CRS will fail `test-qsf.R` and `test-crs.R` even when the cylinders match.

`test-fitting.R` calls unexported `fit_circloid_cpp`. The test target can see a Swift test hook for that. It does not need to be on the public API. The synthetic circle/ellipse cases in that file do not need a LAS reader. The eight real slices do.

Numeric locks to keep, from the R tests: DBH `0.2228` and `0.2093` at tolerance `0.005`, branch-order count `823` at order 3, quality count `93` at quality 5, QSF log tree IDs for `W2` and `W3`, filter lengths `7`, `47`, and `1`.

## What this discussion is not deciding yet

LAS reader library versus a frozen point dump is still open. `flag_buffer` is implemented in Swift. The Swift/C++ wrapper is the C API in `apple/bridge`.
