# Swift API matched to the R package

**Status:** target for the Swift work. No Swift package yet. Supersedes the smaller spike in `notes/02-swift-api.md`. macOS build details are in `notes/04-openmp-apple.md`.

Three goals:

1. A Swift API whose names, arguments, defaults, and results follow the R API in `NAMESPACE`.
2. Swift tests that follow `tests/testthat/` case by case.
3. C++ build changes so that one binary runs on macOS 27 on machines with 16 GB to 256 GB of unified memory.

## 1. Swift API

The R API is a thin layer over C++. Most exported functions take a lidR `LAS`, pull out `las@data`, call an Rcpp entry, and return the same `LAS` or a data frame with a class (`qsm`, `qsf`). Swift should expose those same names and skip lidR, Rcpp, and `sf`.

Call shape to preserve:

```text
cloud = hybridHomogeneization(cloud)
cloud = segmentGround(cloud)
cloud = woodLikelihood(cloud)
cloud = segmentSemantic(cloud)
seeds = findSeeds(cloud)
cloud = segmentInstance(cloud, seeds)
cloud = flagBuffer(cloud, seeds, distance: -0.75)
cloud = flagSmallTrees(cloud, minHeight: 1)
forest = qsf(cloud)
```

Swift names can be the Swift form of the R names (`segmentGround` for `segment_ground`). Arguments, defaults, and result fields stay aligned with R. `arbor_parameters_default` becomes a Swift value with the same nested fields as `arbor::settings::ArborParameters` in `src/api/arbor.h`. Constants stay `ARBORTREE = 0`, `ARBORLOW = 1`, `ARBORUNDERSTORY = 2`, `ARBORBUFFER = 3`.

### Match these R exports

Pipeline and clouds: `hybrid_homogeneization`, `segment_ground`, `wood_likelihood`, `segment_semantic`, `find_seeds`, `segment_instance`, `resolve_oversegmentation`, `flag_buffer`, `flag_small_trees`, `colorize_trees`, `transfer_attributes`, `add_range`, `filter_range`, `read_trajectory`, `add_single_tree_ground`, `extract_tree_context`.

Models: `qsm`, `qsf`, `qsm_read`, `qsm_write`, `qsf_read`, `qsf_write`, `qsm_dbh`, `qsm_height`, `qsm_volume`, `qsm_stem`, `qsm_merchantable`, `qsm_nostump`, `qsm_stats`, `qsm_treeid`, `qsm_message`, `qsf_merchantable`, `qsf_segment_semantic`, `available_allometries`.

Logs: `qsf_log`, `qsf_filter`, and the `qsf_filter_*` wrappers (`ok`, `sapling`, `nomeasure`, `broken`, `flagged`, `warnings`, `errors`, `messages`).

CRS: R stores it with `sf::st_crs` on the `qsm` / `qsf` object (`R/crs.R`). Swift keeps the same CRS string (and EPSG when that is all the caller has) on the cloud and on each model. It does not import `sf`.

### Leave these on the R side

Plots and GUI helpers (`plot`, `plot_semantic`, `plot_instance`, `plot_qsm`, `plot_likelihood`, `plot_passage`, `plot_semantic_instance`, `qsf_treemap`, `add_dbh3d`, `foliage.colors`), `install_cmd_tools`, and anything that exists only to satisfy lidR or `sf` S3 methods. A Swift viewer can come later. It is not part of matching the processing API.

Unexported C++ used by tests still needs a Swift entry if its test is in scope. `tests/testthat/test-fitting.R` calls `arbor:::fit_circloid_cpp`. That function is not in `NAMESPACE`. The Swift test target can see it without putting it on the public API.

### Types

| R | Swift |
| --- | --- |
| lidR `LAS` / `data.table` | A point-cloud type owning the same columns the C++ reads and writes: X, Y, Z, Classification, hag, pwood, foliage, passage, UserData, treeID |
| `qsm` data frame plus attributes `id`, `name`, `message`, `crs` | A struct of cylinder rows plus those attributes |
| `qsf` named list of `qsm` | A dictionary or array of those models, keyed the same way |
| `arbor_parameters_default` | A struct tree filled from the C++ defaults |

Stages mutate the cloud the way the R functions mutate the `LAS`. Copying the cloud into Swift arrays between stages is optional for tests and too expensive for a full scene.

The C++ boundary stays the non-R build (`PointCloudDefault`, no `USING_R`). A small C ABI in front of `src/api/arbor.h` is the stable way to do this. Swift/C++ interop can be tried later. GPL-3 still covers a Swift program that links the library.

## 2. Swift tests

R tests live in `tests/testthat/`. Swift tests should use XCTest (or Swift Testing) and the same fixtures, assertions, and tolerances. Where an R test checks an S3 class, the Swift test checks the corresponding type and fields.

| R file | What it locks down | Swift |
| --- | --- | --- |
| `test-pipeline.R` | Attribute presence and types, `UserData` codes after semantic and instance segmentation, buffer flags, `qsf` length `22` with tolerance `1` on `9x9.laz` | Same pipeline and the same numbers, on a frozen point set |
| `test-qsm.R` | DBH `0.2228` and `0.2093` (tolerance `0.005`), write/read of `.qsm` and `.csv`, branch-order and quality counts, disconnected graph error, one-row placeholder, topology repair | Same thresholds and files |
| `test-qsf.R` | CRS name `NAD83 / MTM zone 7`, stats columns `treeID`, `V`, `H`, `.qsf` round-trip | Same, with a CRS string instead of an `sf` object |
| `test-qsf_log.R` | Codes `W2` and `W3`, tree ID lists, filter lengths `7`, `47`, `1` on `inst/extdata/oak-plantation.qsf` | Same file, same IDs |
| `test-crs.R` | `st_crs` get/set, including EPSG `32734` | Set and read the CRS; skip the `sf` class checks |
| `test-fitting.R` | Circle, ellipse, and circloid centers, radii, arc, `shape_type` on synthetic slices, plus a few real slices | Same inputs and tolerances. Needs `fit_circloid` visible to the test target |

`test-pipeline.R` reads `inst/extdata/9x9.laz` through lidR with `-keep_random_fraction 0.6`. That subset is not a fixed point list, so a Swift port of the test cannot treat a later R run as a bit-exact oracle. Freeze one decimated cloud (the points the R test actually kept, or a checked-in copy) and run both sides on that. `test-qsm.R` and `test-qsf.R` also need `tree_qsm.laz` and `tree_6307.laz`. `oak-plantation.qsf` is already in the tree and is the one fixture that does not need lidR.

Arbor’s C++ does not read LAS/LAZ. The R tests rely on lidR for that. The Swift test target needs its own reader, or fixtures exported once to a simple array format. The processing assertions stay on Arbor’s outputs, not on the reader.

## 3. macOS 27 build, 16 GB to 256 GB

One arm64 binary. RAM size is not a compile-time switch. The same library has to start and produce the same results on a 16 GB Mac and a 256 GB Mac Studio. Scene size and thread count are what change.

Build changes, from `notes/04-openmp-apple.md`:

- Apple Clang, C++20, `USING_R` off, so `PointCloud` is `PointCloudDefault`.
- Link Homebrew `libomp` with `-Xpreprocessor -fopenmp` and `-lomp`. `arbor.pro` today passes GCC’s `-fopenmp` on every Unix host, which Apple Clang rejects. That file needs a macOS branch.
- Do not compile `src/R/` or `src/misc/read_adtree_skeleton.cpp` into this library. Do compile `src/vendor/libqsf`, which `arbor.pro` currently omits while `QSF.cpp` calls it.
- No `#pragma omp target`. Apple GPU cores are not an OpenMP device. See `notes/04-openmp-apple.md`.

Memory behavior the build must allow, without a second binary:

- **16 GB.** Below the book’s 32 GB note for a 5,000–6,000 m² batch. The unit-test scenes (the 9×9 m cloud and a single tree) are the runs that must succeed here. A hectare-scale cloud can exhaust RAM. Default thread count stays at the performance-core count or lower, because each `qsf` thread holds a QSM and each graph thread holds an edge buffer.
- **256 GB.** Same code. Larger batches fit, and more trees can be in flight. Raise `OMP_NUM_THREADS` at run time. Do not bake 256 GB or a thread cap into the compiler flags.

The Swift side can read available memory and set `OMP_NUM_THREADS` before the first Arbor call. That is a runtime default, not a change to the segmentation or QSM algorithms.

## Out of scope for this note

Plots, ArborStudio, a Metal port, and matching lidR’s file filters inside Arbor. Those can sit on top of this API after the tests above pass.
