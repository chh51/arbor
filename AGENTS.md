# Arbor agent notes

Local working copy of [r-lidar/arbor](https://github.com/r-lidar/arbor) **1.1.2**: a C++ forest-scan engine with an R package API. License is **GPL-3**.

Do not change engine code while filling in the investigation notes, unless a note explicitly asks for a code change.

## Notes

Working notes live in `notes/`. Update the status line at the top of a note when that track moves.

| Note | Question |
| --- | --- |
| `notes/01-cpp.md` | What is the C++ engine, and what is actually public? |
| `notes/02-swift-api.md` | Early Swift constraints. Superseded by `notes/05-swift-r-api.md`. |
| `notes/03-macos.md` | Why does the R package run slowly on macOS? |
| `notes/04-openmp-apple.md` | Can OpenMP use this Mac’s CPU, memory, and GPU cores? |
| `notes/05-swift-r-api.md` | Swift API and tests matched to the R package, plus the macOS 16–256 GB build. |
| `notes/06-swift-xcode.md` | Xcode C++ library first, then the Swift processing API, then Swift Testing on the R fixtures. |

## Layout

| Path | Role |
| --- | --- |
| `src/api/arbor.h` | Public C++ entry (`arbor::segment`, `seeds`, `qsm`, `dtm`, `utils`) |
| `src/pointcloud/` | `PointCloud` is `PointCloudDataFrame` when `USING_R`, otherwise `PointCloudDefault` |
| `src/segment/`, `src/seed/`, `src/qsm/`, `src/qsf/`, `src/dtm/`, `src/fitting/` | Pipeline stages |
| `src/R/` | Rcpp adapters. R-only |
| `src/vendor/` | Third-party. Do not edit unless a vendored bug is the task |
| `R/` | R API |
| `src/Makevars` | R package compile (`-DUSING_R`) |
| `arbor.pro` | qmake shared library, C++20, no R |

`NAMESPACE` and `src/RcppExports.cpp` are generated. Do not hand-edit them.

## Builds

```r
# R package, from the repo root
install.packages(".", repos = NULL, type = "source")
testthat::test_local()
```

```bash
# Standalone shared library (qmake). Output: libarbor/
qmake arbor.pro && make
```

The two builds do not compile the same files. Compare `src/Makevars` `OBJECTS` with `arbor.pro` `SOURCES` before assuming a symbol exists in both.
