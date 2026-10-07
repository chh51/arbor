# 1. C++ analysis

**Status:** inventory only. Not a finished reading of the algorithms.

Arbor processes mobile / terrestrial laser scans of forest: ground, wood vs foliage, individual trees, then a quantitative structure model (QSM) per tree. The engine is rule-based C++. R is the supported API.

## Two compile modes

`src/pointcloud/PointCloud.h` switches the point type:

- `-DUSING_R` (R package, `src/Makevars`): `PointCloud` is `PointCloudDataFrame`, backed by Rcpp.
- Otherwise (`arbor.pro`): `PointCloud` is `PointCloudDefault`, a `std::vector` container. That header uses `std::span` (C++20).

`src/api/arbor.h` is the non-R entry. It does not include Rcpp. Callers pass `PointCloud` and `settings::ArborParameters`.

## Public C++ surface (`arbor.h`)

| Namespace | Functions | Inputs |
| --- | --- | --- |
| `arbor::segment` | `segment_ground`, `segment_semantic`, `segment_instance`, `dist2root` | cloud, optional seeds or DTM, parameters |
| `arbor::seeds` | `find_seeds` | cloud, parameters |
| `arbor::qsm` | `qsm`, `qsf` | cloud, parameters (`qsf` also takes `min_height`) |
| `arbor::dtm` | `dtm` | cloud |
| `arbor::utils` | `homogeneization`, `sor`, `anisotropy`, `smooth3d` | cloud |

Parameters are nested structs (`Global`, `Woodlikelihood`, `Graph`, `Semantic`, `Seed`, `Instance`, `Qsm`) with a template `visit()` used by the R binding. `slice_at` on seeds is a `std::vector<double>` and is not visited.

Services (`src/utils/services.h`): a process-wide `ServiceLocator` for a logger and a progress-bar factory. The engine calls these; it does not take them as arguments.

## Modules (R object list in `src/Makevars`)

| Directory | Role |
| --- | --- |
| `pointcloud/` | Point storage, attributes, nanoflann KD-tree |
| `segment/` | Ground, semantic, instance segmentation, graph |
| `seed/` | Stem-slice seed detection |
| `qsm/` | Skeleton, radius, pipes, mesh, read/write |
| `qsf/` | Forest of QSMs; file I/O uses `std::filesystem` and `libqsf` |
| `dtm/` | Terrain |
| `fitting/` | Circle, ellipse, polynomial, Fourier fits |
| `utils/` | kNN, SOR, grid, anisotropy, allometry, progress |
| `vendor/` | nanoflann, dbscan, hporro, ptd, libqsm, libqsf |
| `R/` | Rcpp only. Not part of `arbor.pro` |
| `experimental/`, `misc/` | In the R object list. Not in `arbor.pro` |

## `arbor.pro` vs `src/Makevars`

`arbor.pro` builds a shared library with `-std=c++20` and `-DNOGDAL`. It compiles `src/pointcloud/*.cpp` and the stage directories, and it does **not** list:

- `src/vendor/libqsf/*.cpp` — but `src/qsf/QSF.cpp` calls `libqsf`
- `src/experimental/`
- `src/misc/read_adtree_skeleton.cpp` (includes Rcpp)
- anything under `src/R/`

Whether that qmake file links today is an open check for track 3, not a conclusion.

## Still to read

- [ ] Attribute set on `PointCloudDefault` / `PointCloudBase` (what a Swift caller must fill before each stage)
- [ ] Ownership: which APIs mutate the cloud in place vs return a new cloud or `QSM`
- [ ] What `QSM` and `QSF` contain after `qsm()` / `qsf()`, and how they write `.qsm` / `.qsf`
- [ ] Which stages require which attributes (tree id, pwood, height above ground, DTM)
- [ ] Threading: where `#pragma omp` runs, and what `ServiceLocator` assumes about threads
- [ ] Pipeline order the R package uses (`R/cmd_segment.R`, `R/qsf.R`) vs what `arbor.h` allows out of order
