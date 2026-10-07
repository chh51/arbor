# OpenMP on a high-memory Apple Silicon Mac

**Status:** analysis from the Arbor source and the current LLVM/OpenMP targets. No build has been run on this machine.

Use case: run Arbor on macOS 27 on a Mac, Mac mini, or Mac Studio that has a lot of unified memory and a lot of GPU cores. Can OpenMP use that memory and those GPU cores, and can Arbor’s existing C++ pick that up with almost no source changes?

**Answer:** OpenMP can use the CPU cores and the large RAM. It cannot use the Apple GPU cores. Arbor’s existing `#pragma omp` loops can take the CPU path with a build-flag change and no algorithm change.

## What those machines actually offer

Apple Silicon shares one memory pool between CPU and GPU (unified memory). A Mac Studio can hold far more than the 32 GB the Arbor book asks for. The GPU core count is usually much larger than the CPU core count. The CPU is split into performance cores and efficiency cores.

OpenMP, as Arbor uses it, starts operating-system threads and splits loops across them. Those threads run on the CPU. A thread does not occupy a GPU core.

## Two different OpenMP features

| Feature | Syntax Arbor would need | What it runs on | Available on these Macs |
| --- | --- | --- | --- |
| Host parallelism | `#pragma omp parallel for` and the related clauses already in the tree | CPU threads | Yes, after linking LLVM `libomp` |
| Device offload | `#pragma omp target` and a device runtime (`libomptarget`) | A GPU the compiler can emit code for | No Apple GPU target |

Host OpenMP is [LLVM’s `libomp`](https://github.com/llvm/llvm-project/tree/main/openmp), the library Homebrew installs as `libomp`. Docs: [openmp.llvm.org](https://openmp.llvm.org/). Apple Clang can compile the pragmas; it does not ship this runtime. The usual flags are `-Xpreprocessor -fopenmp`, plus the Homebrew include and lib directories and `-lomp`.

Device offload is a second runtime. LLVM’s supported GPU device architectures are NVIDIA (`nvptx`) and AMD (`amdgcn`). Clang’s support page lists offload to x86_64, AArch64, PPC64, NVIDIA GPUs, and AMD GPUs. AArch64 in that list is the CPU used as an offload device, not the Apple GPU. There is no upstream Metal plugin for `libomptarget`. An experimental LLVM Metal/AIR backend has been proposed and is not an OpenMP target. Apple’s compute API for those GPU cores is Metal.

So “turn OpenMP on” on this Mac means “run Arbor’s loops on several CPU threads.” It does not mean “move those loops onto the GPU cores.”

## Memory

The large unified pool helps Arbor directly. The book’s 32 GB figure is about holding one batch of the point cloud, the graphs, and the QSMs. A 64–192 GB (or larger) machine can hold a bigger batch before the process starts paging. That benefit shows up whether OpenMP is on or off. OpenMP does not have a switch that retunes Arbor for Apple unified memory.

OpenMP does change how much RAM the parallel parts use:

- `GraphBuilder.cpp` gives each thread its own edge buffer, then merges under `#pragma omp critical`. More threads means more live buffers.
- `qsf_api.cpp` builds one QSM per thread (`schedule(dynamic)` over trees). Several large trees in flight at once is the main RAM multiplier.

On a machine with a lot of memory, raising the thread count toward the number of performance cores is reasonable. Past the point where several QSMs no longer fit, extra threads make the run slower. `OMP_NUM_THREADS` is the control. Set it from the shell. Arbor does not need a code change for that.

Apple’s efficiency cores are a poor place for these loops (kNN, graph costs, cylinder fitting). Pinning or limiting threads to the performance-core count is a runtime setting (`OMP_NUM_THREADS`, and if needed `OMP_PROC_BIND` / `OMP_PLACES`). It is not an Arbor source change. OpenMP will not automatically leave the efficiency cores idle.

## Why the GPU cores stay idle

Arbor’s parallel regions are irregular CPU work:

- kNN queries against a nanoflann tree, then a cost calculation, then a thread-local edge list (`GraphBuilder.cpp`)
- point loops with a `reduction` for height above ground (`segment_ground_api.cpp`)
- semantic and instance segmentation loops
- one full QSM build per tree, dynamically scheduled (`qsf_api.cpp`)

That pattern matches CPU threads. A GPU wants a regular numeric kernel over a big array, with little pointer chasing and few locks. `#pragma omp critical` around graph and QSM updates is the opposite of that. Even a working Metal offload plugin would not speed these functions up until the algorithms were rewritten as GPU kernels. That rewrite is a new implementation, not a flag.

## Minimal change so Arbor uses the CPU OpenMP path

The pragmas and `src/myomp.h` can stay as they are. When the compiler defines `_OPENMP` and the link line includes `libomp`, the stubs in `myomp.h` are skipped and the existing loops run in parallel. The same source already runs single-threaded when `_OPENMP` is unset, which is what the R macOS binary does today.

What has to change is the build, not the algorithms:

1. Install the runtime: `brew install libomp` (files under `/opt/homebrew/opt/libomp` on Apple Silicon).
2. Compile and link with Apple Clang roughly as:
   - `-Xpreprocessor -fopenmp`
   - `-I/opt/homebrew/opt/libomp/include`
   - `-L/opt/homebrew/opt/libomp/lib -lomp`
3. For the qmake library, `arbor.pro` currently adds GCC’s `-fopenmp` for every Unix system, including macOS. That flag needs a Mac branch that uses the flags above. That is a build-file edit.
4. For the R package, `src/Makevars` already passes `$(SHLIB_OPENMP_CXXFLAGS)`. On stock macOS R those variables are empty, so a Mac user who wants OpenMP sets them in the R Makevars (or a `Makevars.darwin`) to the `libomp` flags. The CRAN/r-universe binary will stay single-threaded until someone builds from source this way.

No `#pragma omp` needs to become `#pragma omp target`. Adding `target` would be the large change, and there is no Apple device to compile it for.

## What this does not deliver

- The GPU cores on a Mac Studio stay unused by Arbor.
- The book’s “do not use a Mac” line is about the one-thread R binary. A source build linked to `libomp` is outside that warning. It still uses CPU cores only.
- Speed will track performance-core count and memory bandwidth, in the same way it does on a Linux laptop. It will not scale with the GPU core count printed on the spec sheet.

## Sources

- Arbor book, Minimum Requirements: [r-lidar.github.io/arbor_book](https://r-lidar.github.io/arbor_book/)
- LLVM OpenMP runtime and build docs: [openmp.llvm.org](https://openmp.llvm.org/), [Building the OpenMP libraries](https://openmp.llvm.org/Building.html) (GPU device architectures listed there are AMD and NVIDIA)
- Clang OpenMP support: [clang.llvm.org/docs/OpenMPSupport.html](https://clang.llvm.org/docs/OpenMPSupport.html)
- OpenMP specification: [openmp.org/specifications](https://www.openmp.org/specifications/)
