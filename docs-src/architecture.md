# Architecture

This page covers how polka is built inside: the source adapters, the filters, the CPU and CUDA merge engines and the thread model. The order of the stages is in [Pipeline](pipeline.md), parameters are in [Configuration](configuration.md), the motion model is in [Deskew](deskew.md) and the numbers are in [Performance](performance.md). File paths below are relative to the repository root and link to the `humble` branch.

## Components

polka is one node, `polka::PolkaNode`, registered as a component and also built as a standalone executable. It owns everything else.

| Part | Files | Job |
|---|---|---|
| Node | [`src/polka_node.cpp`](https://github.com/Pana1v/polka/blob/humble/src/polka_node.cpp) | Output timer, TF lookups, engine choice, publishing, runtime reconfigure, diagnostics |
| Source adapter | [`src/source_adapter.cpp`](https://github.com/Pana1v/polka/blob/humble/src/source_adapter.cpp) | One per source: subscribe, validate, convert, stamp point times, deskew, filter, store |
| Filters | [`src/filters/`](https://github.com/Pana1v/polka/tree/humble/src/filters) | Range, angular and box filters behind one interface |
| Merge engines | [`src/merge_engine/`](https://github.com/Pana1v/polka/tree/humble/src/merge_engine) | `CpuMergeEngine` and `CudaMergeEngine` behind `IMergeEngine` |
| Output | [`src/output/`](https://github.com/Pana1v/polka/tree/humble/src/output) | `OutputPipeline` (CPU post-merge work) and `ScanBuilder` (LaserScan messages) |
| Types | [`include/polka/types.hpp`](https://github.com/Pana1v/polka/blob/humble/include/polka/types.hpp) | Point type and config structs |

The internal point type is `polka::PointXYZIT`: `x`, `y`, `z` with PCL's SSE padding, `float intensity`, and `double time`. `time` holds each point's absolute acquisition time in seconds. It is a double because a Unix time in seconds needs more than float precision to keep microseconds. It is rebased to an offset from the header stamp, or dropped, only at publish time.

## The output tick

All merging happens in `PolkaNode::output_callback()`, which a wall timer fires at `output_rate` Hz. Sources never trigger a merge. They only replace their latest cloud. Each tick:

1. **Gather.** For each source, take its latest cloud. A source older than `source_timeout` is still used, with its last cloud and last TF, until it is also older than `source_stale_reuse_window`. Then it is dropped from the merge.
2. **Place.** Look up the source frame in `output_frame_id` at the cloud's own stamp. If TF has not reached that stamp yet (`ExtrapolationException`), use the newest transform. Any other TF error falls back to the last transform that worked for that source. A source whose TF has never resolved therefore merges with an identity transform until it does.
3. **Stamp.** Pick the output stamp from the source stamps with `timestamp_strategy`. With `suppress_duplicate_timestamps` on, a tick whose stamp equals the last published one publishes nothing.
4. **Align.** With motion compensation on, each source transform is pre-multiplied by a motion delta over `source stamp - output stamp`. The model is described in [Deskew](deskew.md).
5. **Merge and publish.** On the GPU, one `merge_pipeline()` call does the merge and every output stage. On the CPU, `merge()` is followed by `OutputPipeline::process()` and `ScanBuilder::from_cloud()`.

## Thread model

polka does not create threads of its own. Every callback (source subscriptions, IMU and odometry subscriptions, the output timer, the diagnostics timer, the parameter callback and the one-shot reconfigure timer) sits in the node's default mutually exclusive callback group. The standalone executable spins with `rclcpp::spin()`, which is single-threaded. In a multithreaded component container the callback group still runs one callback at a time. So a source callback and the output tick never run at the same time.

The shared state is still guarded, so the code stays correct if that ever changes:

- **Latest cloud.** The adapter publishes its cloud with `std::atomic_store` on a `shared_ptr`, and the tick reads it with `std::atomic_load`. Latest wins: three messages between ticks means the tick sees the third. A cloud the tick holds stays alive after the adapter replaces it, because the tick holds its own reference. libstdc++ implements these free functions with a small internal lock pool, so they are short critical sections, not lock-free.
- **Source metadata.** Frame id and last stamp sit behind a per-adapter `std::mutex`.
- **IMU and odometry.** [`ImuBuffer`](https://github.com/Pana1v/polka/blob/humble/src/imu_buffer.cpp) and [`OdomBuffer`](https://github.com/Pana1v/polka/blob/humble/src/odom_buffer.cpp) keep their ring buffers behind a mutex.

Runtime reconfigure runs in two phases. The on-set callback only validates, because Humble calls it before the new values are committed. On success it arms a zero-delay timer that re-reads the committed parameters and applies the diff on the executor, between ticks.

## Source adapters

Each source gets one `SourceAdapter` ([`include/polka/input/source_adapter.hpp`](https://github.com/Pana1v/polka/blob/humble/include/polka/input/source_adapter.hpp)). It subscribes with the source's QoS (`best_effort` or `reliable`, history depth) to either a PointCloud2 or a LaserScan topic. PointCloud2 goes through `point_cloud_transport` when polka is built with it, and a plain subscription otherwise.

`pc2_callback()` validates fields and detects the time field on the first message, converts with `pcl::fromROSMsg()`, fills each point's `time`, deskews in the sensor frame if enabled, runs the per-source filters (CPU engine only), and stores the cloud.

### Field validation

The first message from each source is checked for `x`, `y` and `z` fields of type `FLOAT32`. If any is missing, the adapter logs an error once and drops every later message from that source. Diagnostics report the source as an error from then on. This catches a wrong topic or an unusual driver at startup, instead of failing inside the PCL conversion on every message. A missing `intensity` field is only a warning.

### Per-point time

`time` is filled before any filter runs, so it survives the same compaction as the coordinates. Each point gets `header stamp + decoded offset`, or just the header stamp when the source has no usable time field. A field whose offsets decode to more than a plausible scan length is rejected, since a misread unit would throw points across the map. Decoding is in [`include/polka/input/point_time_decoder.hpp`](https://github.com/Pana1v/polka/blob/humble/include/polka/input/point_time_decoder.hpp).

### LaserScan sources

`scan_callback()` projects the scan with `laser_geometry::LaserProjection::projectLaser()`. Each reading becomes `(r cos θ, r sin θ, 0)` in the scan's frame. Readings outside `[range_min, range_max]`, including `inf` and `NaN`, are dropped by the projection. The projected cloud then goes through the same field validation and filters as a PointCloud2. A projected scan has no per-point time, so every point gets the header stamp, and LaserScan sources are never deskewed per point. Diagnostics count bandwidth as the LaserScan received, not the larger projected cloud.

## Filters

### IFilter and in-place compaction

All CPU filters implement one interface ([`include/polka/filters/i_filter.hpp`](https://github.com/Pana1v/polka/blob/humble/include/polka/filters/i_filter.hpp)):

```cpp
class IFilter
{
public:
  virtual void apply(CloudT & cloud, const std::string & frame_id) = 0;
  virtual ~IFilter() = default;
};
```

`build_filter_chain()` in [`filter_chain.hpp`](https://github.com/Pana1v/polka/blob/humble/include/polka/filters/filter_chain.hpp) turns a `FilterParams` into a vector of filters: range, then angular, then box, each only if enabled. The angular filter is also skipped when its range list is empty.

Every filter compacts the cloud in place with a read cursor and a write cursor:

```cpp
size_t j = 0;
for (size_t i = 0; i < cloud.size(); ++i) {
  if (keep(cloud[i])) {
    cloud[j++] = cloud[i];
  }
}
cloud.resize(j);
cloud.width = j;
cloud.height = 1;
cloud.is_dense = true;
```

It is one O(n) pass with no allocation. The write cursor never passes the read cursor, so no unread point is overwritten. Kept points stay in their original order. The output is always unorganized (`height = 1`).

### Range filter

[`src/filters/range_filter.cpp`](https://github.com/Pana1v/polka/blob/humble/src/filters/range_filter.cpp) compares squared 3D distance against squared bounds computed once in the constructor, so there is no `sqrt` per point. A point with a `NaN` coordinate fails both comparisons and is dropped.

### Angular filter

[`src/filters/angular_filter.cpp`](https://github.com/Pana1v/polka/blob/humble/src/filters/angular_filter.cpp) keeps or drops points by azimuth in the XY plane. Ranges are given in degrees in `[0, 360]`. A range with `lo > hi` wraps through 0°, so `[270, 90]` is the front half.

It does not compute `atan2` per point. The constructor stores the unit vectors of each range's bounds, and the test is two 2D cross products:

```
cross_lo = lo.x * y - lo.y * x     // >= 0: point is counterclockwise of lo
cross_hi = hi.x * y - hi.y * x     // <= 0: point is clockwise of hi
inside   = span > 180° ? (cross_lo >= 0 || cross_hi <= 0)
                       : (cross_lo >= 0 && cross_hi <= 0)
```

A span over 180° is not a convex wedge, so it is the union of two half-planes instead of the intersection. `invert` keeps the complement. The CUDA kernels use the same test. The CPU speedup from dropping `atan2` is in [Performance](performance.md).

### Box filter and self-filter

[`src/filters/box_filter.cpp`](https://github.com/Pana1v/polka/blob/humble/src/filters/box_filter.cpp) is an axis-aligned box test with an `invert` flag. A point is kept when `inside != invert`:

| inside | invert | result |
|---|---|---|
| yes | no | keep |
| no | no | drop |
| yes | yes | drop |
| no | yes | keep |

The per-source and output box filters use `invert = false` and crop to the box. The self-filter (robot footprint) is the same class with `invert = true`: `OutputPipeline` appends one inverted `BoxFilter` per exclusion box after the output chain. Several boxes compose because each one removes only its own volume.

## CPU merge engine

[`src/merge_engine/cpu_merge_engine.cpp`](https://github.com/Pana1v/polka/blob/humble/src/merge_engine/cpu_merge_engine.cpp) is a single-threaded loop:

1. Sum the input sizes and `resize()` the output once.
2. For each source, cast its `Eigen::Isometry3d` to `Eigen::Affine3f` once, not once per point.
3. Transform each point with that matrix. Copy `intensity` and `time` unchanged, since a rigid transform changes neither.
4. Skip points with a non-finite coordinate, then shrink the output to the points written.

Per-source filters have already run in the adapter, in the sensor frame. After the merge, `OutputPipeline::process()` ([`src/output/output_pipeline.cpp`](https://github.com/Pana1v/polka/blob/humble/src/output/output_pipeline.cpp)) runs the output filters, self-filter boxes, height cap and `pcl::VoxelGrid` in the output frame. `ScanBuilder::from_cloud()` ([`src/output/scan_builder.cpp`](https://github.com/Pana1v/polka/blob/humble/src/output/scan_builder.cpp)) flattens to a LaserScan. It keeps points inside the flatten Z band, bins them by `atan2(y, x)`, and keeps the nearest XY range per bin. Empty bins are `inf`.

`pcl::VoxelGrid` averages each voxel, `time` included, and its centroid math is float. With per-point timestamps and voxel both on, the CPU path warns once that `time` loses precision.

## CUDA merge engine

[`src/merge_engine/cuda_merge_engine.cu`](https://github.com/Pana1v/polka/blob/humble/src/merge_engine/cuda_merge_engine.cu) is built when CMake finds a CUDA compiler and `WITH_CUDA` is on (the default). GPU constants are in [`include/polka/merge_engine/cuda_types.cuh`](https://github.com/Pana1v/polka/blob/humble/include/polka/merge_engine/cuda_types.cuh).

### Build, setup and fallback

- **Architectures.** Code is built for sm_75, sm_86, sm_87 and sm_89, plus sm_89 PTX so newer GPUs can JIT-compile it.
- **Runtime.** `polka_cuda` links `CUDA::cudart_static`. A CUDA build needs only the NVIDIA driver at run time, not a CUDA toolkit or `libcudart`.
- **Engine choice.** At construction, with `enable_gpu: true` (read-only), the node calls `cudaGetDeviceCount()`. With no device it logs a warning and uses the CPU engine. With a device it builds `CudaMergeEngine`.
- **Setup failure.** Every allocation and the stream creation in the constructor go through `POLKA_CUDA_REQUIRE`, which throws `std::runtime_error`. The constructor frees what it already allocated and rethrows. The node catches the error, logs `CUDA device unusable (...), falling back to CPU`, and builds `CpuMergeEngine`. This covers a GPU that is present but busy or unavailable, as seen after suspend and resume.
- **Errors after setup.** Copies and kernel launches use `POLKA_CUDA_CHECK`, which prints to stderr and carries on. There is no switch to the CPU engine mid-run.
- **Source set changes.** Device buffers are sized from the source count. When a runtime reconfigure changes `source_names`, the CUDA engine is rebuilt. That rebuild has no CPU fallback.

### Preallocated buffers

Everything is allocated once in the constructor and reused every tick. The capacity is 200,000 points per source, times the number of sources.

| Buffer | Size | Use |
|---|---|---|
| `d_buf_a`, `d_buf_b` | 16 B per point each | `float4 {x, y, z, intensity}`, ping-pong between stages |
| `d_time_a`, `d_time_b` | 8 B per point each | `double` time, parallel to the point buffers |
| `d_count_a`, `d_count_b` | 4 B each | Output counters |
| `d_tf` | 64 B | Current source transform |
| `d_voxel_keys`, `d_voxel_points`, `d_voxel_time` | 2^18 slots, 28 B per slot | Voxel hash table |
| `d_scan_ranges_int`, `d_scan_ranges_float` | 4,096 bins, 4 B each | Scan flatten |
| `h_buf_in/out`, `h_time_in/out` | Same as the device point and time buffers | Pinned host staging |

That is about 9.6 MB of device memory per source, plus 7.3 MB for the voxel table, plus the same 9.6 MB per source of pinned host memory. Two sources come to roughly 27 MB on the device.

The host buffers are pinned (`cudaMallocHost`) because `cudaMemcpyAsync` from pageable memory is staged through a driver-side pinned copy anyway. Pinning removes that hidden copy and lets the transfer run without blocking the host. A tick still allocates a few small host vectors and the output `CloudT`, but nothing on the device.

### Per-source kernel

`merge_pipeline()` packs every source into `h_buf_in` and `h_time_in` in one pass. Intensity goes into the `w` lane of each `float4`, and time goes into the parallel double array. One host-to-device copy then moves the whole tick's points, with a second copy for the times.

Then, per source, the host flattens the transform into `d_tf` and launches `fused_transform_filter_kernel` with 256 threads per block, one thread per point:

1. Range test on the squared distance, if the range filter is enabled.
2. If it passes, the angular cross-product test and the box test.
3. If all pass, transform with the 3x4 matrix and keep the point with its time.
4. Reserve an output slot and write.

The per-source filters run on the point before the transform, in the sensor frame, the same as on the CPU path. Filter parameters go to the kernel by value as a `GpuFilterParams` struct, with angular bounds precomputed on the host as cosine and sine pairs.

### Slot reservation

Threads do not know ahead of time how many points pass, so each kept point reserves its output index with an atomic add. This is done in two levels to keep contention on the global counter low:

```cpp
__shared__ int local_count;   // per block
__shared__ int output_base;
...
if (keep) local_idx = atomicAdd(&local_count, 1);          // shared memory
__syncthreads();
if (threadIdx.x == 0 && local_count > 0)
  output_base = atomicAdd(output_count, local_count);      // one global add per block
__syncthreads();
if (keep) output[output_base + local_idx] = out_point;
```

`atomicAdd` returns the old value, so no two threads get the same index. It is one pass and one launch, with no prefix-sum stage. Output order depends on block scheduling, so GPU output order is not deterministic. The output filter kernel uses the same pattern. Voxel compaction does a plain global `atomicAdd` per occupied slot.

### Transform layout

Eigen matrices are column-major. The host copies the 4x4 matrix into a row-major `float[16]`, and the kernel multiplies by the top 3x4 only, since the bottom row of a rigid transform is `[0 0 0 1]`:

```cpp
float ox = tf[0]*p.x + tf[1]*p.y + tf[2]*p.z  + tf[3];
float oy = tf[4]*p.x + tf[5]*p.y + tf[6]*p.z  + tf[7];
float oz = tf[8]*p.x + tf[9]*p.y + tf[10]*p.z + tf[11];
```

`d_tf` is a single 64-byte buffer overwritten for each source. That is safe because the copy and the kernel that reads it are queued on the same stream, which runs them in order.

### Output stages on the GPU

After the merge, the merged points stay on the device for the remaining stages, swapping between the `a` and `b` buffers:

- **`output_filter_kernel`.** One pass for output range, height cap, output box, up to 8 self-filter boxes and output angular. It runs only if at least one of those is enabled.
- **`voxel_insert_kernel` and `voxel_compact_kernel`.** Each point hashes its voxel index into a 2^18-slot table with `atomicCAS` and up to 16 quadratic probes. The first point to claim a voxel is kept as is, with its own time. The compact kernel then copies occupied slots out. This keeps one real point per voxel, not a centroid. A point that finds no free slot within 16 probes is dropped, and two voxels whose hashes collide merge into one.
- **`flatten_kernel` and `scan_decode_kernel`.** Run only when the scan output is enabled. Each point in the Z band does an `atomicMin` on its bin, using the float range reinterpreted as an int. That is valid because non-negative IEEE floats order the same as their bit patterns. Bins are seeded with `range_max`, and the decode kernel turns any bin still at that value into `inf`. At most 4,096 bins are used.

### Stream and synchronization

All work goes on one CUDA stream created in the constructor. The host must synchronize whenever it needs a count to size the next launch: after the merge, after the output filter, and after voxel compaction. One final synchronization covers the device-to-host copies of the points, times and scan ranges into the pinned buffers. A tick with every stage on synchronizes four times. The host then unpacks the `float4` lanes back into `PointXYZIT`.

`CudaMergeEngine::merge()` also exists and does the merge only, with per-call vectors instead of the pinned buffers. The node never calls it: on the GPU it always uses `merge_pipeline()`.

### Limits

| Limit | Value | When exceeded |
|---|---|---|
| Points per source | 200,000 | Truncated, with a warning |
| Total points | 200,000 x sources at construction | Truncated, with a one-time warning |
| Angular ranges per filter | 8 | Truncated, with a warning |
| Self-filter boxes | 8 | Truncated, with a warning |
| Voxel table | 262,144 slots, 16 probes | Points that find no slot are dropped |
| Scan bins | 4,096 | Bins beyond are not produced |

## CPU and CUDA differences

| | CPU engine | CUDA engine |
|---|---|---|
| Per-source filters | In the source callback | In the merge kernel |
| Per-source range when disabled | Not applied | Bounds still applied |
| Output order | Deterministic, source by source | Depends on block scheduling |
| Non-finite points | Skipped in merge | Fail the range test |
| Voxel | `pcl::VoxelGrid` centroid, float `time` | First point per voxel, exact `time` |
| Scan flatten | `ScanBuilder::from_cloud()` | `flatten_kernel`, ranges returned to the host |
| Diagnostics kept-point count | Reported | Unknown, reported as -1 |

Whether the GPU is faster depends on how much per-point work the pipeline does. See [Performance](performance.md).
