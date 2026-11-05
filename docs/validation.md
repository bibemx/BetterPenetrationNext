# Validation scope — 1.0.9

User visual/stability acceptance was received on **2026-11-05** (Asia/Irkutsk). This accepts the tested scene and settings; it is not clinical validation or a guarantee for every animation.

The accepted simulation DLL was SHA256 `E77EDF397679A06820A91A30D95EA381B1BD28C57D70DB8DE4722FFC97226A9B`. Publication keeps its solver and geometry implementation. Packaging aligns embedded defaults and B/N shortcuts with the tested configuration; the release DLL is rebuilt and its own hash is listed in the release checksums.

The packaged defaults use the user's last saved configuration at release acceptance: SourceLengthScale **0.7727273**, CanalLengthScale **1.384297**, ContactDamping **5.419422**. The instrumented before/after timing and motion traces below used the earlier **0.8367769 / 1.421488 / 1** values. They describe that measurement, not a benchmark of the later user-adjusted preset. Native configuration tests are repeated with the actual packaged defaults.

| Check | Scope/result |
|---|---|
| Core regression | 173 tests: constraints, open shells, separate seam identities, finite source BVH cuts, angular wrapping, active sector release, recovery and allocation checks |
| Recorded geometry | 74 actual captured Source/Receiver states with skin matrices, morphs and frozen settings; production `ReactiveReceiverRig.Step` |
| Entrance regression | 11,026 central aperture rays on those states, no exterior bridging hits; zero release jump and final recovery error |
| Native Unity | Unity 2019.4.9f1: 18 passes on actual meshes, UV seam/skin identities, Blend Shapes and Step → MeshDeformationBinding → BakeMesh integration; user acceptance was a separate check |
| Active user scene | Same animation state and speeds before/after; 450 render frames without geometry capture during the trace; 8 subsequent native surface probes and 32 active result states |

Actual warm-run material-tip acceleration relative to the animated base: maximum **14.63 → 4.64**, 95th percentile **2.85 → 2.31** Unity units/s². This measurement includes animation motion and must not be labelled entirely numerical jitter.

Full-frame intervals varied: before mean/p95 **29.77/36.38 ms**, first after **25.68/29.57 ms**, warm repeat **31.45/37.37 ms**. The first after trace included one 305 ms frame. Background load, focus and GPU waits were not held experimentally identical, so no full-frame performance gain is established.

A separate .NET replay of 2,045 C1-interpolated states derived from the 74 captures measured **10.88 → 10.13 ms** per numerical step and zero warm managed allocations. These are interpolated test states, not additional measured Runtime frames or KKS FPS.

Remaining limits: the general minimum-contour-radius proxy still reported up to 8.15 mm residual in the sampled active scene; it is not an independent full native-mesh containment test. Complete coverage in every pose, absence of all self-intersections, all-card compatibility and clinical tissue properties are not established. Material parameters and the 4.5-second recovery are configurable visualization assumptions.

The clean-source build, regenerated defaults and installer fresh/upgrade/rollback checks are repeated for publication. The working user installation is not overwritten by the publication process.
