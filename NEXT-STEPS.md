# Next steps

What `hypxrcompose` v1 does not do, why, and what closing each gap needs. The framing is research
27's: v1 is the reprojection composite (§5), v2 is grade-B replay, rolling-shutter correction, and
depth-assisted eye reprojection (§5, §5.1, §3 footnote 3).

Everything here is upgradeable against takes recorded today. That is the property the architecture
was chosen for: nothing in a bundle is derived, so a better compositor replays an old take better.
The only irreversible decisions are capture-side ones — telemetry not recorded is gone forever.

---

## Gap 1 — Grade-B replay: re-render the quads instead of reusing the matte

**What v1 does.** The overlay is a recorded RGBA image per eye. The compositor warps it into the
output camera and blends it. The overlay's resolution, its content, and its per-monitor styling are
all baked in at capture time.

**What v2 wants.** Research 27 §2 (S2 grade B): screencopy each XR monitor's headless output plus the
quad pose/size/visibility telemetry, and have the compositor re-render the quads itself. That is what
makes "re-run the take at 4K", "make the monitors more transparent", and novel camera paths possible,
because the foreground stops being an image sampled from one viewpoint and becomes geometry.

**What it needs.**

1. *Bundle*: **the quad records now exist and are recorded** — `telemetry.jsonl` carries a `quads`
   array per record with `index`, `name`, `pose`, `size`, `visibility`, `view_space`, `swapchain`,
   `image`, `array_layer`, and `rect`. `hypxrcompose` parses and validates them today without
   compositing them, so a v2 starts from checked data. Two semantics must be honoured exactly, and
   both are easy to get wrong:

   - **`pose` is head-relative.** The layer's STAGE pose at time *t* is `head(t) ∘ pose`. Reading it
     as a world pose puts every layer in the wrong place the instant the wearer moves.
   - **`view_space` decides what that means over time.** `true` = head-locked: the layer stays at
     this head-relative pose. `false` = room-anchored: the layer had a fixed STAGE pose, and what was
     recorded is that pose relative to the head at *t*, so a replay must re-anchor rather than carry
     it along. The synthetic bundle contains one of each, and the test suite asserts the
     room-anchored one re-anchors to a constant STAGE pose across the whole take.

   `swapchain` is stable within a session only, so it identifies a texture *within* a take and must
   not be persisted across takes. What is still missing is the pixels: a `monitors/<name>.mp4` per XR
   monitor with screencopy timing, plus the mapping from `(swapchain, image, array_layer, rect)` to a
   region of one of those files. **That part is capture-side and should be recorded now.**
2. *Compositor*: a textured-quad rasterizer. The existing kernel is a full-screen inverse warp with
   no geometry stage; grade B needs an actual draw per quad with depth sorting (or back-to-front
   painting, which for a handful of quads is simpler and exact), and the layer blend the session used.
3. *Choice to make*: with grade B available, does grade A stay? Yes — it is the ground truth the
   re-render is checked against. A useful acceptance test is "grade B re-rendered into the recording
   eye's pose and fov must match grade A to within resampling error", which the synthetic bundle can
   express directly by having `synth` emit both.

**Effort.** The rasterizer is small. The telemetry schema extension and getting the compositor's
notion of a quad to agree with HypXRland's are the real work. Two or three rounds.

---

## Gap 2 — Rolling shutter

**What v1 does.** Nothing. Each camera frame is treated as instantaneous, sampled at the pose that
held at `t_device_ns + exposure_ns/2`. Under fast head rotation the image is skewed across its
readout, exactly as it is in every Quest capture today (research 27 §3 footnote 3).

**What v2 wants.** Per-scanline pose correction: the pose used for a background sample should depend
on the *row* the sample lands on, not on one instant per frame.

**What it needs.**

1. *Bundle*: the readout direction and the readout duration per camera (Android exposes
   `SENSOR_ROLLING_SHUTTER_SKEW`), in the camera calibration header. Plus a pose track at IMU rate
   rather than at render rate — poses are cheap, and this is the other thing that must be **recorded
   now** to be correctable later. A `poses.jsonl` at 500–1000 Hz alongside `telemetry.jsonl` would do
   it.
2. *Compositor*: in the background sampler, replace the single `uCamRotInv`/`uCamPos` with a pose
   evaluated at `t_frame + skew * (row / height)`. Because the kernel is an inverse warp, the row in
   question is the *source* row, so the sample needs one iteration: project with the frame-centre
   pose, take the resulting row, re-evaluate the pose there, project again. Two iterations converge
   for any plausible skew.
3. *Test*: `synth` renders its camera frames instantaneously today. Give it a `--rolling-shutter-ms`
   that renders each row from the pose at that row's instant, and the existing marker assertions
   immediately become a rolling-shutter test: without correction the marker misses by a predictable
   amount, with correction it does not.

**Effort.** The compositor change is contained. The IMU-rate pose track is a host-side capture change
and the skew figure is a device-side characterization item.

---

## Gap 3 — Depth-assisted reprojection to the true eye positions

**What v1 does.** Assumes a single depth for the whole background (`--bg-depth`, default 2.0 m) and
reprojects the camera image onto that sphere. The passthrough cameras sit near but not at the eyes,
so content away from the assumed distance carries a scale/parallax error. Research 27 §5.1 accepts
this explicitly; it is the artifact every Quest recording already has.

**Measured, on the synthetic rig** (48 mm eye-to-lens offset, wall at 2 m, assumed depth 2 m,
480×360 pane): worst-case **0.94 px**. That is the residual from the sphere-versus-plane mismatch
alone. Deliberately using a wrong assumed depth (10 m instead of 2 m) moves the image by a few
pixels, which the test suite asserts against a closed-form prediction — so the error scales the way
the geometry says it should, and a real room with content from 0.3 m to 10 m will show
correspondingly more.

**What v2 wants.** A per-pixel depth from the stereo camera pair, and reprojection to the true eye
position instead of onto a sphere.

**What it needs.**

1. *No bundle change.* Both cameras and their extrinsics are already recorded; this is purely a
   compositor upgrade, which is the point of recording sources rather than composites.
2. *Compositor*: stereo matching between the two rectified camera images (a coarse block matcher or
   semi-global matching is enough — this is a background, not a metrology task), then a per-pixel
   depth in place of the scalar `uBgDepth`. The kernel already takes a depth per sample; it becomes a
   texture lookup rather than a uniform.
3. *Disocclusion*: reprojecting to a different centre exposes surfaces the camera never saw. v1's
   single depth cannot produce holes; a per-pixel depth can. The cheap answer is to fall back to the
   assumed-depth sample in holes, which degrades to exactly v1 behaviour locally.
4. *Test*: give `synth` a second plane at a different depth (a "table" in front of the wall). The
   existing marker predictions then differ measurably between the v1 and v2 models, and the test can
   assert that v2 lands markers on both planes where v1 only manages the one at the assumed depth.

**Effort.** The largest of the three, and the one with the most tuning. It shares its machinery with
gap 2 (both are "the pose/depth for a sample is not constant across the frame").

---

## Smaller gaps

| Gap | Note |
|-----|------|
| **`stage_correction` is recorded but never applied.** | v1 reads the stamped eye poses as already corrected (research 27 §3 footnote 2 speaks of "the *applied value* per frame"). If the producers' semantics turn out to be "the correction still to apply", `Render.cpp` needs one composition and `validate` needs to say so. Until the producers settle this, it is the interpretation most likely to be wrong. |
| **Output camera is one of two presets.** | The framing menu research 27 §5 describes — fixed tripod, orbit, arbitrary keyframed path — is a `SPose` per output frame, which the render loop already consumes. It needs a path format and a CLI, not new geometry. |
| **MV-HEVC / spatial video output.** | `--eye stereo-sbs` covers the immediately shareable case. MV-HEVC is a mux-time concern (§5.1) and lands in `Ffmpeg.cpp`'s writer spec, not in the kernel. |
| **Nearest-neighbour source resampling.** | Output frames pick the nearest source frame in time. At 30 Hz cameras against 45 Hz output that visibly stutters, and the same applies to overlay frames lost to the readback queue. Motion-compensated interpolation is the fix; simple frame blending is not (it doubles edges). Until then, matching `--fps` to the camera rate is the honest option for camera-dominant cuts. |
| **A dropped overlay frame is held, not synthesized.** | When a record carries `"dropped": true` the composite reuses the nearest surviving overlay frame, reprojected from *its* pose. That is correct rather than merely convenient - the pixels really were rendered from that viewpoint - but a long readback stall shows as a static overlay over a moving background. Grade-B replay (gap 1) removes the problem entirely, since it re-renders rather than resamples. |
| **Straight-alpha edge bleed.** | A `"premultiplied"` bundle — what the host producer emits — filters correctly by construction. A `"straight"` one still bleeds unassociated colour at matte edges under bilinear sampling, because association happens after the filter. Associating on upload would fix it, at the cost of a conversion pass. |
| ~~**Per-frame fov is taken from the nearest telemetry record.**~~ — **fixed: the output frustum is derived once and held.** | The output camera's frustum is now built from the union of every frustum the eye recorded, padded to the pane's aspect, and fixed for the whole render (`deriveOutputFrusta`). So it neither jitters with the per-record stamp — the reference take stamps 2006 distinct fovs for one eye — nor needs interpolating. The per-record fov is still used where it belongs: to sample the overlay, which was rendered with it. |
| **Colour management is sRGB-only.** | Compositing is now correct: sources decode from sRGB to linear light (in hardware, before filtering), blend premultiplied, and encode once on output. What is still assumed is that *everything is sRGB* — a wide-gamut or HDR overlay tap would need its actual primaries and transfer function in the manifest, and the output would need somewhere to put values above 1.0. |
| **libav\* linkage.** | v1 talks to `ffmpeg` over pipes deliberately (see README). A v2 that wants hardware decode straight into a GL texture — which is where the throughput ceiling is — should link libav\*; the seam is `src/Ffmpeg.cpp`. See "Throughput" below for why this is now the *first* thing to do rather than the last. |

---

## Output framing: two frusta, two consumers (fixed 2026-08-17, corrected after a headset viewing)

A recorded eye frustum is asymmetric and its angular aspect is whatever the runtime picked. On the
reference take, eye 0 is `l=-0.9425 r=0.6981 u=0.7679 d=-0.9599`: an angular aspect of 0.9255 and an
optical axis at **62.1%** of the width (eye 1 mirrors it at 37.9%). v1 handed that fov straight to
the shader, which maps it linearly onto whatever pane was asked for — so the picture was stretched by
the ratio of the two aspects: **3.1%** at the take's own buffer size, **1.92x** at 1920x1080.

The first fix built each eye's output camera from its own recorded frustum, padded to the pane. That
is geometrically the truest record and it is **wrong for viewing**, which only showed up in a
headset: with the optical axes preserved at 62.1% and 37.9%, a feature at infinity lands **349 px
apart** at 1440 per eye. The wearer reported each eye's content sitting to one side, the frames
disagreeing at the edges, and fusion breaking. A constant disparity at infinity is not something a
viewer can fuse, and frame edges at different visual angles fight the content.

So there are two modes, because there are two consumers:

| `--frustum` | What it does | For |
|---|---|---|
| **`presentation`** (default) | **One** frustum for both eyes: the per-edge median across records, intersected across eyes, symmetrized about forward, cropped to the pane. Each eye renders through it from its **own position** but a **common orientation** (a parallel rig). | Flat SBS viewing, any player, a person fusing two panes. Parallax is carried by the content and never by frame placement. |
| `recorded` | Each eye keeps its own asymmetric frustum, padded to the pane, both at one scale. | Analysis, and a headset-native player that re-projects per eye. |

Measured on the reference take at 1440x1080 per eye, presentation mode: shared frustum
`l=-0.6981 r=0.6981 u=0.5617 d=-0.5617`, 858.1 px per tangent, optical axis at the pane centre,
**disparity at infinity 0.0 px** (was 349), and content parallax of 39.8 px at 1.48 m, 29.4 px at
2 m, 11.8 px at 5 m. Vertical disparity is zero by construction. It keeps 100% of the horizontal
field the eyes recorded and 53% of the vertical — filling a 4:3 frame from a nearly-square field
costs that, and it is a crop rather than a stretch.

Two details worth knowing. The per-edge **median** matters: the take's stamped fov is not constant
(eye 0's `u` runs from 0.3964 to 0.7679), and a strict intersection would let the single narrowest
frame decide the framing for all 4604 records. And the **common orientation** matters as much as the
shared frustum: keeping each eye's stamped orientation would toe the cameras in and reintroduce a
disparity at infinity, which is the thing the shared frustum exists to remove.

Verified end to end on a fixture carrying the real take's frustum: a feature at infinity lands within
**0.029 px** in both panes; stereo disparity is the IPD parallax and nothing else (11.96 px measured
against 11.39 predicted); squares stay square to **0.43%**, against the 44% the old mapping
introduced on the same pane. The `recorded` arm of the same test asserts the contrast — 80.8 px of
constant disparity on the synthetic frustum — so the mode that broke fusion cannot quietly become
the default again.

---

---

## Camera background: what the first real camera take convicted, and what is still open

Live report on the first camera render: *"passthrough is not covering the full scene - it's like 3/4
black, only the top center has passthrough."* Two terms, one fixed and one open.

**Convicted and fixed: the principal point was in the wrong coordinate frame.** Android states
intrinsics against the sensor's `active_array` — 1280×1280 here — and delivers a 1280×960 stream
cropped from it. `cy = 638.6` is the centre of the array and 66.5% down the image, so the compositor
believed the camera saw 36.4° above its optical axis and 20.3° below, and discarded the bottom of
every frame. `fx == fy` exactly, and the array is 1:1 against a 4:3 output, which is what proves it
is a centre crop rather than a squash. Rebasing puts the principal point at 49.9% of the image and
the field at a symmetric 28.9°/29.0°. Measured on the real take, left pane at 1440×1080: camera
coverage **50.1% → 59.2%**, vertical extent **63.6% → 75.9%** of the pane.

**Also convicted, and already fixed by the presentation framing:** the render the wearer saw predates
it and used the per-eye recorded frusta, whose padding makes the pane cover ~3.6× the solid angle.
Measured on that same take: **20.9% coverage, x [33%, 84%], y [0%, 47%]** — "3/4 black, top centre",
almost exactly the report. Under the presentation frustum the same frames give 50%, and 59% with the
rebase.

**Aimed, not cropped: `--bg-align auto` (default).** The coverage that remained after the rebase was
an aiming residue, not a field limit — but the field is tighter than it looks, so both halves are
worth stating. The output frustum is ±0.6981 **radians**, i.e. **80.0° horizontal by 64.4° vertical**;
the camera is **73.0° by 57.9°**. The camera is therefore *narrower* than the output in both axes and
full coverage is not available: the ceiling is ~88% per axis in tangent space, ~77% of the area.
Within that ceiling, what was costing coverage was aim. `--bg-align auto` drops the swing from the
recorded extrinsic — keeping its roll — so each camera's optical axis points along the output's
forward. Measured on the real take, left pane at 1440×1080:

| | coverage | x | y (top-down) |
|---|---|---|---|
| recorded extrinsic, 2.0 m | 59.2% | 4–97% | **0–76%** (top-biased) |
| `--bg-align auto`, 2.0 m | **64.8%** | 8–92% | **8–93%** (centred) |
| `--bg-align auto`, 1.0 m (defaults) | 59.6% | 9–91% | 11–93% |

**It is a trade, and the suite measures both halves.** Dropping the swing re-registers the background
against the world by exactly the angle discarded. On the synthetic take, where the extrinsic *is*
ground truth, that is a defect: a world feature drifts **8.4 px** under `auto` against **0.46 px**
under `recorded`, while the optical axis moves from 8.3 px off-centre to 0.7 px. On the real take the
recorded swing is measured against the IMU rather than the head and is therefore wrong to begin with,
so replacing it with "forward" is the better guess — but it *is* a guess, and `--bg-align recorded`
is the mode to return to the moment a real `imu_to_head` exists.

**Swim, part 1: temporal — already correct, and now timed off the right clock.** The camera image was
already reprojected from the head pose at the frame's own capture instant rather than at the output
instant, which is what keeps the room from lagging the overlay across a 30 Hz capture against a 45 Hz
output. What changed is which stamp that instant is read from: `t_xr_ns` now takes precedence over
`t_device_ns`, because the clock series maps host time against XR time while `t_device_ns` carries
whatever `t_device_ns_domain` says — `CLOCK_MONOTONIC` on this take. On the real bundle the two differ
by a **constant 104 ns**, so the change buys nothing today; it is made now because the take where they
diverge is the one nobody will debug. `validate` says which path engaged.

**Swim, part 2: depth — the default moves from 2.0 m to 1.0 m.** With the assumed-depth model, residual
swim is proportional to depth error, and it shows up as a jump exactly when the compositor switches
camera source frame: world-locked content should not move at a source change. Measuring that excess
against the real take:

| `--bg-depth` | 0.5 | 0.75 | **1.0** | 1.5 | 2.0 | 3.0 | 5.0 |
|---|---|---|---|---|---|---|---|
| excess jump at a source change | −0.23 | −0.07 | **+0.03** | +0.13 | +0.17 | +0.23 | +0.26 |

It crosses zero at **~0.9 m** — desk distance, which is where a seated session's content is — so the
default is now 1.0 m. At the old 2.0 m the jump was nearly six times larger. This is a scene property
rather than a constant: a take shot across a room wants a larger value, and the coverage table above
shows the cost of choosing a near depth (a closer assumed surface means more parallax between the lens
and the eye, so the image shifts further). The real answer is per-pixel depth from the stereo pair,
which is gap 3 above.

~~**Still open: the 10.87° extrinsic pitch.**~~ — **CLOSED: it was our own conversion bug, not a
frame problem.** Two things were wrong with the reasoning that left this open, and both are now
settled from Meta's shipping code rather than from our own output:

- **`GYROSCOPE` is enum reuse, not a frame.** `LENS_POSE_REFERENCE` reports that enumerant, but on
  Quest the pose is **head-relative**: Meta's samples compose it directly onto the Head node with
  nothing but a 180° X flip, unchanged across 15 months of SDK versions, and the native docs say
  "relative to the center of the HMD". There is no `imu_to_head` constant to wait for and nothing for
  the producer to add.
- **Our stored `extrinsics_head_to_camera` is missing a conjugate.** Android documents the raw
  quaternion as *sensor to camera* (p′ = Rp), so a head-to-camera needs the inverse. Dropping it
  mirrors the cant: cameras that really point **10.86° DOWN** were stored pointing **10.86° UP**, a
  **21.7° error** — which is exactly the residue that pushed the passthrough into the top of the
  frame and was mistaken for an unresolvable frame offset.

The correct conversion, in OpenXR axes: the translation is `LENS_POSE_TRANSLATION` **unchanged** (the
Android sensor frame shares the OpenXR head axes — no z-flip), and
`q_head→camera = conjugate(raw_xyzw) ⊗ Rx(180°)`. For a bundle that stored the mirrored value with no
raw to fall back on, `q_correct = Rx(180°) ⊗ q_stored⁻¹ ⊗ Rx(180°)` gives the same answer — verified
identical on both cameras of the reference take, and unit-tested against the device's ground truth.

**Policy: the raw wins.** Whenever the header carries `extrinsics_android_raw` — which both real
takes do — the loader recomputes `head_to_camera` from it and uses that, in memory only; the bundle
is never rewritten. It cross-checks against the stored value and says in `validate` whether the
stored one was correct or is a *repaired legacy conversion*, with the disagreement angle. On the
reference take that reads 21.72° (L) and 21.48° (R). When no raw is present the stored value is
trusted as-is.

**What this means for framing, and `--frustum camera`.** With truthful geometry the L camera covers **+18.0° to −39.9°
vertically** and **±36.4° horizontally** against a pane of ±32.2° by ±40.0°, sitting about 0.4° off
forward in azimuth. So under the (now default) `--bg-align recorded` there is an **uncovered band at
the top of the pane** — about 14° of the 64.4° vertical field. That is the real rig: the cameras look
down, and nothing above +18° was ever photographed. It is not a defect to fight — but it is a
defect to *show*, which is what `--frustum camera` is for: it clips the shared frustum to the
intersection of both cameras' coverage, so no output pixel is frame that no camera photographed.
Measured on take4 with the overlay switched off, so the number is the background's alone: the
camera-clipped frame is **99.8–100.0% covered**, against **59.5–60.9%** for the same frames under
`presentation`. The residual hundredths of a percent are a one-pixel border, from the inscribed
rectangle being sampled at 64 points per image edge.

Two properties worth stating because they are what make one clip good for a whole take. Coverage is
**pose-independent**: the camera is rigid to the head and, under presentation framing, so is the
output camera, so the camera's rotation relative to it is just the extrinsic and the lens-to-eye
offset is constant in that frame. Nothing depends on where the head is pointing. (Under `--framing
stabilized` the output rotates against the head by the smoothing residual, so the clip becomes
approximate near the edges, and the tool warns.) And the clip includes **parallax**, not just angles:
the lens sits 6 cm forward of the eye, so an angle-only intersection leaves a sliver of uncovered
frame at the assumed depth.

coordinates, and the resulting coverage — so this class of problem is visible without a headset.

---

## The output clock, and what still resamples (2026-08-20)

Reported from a viewing of take4: *"XR content seems a little jittery relative to background"*.
`--clock camera` addresses the background's half of that by making the output frame sequence the
camera frame sequence. Measured on take4, background only (overlay switched off in a symlink replica
so the figure is the background's alone), over 260 frames:

| | source-change pairs | held pairs | excess jump | alternation |
|---|---|---|---|---|
| `--clock overlay` (45 Hz grid) | 160 | 99 | **+3.315** | 6.555 |
| `--clock camera` (29.86 Hz) | 259 | 0 | **0 by construction** | **3.923** |

The excess-jump metric — how much more the image changes when the compositor switches camera source
frame than when it holds one — goes to zero because there are no held frames left: the mapping is
1:1. The alternation figure is the like-for-like one, and it falls 40%.

**Take4's real cadence**, which is why the container is what it is: 1111 left-camera frames over
37.18 s, average **29.8524 Hz**, interval median 33.333 ms, four dropped frames showing as 50.0 ms
gaps, stdev 1.65 ms. On a constant-rate grid at that measured average the worst frame sits **15.84 ms**
from its true stamp (mean 5.21 ms). That is accepted rather than fixed: the geometry carries no timing
error — every frame is *composed* at its own stamp — so what the grid costs is a playback wobble of at
most half a frame, against the spatial jitter it removes. **True VFR output is the exact fix** and the
obvious next step; it needs timestamps per frame, which the current rawvideo-over-a-pipe writer cannot
carry.

**What still resamples: the overlay.** It is chosen by the ordinal rule from the nearest record, up to
half an overlay period away (~11 ms at 45 Hz). Under the camera clock it is reprojected at the quads'
own distance — 1.92 m on take4, the median over 14202 layer records — instead of at infinity, which
removes the head-translation term for content at that distance. This is a whole-view warp at one
depth, **not** a per-quad homography, and the reason is worth recording: a v1 bundle's overlay is one
composited eye view with no per-quad masks and no way to attribute a pixel to a layer, so the exact
per-quad reprojection the geometry would allow has nothing to apply itself to. Its residual is the
parallax difference between a quad at the chosen depth and one elsewhere: two quads 0.5 m apart in
depth, seen 11 ms apart with the head at 0.5 m/s, disagree by about 0.3 mrad — well under a pixel at
these resolutions. Grade-B replay (gap 1) removes the question by re-rendering the quads.

---

## Throughput — measured on the first real take, and where the ceiling actually is

The reference take: `hypxrtake-20260817-115453-230`, 91.9 s, 4604 telemetry records, 2101 overlay
frames per eye at 2064×2162 ffv1 RGBA (2.3 + 2.5 GB). The reference render: left eye, 1920×1080,
`--framing asis`, libx264 crf 18 preset medium, 4137 output frames. The machine: Ryzen AI 9 HX 370,
12 cores / 24 threads, RTX 5070 Laptop.

| | before | after |
|---|---|---|
| `validate` | **≈840 s** (≈14 min) | **1.57 s** cold page cache, **0.63 s** warm |
| `render` | 70.1 s (1.31× realtime) | 69.5 s (1.32× realtime) |

`validate` was the whole win, and it was pure waste: `ffprobe -show_entries frame=pts_time` *decodes*
every frame to learn a number the container already holds. Counting packets instead gives the same
2101 per eye, and the alignment rule is enforced identically either way. `--deep` keeps the decoding
path for when a file is suspected of being truncated rather than merely large.

`render` did not move, and the reason is worth writing down because it is the opposite of what this
repo previously assumed.

**The composite is not decode-bound with an idle machine behind it. It is machine-bound.** Phase
split on the reference render: decode 27.1 s, GPU 18.1 s, encode 19.5 s, and the whole run keeps
**18 of 24 hardware threads busy** — because ffmpeg's ffv1 decode and x264's encode each already
thread across everything available. The decisive measurement is one that does not involve this
tool's own parallelism at all: **running two whole renders side by side takes exactly twice as long
as running one** (scaling 1.05 of a possible 2.0; four at once, 1.06 of 4.0). There is no idle
capacity to harvest.

So segment-parallel rendering, which is implemented and correct, does not pay here: measured
`--jobs 4` = 1.07× *slower* than `--jobs 1`, `--jobs 8` = 1.10× slower. The default is 1. It stays in
the tool because the finding is about this workload on this machine — it pays wherever the per-frame
cost is *not* already spread across every core — and because `--segment i/k` is how a long take gets
split across several machines or resumed after a failure.

What would actually move the number, in order of measured value:

1. **Stop pushing raw RGBA through pipes.** Every output frame moves ~70 MB between processes:
   17.8 MB out of the decoder, the same again into the GL upload, 8.3 MB back out of the readback,
   8.3 MB into the encoder. At 45 fps that is several GB/s of pure memory traffic on a laptop part
   whose bandwidth is shared with the iGPU. This is the ceiling, and it is exactly the libav\*
   linkage item above: decode into a hardware frame, hand it to GL as a texture, never round-trip
   through the CPU. Nothing else on this list is worth doing first.
2. **Take the encode off the CPU.** `--codec hevc_nvenc` measured 1263 → 692 CPU-seconds, roughly
   halving the machine's load; wall time gained 1.27× on a *contended* machine and nothing on a
   quiet one, which is the signature of a bottleneck that has moved elsewhere (see 1). `--crf` now
   maps to NVENC's constant-quality VBR, so quality is comparable rather than the default bitrate
   target it silently used before. Caveat from the bench: on this laptop the dGPU is not always
   present (`CUDA_ERROR_NO_DEVICE` when it has powered down), so this cannot be a default.
3. **Change what capture writes** — see the two capture-side notes below.

### Capture-side: `utvideo` is the decode-fast, disk-heavy trade, and it already works end to end

`manifest.overlay.encoder` already accepts `utvideo` (gbrap), the loader validates it, and the
compositor decodes it, so this is a producer-side switch with no compositor work at all. Measured on
the real overlay's own frames, transcoded ffv1 → utvideo:

| | ffv1 (as recorded) | utvideo |
|---|---|---|
| decode, 24 threads | 60.3 fps | **257.3 fps** (4.3×) |
| decode CPU per frame | 320 ms | **46.5 ms** (6.9× less) |
| size per frame | 1.10 MB | 3.06 MB (2.8× more) |

The CPU figure is the one that matters given the finding above: ffv1 buys its compression with a lot
of CPU, and — worth knowing — its *threading* is expensive too. Single-threaded ffv1 decode costs
140 CPU-ms per frame against 320 CPU-ms at 24 threads: 2.3× the CPU for 7.5× the wall. So a take
recorded as utvideo costs 2.8× the disk during capture and hands most of a core back per decoder.
For a 92-second take that is 2.3 GB → 6.4 GB per eye, which is the real question for the producer,
not the compositor.

### Capture-side: the NVENC lossless "filming tier"

Approved as a roadmap item, not built. The idea is that the overlay tap stops being a lossless CPU
codec and becomes a hardware-encoded pair of streams:

1. *Split the planes.* NVENC cannot carry alpha. So the overlay is written as **two** streams — colour
   as 4:4:4 and the matte as a luma-only stream — rather than one RGBA stream. They are the same
   frame count, aligned by the same ordinal rule the contract already specifies, so the bundle grows
   a second file per eye and nothing else changes semantically.
2. *Encode losslessly.* `hevc_nvenc` with lossless 4:4:4 keeps the "nothing in a bundle is derived"
   property that makes old takes replayable by a better compositor. Lossy here would be a
   capture-side decision that can never be undone, which is the one class of mistake this
   architecture exists to avoid.
3. *Decode with NVDEC.* The point is symmetric: a take encoded on the GPU should be decoded on the
   GPU, straight into a texture, which is item 1 of the throughput list. This is what makes the
   filming tier worth building — not the capture saving, but that it removes the CPU from both ends.

The compositor work is real but contained: `SOverlayInfo` grows a matte path per eye, the loader
validates the two streams against each other, and `ComposeGL` samples two textures instead of one.
The alignment rule, the alpha association, and the linear-light composite are all unchanged.

---

## Open questions for the producers

These are the ones the compositor cannot answer alone; they are also listed in README's
Interpretations table.

1. ~~**Overlay alpha association**~~ — **settled: `"premultiplied"`, multiplied in linear light with
   the sRGB encode after.** `"straight"` remains supported, and an absent field still means straight
   for older bundles.
2. ~~**Overlay pts epoch**~~ — **settled.** Overlay frames align to telemetry by ordinal: the n-th
   frame is the n-th record without `"dropped": true`. Container pts are a nominal timeline at
   `target_hz` and are never used to align. `manifest.overlay.pts_epoch_ns` is obsolete and warned
   about.
3. ~~**A head pose per telemetry record**~~ — **settled: `head` is recorded per record, in STAGE
   space at the eyes' instant.** The eye-midpoint derivation survives only as a fallback.
4. **Camera calibration header shape and `cam` key domain** — v1 accepts several spellings; pick one.
   The same applies to the quad records' `size`, `visibility`, and `rect` spellings.
5. **`extrinsics_head_to_camera` axis convention** — v1 requires OpenXR axes (camera looks down −Z).
   Android's `LENS_POSE_ROTATION` is not in that frame; the conversion belongs device-side.
6. ~~**`stage_correction` semantics**~~ — **settled: informational only, never applied.** v1 already
   treats it that way.
7. **Camera timestamp domain** — research 27 open question 5. `timestamp_source` in the header is
   read and warned about when missing, but v1 cannot compensate for a domain it is not told about.

## stereo-check: automated disparity sanity on real renders (validated by hand 2026-08-17)
A real-bundle stereo verifier, proven manually on the first real SBS render: split panes,
phase-correlate horizontal bands → require dy == 0 everywhere and dx within tolerance of the
telemetry-derived expectation: distant-feature shift from the per-eye frustum asymmetry
(Δtan of the eye fov centers × px-per-tan) plus IPD/depth parallax for content depth taken
from the quad records. First real measurement: 381–384 px measured vs ~377 px predicted
(349 frustum + 28 parallax at 1.48 m), dy = 0 in all bands. Ship as
`hypxrcompose stereo-check <take> <render>` and run it in the synth end-to-end suite too.

## Stereo output container: prefer MKV for auto-detection (field finding 2026-08-17)
The H.264 frame-packing SEI written into MP4 stereo outputs is NOT promoted into mpv's
`video-params/stereo-in` on the field mpv build — only container-level signaling is
(Matroska StereoMode; verified live: the SEI-only MP4 failed to auto-tag, a stream-copy
remux with `-metadata:s:v stereo_mode=left_right` into .mkv tags instantly). Change:
when --eye stereo-sbs and --out ends in .mp4, print a one-line notice recommending .mkv;
consider defaulting stereo output to .mkv when the user gives no extension. Keep writing
the SEI for MP4 (other players read it); MKV StereoMode stays the reliable path for the
HypXRland auto-tag flow.
