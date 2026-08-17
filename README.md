# hypxrcompose

The offline compositor for `.hypxrtake` bundles — phase 3 of
[research 27, *Full-fidelity capture*](https://example.invalid/docs/openxr/research/27-full-fidelity-capture.md).

A `.hypxrtake` bundle is a recording of an XR session captured *at its sources*: the host's
composited eye buffers with their alpha matte, the host's pristine application audio, the headset's
passthrough cameras and microphone, and — the thing that makes the rest usable — a per-frame record
of every pose, field of view, and clock offset involved. Nothing in a bundle is composed. Everything
is source.

`hypxrcompose` turns one into a finished video. Because composition is deferred rendering against
recorded poses, the framing is a decision made in post: wearer's view as lived, a stabilized wearer's
view, either eye, or stereo side-by-side, all from the same take.

```
hypxrcompose validate <take>                       # is this bundle well formed?
hypxrcompose synth <out-take>                      # make one with known geometry, no hardware
hypxrcompose render <take> --out film.mp4          # compose it
```

## Status: v1

v1 is the **reprojection composite** research 27 §5 describes:

- **Background** — the passthrough camera video, projected through its recorded intrinsics
  (pinhole + Brown-Conrady) and extrinsics into the chosen output camera. Takes without cameras get a
  world-locked checker or a solid colour instead.
- **Foreground** — the Grade-A RGBA overlay, reprojected from its stamped per-eye pose and field of
  view into the same output camera, and alpha-blended over the background.
- **Output camera** — the recorded eye poses (`--framing asis`) or a zero-phase Gaussian smoothing of
  the head track with each eye's offset re-applied (`--framing stabilized`).
- **Audio** — the app track and, when present, the mic track, each placed on the output timeline
  through its start stamp and (for device-clocked tracks) the recorded clock series, summed with a
  limiter.
- **Stereo** — `--eye stereo-sbs` renders both eyes into one frame. Stereo is native to the design:
  every source in the bundle is already per-eye.

What v1 does not do is in [NEXT-STEPS.md](NEXT-STEPS.md): grade-B quad replay, rolling-shutter
correction, and depth-assisted reprojection to the true eye positions.

## The bundle contract

This is the format `validate` arbitrates and `render` consumes. Where this document and research 27
§4 differ, this document is current.

```
<take>.hypxrtake/
  manifest.json          {"take_id", "host":{...},
                          "sources":{"overlay","app_audio","cameras","mic"},
                          "overlay":{"width","height","format","encoder","target_hz","eye_count"},
                          "notes":[]}
  telemetry.jsonl        one object per line:
                         {"t_host_ns", "frame",
                          "eyes":[{"pose":{"pos":[3],"quat":[4]},
                                   "fov":{"l","r","u","d"}} x eye_count],
                          "stage_correction":{...}|null, "blend_mode",
                          "dropped": true|false (optional, default false)}
  clock.jsonl            {"t_host_ns", "offset_ns", "rtt_us"}   (device = host + offset)
  overlay/eye0.mkv       RGBA lossless-class, eye 0 = left
  overlay/eye1.mkv       eye 1 = right
  audio/app.flac         host application audio
  audio/app.json         {"start_t_host_ns", "sample_rate_hz", "channels", ...}
  cameras/<id>-camL.mp4  device passthrough cameras
  cameras/<id>-camR.mp4
  cameras/<id>-cameras.jsonl
                         line 1: calibration header — per camera, `intrinsics`
                         {fx,fy,cx,cy,distortion[]} and `extrinsics_head_to_camera` {pos,quat},
                         plus "timestamp_source"
                         then one object per frame:
                         {"cam", "t_device_ns", "exposure_ns", "frame"}
  audio/<id>-mic.flac    device microphone
  audio/<id>-mic.json    {"start_t_device_ns", "sample_rate_hz", "channels", ...}
```

Conventions, fixed once and depended on everywhere:

- **Space** is OpenXR's: right-handed, +X right, +Y up, −Z forward. Every pose in the bundle lives in
  the session's `LOCAL_FLOOR`-derived tracking space.
- **A pose** is `pos` + `quat` in `(x, y, z, w)` order, taking local vectors to world:
  `world = rot * local + pos`. Quaternions must be unit to 1e-3.
- **A field of view** is OpenXR's `XrFovf`: four half-angles in radians, `l` and `d` normally
  negative, an asymmetric frustum with its image plane at z = −1.
- **Camera intrinsics** are OpenCV's: +X right, +Y **down**, +Z forward, pixel centres at
  `(col + 0.5, row + 0.5)`, `distortion` in the order `k1, k2, p1, p2, k3`.
- **Camera extrinsics** (`extrinsics_head_to_camera`) are the lens pose in *head* space, expressed in
  **OpenXR axes** — the camera looks down its own −Z, +Y up — not in Android's sensor frame. The
  device-side producer converts. See Interpretations below.
- **Time** is nanoseconds. `t_host_ns` is the host's monotonic clock, `t_device_ns` the headset's;
  `clock.jsonl` relates them as `device = host + offset`, piecewise-linear between samples and held
  constant outside them.
- **Camera timestamps** are the *start* of exposure; the pose that belongs to a frame is the one at
  `t_device_ns + exposure_ns/2` (research 27 §3 footnote 1).

### The overlay alignment rule

**The n-th frame of each eye's overlay video is the n-th `telemetry.jsonl` record that does not carry
`"dropped": true`, and that record's `t_host_ns` is the frame's true time.** Ordinal, not temporal.

The container's presentation timestamps carry a *uniform nominal timeline* at `overlay.target_hz` and
say nothing about host time — Matroska's timestamp scale is fixed at 1 ms, so it could not carry
`t_host_ns` even if a producer wanted it to. `hypxrcompose` reads the pts only to sanity-check that
nominal cadence against the manifest, and never to align. Producers must run ffmpeg with
`-fps_mode passthrough` so nothing downstream duplicates or drops a frame and breaks the
correspondence.

`"dropped": true` means "this record has no pixels in the overlay video". It covers both causes at
once: decimation, because the overlay is captured at `target_hz` (typically 45) while the session runs
faster, and readback-queue losses, which are irregular. Per-cause counts belong in `manifest.notes`.

`validate` enforces the invariant directly: **per eye, the video's frame count must equal the number
of telemetry records without `dropped`.**

The consequence for composition is worth stating, because getting it wrong is invisible in a still
frame: an overlay frame is reprojected from *its own* record's pose and field of view, not from the
output instant's. Where a record was dropped, the two differ, and using the output instant's pose
would warp the overlay from a viewpoint it was never rendered at.

### Overlay pixel formats

The overlay must carry alpha; a matte is the entire reason the source exists. The accepted encoders
and the pixel format each decodes as:

| `manifest.overlay.encoder` | decodes as |
|----------------------------|------------|
| `ffv1`                     | `bgra`     |
| `png`                      | `rgba`     |
| `utvideo`                  | `gbrap`    |

`x264rgb` is deliberately absent: it cannot carry alpha. A pixel format with no alpha component is an
error, not a warning.

## Interpretations

Every place the contract does not pin something down and this implementation had to choose. Capture-
side producers need to converge on these, or change them here.

| # | Question | What v1 does |
|---|----------|--------------|
| 1 | **Overlay alpha association.** `"format": "rgba"` does not say whether colour is premultiplied. OpenXR composition layers default to premultiplied. | Straight (unassociated) alpha by default. An optional `manifest.overlay.alpha` of `"straight"` or `"premultiplied"` overrides it; anything else is an error. **Producers should emit this key explicitly.** |
| 2 | ~~**`container pts = t_host_ns`**~~ — **settled by the producers, not an interpretation any more.** | Overlay frames align to telemetry by ordinal (see "The overlay alignment rule"). Container pts are nominal and unused. `manifest.overlay.pts_epoch_ns`, which an earlier reading of the contract used, is now warned about as obsolete. |
| 3 | **A head pose is needed but not recorded.** Camera extrinsics are head-relative; the contract carries per-eye poses only. | The head is the midpoint of the two eye poses (position mean, rotation slerped halfway), which is where OpenXR's VIEW space sits. An optional per-record `"head"` pose is used when present. **Producers are encouraged to emit it.** |
| 4 | **Camera calibration header shape.** "intrinsics and extrinsics per cam" does not fix the container. | Three shapes are read: a `cameras` array of entries each with a `cam` key, a `cameras` object keyed by camera, or the camera keys at the top level. `timestamp_source` is read from the header or from each entry. |
| 5 | **Camera keys.** The filenames say `camL`/`camR`; the per-frame `cam` field's domain is unstated. | `"L"`, `"left"`, `"cam0"`, `"caml"`, `"0"`, `"eye0"` and integer `0` all normalize to left/eye 0; the `R` forms to right/eye 1. Anything else is an error. |
| 6 | **The `<id>-` prefix** on camera and mic files. | Discovery is by suffix (`*-cameras.jsonl`, `*-cam{L,R}.mp4`, `*-mic.flac`), so any prefix works; the bare spellings `mic.flac` and `cameras.jsonl` are also accepted, and the mic pair is looked for under `audio/` and at the take root. |
| 7 | **Camera video ↔ sidecar correspondence.** | The sidecar is authoritative for timestamps: its records for a given camera, in file order, correspond one-for-one with that camera's video frames in decode order. A count mismatch is an error. |
| 8 | **`stage_correction`'s shape and semantics.** The contract writes `{...}|null`. | Read as a pose (`pos` + `quat`) and recorded, but **not applied**: the stamped eye poses are taken to already include it, per research 27 §3 footnote 2 ("the *applied value* per frame"). An object of another shape warns. If the producers' semantics turn out to be "the correction that still needs applying", this is the one place that must change. |
| 9 | **Overlay file names.** The contract says `overlay/eye{0,1}.mkv`; research 27 §4 wrote `overlay/{left,right}.mkv`. | Both are accepted, `eye{0,1}` preferred, with a warning on the older spelling. |
| 10 | **Distortion vector length.** | 0, 4, or 5 coefficients are read as OpenCV `k1,k2,p1,p2,k3` with missing terms zero; longer vectors warn and the first five are used. |
| 11 | **Extra files.** | Unknown files and directories are ignored, which is what lets `synth` drop its ground truth at `synth/ground-truth.json` inside a bundle that still validates clean. |

## Usage

### validate

```
hypxrcompose validate <take> [--strict] [--json] [--no-media]
```

Runs the same loader `render` runs, with media probing on, and reports every structural and
referential problem rather than stopping at the first. Exit 0 clean (warnings allowed unless
`--strict`), 1 on errors, 2 if the bundle could not be opened at all.

The referential checks are the interesting ones: overlay frames must have a stamped pose nearby,
camera sidecar records must match the video's frame count, camera timestamps must map onto the
telemetry span once the clock offset is applied (a sign error shows up here), audio start stamps must
land inside the take, and a declared source must actually have its files.

### synth

```
hypxrcompose synth <out-take> [--frames N] [--hz N] [--cam-hz N] [--clock-offset-ms N] ...
```

Generates a complete bundle with no hardware. This is what makes the compositor testable: the scene
is closed-form, so the correct output pixel for a known feature is *computable*. The scene is a
checkerboard wall at a known depth carrying uniquely coloured markers and a frame-identity patch,
plus a floating RGBA "monitor" quad with its own markers and a half-transparent halo.

The generator deliberately makes the easy things hard:

- a nonzero, drifting host↔device clock offset (default +250 ms at +20 ppm) with jitter on each
  sample, so a compositor that forgets to map device time selects visibly wrong frames;
- camera extrinsics that are not the eye poses — a wider baseline (84 mm) than the IPD (63 mm), a
  45 mm forward offset, a 12 mm drop, and a 2° outward splay;
- asymmetric, per-eye-mirrored fields of view;
- a principal point off centre and real Brown-Conrady distortion;
- cameras on their own cadence and phase, and an overlay decimated from the session rate with two
  extra irregular "readback" drops on top, so neither the ordinal alignment rule nor nearest-in-time
  selection can hide behind an accidental 1:1 index map;
- head motion with high-frequency jitter riding on a slow sweep, so stabilization has both something
  to remove and something to keep;
- audio clicks at known instants, one stamped on each clock domain.

Ground truth lands in `synth/ground-truth.json` inside the bundle.

### render

```
hypxrcompose render <take> --out film.mp4
    [--eye left|right|stereo-sbs] [--framing asis|stabilized] [--size WxH] [--fps N]
    [--background auto|camera|checker|solid] [--bg-depth M] [--fg-depth M|inf]
    [--stabilize-ms N] [--mic-gain G] [--no-audio] [--no-limiter]
    [--codec NAME] [--crf N] [--gpu SUBSTR] [--frames-dir DIR] [--report FILE] [--limit N]
```

`--size` is the size of **one eye's pane**; stereo SBS output is therefore `2W x H`, which keeps each
eye's geometry undistorted rather than squeezing two eyes into one frame.

`--report` writes a JSON record of which telemetry record, overlay frame, and camera frame every
output frame was made of, plus the throughput breakdown. It is the first thing to look at when a
composite looks wrong.

## How it works

### The output timeline

Constant rate: output frame *k* sits at `t0 + k/fps`, where `t0` is the first telemetry stamp and
`fps` defaults to the manifest's `target_hz`. Every source is resampled onto it by nearest-in-host-
time selection — an exact 1:1 map when the output rate equals the capture rate, and the least-wrong
choice available for 30 Hz cameras without optical flow.

Poses are the exception: the head pose for a camera frame is *interpolated* (lerp position, slerp
rotation) between the telemetry records bracketing that frame's mid-exposure instant, because a 30 Hz
camera frame lands between 90 Hz telemetry records and rounding would inject half a frame of head
motion into the geometry.

### The clock

`clock.jsonl` samples the offset in host time, so mapping a device timestamp back needs the offset at
the host time being solved for. `hostFromDevice` iterates `h ← device − offset(h)`; the map is a
contraction for any clock that does not run backwards, and at realistic drift it converges in one
step. Between samples the offset is piecewise-linear. Outside them it is **held**, not extrapolated:
a linear extrapolation of a noisy offset estimate diverges, while a held value is wrong by a bounded,
understandable amount. Validate warns when the clock series does not span the take.

### The reprojection kernel

An inverse warp on the GPU: for each *output* pixel, build the ray the output camera looks along,
place a point on it at the assumed depth, and project that point into each source's model — the
camera's pinhole+distortion for the background, the stamped frustum for the overlay. Nothing is
forward-mapped, so there are no seams or holes to fill.

The assumed depth is the knob that exposes v1's honest limitation. The passthrough cameras are near
but not at the eyes, so without depth data a reprojection is exact only for content at the assumed
distance:

- `--bg-depth` (default 2.0 m) is where the background is assumed to be. On the synthetic rig the
  residual error from a wrong assumption is well under a pixel at 480×360 — the artifact every Quest
  recording already has.
- `--fg-depth` (default `inf`) makes the overlay warp rotation-only. With `--framing asis` the output
  camera *is* the recording eye, so the overlay reprojection is exact at any depth; with
  `--framing stabilized` a rotation-only warp is the honest choice absent depth data.

The kernel runs on EGL_MESA_platform_surfaceless plus an FBO — no compositor, no window system, no
display, so it works over ssh and in CI. `--gpu <substring>` pins the device by renderer string,
vendor, or DRM node when EGL's default picks the wrong vendor on a multi-GPU machine (`--gpu list`
enumerates); `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json` is the
blunter instrument.

### Stabilization

Composition is offline, so the whole pose track is in hand and there is no reason to accept the lag a
causal filter (one-euro, EMA) imposes. The filter is a **zero-phase Gaussian**: each output pose is
the time-weighted mean of its neighbours with weights `exp(-dt²/2σ²)`, truncated at 3σ, with the
track's ends clamped. Weighting is by time rather than by sample index, so an irregular cadence does
not silently move the cutoff. Positions average componentwise; rotations average in the tangent space
of the kernel-centre rotation via log/exp, which is exact for a single rotation and correct to second
order in the angular spread.

`--stabilize-ms` is σ. The −3 dB point is `sqrt(ln2/2)/(πσ)`: the 200 ms default passes head motion
below about 0.94 Hz and removes the jitter above it. The eye's offset from the head is re-applied
after smoothing, so the IPD and the stereo geometry survive.

### Audio

Each track is placed by converting its start stamp to an output-timeline sample index — device-
stamped tracks go through the clock series first — and then delayed with `adelay`'s sample-exact `S`
suffix or trimmed with `atrim`'s `start_sample`. Both directions are sample-exact; nothing rounds to
milliseconds. Tracks are summed straight (`amix ... normalize=0`) so `--mic-gain` means what it says,
and an `alimiter` catches the sum rather than pre-attenuating every track. `--no-limiter` turns it
off.

### ffmpeg by subprocess, not libav

Deliberate, for v1:

- the libav\* ABI moves, and a tool that must still open takes recorded a year ago is better off
  depending on "an ffmpeg is installed" than on a particular libavcodec soname;
- every command is printable, so `--dump-commands` turns any decode or encode problem into a command
  line you can run by hand;
- the codec menu is whatever the local ffmpeg has, including hardware encoders, with no build-time
  coupling.

The cost is one memcpy per frame through a pipe, which sits far below the GPU and encoder time next
to it at v1 resolutions. A v2 wanting zero-copy hardware frames should link libav\*; the seam is
narrow — `src/Ffmpeg.cpp` — and contained.

## Tests

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build -j8
./build/hypxrcompose_tests
```

Everything runs headless on Mesa surfaceless; no compositor and no hardware are involved.

The end-to-end tests are `synth → render → verify`, and every assertion is quantitative because the
synthetic scene's geometry is closed-form. What each one proves:

| Test | Proves |
|------|--------|
| `OverlayMarkersLandWherePredicted` | Stamped poses survive the write/parse round trip; the GLSL frustum maths agrees with the CPU model; the ordinal rule picks the right overlay frame and reprojects it from *its own* record's pose; resampling to a pane size that is not the capture size lands content correctly. The test asserts that at least one sampled frame is one whose overlay pixels come from a different record, so the dropped-frame path is genuinely covered. |
| `BackgroundMarkersLandWherePredicted` | The whole camera chain closes — intrinsics, distortion, extrinsics, mid-exposure pose interpolation, assumed-depth model. The prediction never touches the distortion model, so a disagreement between the generator's inverse distortion and the shader's forward distortion shows up as a miss. |
| `TheAcceptedParallaxErrorIsSmallAndMeasurable` | Quantifies research 27 §5.1's accepted error for this rig instead of asserting it away. |
| `TheClockOffsetSelectsTheRightCameraFrame` | Device time is mapped into host time before frames are chosen: the selection matches the clock series, is exactly `offset × camera rate` away from what a clock-blind compositor would pick, and the frame-identity patch in the *pixels* decodes to the same answer. |
| `StereoPanesDifferByTheSyntheticIpdParallax` | The two panes are genuinely two eyes, each with its own pose and its own mirrored frustum, and their disparity carries the synthetic IPD's parallax with the right sign. |
| `ChangingTheAssumedBackgroundDepthMovesTheImageByThePredictedParallax` | `--bg-depth` moves the image by exactly the predicted amount. |
| `StabilizedFramingRendersFromTheSmoothedCamera` | The render path uses the smoothed camera, keeps the eye's offset from the head, and warps by direction only at infinite foreground depth. |
| `AudioClicksLandOnTheOutputTimeline` | Both clock domains place correctly: a host-stamped click and a device-stamped click both land within two samples of their predicted output positions. |
| `TheOrdinalAlignmentCountIsEnforced` | Flipping one record's `dropped` flag makes the counts disagree, and validate says so. The rule the whole overlay path rests on is not taken on trust. |
| `AHostOnlyTakeComposesOverTheCheckerBackground` | A bundle with no device sources still composes and claims no camera frames. |

Plus unit tests for the pose algebra against hand-computed cases, the pinhole model against a
hand-evaluated Brown-Conrady expansion, the clock map's interpolation/holding/inversion, the
stabilizer's frequency response and phase neutrality, and validate's error cases.

## Building

Needs a C++23 compiler, CMake ≥ 3.20, EGL and OpenGL ES 3, nlohmann/json, GTest for the suite, and
`ffmpeg`/`ffprobe` on `PATH`.

## Licence

BSD 3-Clause. See [LICENSE](LICENSE).
