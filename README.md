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
                          "head":{"pos":[3],"quat":[4]},
                          "quads":[{...} x layers],
                          "stage_correction":{...}|null, "blend_mode",
                          "dropped": true|false (optional, default false)}
  clock.jsonl            {"t_host_ns", "offset_ns", "rtt_us"}   (device = host + offset)
  overlay/eye0.mkv       RGBA lossless-class, eye 0 = left
  overlay/eye1.mkv       eye 1 = right
  audio/app.flac         host application audio
  audio/app.json         {"start_t_host_ns", "sample_rate_hz", "channels", ...}
  cameras/<id>-camL.mp4  device passthrough cameras
  cameras/<id>-camR.mp4
  <id>-cameras.jsonl     at the take ROOT (the join puts it there; cameras/ also accepted)
                         line 1: calibration header — two parallel maps keyed by camera,
                         `intrinsics` {L:{fx,fy,cx,cy,distortion},R:{...}} and
                         `extrinsics_head_to_camera` {L:{pos,quat},R:{...}}, with
                         "axes"/"timestamp_source"/"t_device_ns_domain"/"clock_anchor"
                         shared once at the top level
                         then one object per frame:
                         {"cam", "t_device_ns", "exposure_ns", "frame", ...}
  <id>-mic.wav           device microphone, at the take ROOT, pcm_s16le
  <id>-mic.json          {"start_t_device_ns", "t_device_ns_domain", "sample_rate_hz",
                          "channels", "clock_anchor", ...}
```

### What the reader tolerates, and why

The layout above is what the **producer actually writes** — confirmed against the first real joined
takes, and what `synth` now emits, so first contact with a real bundle stops being a bug report.
Four earlier guesses were wrong and each needed a hand shim before a real take would load; all four
are now parsed natively, and the older spellings are still accepted because bundles carrying them
exist:

| Field or file | Producer reality | Also accepted |
|---|---|---|
| Device mic | `<id>-mic.wav` + `.json` at the take **root** | `audio/*-mic.flac`, and either container in either place |
| Camera sidecar | `<id>-cameras.jsonl` at the take **root**, videos under `cameras/` | the sidecar under `cameras/` |
| Calibration header | `intrinsics: {L,R}` + `extrinsics_head_to_camera: {L,R}` as parallel maps, shared fields at the top | a `cameras` array, a `cameras` object, or per-cam objects at the top level |
| `distortion` | `null` — the Meta cameras pre-undistort and publish no coefficients | an array of 0/4/5 coefficients |
| `exposure_ns` | `-1` when the device does not report one | any non-negative duration |

`distortion: null` is read as "no distortion", which is the pinhole model with every term zero — not
as a missing field and not as an error. `exposure_ns: -1` is the device's *unknown* sentinel: the
frame is then sampled at its stamp rather than half an exposure later, with one rate-limited warning
rather than one per frame.

**Unknown keys are ignored, everywhere, as policy.** Every reader here looks up the keys it needs and
steps over the rest, at every level — manifest, telemetry records, clock samples, calibration header,
per-frame records. A producer may add fields without breaking a reader that has never heard of them.
The real takes already rely on this: calibration entries carry `camera_id`/`camera_source`/`position`
strings and a duplicate `intrinsics` array beside the named fields, and per-frame records carry
`capture`, `pts_us` and `t_xr_ns`. The synthetic bundle emits all of those, so the policy is
exercised by the suite rather than merely promised here.

A take may also declare `sources.cameras: false` and still ship a calibration sidecar — the first
real joined take does, because the cameras disconnected mid-session and captured nothing. That is
reported as a note, not an error: the manifest is right, there are no camera pixels.

Conventions, fixed once and depended on everywhere:

- **Space** is OpenXR's: right-handed, +X right, +Y up, −Z forward. Eye, head, and camera poses are in
  the client's **STAGE** space.
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

### Head pose and composition layers

Each record carries **`head`**, a pose at the same instant and in the same space as the eyes. Camera
extrinsics and quad poses are relative to it. When it is absent — only in bundles written before it
existed — the head falls back to the midpoint of the eyes, which is where OpenXR's VIEW space sits;
validate says so, and rejects a bundle that carries it on some records but not others.

Each record also carries **`quads`**, one entry per composition layer, in composition order back to
front:

```
{"index", "name" (currently always null), "pose", "size" (metres), "visibility",
 "view_space" (bool), "swapchain", "image", "array_layer", "rect"}
```

Three semantics here are load-bearing, and getting any of them wrong puts layers in the wrong place —
or in the wrong eye:

1. **`pose` is head-relative.** The layer's pose in STAGE space at time *t* is `head(t) ∘ pose`.
2. **`view_space` says what that means over time.** `true` = head-locked: the layer stays at this
   head-relative pose always. `false` = room-anchored: the layer had a fixed pose in STAGE, and what
   was recorded is that pose expressed relative to the head at *t*. A replay must re-anchor it rather
   than carry it along with the head.
3. **`visibility` is an `XrEyeVisibility`, not an opacity** — one of `"both"`, `"left"`, `"right"`,
   `"none"`. It says which eye the layer was composed into. A stereo-depth desktop submits a *pair* of
   quads per monitor, sharing one pose, one per eye, taking opposite halves of a side-by-side
   swapchain via `rect`; a HUD submits `"both"`. Ask `composedInEye(0|1)` rather than reading the
   field. A boolean or a 0..1 number is the older opacity spelling and is still read, with a warning.

`swapchain` is stable within a session only. v1 does not composite quads — it uses the recorded
matte — but it parses, validates, and carries them, because telemetry not recorded correctly today is
telemetry v2 cannot use. `validate` warns when a take has no quad records at all: it composes fine,
but grade-B replay is foreclosed for it forever.

### Alpha association and colour space

`manifest.overlay.alpha` says how the overlay video stores colour:

- **`"premultiplied"`** (what the host producer emits): the multiply happens **in linear light** and
  the sRGB encode after it, so a stored byte is `srgb_encode(color_linear · alpha)`.
- **`"straight"`**: the stored byte is `srgb_encode(color_linear)`, unassociated.
- Absent: treated as `"straight"`, for bundles written before the field existed.

Alpha itself is never encoded; only the colour channels are.

The compositor therefore decodes sRGB → composites in **premultiplied linear** → encodes back. Both
source textures are uploaded as `GL_SRGB8_ALPHA8`, so the hardware decodes to linear *before*
filtering — the only place that decode can correctly happen, since filtering encoded values is wrong
at every edge. The single encode is in the fragment shader, on the way into the render target.

This is not a nicety. Blending a 75 %-alpha layer in encoded space instead of linear light lands
about 30 levels away from the right answer, and the test suite asserts against both the correct value
and that specific wrong one.

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
| 1 | ~~**Overlay alpha association**~~ — **settled by the producers.** | `manifest.overlay.alpha` is authoritative: `"premultiplied"` (the host producer's output, multiplied in linear light and sRGB-encoded after) or `"straight"`. Absent still defaults to `"straight"` for older bundles. See "Alpha association and colour space". |
| 2 | ~~**`container pts = t_host_ns`**~~ — **settled by the producers, not an interpretation any more.** | Overlay frames align to telemetry by ordinal (see "The overlay alignment rule"). Container pts are nominal and unused. `manifest.overlay.pts_epoch_ns`, which an earlier reading of the contract used, is now warned about as obsolete. |
| 3 | ~~**A head pose is needed but not recorded**~~ — **settled by the producers.** | Every record carries `head`, in STAGE space at the eyes' instant. The eye-midpoint derivation survives only as a fallback for older bundles, and validate warns when it is used. |
| 4 | **Camera calibration header shape.** "intrinsics and extrinsics per cam" does not fix the container. | Three shapes are read: a `cameras` array of entries each with a `cam` key, a `cameras` object keyed by camera, or the camera keys at the top level. `timestamp_source` is read from the header or from each entry. |
| 5 | **Camera keys.** The filenames say `camL`/`camR`; the per-frame `cam` field's domain is unstated. | `"L"`, `"left"`, `"cam0"`, `"caml"`, `"0"`, `"eye0"` and integer `0` all normalize to left/eye 0; the `R` forms to right/eye 1. Anything else is an error. |
| 6 | **The `<id>-` prefix** on camera and mic files. | Discovery is by suffix (`*-cameras.jsonl`, `*-cam{L,R}.mp4`, `*-mic.flac`), so any prefix works; the bare spellings `mic.flac` and `cameras.jsonl` are also accepted, and the mic pair is looked for under `audio/` and at the take root. |
| 7 | **Camera video ↔ sidecar correspondence.** | The sidecar is authoritative for timestamps: its records for a given camera, in file order, correspond one-for-one with that camera's video frames in decode order. A count mismatch is an error. |
| 8 | ~~**`stage_correction`'s semantics**~~ — **confirmed pinned by the producers: informational only, never applied.** | Read as a pose and recorded; the stamped eye poses already include it. v1's original reading was right. |
| 9 | **Overlay file names.** The contract says `overlay/eye{0,1}.mkv`; research 27 §4 wrote `overlay/{left,right}.mkv`. | Both are accepted, `eye{0,1}` preferred, with a warning on the older spelling. |
| 10 | **Distortion vector length.** | 0, 4, or 5 coefficients are read as OpenCV `k1,k2,p1,p2,k3` with missing terms zero; longer vectors warn and the first five are used. |
| 12 | ~~**Quad record field shapes.**~~ — **`visibility` PINNED: it is an `XrEyeVisibility`, not an opacity.** | `visibility` is one of the strings `both`, `left`, `right`, `none`, and says *which eye the layer was composed into*. A stereo-depth desktop submits a per-eye **pair** of quads sharing one pose and taking opposite halves of a side-by-side swapchain; a HUD submits `both`. Ask `composedInEye(0\|1)`. A boolean or a 0..1 number is the deprecated opacity spelling: still read, with a warning. `size` reads as `[w, h]` metres or an object with `width`/`height`; `rect` as `[x, y, w, h]` swapchain pixels or an object. `index` must equal the entry's position in the array, since the array is composition order. |
| 11 | **Extra files.** | Unknown files and directories are ignored, which is what lets `synth` drop its ground truth at `synth/ground-truth.json` inside a bundle that still validates clean. |

## Usage

### validate

```
hypxrcompose validate <take> [--strict] [--json] [--no-media] [--deep] [--checksum]
```

Runs the same loader `render` runs, with media probing on, and reports every structural and
referential problem rather than stopping at the first. Exit 0 clean (warnings allowed unless
`--strict`), 1 on errors, 2 if the bundle could not be opened at all.

Frame counts come from the container's packet index, not from decoding. For every encoder the
contract allows an overlay to use — ffv1, utvideo, png — one packet is exactly one frame, so the
count is not an estimate; it is the same number a full decode gives, and the alignment rule is
enforced identically either way. The difference is that on a 92-second two-eye take the fast path
takes **1.6 s** and the decoding one took **fourteen minutes**. `--deep` still decodes, for when a
file is suspected of being truncated rather than merely large, and `--checksum` adds an md5 over
every decoded frame. A codec outside the known intra-only set is counted from the index but says so.

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
- two composition layers with both `view_space` values — a room-anchored monitor and a head-locked
  HUD — recorded head-relative exactly as the producer records them, so a v2 grade-B replayer has a
  bundle with known-correct answers to develop against;
- a 75 %-alpha HUD and a 50 %-alpha marker and halo, stored premultiplied in linear light (or
  straight, with `--alpha straight`), so neither the colour space nor the association can be guessed
  wrong and still pass;
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
    [--jobs N] [--segment i/k] [--decode-threads N]
```

`--size` is the size of **one eye's pane**; stereo SBS output is therefore `2W x H`, which keeps each
eye's geometry undistorted rather than squeezing two eyes into one frame.

The output camera's frustum is **derived from the recorded eye frusta, not copied from them**, and
`--frustum` picks which derivation:

- **`presentation`** (default) — **one** symmetric frustum shared by both eyes, cropped to the pane,
  with each eye rendering through it from its own position but a common orientation. A feature at
  infinity therefore lands at the same pane coordinate in both eyes, vertical disparity is zero, and
  the stereo is carried entirely by the content. This is what a flat side-by-side viewer needs — and
  what a person fusing two panes needs, which is not the same as what is geometrically truest.
- **`recorded`** — each eye keeps its own asymmetric frustum, padded to the pane at a shared scale.
  Truer to what each eye saw, and the right input for analysis or for a headset-native player that
  re-projects per eye. On a flat viewer the two frames sit at different visual angles: on the
  reference take a feature at infinity lands 349 px apart at 1440 per eye, which breaks fusion.

Either way the recorded frustum is asymmetric (on the first real take the optical axis sits at 62.1%
of the eye buffer's width) and its angular aspect, 0.9255 there, is not the aspect of any pane you
would ask for — mapping one onto the other stretched the picture by 1.92x at 1920x1080. The frustum
actually used is published per pane in `--report` as `pane_fov`, and any pixels-per-radian figure
should be read off that rather than off the telemetry.

`--report` writes a JSON record of which telemetry record, overlay frame, and camera frame every
output frame was made of, plus the throughput breakdown. It is the first thing to look at when a
composite looks wrong.

`--jobs N` cuts the output timeline into N chunks and composes them in parallel worker processes,
joining the encoded chunks with a stream copy and muxing audio once at the end. **It defaults to 1,
and on the measured take turning it up made things slower** — the serial pipeline already keeps 18 of
24 hardware threads busy, so there is nothing for a second worker to take. The numbers, and where the
time actually goes, are in [NEXT-STEPS.md](NEXT-STEPS.md#throughput--measured-on-the-first-real-take-and-where-the-ceiling-actually-is).
`--segment i/k` renders one chunk by hand, which is how a long take is split across machines or
resumed after a failure.

### Stereo signalling

A `--eye stereo-sbs` render says so in the file, so a player or an XR compositor can detect it
instead of being told. Two signals are written, because no single one is understood everywhere:

| Output | Signal | Read back with | Who detects it |
|---|---|---|---|
| `.mkv` / `.webm` | Matroska **StereoMode** = `left_right` | `ffprobe -show_entries stream_tags=stereo_mode` | mpv, and the XR desktop's stereo window auto-tagging |
| `.mp4` with libx264/libx265 | H.264/HEVC **frame-packing SEI**, arrangement type 3 (side by side) | `ffprobe -show_frames` → `side_data_type=Stereo 3D` | players that read the bitstream; survives a stream copy, so a `--jobs` join keeps it |
| `.mp4` with any other encoder | none available | — | warns at startup: write `.mkv`, or use libx264/libx265 |

Matroska output gets both. A mono render carries neither — a wrong stereo flag is worse than no flag,
since anything honouring it will show one picture as two half-width ones.

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
| `PartialAlphaCompositesInLinearLightFromAPremultipliedSource` | A 75 %-alpha layer over a known background composites to the linear-light answer and *not* to the encoded-space one, which the test computes explicitly and requires to be at least 15 levels away. Covers both the colour space and the premultiplied association in one measurement. |
| `StraightAndPremultipliedBundlesComposeToTheSamePixels` | Two encodings of identical imagery compose alike to within rounding — so the compositor really is reading the manifest field rather than assuming one. |
| `QuadRecordsRoundTripWithHeadRelativePoses` | Quad records survive parse intact, and `head ∘ pose` re-anchors the room-anchored layer to the same STAGE pose at every record (measured: 4.5 × 10⁻¹³ mm) while the head-locked one's STAGE pose follows the head. A v2 reading these as world poses fails here. |
| `AHostOnlyTakeComposesOverTheCheckerBackground` | A bundle with no device sources still composes and claims no camera frames. |

Plus unit tests for the pose algebra against hand-computed cases, the pinhole model against a
hand-evaluated Brown-Conrady expansion, the clock map's interpolation/holding/inversion, the
stabilizer's frequency response and phase neutrality, and validate's error cases.

## Measured throughput

A 180-record 90 Hz take with a 45 Hz overlay and 30 Hz cameras, all sources at 1280×960, composed on
a Ryzen AI 9 HX 370 laptop with both an AMD 890M (radeonsi) and an NVIDIA RTX 5070:

| Output | Background | GPU | fps | Mpix/s | decode | gpu | encode |
|--------|-----------|-----|----:|------:|-------:|----:|-------:|
| 1280×960 mono | camera | NVIDIA | 124.8 | 153 | 0.42 s | 0.14 s | 0.16 s |
| 2560×960 stereo SBS | camera | NVIDIA | 64.3 | 158 | 0.83 s | 0.25 s | 0.32 s |
| 2560×960 stereo SBS | checker | NVIDIA | 84.1 | 207 | 0.51 s | 0.24 s | 0.32 s |
| 2560×960 stereo SBS | camera | AMD 890M | 64.7 | 159 | 0.79 s | 0.29 s | 0.31 s |

Two things to read out of that. First, composition runs comfortably faster than real time even in
stereo — a two-minute take composes in well under a minute. Second, the integrated AMD GPU and the
discrete NVIDIA one are within one percent of each other, which is the clearest possible statement
that **this pipeline is not GPU-bound**: roughly 60 % of the wall time is decoding source video
through pipes and about 18 % is the kernel. The first optimization worth making is not a faster
shader, it is not paying for `rawvideo` over a pipe (see NEXT-STEPS).

At the smaller resolutions the test suite uses (1920×720 stereo from 640×480 sources) the same rig
turns in 100–170 fps and 140–230 Mpix/s.

## Building

Needs a C++23 compiler, CMake ≥ 3.20, EGL and OpenGL ES 3, nlohmann/json, GTest for the suite, and
`ffmpeg`/`ffprobe` on `PATH`.

## Licence

BSD 3-Clause. See [LICENSE](LICENSE).
