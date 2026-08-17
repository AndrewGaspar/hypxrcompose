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

1. *Bundle*: a `monitors/<name>.mp4` per XR monitor with screencopy timing, and a per-frame quad
   record in `telemetry.jsonl` — pose, extent in metres, visibility/opacity, curvature if any, and
   which monitor texture each quad shows. **This is capture-side and should be recorded now**, even
   though nothing consumes it yet.
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
| **Nearest-neighbour source resampling.** | Output frames pick the nearest source frame in time. At 30 Hz cameras against 60 Hz output that visibly stutters. Motion-compensated interpolation is the fix; simple frame blending is not (it doubles edges). Until then, matching `--fps` to the camera rate is the honest option for camera-dominant cuts. |
| **The overlay's straight-alpha edges.** | Bilinear sampling of straight-alpha RGBA bleeds background colour at matte edges. Premultiplying on upload would fix it; the manifest key that says which association the file uses (Interpretation 1) has to be settled first. |
| **Per-frame fov is taken from the nearest telemetry record, not interpolated.** | Correct for `asis` at capture rate. If output rates diverge from capture rates, the frustum should interpolate the way the pose does. |
| **No colour management.** | Everything is treated as 8-bit sRGB-ish and blended in that space. Alpha blending in a non-linear space is wrong at partial alpha, visibly so on the halo edges of a bright overlay. Blending in linear light needs the transfer function the overlay tap actually wrote. |
| **libav\* linkage.** | v1 talks to `ffmpeg` over pipes deliberately (see README). A v2 that wants hardware decode straight into a GL texture — which is where the throughput ceiling is — should link libav\*; the seam is `src/Ffmpeg.cpp`. |
| **Throughput.** | On the measured rig the composite is dominated by decode and pipe traffic, not by the GPU. The first optimization is not a faster kernel, it is not paying for `rawvideo` over a pipe twice. |

---

## Open questions for the producers

These are the ones the compositor cannot answer alone; they are also listed in README's
Interpretations table.

1. **Overlay alpha association** — straight or premultiplied? Please emit `manifest.overlay.alpha`.
2. **Overlay pts epoch** — Matroska cannot store nanosecond pts. Either offset the muxer's timestamps
   so the pts are absolute host nanoseconds to the millisecond, or set
   `manifest.overlay.pts_epoch_ns` and start the pts at zero. v1 guesses when neither is present, and
   says so.
3. **A head pose per telemetry record** — v1 derives it as the eye midpoint. Emitting `"head"`
   removes an assumption from the camera extrinsic chain.
4. **Camera calibration header shape and `cam` key domain** — v1 accepts several spellings; pick one.
5. **`extrinsics_head_to_camera` axis convention** — v1 requires OpenXR axes (camera looks down −Z).
   Android's `LENS_POSE_ROTATION` is not in that frame; the conversion belongs device-side.
6. **`stage_correction` semantics** — applied already, or still to apply?
7. **Camera timestamp domain** — research 27 open question 5. `timestamp_source` in the header is
   read and warned about when missing, but v1 cannot compensate for a domain it is not told about.
