# Codex Task: Release Robotrol v7.1

## Goal
Bump the version string to V7.1, commit all pending changes, tag the release,
push to GitHub, and create a GitHub release.

File names do NOT change — the main script stays `Robotrol_FluidNC_v7_0.py`
(keeping the v7.x series filenames). Only the displayed version string bumps.

---

## Step 1 – Version bump

Edit the following strings (case-exact, two files):

**Robotrol_FluidNC_v7_0.py** — two lines:
  - Line 2:  `# 34 Robotrol V7.0`       →  `# 34 Robotrol V7.1`
  - Line 531: `root.title("Robotrol V7.0")` →  `root.title("Robotrol V7.1")`

**pickplace_ui.py** — one line:
  - `"Setup + Calibration (v7.0) - Step by step\n"` → `"Setup + Calibration (v7.1) - Step by step\n"`

**README.md** — one line:
  - `main UI core (window title: V7.0)` → `main UI core (window title: V7.1)`

---

## Step 2 – Syntax check

Run (all must pass with no output):
```
python -m py_compile Robotrol_FluidNC_v7_0.py pickplace_ui.py
```

---

## Step 3 – Stage and commit

Stage exactly these files (nothing else — avoid .claude/, data/tnt_*.png,
robosim_visualizer_v90.spec unless you are sure they belong):

```
git add Robotrol_FluidNC_v7_0.py
git add Robotrol_FluidNC_v7_0.spec
git add pickplace_ui.py
git add configs/camera.json
git add data/GCODE_help.txt
git add data/examples/test_marlin_interpreter.gcode
git add data/examples/tobias_cursive.gcode
git add README.md
```

Commit message:
```
feat: release robotrol v7.1 — GCODE tab redesign + Pick & Place polish

GCODE tab (formerly "Plane G2/G3"):
- Renamed tab to "GCODE"
- Replaced single Scale with Scale X / Scale Y (independent axes)
- Added Mirror X / Mirror Y checkboxes
- Added Rotate combobox (0° / 90° / 180° / 270°)
- Built horizontal UI: Transform controls (left) + live preview canvas (right)
- Preview draws UV path in blue (draw) / gray dashed (rapid) / red origin dot
- Help button opens GCODE_help.txt describing use case and workflow
- Arc executor (_plane_exec_gcode) updated: _transform_uv() closure applies
  scale_x/y → mirror → rotate for both end-points and I/J arc offsets

New example files:
- data/examples/test_marlin_interpreter.gcode  (interpreter unit test)
- data/examples/tobias_cursive.gcode           (cursive "Tobias" pen-plotter demo)
- data/GCODE_help.txt

Pick & Place tab:
- "Pick&Place" → "Pick & Place" consistent across tab label, error messages,
  and help text
- Calibration sub-tab order: Camera → Base-Cam → Marker-Obj → Perception
- Labels, info text, and help procedure revised for clarity (26-step guide)

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
```

---

## Step 4 – Tag

```
git tag v7.1
```

---

## Step 5 – Push

```
git push origin main
git push origin v7.1
```

---

## Step 6 – GitHub release

```
gh release create v7.1 \
  --title "Robotrol v7.1" \
  --notes "## What's new in v7.1

### GCODE tab (formerly \"Plane G2/G3\")
- Tab renamed to **GCODE**
- Independent **Scale X / Scale Y** for non-uniform scaling
- **Mirror X / Mirror Y** toggles
- **Rotate** selector (0° / 90° / 180° / 270°) — rotates path in UV plane
- Live **preview canvas** — blue = draw moves, gray dashed = rapids, red dot = G-code origin
- **Help** button opens built-in usage guide
- Arc executor updated with full transform pipeline (scale → mirror → rotate) for both end-points and I/J offsets

### New example files
- \`data/examples/test_marlin_interpreter.gcode\`
- \`data/examples/tobias_cursive.gcode\`
- \`data/GCODE_help.txt\`

### Pick & Place tab
- Consistent \"Pick & Place\" label everywhere
- Calibration sub-tab order: Camera → Base-Cam → Marker-Obj → Perception
- Revised labels and 26-step calibration guide"
```

---

## Done
After completing the above, confirm:
- `git log --oneline -3` shows the new commit
- `git tag` lists v7.1
- The GitHub release page is live
