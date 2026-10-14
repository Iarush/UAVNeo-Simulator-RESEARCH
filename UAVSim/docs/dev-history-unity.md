# Unity / C# side — development history

Reconstructed from Claude Code session transcripts, parsed programmatically (user
messages and assistant text only; tool-result blobs dropped, file edits recovered
separately from tool-call payloads).

## Sources

| Session | Span | Role |
|---|---|---|
| `a30b06be` | 2026-07-27 04:44 → 07-28 15:10 | The bulk of the bug work: STL parsing, axis conventions, propellers, the `CustomDroneBuilderUI` rewrite and its failures |
| `b5d78f3e` | 2026-07-28 15:18 → 07-29 14:41 | `ImportPageUI` creation and Inspector wiring |
| `44bace66` | 2026-07-28 15:18 → 08-01 03:15 | Continuation of `b5d78f3e`; `MotorQuadrants`, `NodePlacementUI`, the `ReplaceDefaultVisual` rewrite |

> **Note on the session list.** `44bace66.jsonl` carries `sessionId: b5d78f3e-…` in its
> first record — it is a **resume of `b5d78f3e`**, not an independent session. Their
> first ~353 transcript lines are identical. `b5d78f3e`'s only unique content is its
> final branch (2026-07-29 14:41), a read-only loadout audit that was cut off after
> one assistant turn. Everything else attributed below to `b5d78f3e` also appears in
> `44bace66`.

### Every C# file edited, across all three sessions

Recovered by enumerating `Edit`/`Write` tool calls:

| File | Edits | Sessions |
|---|---|---|
| `CustomFrameLoader.cs` | 26 | `a30b06be`, `44bace66` |
| `CustomDroneBuilderUI.cs` | 7 (1 full rewrite + 6 repairs) | `a30b06be` |
| `NodePlacementUI.cs` | 4 (1 create) | `44bace66` |
| `ImportPageUI.cs` | 5 (1 create) | `b5d78f3e`/`44bace66` |
| `StlMeshLoader.cs` | 3 (1 create) | `a30b06be` |
| `MainMenu.cs` | 4 | `b5d78f3e`/`44bace66` |
| `MotorQuadrants.cs` | 1 (create) | `44bace66` |
| `PropellerSpin.cs` | 1 | `a30b06be` |
| **`Flight.cs`** | **1** | `a30b06be` |
| `AxisDiagnosticBuilder.cs`, `SanityCheck.cs`, `CustomDroneBuilderSceneSetup.cs` | 10 | `a30b06be` (temporary, all deleted) |

---

## 1. Bug timeline

### ⚠️ One of the four bugs you named is not in these transcripts

> **Flag — "Flight.cs silently overwriting custom COM/inertia every `Start()`".**
> I could not corroborate this. **`Flight.cs` was edited exactly once across all three
> sessions** (`a30b06be`, 2026-07-27 13:38), and that edit added a single diagnostic
> `Debug.Log` line — nothing else:
>
> ```csharp
> Debug.Log($"[Flight] Applied to rigidbody: mass={this.rBody.mass:F4} centerOfMass={…} …");
> ```
>
> No transcript records diagnosing or fixing a `Start()` clobber. What *is* recorded
> is the ordering **as a working design** (`44bace66`): lines 1 and 3 of the startup
> log "come from `CustomFrameLoader.Awake()` (`[DefaultExecutionOrder(-100)]`); 4–6 are
> `Start()` calls, so they always come after."
>
> Possible explanations I cannot choose between: the bug was found in a session not
> among these three; it was fixed before 2026-07-27; or the recollection is of the
> *propeller* lifecycle-ordering bug (#4 below), which is a genuine Awake-vs-`Start`
> ordering defect that was diagnosed and fixed. I am not asserting any of these.
> `Flight.cs` is recorded in CLAUDE.md as never audited.

The other three you named are all present: #2 (152 m drone), #3/#5/#7 (axis swap and
winding), #4 (propellers ignoring custom motor placement).

### Chronological list

**#1 — `ReplaceDefaultVisual` looked for `MeshFilter` on the wrong GameObject**
`a30b06be`, 07-27 04:44
*Symptom:* mesh swap silently did nothing. *Cause:* `GetComponent<MeshFilter>()` on the
script's own GameObject, but `CustomFrameLoader` lives on `Drone` while the FBX instance
is a child. *Fix:* `GetComponentInChildren<MeshFilter>(true)`. The old `AddComponent`
fallback was removed too — it "was silently creating an empty MeshFilter/MeshRenderer on
the root when the lookup failed, masking the real problem."

**#2 — no mm→m scaling: the 152-metre drone**
`a30b06be`, 07-27 05:25
*Symptom:* absurdly oversized mesh. *Cause:* "no mm-to-meters (or any) scaling currently
exists anywhere in the STL parsing path." *Fix:* `0.001f` applied in `AddTriangle`, chosen
because it "is the single place both binary and ASCII parsing paths converge before
vertices are pushed into the mesh list." Applied to positions only, not the normal.

**#3 — CAD Z-up vs Unity Y-up**
`a30b06be`, 07-27 05:48
*Fix as first written:* `SwapYZ`, i.e. `(x,y,z) → (x,z,y)`, applied to normal and all
three vertices. Flagged **in the same message** as a reflection, not a rotation:
"swapping two axes is a mirror reflection, not a rotation — it flips triangle
winding/handedness." That flag is what produced #5.

**#4 — propellers hardcoded to the original FBX positions**
`a30b06be`, 07-27 05:53
Added `PropellerSpin.SetPropellerPositions(Vector3[])` and
`CustomFrameLoader.RepositionPropellers()`. Two premise corrections were found while
implementing:

1. *`FindPropellers()` has no FL/FR/BL/BR order.* It sorts alphabetically
   (`Propeller_0`..`_3`) "so the array index matches `Flight.MotorSpeeds` order."
   Assigning `positions[i] → propellers[i]` "would silently scramble the
   `Flight.MotorSpeeds ↔ propeller` correspondence." Instead each incoming corner is
   matched to **whichever propeller currently occupies that quadrant**.
2. *Lifecycle ordering.* `CustomFrameLoader` is `[DefaultExecutionOrder(-100)]` and calls
   from `Awake()`, but `FindPropellers()` runs in `Start()` — "all `Awake()`s run before
   any `Start()`, so `propellers` would still be `null`." Fixed by having
   `SetPropellerPositions` call `FindPropellers()` itself when `propellers` is null.

An unresolved caveat was raised at the time: `SetPropellerPositions` writes
`localPosition`, "relative to each propeller's *immediate* parent, not necessarily the
`Drone` transform," and the FBX hierarchy could not be inspected.

**#5 — winding-order swap added to compensate for the #3 mirror**
`a30b06be`, 07-27 05:58
*Symptom reported:* mesh "visually flicker/appear to spin as lighting hits inverted
faces." *Fix:* swap `b` and `c` after the axis swap. The reasoning was worked through
rather than assumed:

> the Y/Z swap is a reflection (determinant -1), so `cross(Sb-Sa, Sc-Sa) = -S(cross(b-a, c-a))` …
> Reordering to `(a, c, b)` flips the sign of that cross product back

Consequently no separate normal negation was needed, and the zero-normal cross-product
fallback — computed *after* the reorder — came out consistent automatically.

**#6 — `com` applied raw, without the axis conversion**
`a30b06be`, 07-27 13:22
*Cause:* `com`, `moments` and `rotation` were "read straight off the UDP response and used
**as-is** — none of them go through the `SwapYZ` + mm→m conversion the mesh vertices get."
*Fix:* `com` converted. `moments`/`rotation` were **deliberately left alone** at this point
because `SwapYZ` is determinant −1 and "Unity's `Quaternion` can only represent proper
rotations — so naively applying the same fix here isn't valid math."

**#7 — mesh upside-down; `SwapYZ` replaced by a proper rotation, winding swap removed**
`a30b06be`, 07-27 13:32
*Symptom:* user reported "the large body extrusion part of the drone is facing upwards…
that is really the bottom." *Fix:* a proper +90° rotation about X, `(x,y,z) → (x,-z,y)`.
Three consequences, all in one change:
- the winding swap from #5 was **removed** — "that was only needed to compensate for the
  old transform being a mirror… this new transform is a genuine rotation (det = +1), so
  triangle winding comes out correct on its own";
- `com` switched to the new rotation;
- `rotation` became composable and was fixed: `rotation = ZUpToYUpRotation * rawRotation`.
  "I'd flagged this as unsafe to touch earlier specifically because the old reflection
  couldn't be composed with a quaternion. Now it can."

So #5 was a correct fix for #3 that became wrong the moment #3's approach changed.
Both the swap and its removal are in the record.

**#8 — `RepositionPropellers` read node positions with no conversion**
`a30b06be`, 07-27 13:44
*Evidence:* propeller Y came out `±0.695`/`±0.655` with Z collapsed to `0.000` for all
four — "the STL's Z-up 'height' data landing straight into Unity's Y slot un-rotated."
*Fix:* route motor positions through `ConvertZUpToYUp` before classifying.

> Important qualifier recorded three turns later (07-27 13:52): this fix is **visual
> only.** Motor mixing "never reads any propeller's actual position — it uses a single
> scalar `armEffective` … plus hardcoded per-motor sign constants." So #8 could not have
> caused the physics instability being chased.

**#9 — `com` over-scaled (regression introduced by #6's fix)**
`a30b06be`, 07-27 13:44
*Symptom:* `com=(0.0000, 0.0000, 0.0000)` — "suspiciously exact for a real part."
*Cause:* the mm→m factor applied to `com` in #6 was wrong; live telemetry showed
`moments` (0.0047–0.0100) and `armLength` (0.2128) arriving already in SI units.
*Fix:* scale factor removed. Next run logged `com=(0.0000, -0.0236, 0.0002)`.

**#10 — `ReplaceDefaultVisual` assigned the STL to a propeller's MeshFilter**
`a30b06be`, 07-27 14:17
*Symptom:* "it spins visually, but the spin does not do anything" — user credited a
friend's theory that a parent was spinning its children. *Cause:*
`GetComponentInChildren<MeshFilter>(true)` "has no filtering at all against the propeller
meshes… Unity's traversal order isn't guaranteed to hit the main body first."
`PropellerSpin.Update()` then rotates that transform "since it only spins the local
transform and never touches the rigidbody or `MotorSpeeds`." A second fragility was fixed
in the same pass: `MeshRenderer` was found by a *separate* `GetComponentInChildren`
search, so it could resolve to a different GameObject. *Fix:* skip children named
`Propeller_*`, take the renderer from the same GameObject as the filter. **This fix was
itself insufficient — see #19.**

**#11 — `ReadFloat` missing after the `StlMeshLoader` extraction**
`a30b06be`, 07-27 15:27
Compile error. `CustomFrameLoader` kept its own copy for the UDP response; the new
`StlMeshLoader` had none. Fixed by giving `StlMeshLoader` a private copy. A sweep
confirmed "`ReadFloat` is the only missing helper."

**#12 — `GetComponentInChildren<T>()` skips inactive objects**
`a30b06be`, 07-27 15:33 → 17:43
*Symptom:* `CustomDroneBuilderUI was not found in MainMenu hierarchy`, repeatedly.
*Cause:* `MainMenu.Awake()` called `GetComponentInChildren<CustomDroneBuilderUI>()`
without `includeInactive`. Recurred on a *different* GameObject fileID later, and a
scene-save race was identified: with `Main.unity` open live, "if your in-memory scene
still has the *old* broken state and you hit Ctrl+S before picking up this disk edit,
it'll overwrite my fix again — exactly what seems to have happened last time." The same
class of bug was fixed again in `MainMenu.cs` on 07-29 with an explicit comment:
"Include inactive objects so the pane is still found once `Start()` has hidden it."

**#13 — pre-existing `MainMenu.Start()` NullReferenceException**
`a30b06be`, 07-27 15:47
Unrelated to the custom-drone work: the `NoUsername` pane was saved inactive. Found
while chasing #12. All five pane lookups were cross-checked in the same pass.

**#14–#16 — the procedural `CustomDroneBuilderUI` failure chain**
`a30b06be`, 07-27 21:48 → 22:42. See §3.
- **#14** `nodePlacementPage` was created full-screen but never `SetActive(false)` at
  construction — only later inside `ResetWizard()`. If `OnEnable()` didn't fire cleanly,
  "both pages stay active simultaneously, full-screen, as siblings," the later one
  covering the former. Fixed at construction time.
- **#15** `EnsureUIBuilt()` set `uiBuilt = true` *before* calling `BuildUI()`, so a throw
  partway through left the flag set and the UI permanently half-built.
- **#16** `Button.targetGraphic` was never assigned, "so Unity has no `Image` to reference
  for the button's visual states." A sweep confirmed the dropdown template's `Toggle`
  already had its own set correctly. Explicitly *not* claimed as the root cause: "I'm not
  certain yet this was *the* crash… I still don't have the actual stack trace."

**#17 — `ImportPageUI` backdrop not drawn / panel not blocking the menu**
`b5d78f3e` + `44bace66`, 07-29 14:01 → 14:27
A cluster of scene-level issues: the import UI rendered on top of the live main menu
rather than covering it; the panel "already exists and is set up correctly — it's just
being suppressed by one leftover component." `MainMenu.cs` was retargeted from
`CustomDroneBuilderUI customDroneBuilderPane` to `ImportPageUI customDroneImportPane`
in the same window.

**#18 — `DebugFrameTestSetup`'s hardcoded personal path, live on the normal route**
`44bace66`, 07-29 15:10
*Status at audit:* **PARTIAL** — "clean in C#, still live in the scene." Both C# defaults
were `""`, but `Main.unity:18750` serialized
`TestSTLPath: C:\Users\Owner\Downloads\Part Studio 1 - Part 1.stl`, and a serialized
Inspector value overrides a field initializer. Because `DebugFrameTestSetup` is
`[DefaultExecutionOrder(-200)]` versus `CustomFrameLoader`'s `-100`, and the statics
persist across scene loads, "any level launched from the menu — including one where the
user never opens the import panel — inherits it." Not fixed: it requires a scene edit.

**#19 — `ReplaceDefaultVisual`'s name test only checked the node's own name**
`44bace66`, 07-31 19:59 → 08-01 02:59
The #10 fix was incomplete. The exclusion is
`candidate.gameObject.name.StartsWith("Propeller_")` — "there is no `GetComponentInParent`,
no `transform.parent` loop, no path check anywhere in the method. **So yes: if a
propeller's mesh sits on a CHILD of `Propeller_*`, that child is NOT excluded.**"

The prefab could not be enumerated statically — `DronePlayer.prefab` has zero MeshFilters
of its own, sourcing everything from a binary FBX whose `.meta` has
`internalIDToNameTable: []`. What the prefab did reveal: **7273** `m_Layer` overrides
(≈ node count) against only **5** `m_Name` overrides (`Propeller_0/1/2/3`, `DroneModel`).

Two symptoms were explicitly separated: the spin is diagnostic (a mesh can only spin if
it is a descendant of a `Propeller_*`), while the propellers being invisible "proves
nothing" — the old code disabled *every* renderer except the chosen one anyway.

*Fix (07-31 → 08-01):* replaced entirely with explicit serialized references —
`customFrameVisual` (a `MeshFilter` on a purpose-built child) and `defaultBodyObjects`
(a `GameObject[]` the user populates). Removal was verified by count: the
~7273-renderer loop, the per-renderer `GetComponentInParent<Camera>()`, the
`GetComponentsInChildren<MeshFilter>(true)` search and the `StartsWith("Propeller_")`
test all went to **0** occurrences.

**#20 — NaN `armLength` reaching the motor mixer**
`44bace66`, 07-31 05:48 ("FIX 3", implemented in isolation on request)
A non-finite `armLength` would propagate `CustomFrameLoader:73 → Flight.cs:466 → Flight.cs:50`
`armEffective => armLength * sqrt(2)/2`, giving "a NaN moment arm in the motor mixing."
Closed with an `IsFinite` guard.

**#21 — pipeline would run with an invalid motor layout**
`44bace66`, 07-31 05:53 ("FIX 2")
`TryValidateMotorLayout` inserted after `File.Exists`, before any socket traffic. Side
effect noted at the time: `RepositionPropellers`' own duplicate/missing-corner checks
became "dead in practice."

**#22 — "the custom STL stopped loading" after the #19 rewrite**
`44bace66`, 08-01 03:15 — *diagnosis only, session ends here*
Diagnosed as an **unassigned `customFrameVisual`**, which logs a red error, destroys the
mesh and falls back to the stock drone — while `Awake()` continues, so all six normal log
lines still appear. "Only the visual is missing; physics is still custom. That's the
pre-existing quirk I flagged, and it makes this failure easy to misread as 'nothing
happened.'" Two decoys were named: an empty `defaultBodyObjects` (yellow warning only —
the STL loads but the default body draws over it), and a missing Material (renders
magenta, no console output).

> **Flag.** #22 was never confirmed fixed. The transcript ends immediately after the
> diagnosis, with the prefab wiring left to the user.

---

## 2. STL parsing

All of the below lives in `StlMeshLoader.cs`, extracted from `CustomFrameLoader.cs` on
2026-07-27 14:40 (`a30b06be`) so "both the flight-loading path and the new UI's preview
call the same `TryLoadStl`/`ConvertZUpToYUp`, no duplication."

### Binary vs ASCII

Two parse paths — `ReadBinaryStl` and `ReadAsciiStl` — selected by a branch at
`StlMeshLoader.cs:51-54`. **They converge on a single `AddTriangle`** (binary calls it at
`:90`, ASCII at `:120`). That convergence is the load-bearing design fact: every
correction below is applied once, inside `AddTriangle`, rather than twice.

Normals are handled entirely manually — there is **no `RecalculateNormals()` call
anywhere.** The resolved normal is pushed three times, once per vertex (flat/faceted
shading, no smoothing), and `mesh.SetNormals(normals)` assigns exactly what `AddTriangle`
produced. `RecalculateBounds()` is called, but "bounds ≠ normals."

The zero-normal fallback `Vector3.Cross(b - a, c - a).normalized` fires when the STL
omits or degenerates the facet normal, and is deliberately computed **after** the axis
conversion so it stays consistent with it.

### mm → m scaling

Constant at `StlMeshLoader.cs:14` (`MillimetersToMeters = 0.001f`), applied at `:146-148`.
Audited 2026-07-29 (`44bace66`) as **LANDED, exactly once**: "A project-wide grep returns
only those three multiplication sites." `CustomFrameLoader` references the constant only
in comments that document *not* re-applying it to node positions or the returned centre
of mass.

Needed because CAD exporters (the reference file is an OnShape "Part Studio" export) emit
millimetres while Unity is metre-based — see bug #2.

### Axis conversion

`StlMeshLoader.cs:22` — `Quaternion.AngleAxis(90f, Vector3.right)` — applied to the normal
and all three vertices at `:137-140`. Order note from the audit: "the rotation is applied
before the mm scaling. That's harmless — a rotation and a uniform scale commute."

Needed because CAD tools are Z-up and Unity is Y-up (bug #3).

### Winding-order correction — deliberately absent

Audited as **MISSING**, and the audit concluded that is correct:

> `StlMeshLoader.cs:151-153` adds vertices in source order `a, b, c` … **There is no
> 2nd/3rd swap anywhere in the file.** This is deliberate, not an oversight … a
> determinant +1 transform cannot flip winding, and adding a swap on top of it would
> invert your faces.

The history matters here: a winding swap **was** added (bug #5) when the conversion was
the determinant −1 `SwapYZ`, and **was removed** (bug #7) when the conversion became a
proper rotation. The current absence is the second half of that pair.

The audit also raised the trade-off this creates, and did not resolve it:

> CAD exporters (SolidWorks/OnShape) are right-handed Z-up; Unity is left-handed Y-up.
> Changing handedness *requires* a determinant −1 transform. The code's rotation is
> determinant +1 … which means the loaded mesh is mirrored relative to the source model.

Assessed as "nearly invisible" on a symmetric X-quad, visible "on chiral features
(asymmetric mounts, embossed text, handed part geometry)," and recommended as "one open
question rather than two separate fixes."

---

## 3. Node-placement UI

> **Two corrections to the framing in the request, both evidenced.**
>
> **1. There was never an `importPage` field on `MainMenu.cs`.** The field
> `MainMenu.cs` carried was `private CustomDroneBuilderUI customDroneBuilderPane`,
> renamed to `private ImportPageUI customDroneImportPane` on 2026-07-29 14:18
> (`44bace66`, recovered from the edit payload). `importPage` is a **private field
> inside the procedural `CustomDroneBuilderUI`**, created by the rewrite described
> below — it never appeared on `MainMenu`.
>
> **2. The inherited `CustomDroneBuilderUI` was not "half-built scaffolding" with a
> partly-working UI.** It was a dead stub, and the whole thing was thrown away and
> rewritten from scratch on 2026-07-27 14:49.

### What was inherited (before 2026-07-27 14:34)

From the user's own brief opening the work: the UI "was created by duplicating an
unrelated Settings page and is non-functional/chopped." Confirmed on inspection:

> The old `CustomDroneBuilderUI.cs` was indeed a non-functional stub — every serialized
> field was null, it only had a "Background"/"Title" child pair (no real wizard content),
> and it would NullReferenceException on `Start()` if ever shown.

Every `[SerializeField]` on the component in `Main.unity` was `{fileID: 0}`.

One further inherited defect was found in the same pass: a button in `Main.unity` "literally
named `"Custom Model"` with label text **"Add Custom Model"**, sitting exactly on top of the
real Exit button — but wired to call `Exit()`. Clear copy-paste-and-abandon artifact." It was
relabelled and rewired to `MainMenu.ShowCustomDroneBuilder()`, which also fixed the
overlapping-buttons bug.

### What was built: the procedural `CustomDroneBuilderUI` (2026-07-27, `a30b06be`)

A complete rewrite that **builds its entire UI in code** — "no hand-authored prefab." The
scene contains only "a new `CustomDroneBuilder` GameObject… just a `RectTransform` + the
`CustomDroneBuilderUI` component, nothing else. It's an empty shell by design."

Runtime-built hierarchy, as reported on 2026-07-28:

```
CustomDroneBuilder (black translucent background)
├─ ImportPage
│   ├─ Title / BrowseButton (SimpleFileBrowser) / StatusText
│   ├─ DensityRow → DensityLabel, DensityInput (defaults to 1250)
│   └─ ButtonRow → CancelButton, NextButton
├─ NodePlacementPage  (starts hidden)
│   ├─ PreviewColumn → Instructions, PreviewImage (RawImage + DronePreviewOrbit), BackButton
│   └─ NodeListColumn → ListTitle, NodeListScroll, ValidationWarning, ConfirmButton
└─ NodePopup  (starts hidden)
    └─ PopupBox → Title, TypeRow (NodeType dropdown), MassRow, PopupButtonRow
```

Plus a separate 3D rig, deliberately parked far from the scene:

```
DronePreviewRoot          ← world position (0, 20000, 0), "so it can never overlap anything else"
├─ PreviewLight
├─ PreviewMesh            ← MeshFilter + MeshRenderer + MeshCollider; the loaded STL
└─ PreviewCamera          ← renders to a RenderTexture shown in PreviewImage
```

**Node placement** worked by raycast, not snapping. `DronePreviewOrbit.cs` (new, and the
one piece later judged reusable) provides "drag-to-orbit plus click-to-raycast, with
public `Target`, `PreviewCamera`, `RaycastTarget` fields." A click raycast against the
`MeshCollider` opened a popup with a `NodeType` dropdown and a mass field; confirming
appended to a running list.

> **Flag — no snapping.** The request asks about "node placement/snapping." I found no
> snapping, quantisation, or grid logic anywhere in the transcripts. Placement is a raw
> surface raycast hit point. Marker colour per node type exists (`:298`); snapping does not.

**How it populated the statics** (from the later audit, `44bace66` 2026-07-30):
`OnConfirmClicked` validated ≥1 node with a non-blocking Battery warning, then cleared and
repopulated `CustomFrameLoader.SelectedNodes`, set `SelectedSTLPath` and `SelectedDensity`,
and called `mainMenu.BeginSimulation()` — "the exact same transition the default flight
path uses." Export at `:323-332` applied a **Y-up → Z-up** conversion, because points
picked off a Y-up preview mesh must be converted back into the STL frame on the way out.
`StlMeshLoader.ConvertYUpToZUp` was added for exactly this.

### Why it was abandoned

Bugs #14–#16 above, over roughly seven hours, none of which the assistant could observe
directly. The user's verdict on 2026-07-28 15:10 (`a30b06be`, final turn):

> "no no no, I want to rebuild the UI there are way too many errors with the past UI, I
> will create the elements and then I will need you to link up the [scripts]"

### What replaced it

**`ImportPageUI.cs`** — written 2026-07-28 15:21 (`b5d78f3e`) against a hierarchy the user
hand-built (`CustomDroneImportPanel` → Title / BrowseButton / SelectedPathText /
DensityLabel / DensityInput / StatusText / ConfirmButton, all TMP). The brief was explicit:

> Do NOT create, destroy, move, or restructure any GameObjects in the scene — I've already
> built the hierarchy by hand. Only write the script itself.

A mismatch was flagged before any code was written: the existing procedural UI "uses legacy
`UnityEngine.UI` widgets (`Text`, `InputField`, `Button`), not TextMeshPro, and it ignores
whatever's manually built under its GameObject," whereas the hand-built panel was TMP and
had no node-placement or preview step at all.

**`NodePlacementUI.cs`** — written 2026-07-31 05:24 (`44bace66`), after a read-only audit
(2026-07-30) established what could be salvaged. That audit's verdict on the procedural UI:

> **Why it's unusable for your hand-built panel:** every one of those members is `private`,
> and the class builds its own UI procedurally … There are no public `TMP_*` fields to drag
> onto. It also generates legacy `UnityEngine.UI` widgets, not TMP.

Binds six fixed rows located **by name at runtime** under `NodeListContainer` — "only four
drag-assignments total." Three position-field names were corrected on 2026-07-31 14:41
(`PosXInput → XPosInput`, and the same for Y/Z) to match the hierarchy the user had built.
On 2026-07-31 15:01 the enum member `OffboardComputer` was renamed `CompanionComputer` so
dropdown labels and enum names matched end to end; it was verified first that "no code
switches on it," making the rename safe for the wire protocol.

**`MotorQuadrants.cs`** — written 2026-07-31 04:50 (`44bace66`) as the shared FL/FR/BL/BR
contract, so the UI validator and the loader could not drift apart.

---

## 4. `CustomFrameLoader` and `Flight.cs` integration

### Execution order

`CustomFrameLoader` is `[DefaultExecutionOrder(-100)]` and fires from `Awake()`. The
contract is **populate the statics, then load the flight scene** — there is no entry-point
method (audited 2026-07-30, `44bace66`: **MISSING** — "no such method exists by design").

`Awake()` sequence, as recorded: `TryGetMassProperties` (UDP, 5 s timeout) →
`StlMeshLoader.TryLoadStl` → `ReplaceDefaultVisual` → `Flight.ApplyCustomMassProperties`
→ `RepositionPropellers`.

The full expected startup log, used as a staging diagnostic (`44bace66`, 2026-08-01):

| # | Line | Source |
|---|---|---|
| 1 | `[CustomFrameLoader] Mass properties: com=… moments=… rotation=… armLength=…` | `CustomFrameLoader.cs:219` |
| 2 | *(silence — mesh load and visual swap log nothing on success)* | — |
| 3 | `[CustomFrameLoader] Motor positions (Unity-local, post-conversion): FL=… FR=… BL=… BR=…` | `CustomFrameLoader.cs:141` |
| 4 | `[PropellerSpin] Propeller_0 at local(…) → Front Left, target=CW, …` ×4 | `PropellerSpin.cs:84` |
| 5 | `[PropellerSpin] Found 4 propellers, Flight=…` | `PropellerSpin.cs:87` |
| 6 | `[Flight] Applied to rigidbody: mass=… centerOfMass=… inertiaTensor=… armLength=… armEffective=…` | `Flight.cs:493` |

"Lines 1 and 3 come from `CustomFrameLoader.Awake()`…; 4–6 are `Start()` calls, so they
always come after."

Two other ordering facts recorded:
- `DebugFrameTestSetup` at `-200` runs **before** `CustomFrameLoader` at `-100`, so its
  values land first (bug #18).
- `PropellerSpin.FindPropellers()` runs in `Start()`, which is what made bug #4's second
  premise issue real.

### How UDP results get applied

`TryGetMassProperties` parses the 45-byte response, then converts before handing off:
`com` through `ConvertZUpToYUp` (metres already — bug #9), `rotation` composed as
`StlMeshLoader.ZUpToYUpRotation * rawRotation` (`CustomFrameLoader.cs:194`), `moments`
passed through unchanged since "only the frame they're expressed in does [change], and
that's exactly what the rotation composition handles."

Validation before acceptance: all values finite, quaternion `sqrMagnitude > 0`, all three
moments `> 0`, `armLength > 0` — the last added as bug #20's fix.

`Flight.ApplyCustomMassProperties(com, principalMoments, inertiaRotation, newArmLength)`
receives them; `rBody.centerOfMass` and `rBody.inertiaTensorRotation` are what it sets.
Confirmed 2026-07-27 13:21: it "doesn't touch the mesh at all, it only sets
`rBody.centerOfMass` and `rBody.inertiaTensorRotation` for physics."

### What was changed in `Flight.cs`, and why

**One line, once.** A diagnostic `Debug.Log` at `Flight.cs:494`, added 2026-07-27 13:38
"rather than guess further," paired with an equivalent log in `CustomFrameLoader`. Its
purpose was to settle the units question empirically instead of by inference — and it did:
the logged `moments` and `armLength` are what proved the service returns SI units and
triggered the bug #9 fix.

No other `Flight.cs` change exists in these three sessions. Two structural findings were
made *by reading* it, neither acted on:

- **Motor mixing is analytical and ignores geometry.** `Flight.cs:730-743` uses the scalar
  `armEffective` plus fixed sign arrays (`Flight.cs:425,430,435`):
  ```
  motorRollSign  = { -1, 1, 1, -1 }
  motorPitchSign = { -1, -1, 1, 1 }
  motorYawSign   = { -1, 1, -1, 1 }
  ```
  "the physics model **always assumes a perfectly symmetric, idealized X-configuration
  quad** … regardless of what your actual STL geometry looks like."
- **`rBody.mass` is never set from the response.** Logged as `2.0000` — the prefab default.
  "the response is only com+moments+rotation+armLength, 45 bytes total." Flagged, not fixed.

---

## 5. Design decisions and reasoning

**Single conversion funnel.** Every correction (scaling, rotation, and formerly winding)
goes in `AddTriangle` because it is the one point both parse paths converge. Chosen at the
first opportunity (bug #2) and it held: the 2026-07-29 audit could verify "applied exactly
once" with a three-site grep.

**Keep linear algebra out of Unity.** `moments` are passed through untouched and only the
frame is rotated. Consistent with the service contract, which ships principal moments plus
a quaternion precisely so "the C# side only needs to unpack floats and assign them."

**Refuse to compose a reflection with a quaternion.** When `com` was fixed but
`moments`/`rotation` were not (bug #6), the stated reason was that a determinant −1
transform cannot be represented as a `Quaternion`, and "getting it wrong would silently
corrupt flight dynamics rather than fail loudly." The fix was deferred until the transform
became a proper rotation (bug #7) — at which point it was applied immediately.

**Match by quadrant, never by array index.** Both in `SetPropellerPositions` (bug #4) and
in `MotorQuadrants`. Index matching "would silently scramble the
`Flight.MotorSpeeds ↔ propeller` correspondence."

**Explicit references over name heuristics.** The bug #19 rewrite rejected a name-based fix
even though one was available, because it "still picks an arbitrary first part out of
thousands as 'the body'" — "fixes the symptom you noticed without fixing the selection
problem underneath." The chosen design also "matches how the rest of this project is
wired: hand-built objects, explicit drag-assigned references."

**`defaultBodyObjects` as a list, not a single root.** Because the FBX is binary and its
structure was genuinely unknown: "**I cannot tell you whether the body is separable from
the propellers under one root.** … So rather than guess a single root, the field is a
**list you populate**."

**No procedural UI, after the fact.** The hard constraint in CLAUDE.md was earned by the
`CustomDroneBuilderUI` experience (§3). Everything after 2026-07-28 binds to hand-built
objects.

**Store node values as raw strings, not parsed floats** (persistence proposal, 2026-08-01):
"your rules make empty and `"0"` semantically different, and a parse/format round-trip
would destroy that distinction (and rewrite `0.150` as `0.15`)."

**Enum pinned to the wire.** `NodeType : byte` with explicit values, written to the socket
as its raw byte. "reordering the enum would silently change the protocol." This is why the
`OffboardComputer → CompanionComputer` rename was preceded by checking that nothing
switched on it.

**On the "do not modify `CustomFrameLoader`" rule** (`44bace66`, 2026-07-31): its
provenance was audited and found to be **conversational only** — "It is not recorded
anywhere in the repo… The instruction came from your opening message this session… a
future session reading only CLAUDE.md would not know." That audit is what put the
modification policy into CLAUDE.md.

---

## 6. Open questions / unfinished items

**1. `customFrameVisual` prefab wiring was never confirmed done** (bug #22). The last turn
of `44bace66` diagnoses "the custom STL stopped loading" as an unassigned field and hands
the user an 8-step prefab procedure. No transcript confirms it was carried out or that the
symptom cleared. **This is the most recent unresolved state in the Unity work.**

**2. Whether the body is separable from the propellers in the FBX.** Unanswered by
necessity — binary FBX, `internalIDToNameTable: []`, ~7273 nodes, 5 named. The user was
asked to look and the answer is not in the record. The reparenting alternative carries its
own flagged risk: `DroneModel` "does override its `m_LocalScale` and `m_LocalRotation`," so
reparenting changes what `localPosition` means, "a behaviour change on the default drone too."

**3. The mirror.** Raised 2026-07-29 as a matched pair with the winding decision, assessed
as harmless on symmetric frames and wrong on chiral geometry, explicitly deferred.

**4. `rBody.mass` is never set from the service.** Flagged twice, never actioned. Whether
the protocol omission is intentional is unrecorded.

**5. Node-panel value persistence — proposed, never implemented.** A full design landed on
2026-08-01 03:15 (`PlayerPrefs`, one key `UAVSim.NodePlacement.Rows.v1`, raw strings,
`OnEnable` restore, a Clear button, registry path
`HKEY_CURRENT_USER\Software\MIT Beaver Works\UAVSim`). It ends "Confirm and I'll implement."
**The session ends there.** Two caveats were pre-flagged: keys are positional, and STL path
and density are not covered.

**6. `ReplaceDefaultVisual` is still `void` and `Awake()` continues after it fails.**
Called out as "one pre-existing quirk I did not change" — "an unassigned
`customFrameVisual` gives you the default visual with custom physics," despite an in-code
comment claiming failures never partially change the drone.

**7. Duplicate quadrant logic in `PropellerSpin.cs`.** Two more copies at `:59-60` and
`:120-121`, operating in Unity drone-local space rather than the CAD frame, so
`MotorQuadrants.Classify` "cannot be dropped in: it would apply `ConvertZUpToYUp` to an
already-Unity vector." Full unification needs a `ClassifyUnityLocal` overload. Not done.

**8. `TestSetup` still in `Main.unity`** (bug #18) — requires a scene edit, left to the user.

**9. Main-menu "custom drone armed" indicator + Clear** — proposed in detail 2026-07-31,
not implemented. Noted as "currently the only way back to the stock drone without
restarting, since the statics live for the whole process."

### On camera zoom and propeller animation specifically

> **Flag — I found no unresolved camera-zoom item.** The only occurrences of "zoom" in any
> of the three sessions are workflow remarks while viewing the temporary axis diagnostic:
> the user's "I will just zoom in ( I can not find the new pieces)" and an instruction to
> "zoom in further with your scroll wheel." Both were resolved in-session by making the
> diagnostic auto-select its root object, and the whole diagnostic was deleted on
> 2026-07-28. Nothing about a camera-zoom feature or defect appears. If there is such an
> item, it is not in these transcripts.

**Propeller animation** has three distinct threads, of which two are closed and one is open:

- *Closed — the mesh spinning.* Bugs #10 and #19. Cause was `ReplaceDefaultVisual`
  assigning the STL to a node under a `Propeller_*` transform, which `PropellerSpin`
  rotates. Fixed properly by the explicit-reference rewrite.
- *Closed as retracted — the Animator theory.* Recorded in CLAUDE.md as ruled out. Its
  provenance is flagged inside the transcript itself: the Animator/animation-clip theory
  and the `AddForceAtPosition` local-vs-world theory "are from prior sessions I have no
  record of. I recorded both as retracted per your instruction, but tagged" as unverified.
  **They are not verifiable from these three sessions.**
- *Open — propeller visibility and spin on a custom frame.* Raised as a design question
  on 2026-07-31 ("a custom frame currently flies with **no visible propellers**, while
  `RepositionPropellers` still moves them"), answered by the user as a hard requirement
  ("propellers must STAY VISIBLE"), and implemented as the `defaultBodyObjects` list —
  but whether that list can actually exclude the propellers depends on open question #2.
  A runtime warning was added that "names the exact propeller and the root containing it"
  if wired wrong. Whether it ever fired is not recorded.

---

*Compiled 2026-08-22 from read-only analysis of Claude Code session transcripts. No project
source files or transcript files were modified.*
