# Fuse2rob

Fuse2rob is a Fusion 360 script for exporting robot assemblies from Fusion 360 to URDF, with optional STL mesh export. It is designed for workflows where the Fusion model is the source of truth for link geometry, joint placement, mass properties, and basic robot structure.

The project is based on ideas from `fusion2urdf` and `Fusion2PyBullet`, but has been adapted for the current Fuse2rob workflow, including Isaac Sim-oriented export behavior and handling for virtual links used in closed-chain-style mechanisms.

## What It Exports

Fuse2rob generates a robot description package containing:

```text
<robot_name>_description/
├── urdf/
│   └── <robot_name>.urdf
└── meshes/
    ├── <link_name>.stl
    └── <link_name>_<body_name>_<index>.stl
```

Depending on the selected settings, the URDF can include:

- links with inertial properties from Fusion 360
- joints from Fusion 360 joint definitions
- visual meshes
- collision meshes
- sub-mesh exports for links made from multiple visible bodies
- an Isaac Sim orientation correction link/joint
- optional pyBullet starter script

## Main Features

- Fusion 360 GUI for export settings.
- URDF export from the active Fusion design.
- STL mesh export from visible bodies.
- Optional sub-mesh export.
- Joint preview before generation.
- Configurable document units and target units.
- Configurable joint parent/child interpretation.
- Inertia calculation using Fusion 360 physical properties.
- Basic Isaac Sim compatibility support.
- Virtual-link handling for mechanisms that need a URDF tree representation.

## Requirements

- Autodesk Fusion 360
- Python environment provided by Fusion 360
- A Fusion design organized as components and joints
- At least one grounded component to act as the base/root link

Fuse2rob is intended to run inside Fusion 360 as a script or add-in. It is not a standalone command-line tool.

## Installation

1. Download or clone this repository.
2. Open Fusion 360.
3. Go to **Utilities → Scripts and Add-Ins**.
4. Click the green **+** button to add a script.
5. Select the Fuse2rob script folder.
6. Run the script from **My Scripts**.

The script entry point is:

```text
fuse2bot.py
```

When launched, it opens a Fusion 360 dialog for export settings.

## Fusion Model Setup

Before exporting, prepare the Fusion model carefully:

1. Each moving robot part should be a Fusion component.
2. Bodies that should be exported must be visible.
3. Hidden bodies are ignored for mesh export and subtracted from mass calculations where possible.
4. Define Fusion joints between components.
5. Ground the component that should become the robot base.
6. Use clear component and body names; Fuse2rob will sanitize names for URDF compatibility.

Names are converted to URDF-safe names by:

- replacing spaces and `:` with safe characters
- replacing `-` with `_`
- converting to lowercase
- simplifying version suffixes like `v28_1`

## Running an Export

1. Open the Fusion design.
2. Run Fuse2rob.
3. Choose an output directory.
4. Select the export options.
5. Use **Preview Links** to inspect joint parent/child relationships.
6. Click **Generate**.

Fuse2rob creates a `<robot_name>_description` folder in the selected output directory.

## Export Options

### Save Mesh

When enabled, Fuse2rob exports STL meshes to the `meshes/` directory.

Disable this when you only want to regenerate the URDF quickly.

### Sub Mesh

When enabled, each visible body under a top-level component can be exported as an individual STL and referenced separately in the URDF.

This is useful when one robot link is made from multiple bodies.

### Mesh Resolution

Controls STL export quality:

- `Low`
- `Medium`
- `High`

Higher resolution produces larger STL files.

### Inertia Precision

Controls Fusion 360 physical-property calculation accuracy:

- `Low`
- `Medium`
- `High`

Use higher precision when mass properties are important for simulation.

### Document Units and Target Units

Fuse2rob can convert from the Fusion document units to the target URDF units.

Supported units:

- `mm`
- `cm`
- `m`

Typical simulation output should use meters.

### Joint Component 1

Controls whether Fusion joint component 1 is interpreted as the URDF parent or child.

Options:

- `Parent`
- `Child`

Use **Preview Links** to verify the resulting parent-child structure before generating.

### Target Platform

Current options:

- `IsaacSim`
- `None`
- `pyBullet`

For `pyBullet`, Fuse2rob can also write a small `hello_bullet.py` loading example.

## Isaac Sim Notes

Fuse2rob currently writes an additional fixed joint for Isaac Sim orientation correction:

```xml
<link name="world_corrected"/>
<joint name="world_to_base" type="fixed">
  <parent link="world_corrected"/>
  <child link="<base_link>"/>
  <origin xyz="0 0 0" rpy="1.5708 0 0"/>
</joint>
```

When importing into Isaac Sim:

- enable self-collision if robot links should collide with each other
- visualize collision geometry to confirm mesh import
- verify that `package://meshes/...` paths resolve correctly
- check joint axes and limits after import
- avoid relying on URDF alone for true closed-chain dynamics

## Virtual Links

URDF is a tree format and does not represent closed-loop mechanisms directly. Fuse2rob handles duplicate-child situations by creating virtual links with a `_virtual` suffix.

This is useful for mechanisms such as four-bar linkages where Fusion may define relationships that do not map cleanly to a URDF tree.

Current virtual-link behavior:

- virtual links are created when a link would otherwise be assigned more than one parent
- real and virtual links can share mass and inertia
- virtual links can be exported without visual or collision meshes
- a near-locked virtual anchor joint is created between the real link and virtual link

The virtual mesh behavior is controlled in `parts.py`:

```python
self.include_virtual_meshes = False
```

With this set to `False`, virtual links export inertial data only. To restore visual and collision meshes for virtual links, set it to `True`.

## Collision Meshes

Real links receive collision meshes by default. Virtual links may intentionally omit collision meshes when `include_virtual_meshes` is disabled.

If parts do not collide in Isaac Sim, first check:

1. The URDF contains `<collision>` tags for the real links.
2. The STL files exist in the expected `meshes/` folder.
3. Isaac Sim resolved the mesh paths correctly.
4. Isaac Sim self-collision was enabled during import.
5. Collision visualization shows the expected geometry.

## Output Details

### URDF

The URDF writer creates:

- XML header
- robot tag
- default silver material
- Isaac Sim world correction link/joint
- link definitions
- joint definitions

### Meshes

Mesh export uses visible Fusion bodies. For each visible top-level occurrence, Fuse2rob exports a component-level STL. If sub-mesh export is enabled, it also exports individual body STLs.

## Code Structure

```text
fuse2bot.py
core/
├── ui.py
├── manager.py
├── parser.py
├── parts.py
├── io.py
├── transforms.py
└── utils.py
```

### `fuse2bot.py`

Fusion 360 script entry point. It gets the active application, design, and root component, then launches the UI.

### `ui.py`

Builds the Fusion 360 dialog and handles button events for preview and generation.

### `manager.py`

Converts UI settings into parser/export configuration. It controls units, mesh accuracy, inertia accuracy, joint order, and target platform.

### `parser.py`

Reads the Fusion design structure, extracts bodies, joints, mass properties, links, virtual links, and joint definitions.

### `parts.py`

Defines URDF `Link` and `Joint` XML generation.

### `io.py`

Writes the URDF file and exports STL meshes.

### `transforms.py`

Contains inertia transform utilities, including conversion from world-origin inertia to center-of-mass inertia.

### `utils.py`

Contains URDF name formatting helpers.

## Limitations

- URDF does not natively support closed-loop mechanisms.
- Virtual links are a workaround, not a full physics constraint model.
- Isaac Sim may require additional import settings for self-collision and articulation behavior.
- Mesh path resolution depends on how the generated package is loaded.
- Only visible bodies are exported.
- Fusion joint setup quality directly affects URDF quality.

## Recommended Workflow

1. Keep the Fusion component tree clean.
2. Ground exactly the intended base component.
3. Hide construction/helper bodies that should not export.
4. Define all required joints in Fusion.
5. Run Fuse2rob and preview joint relationships.
6. Generate URDF and meshes.
7. Inspect the URDF for expected links, joints, inertials, visuals, and collisions.
8. Import into Isaac Sim with self-collision enabled when needed.
9. For closed-chain mechanisms, validate behavior in simulation and tune the virtual-link workaround if necessary.

## Troubleshooting

### The wrong component is the base link

Check which Fusion component is grounded. Fuse2rob uses the grounded component as the base/root link.

### Joint parent and child are reversed

Change the **Joint Component 1** option and use **Preview Links** before exporting again.

### Meshes are missing

Check that:

- `Save Mesh` is enabled
- the bodies are visible in Fusion
- STL files were written to `meshes/`
- URDF mesh paths match the generated filenames

### Parts do not collide in Isaac Sim

Check that:

- the URDF has `<collision>` tags
- Isaac Sim imported the collision geometry
- self-collision was enabled during import
- virtual links are not expected to collide if virtual meshes are disabled

### Simulation behaves strangely around virtual links

Check:

- virtual link mass and inertia split
- virtual anchor joint limits
- joint axes
- closed-chain assumptions
- whether the mechanism should instead be controlled with an Isaac/PhysX constraint or controller

## Credits

Fuse2rob builds on ideas from:

- `fusion2urdf` by syuntoku14
- `Fusion2PyBullet` by yanshil

Thanks to the original authors and communities for making Fusion-to-robot-description workflows accessible.

## Project Status

Fuse2rob is a practical internal/export tool for Fusion 360 to URDF workflows. It is especially focused on getting useful robot models into simulators such as Isaac Sim, while preserving a simple and inspectable codebase.
