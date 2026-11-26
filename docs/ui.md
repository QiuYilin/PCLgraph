# UI

The `pclgraph-ui-example` runs the taskgraph ImGui workbench with the PCL nodes registered.

## Point Cloud

The example registers one monitor kind, `point-cloud`, titled **Point Cloud**. taskgraph owns the window, docking, and the source toolbar. Each window keeps its own camera and point size.

- **View > Monitor** opens one window when none is open, using this kind.
- **New Monitor**, on the title bar immediately to the right of **Help**, opens another independent window. The title stays **Point Cloud**.
- **Source** and **Follow Selection** come from taskgraph. Follow is on by default and tracks the canvas selection. **Source** pins one `graph / node / port` output.
- The content area is a VTK view of `CloudData` that has XYZ. **Fit** resets that window's camera. **...** changes that window's point size. The status line shows the point type and point count.
- The window starts floating and can be dragged into the dockspace.

## Output

The Output panel summarizes `CloudData` and `TransformData`, including the point type, point count, organized size and fields, and a transform's translation, angle, and axis.
