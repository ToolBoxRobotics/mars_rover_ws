# Meshes

No mesh files ship with this workspace yet. The active model
(`rover.urdf.xacro` and everything it includes) uses simple box/
cylinder primitives throughout, matching this project's established
style, and doesn't need any mesh files to render.

`rover_description/reference/opportunity_style_template.urdf.xacro` -
a corrected, valid reference template adapted from a user-supplied
file, not currently wired into the active model - does reference mesh
files here (`package://rover_description/meshes/body_box.stl`,
`arm_j1.stl`, etc.) that don't exist. That file's own xacro expansion
and structural validity don't require the files to actually be
present (verified: `xacro` expansion and a full link/joint/transmission
consistency check both pass cleanly without them) - they'd only be
needed to actually see the model rendered in RViz or Gazebo.

If mesh-based visuals are wanted later, drop the corresponding `.stl`
files here with names matching the `<xacro:property>` declarations at
the top of that reference file, and it should render as-is.
