# Leviathan (Team 9143, Robot A) - articulated model for AdvantageScope

The component order here **must** match the order in which the robot code
publishes the component poses - `RobotState/ComponentPoses` and
`Draggables/Components3d`, both filled in `Dashboard.update()` using the
indices in `Constants.LoggingConstants`:

| File          | Component index | Part                                              |
|---------------|-----------------|---------------------------------------------------|
| `model.glb`   | (none)          | Drive base and static structure; follows the robot pose |
| `model_0.glb` | 0               | Elevator middle stage (rises at half the carriage speed) |
| `model_1.glb` | 1               | Elevator carriage                                 |
| `model_2.glb` | 2               | CorAl arm (pitches about the pivot)               |

`config.json` holds the model name ("Leviathan - 9143A"), the glTF-to-field
rotation and each component's zeroed position / rotation.

**The `.glb` files are not in the repository** - the robot's CAD is not
distributed, and `.glb` / `.step` files under `advantageScopeAssets` are
git-ignored. Export the four parts above from the CAD as glTF binaries (for
example STEP -> glTF with CAD Assistant or Blender), name them as in the
table and put them in this folder.

To use the model, point AdvantageScope at the parent `advantageScopeAssets`
folder (*Help > Use Custom Assets Folder*) and bind the robot's components to
`AdvantageKit/RealOutputs/RobotState/ComponentPoses`. See "3D mechanism
animation" in the repository README.

Check on the first AdvantageScope session after a re-export: if the glTF
components come out in a different order, either re-export in the order above
or remap them in `config.json`. If a part floats or swings the wrong way,
adjust its `zeroedPosition` / `zeroedRotations` there.
