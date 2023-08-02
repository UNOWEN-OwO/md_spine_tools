# Yu-Gi-Oh Master Duel Live2D Spine Animation Blender Import Tools

Blender Python addon to import spine2d mesh and animation data to 3d model in blender.

## Background

This addon is designed mostly for Master Duel Spine animation data that uses spine 4.0, also due the engine difference, part of the animation data will be lost or unable to replicate perfectly in blender during the importing (or just did not find the correct way).

So the purpose for this addon is let me able to have spine animation converted to a more general model format for those non-spine support environment.

For official uses, Spine has published many runtimes that support most of the game engine and environments.

Spine runtimes: http://esotericsoftware.com/spine-runtimes

Spine json format: http://esotericsoftware.com/spine-json-format

Spine user guide: http://esotericsoftware.com/spine-user-guide

![winda](https://github.com/UNOWEN-OwO/md_spine_tools/assets/41463621/e5f5e80e-788d-4584-9445-4ff7eebb84da)

## Requirements

- Only tested on Blender 3.5 and 3.6, should work on 3.3+.
- Python 3.7 as the blender python version is 3.7.
- Spine 4.0 json format, some Master Duel Spine animation may not load correctly event on Spine's official editor.

![ss](https://github.com/UNOWEN-OwO/md_spine_tools/assets/41463621/53cc934f-c31a-4513-a5c5-16a20d8d89f1)

## Installation

1. Place this addon folder to your blender addon folder: `\Blender\3.X\scripts\addons`
2. Enable this addon in blender addon list: `Edit > Preferences > Add-ons > Import-Export: MD Spine Tools`

![zeus](https://github.com/UNOWEN-OwO/md_spine_tools/assets/41463621/439c30d7-7b2d-4bc2-8095-004c6cce5bbe)

## Usage

Place the spine json file, atlas text file and atlas images (non-separate) in the same folder.

In 3D View's sidebar, select the spine json file and atlas text file first to import spine mesh and material. **This will remove all existing objects in `bpy.data` like `meshes` `materials` `armatures` `actions` `collections`**

Then select animation to be load (default last animation, as MD uses the latest).

**Separate Material** will create material due to atlas images but not mesh, will disable the rgba keyframe feature (blend in and out).

**Layer Gap** states the render order and distance between each mesh, change its location on object mode to adjust order.

**IK Pole** will create a guide bone for ik constraint, not automatically bind with the modifier, blender does not have ik positive/negative option, if a bone does not bend correctly, you can fix it by bind it at the IK modifier to the ik pole and adjust the pole angle.

**Create Static Action** will create an action for the spine when no animation frame is applied.

**Toggle Armature Constrain** toggle the transform constrain from `rootControl` to `root`. The `rootControl` armature applies the actual animation, controlling the `root` armature, it is where actions apply. However, due to how spine animation works, the `rootControl` armature is impossible to edit in edit mode, where `root` armature can represent the static pose at edit mode.

**Alternative Mesh** Due to the difference of blender and spine handle weights and rotation, some meshes may not load correctly, this option will generate meshes for each `mesh` attachment in `skin` with an alternative approach and stored in `AlternativeMesh` collection.

![ezgif-1-af337bb720](https://github.com/UNOWEN-OwO/md_spine_tools/assets/41463621/9658bac1-38e1-4ec3-98f9-5d0f78b9aaab)

## Known Issues & Current Limitations

- Too small layer gap can cause some meshes render incorrectly on order, slightly adjust the viewport will render normally.
- Nearly impossible to edit mesh and armature in edit mode if load with animation, you can use legacy load to fix it if you have any enquires.
- No export to spine data, Spine's official editor already has this feature.

The following limitations are for general spine data.

- Unsupported for binary spine data.
- Unsupported for separated atlas images.
- Unsupported for rgb keyframe, only alpha.
- May need to dig deeper on transform constraint.
- Only the default skin will be loaded.
- Unsupported data: `Shear`.
- Unsupported skin data: `linkedmesh`, `boundingbox`, `point`.
- Unsupported animation data: `deform`, `attachments`.

## Contribution

Feel free to open an issue or pull request.

As this addon has already served my purpose, I am not guarantee to maintain this addon in the future.

## License

[MIT](LICENSE) © UNOWEN-OwO
