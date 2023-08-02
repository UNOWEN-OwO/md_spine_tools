import bpy
import bmesh
import json
import math
import mathutils
from pathlib import Path as _Path
import re

from . import MDST_LOGGER


# vertices: For each vertex either an x,y pair or, for a weighted mesh
#   first the number of bones which influence the vertex,
#   then for that many bones: bone index, bind position X, bind position Y, weight.
#   A mesh is weighted if the number of vertices > number of UVs.
class Vertex:
    def __init__(self, vertex_data, single_bone_idx=None):
        self.vertex_data = []
        if single_bone_idx:
            self.bone_idx = [single_bone_idx]
            self.bone_weight = [1.0]
            self.bone_count = 1
            self.vertex_data.append([vertex_data.pop(0), vertex_data.pop(0)])
        else:
            self.bone_idx = []
            self.bone_weight = []
            self.bone_count = vertex_data.pop(0)
            for i in range(self.bone_count):
                self.bone_idx.append(vertex_data.pop(0))
                self.vertex_data.append([vertex_data.pop(0), vertex_data.pop(0)])
                self.bone_weight.append(vertex_data.pop(0))

        if sum(self.bone_weight) != 1.0:
            self.bone_weight[0] = 1.0 - sum(self.bone_weight[1:])

    def local_pos(self):
        return [
            sum(self.vertex_data[idx][0] * self.bone_weight[idx] for idx in range(self.bone_count)),
            0,
            sum(self.vertex_data[idx][1] * self.bone_weight[idx] for idx in range(self.bone_count))
        ]

    def global_pos(self, bone_list):
        return [
            sum(((self.vertex_data[idx][0] * math.cos(bone_list[b_idx].abs_rotation) - self.vertex_data[idx][1] * math.sin(bone_list[b_idx].abs_rotation)) * bone_list[b_idx].abs_scale_x + bone_list[b_idx].abs_x) * self.bone_weight[idx] for idx, b_idx in enumerate(self.bone_idx)),
            0,
            sum(((self.vertex_data[idx][1] * math.cos(bone_list[b_idx].abs_rotation) + self.vertex_data[idx][0] * math.sin(bone_list[b_idx].abs_rotation)) * bone_list[b_idx].abs_scale_y + bone_list[b_idx].abs_y) * self.bone_weight[idx] for idx, b_idx in enumerate(self.bone_idx))
        ]

    def euler_pos(self, bone_list, mode='xyz', t='global'):
        x, y, z = eval(f'self.{t}_pos(bone_list)')
        local = locals()
        return [eval(i, local) for i in mode]


class Bone:
    def __init__(self, idx, bone_data):
        self.length = 0
        self.x = self.y = self.abs_x = self.abs_y = self.dx = self.dy = 0
        self.rotation = self.abs_rotation = self.roll = 0
        self.scaleX = self.scaleY = self.abs_scale_x = self.abs_scale_y = 1.0
        self.transform = 'normal'

        self.shearX = self.shearY = 0

        for k, v in bone_data.items():
            setattr(self, k, v)

        if self.shearX or self.shearY:
            MDST_LOGGER.warning('Shear is not supported')

        self.bone_idx = idx
        self.rotation = math.radians(self.rotation)

        if hasattr(self, 'transform') and self.transform == 'noRotationOrReflection':
            self.rotation = 0

    def set_parent(self, parent):
        self.parent_bone = parent

        self.abs_rotation = self.rotation + self.parent_bone.abs_rotation
        self.abs_scale_x = self.scaleX * self.parent_bone.abs_scale_x
        self.abs_scale_y = self.scaleY * self.parent_bone.abs_scale_y

        self.abs_x = self.parent_bone.abs_scale_x * (self.x * math.cos(self.parent_bone.abs_rotation) - self.y * math.sin(self.parent_bone.abs_rotation)) + self.parent_bone.abs_x
        self.abs_y = self.parent_bone.abs_scale_y * (self.y * math.cos(self.parent_bone.abs_rotation) + self.x * math.sin(self.parent_bone.abs_rotation)) + self.parent_bone.abs_y
        # self.abs_x2 = self.abs_x + self.length * math.cos(self.abs_rotation) * self.abs_scale_x
        # self.abs_y2 = self.abs_y + self.length * math.sin(self.abs_rotation) * self.abs_scale_y

        self.dx = self.abs_scale_x * self.length * math.cos(self.abs_rotation)
        self.dy = self.abs_scale_y * self.length * math.sin(self.abs_rotation)

        roll = self.abs_rotation % (2 * math.pi)
        self.roll = - roll if roll < math.pi else (math.pi - (roll % math.pi))


class IK_Bone:
    def __init__(self, ik_data, bone_dict):
        self.order = self.softness = 0
        self.bendPositive = self.stretch = self.compress = self.uniform = False

        self.mix = 1.0

        for k, v in ik_data.items():
            setattr(self, k, v)

        self.target_bone = bone_dict[self.target]
        self.parent_bone = bone_dict[self.bones[0]]
        self.child_bone = bone_dict[self.bones[-1]] if len(self.bones) > 1 else None
        self.chain_length = len(self.bones)


###
# rotation: The rotation to offset from the target bone. Assume 0 if omitted.
# x: The X distance to offset from the target bone. Assume 0 if omitted.
# y: The Y distance to offset from the target bone. Assume 0 if omitted.
# scaleX: The X scale to offset from the target bone. Assume 0 if omitted.
# scaleY: The Y scale to offset from the target bone. Assume 0 if omitted.
# shearY: The Y shear to offset from the target bone. Assume 0 if omitted.
# rotateMix: A value from 0 to 1 indicating the influence the constraint has on the bones, where 0 means no affect, 1 means only the constraint, and between is a mix of the normal pose and the constraint. Assume 1 if omitted.
# translateMix: See rotateMix.
# scaleMix: See rotateMix.
# shearMix: See rotateMix.
# local: True if the target's local transform is affected, else the world transform is affected. Assume false if omitted.
# relative: True if the target's transform is adjusted relatively, else the transform is set absolutely. Assume false if omitted.
class TK_Bone:
    def __init__(self, tk_data, bone_dict):
        self.mixRotate = self.mixX = self.mixScaleX = self.mixShearY = 0

        for k, v in tk_data.items():
            setattr(self, k, v)

        self.target_bone = bone_dict[self.target]
        self.bone_list = [bone_dict[bone] for bone in self.bones]


###
# positionMode: Determines how the path position is calculated: fixed or percent. Assume percent if omitted.
# spacingMode: Determines how the spacing between bones is calculated: length, fixed, or percent. Assume length if omitted.
# rotateMode: Determines how the bone rotation is calculated: tangent, chain, or chain scale. Assume tangent if omitted.
# rotation: The rotation to offset from the path rotation. Assume 0 if omitted.
# position: The path position. Assume 0 if omitted.
# spacing: The spacing between bones. Assume 0 if omitted.
# rotateMix: A value from 0 to 1 indicating the influence the constraint has on the bones, where 0 means no affect, 1 means only the constraint, and between is a mix of the normal pose and the constraint. Assume 1 if omitted.
# translateMix: See rotateMix.
class Path:
    def __init__(self, path_data, bone_dict, attachments):
        for k, v in path_data.items():
            setattr(self, k, v)

        self.bones_list = [bone_dict[bone] for bone in self.bones]

        for k, v in list(attachments[self.target].values())[0].items():
            setattr(self, k, v)

        # FIXME is there a chance path has only one vertex group?
        self.vertices = load_vertex(self.vertices)


class Slot:
    def __init__(self, slot_data, bone_dict, slot_idx):
        for k, v in slot_data.items():
            setattr(self, k, v)
        self.bone_obj = bone_dict[self.bone]
        self.slot_idx = slot_idx


class Atlas:
    def __init__(self, atlas_data, atlas_image):
        data = [i.strip() for i in atlas_data.split('\n') if i]
        self.name = data.pop(0)
        self.atlas_image = atlas_image
        self.rotate = False
        self.bounds = []
        for entry in data:
            k, v = entry.split(':')
            if ',' in v:
                setattr(self, k, [int(i) if i.strip('- ').isdigit() else eval(i.strip().capitalize()) if bool(i) else i.strip() for i in v.split(',')])
            else:
                setattr(self, k, int(v) if v.strip('- ').isdigit() else eval(v.strip().capitalize()) if bool(v) else v.strip())

        # Spine Atlas 4.0 uses bool instead of degrees
        self.rotate = int(self.rotate)
        if self.rotate == 1:
            self.rotate = 90

        # Spine Atlas 4.1 uses bounds
        if self.bounds:
            self.xy = self.bounds[:2]
            self.size = self.bounds[2:]


class AtlasImage:
    def __init__(self, atlas_image_data):
        self.image = re.search(r'.+\.(png|jpg|jpeg)', atlas_image_data).group(0)
        self.size_x, self.size_y = [int(i) for i in re.search(r'size:(?:| )(\d+),(?:| )(\d+)', atlas_image_data).groups()]
        self.filter_x, self.filter_y = re.search(r'filter:(?:| )(\w+),(?:| )(\w+)', atlas_image_data).groups()

        # Spine Atlas 4.0?
        _format = re.search(r'format:(?:| )(\w+)', atlas_image_data)
        self.format = _format.group(1) if _format else 'RGBA8888'
        repeat = re.search(r'repeat:(?:| )(\w+)', atlas_image_data)
        self.repeat = repeat.group(1) if repeat else 'none'

        # Spine Atlas 4.1
        pma = re.search(r'pma:(?:| )(\w+)', atlas_image_data)
        self.pma = eval(pma.group(1).strip().capitalize()) if pma else None
        scale = re.search(r'scale:(?:| )(\w+)', atlas_image_data)
        self.scale = float(scale.group(1).strip()) if scale else 1

        self.atlas = [Atlas(a[0], self) for a in re.findall(r'(^\w+\n(.|\n)*?)(?=^\w+\n|\Z)', atlas_image_data, re.MULTILINE)]


class RGBA:
    def __init__(self, rgba):
        self.r = int(rgba[0:2], 16) / 255
        self.g = int(rgba[2:4], 16) / 255
        self.b = int(rgba[4:6], 16) / 255
        self.a = int(rgba[6:8], 16) / 255


def create_material(atlas, mesh_name, filepath):
    material = bpy.data.materials.new(mesh_name or atlas.image)
    material.use_nodes = True
    material.blend_method = 'BLEND'
    material.shadow_method = 'CLIP'
    # material.alpha_threshold = 0.5
    # material.use_backface_culling = True
    # material.use_screen_refraction = True
    # material.use_nodes = True
    bsdf_node = material.node_tree.nodes.get('Principled BSDF')
    image_node = material.node_tree.nodes.new('ShaderNodeTexImage')
    image_node.location = (-600, 0)
    if filepath:
        image_node.image = bpy.data.images.load(str(filepath / atlas.image))

    # Assume keyframe does not define RGB
    material.node_tree.links.new(image_node.outputs['Color'], bsdf_node.inputs['Base Color'])
    # TODO create mix node for RGB keyframe

    # Assume keyframe does not define Alpha
    material.node_tree.links.new(image_node.outputs['Alpha'], bsdf_node.inputs['Alpha'])

    if mesh_name:
        # Create node for Alpha keyframe
        mix_node = material.node_tree.nodes.new('ShaderNodeMix')
        mix_node.location = (-300, -300)
        mix_node.inputs[0].default_value = 0
        material.node_tree.links.new(image_node.outputs['Alpha'], mix_node.inputs[2])
        material.node_tree.links.new(mix_node.outputs[0], bsdf_node.inputs['Alpha'])

    return material


def create_constrains(bones, armature_obj, iks, tks, paths, is_armature_control):

    control = '_Control' if is_armature_control else ''
    pose_bones = [armature_obj.pose.bones[bone.name + control] for bone in bones]

    for ik in iks:
        ik_constraint = pose_bones[ik.child_bone.bone_idx if ik.child_bone else ik.parent_bone.bone_idx].constraints.new('IK')
        ik_constraint.target = armature_obj
        ik_constraint.subtarget = ik.target_bone.name + control
        ik_constraint.chain_count = ik.chain_length
        ik_constraint.use_stretch = ik.stretch

        # spine uses softness, default 0, max 160 in spine?
        # there isn't really a 'softness' in blender, hopefully this is close enough
        ik_constraint.influence = (1 - ik.softness / 160) * ik.mix

        ik_constraint.enabled = is_armature_control

        if ik.child_bone and is_armature_control:
            pole_bone = armature_obj.pose.bones[ik.name + '_Pole']
            parent_bone = armature_obj.pose.bones[ik.parent_bone.name + control]
            pole_bone.rotation_mode = 'XYZ'
            angle = parent_bone.rotation_euler[0] + (- math.pi / 2 if ik.bendPositive else math.pi / 2)
            pole_bone.rotation_euler[0] = angle
            pole_bone.location = parent_bone.location + 10 * mathutils.Vector((0, math.cos(angle) - math.sin(angle), math.sin(angle) + math.cos(angle)))
            pole_bone.scale = parent_bone.scale

            # ik_constraint.pole_target = armature_obj
            # ik_constraint.pole_subtarget = pole_bone.name + control
            # ik_constraint.pole_angle = - math.pi / 2

        # unused
        # ik_constraint.use_tail = True
        # ik_constraint.pole_target = armature_obj
        # ik_constraint.pole_subtarget = ik.parent_bone.name + control
        # ik_constraint.pole_angle = math.radians(ik.bendPositive * 180)
        # ik_constraint.use_stretch = ik.stretch
        # ik_constraint.use_location = True
        # ik_constraint.use_rotation = True
        # ik_constraint.use_scale = True
        # ik_constraint.use_stretch = True
        # ik_constraint.use_stretch_rotation = True
        # ik_constraint.use_inverse_kinematics = True
        # ik_constraint.weight = 1
        # ik_constraint.use_chain_offset = True

    # create transform constraints modifier
    for transform in tks:
        for bone in transform.bone_list:
            if transform.mixX == -1:
                tk_constraint = pose_bones[bone.bone_idx].constraints.new('COPY_LOCATION')
                tk_constraint.target = armature_obj
                tk_constraint.subtarget = transform.target + control
                tk_constraint.influence = math.fabs(transform.mixX)
                tk_constraint.use_x = tk_constraint.use_y = tk_constraint.use_z = tk_constraint.use_offset = True
                tk_constraint.invert_x = tk_constraint.invert_y = tk_constraint.invert_z = transform.mixX < 0

                # what is the difference between LOCAL_WITH_PARENT?
                tk_constraint.target_space = tk_constraint.owner_space = 'LOCAL'

                tk_constraint.enabled = is_armature_control

            if transform.mixX and transform.mixX != -1:
                MDST_LOGGER.warning('Copy Transformation Mode not implemented')
            if transform.mixScaleX and transform.mixScaleX != -1:
                MDST_LOGGER.warning('Copy Scale Mode not implemented')
            if transform.mixShearY and transform.mixShearY != -1:
                MDST_LOGGER.warning('Copy Shear Mode not implemented')

            if transform.mixRotate:
                tk_constraint = pose_bones[bone.bone_idx].constraints.new('COPY_ROTATION')
                tk_constraint.target = armature_obj
                tk_constraint.subtarget = transform.target + control
                tk_constraint.influence = transform.mixRotate
                tk_constraint.use_y = tk_constraint.use_z = False
                tk_constraint.euler_order = 'XYZ'

                tk_constraint.enabled = is_armature_control

    # create path curve
    for path in paths:
        curve = bpy.data.objects.new(path.name + control + '_Curve', bpy.data.curves.new(path.name + control, 'CURVE'))
        bpy.context.scene.collection.objects.link(curve)
        curve.rotation_euler = (math.pi / 2, 0, 0)
        curve_obj = curve.data.splines.new('BEZIER')
        curve_obj.bezier_points.add(path.vertexCount // 3 - 1)
        for bezier_idx, bezier_point in enumerate(curve_obj.bezier_points):
            p_l, p_m, p_r = path.vertices[bezier_idx * 3:bezier_idx * 3 + 3]
            bezier_point.co = p_m.euler_pos(bones, 'xzy')
            bezier_point.handle_left = p_l.euler_pos(bones, 'xzy')
            bezier_point.handle_right = p_r.euler_pos(bones, 'xzy')
            # bezier_point.handle_left_type = 'AUTO'
            # bezier_point.handle_right_type = 'AUTO'

            for idx, bone_idx in enumerate(p_m.bone_idx):
                hook = curve.modifiers.new('HOOK', 'HOOK')
                hook.center = bezier_point.co
                hook.vertex_indices_set([bezier_idx*3, bezier_idx*3+1, bezier_idx*3+2])
                bpy.context.evaluated_depsgraph_get()
                hook.object = armature_obj
                hook.subtarget = bones[bone_idx].name + control
                hook.strength = p_m.bone_weight[idx]

    # create spline ik constraints modifier
    for path in paths:
        spline_ik = pose_bones[path.bones_list[0].bone_idx].constraints.new('SPLINE_IK')
        spline_ik.target = bpy.data.objects[path.name + control + '_Curve']

        spline_ik = pose_bones[path.bones_list[-1].bone_idx].constraints.new('SPLINE_IK')
        spline_ik.target = bpy.data.objects[path.name + control + '_Curve']
        spline_ik.chain_count = len(path.bones_list)


def load_vertex(vertex_data, single_bone_idx=None):
    vertices = []
    while vertex_data:
        vertices.append(Vertex(vertex_data, single_bone_idx))
    return vertices


def load_edge(edges):
    return list(zip(edges[::2], edges[1::2]))


def load_triangle(triangles):
    return list(zip(triangles[::3], triangles[1::3], triangles[2::3]))


def load_json(string):
    return json.loads(string)


def load_spine(mdst_spine):

    data = load_json(mdst_spine.spine_ref.as_string())
    atlas = mdst_spine.atlas_ref.as_string()
    filepath = _Path(mdst_spine.atlas_ref.filepath).parent

    layer_gap = mdst_spine.layer_gap
    separate_material = mdst_spine.chk_separate_material
    alternative_mesh = mdst_spine.chk_alternative_mesh

    if alternative_mesh:
        # create collection:
        alt_collection = bpy.data.collections.new('AlternativeMesh')
        bpy.context.scene.collection.children.link(alt_collection)

    atlas_image = [AtlasImage(img[0]) for img in re.findall(r'^((.|\n)*?\n)(?=^\n|\Z)', atlas, re.MULTILINE)]
    atlas_dict = {a.name: a for atlas in atlas_image for a in atlas.atlas}

    attachments = data['skins'][0]['attachments']

    bones = [Bone(idx, bone_data) for idx, bone_data in enumerate(data['bones'])]
    [bone.set_parent([b for b in bones if b.name == bone.parent][0]) for bone in bones if bone.name != 'root']
    bone_dict = {bone.name: bone for bone in bones}

    slots = {slot['name']: Slot(slot, bone_dict, slot_idx) for slot_idx, slot in enumerate(data['slots'])}
    iks = [IK_Bone(ik_data, bone_dict) for ik_data in data['ik']] if 'ik' in data else []
    tks = [TK_Bone(tk_data, bone_dict) for tk_data in data['transform']] if 'transform' in data else []
    paths = [Path(path_data, bone_dict, attachments) for path_data in data['path']] if 'path' in data else []

    # As mesh have its own keyframe, create material for each mesh instead
    # create material for each atlas
    materials = {atlas.image: (atlas if separate_material else create_material(atlas, None, filepath)) for atlas in atlas_image}
    mask_material = None

    # create armature for control
    armature_control = bpy.data.armatures.new('armature')
    armature_control.name = 'armatureControl'
    armature_control.display_type = 'STICK'
    armature_control_obj = bpy.data.objects.new('rootControl', armature_control)
    bpy.context.scene.collection.objects.link(armature_control_obj)
    bpy.context.view_layer.objects.active = armature_control_obj

    bpy.ops.object.mode_set(mode='EDIT')

    bone_control_objs = [None for _ in bones]
    for bone in bones:
        new_bone = armature_control.edit_bones.new(name=bone.name + '_Control')
        new_bone.select = True
        bone_control_objs[bone.bone_idx] = new_bone

        if bone.name != 'root':
            new_bone.parent = bone_control_objs[bone.parent_bone.bone_idx]
            new_bone.use_connect = False
            new_bone.use_inherit_rotation = True
            new_bone.use_inherit_scale = True

            new_bone.head = (0, 0, 0)
            new_bone.tail = (bone.length or 1, 0, 0)

        else:
            new_bone.head = (0, 0, 0)
            new_bone.tail = (1, 0, 0)

        bone_control_objs[bone.bone_idx] = new_bone

    for ik in iks:
        # disable deform on vertex for ik bones
        bone_control_objs[ik.target_bone.bone_idx].use_deform = False if ik.child_bone else True

        if ik.child_bone:
            # create pole bone
            new_bone = armature_control.edit_bones.new(name=ik.name + '_Pole')
            new_bone.select = True

            new_bone.parent = bone_control_objs[ik.parent_bone.parent_bone.bone_idx]
            new_bone.use_connect = False
            new_bone.use_inherit_rotation = True
            new_bone.use_inherit_scale = True
            new_bone.use_deform = False

            new_bone.head = new_bone.parent.head
            new_bone.tail = new_bone.head + mathutils.Vector((10, 0, 0))

    # create spline ik constraints
    # in Spine they are not connected
    for path in paths:
        parent = armature_control.edit_bones[armature_control.edit_bones.find(path.bones_list[0].name + '_Control')]
        for path_node in path.bones_list[1:]:
            idx = armature_control.edit_bones.find(path_node.name + '_Control')
            armature_control.edit_bones[idx].parent = parent
            parent = armature_control.edit_bones[idx]

    bpy.ops.object.mode_set(mode='OBJECT')

    # create armature
    armature = bpy.data.armatures.new('armature')
    armature.name = 'armature'
    armature.display_type = 'STICK'
    armature_obj = bpy.data.objects.new('root', armature)
    bpy.context.scene.collection.objects.link(armature_obj)
    bpy.context.view_layer.objects.active = armature_obj
    armature_obj.show_in_front = True
    armature_obj.select_set(state=True)

    bpy.context.view_layer.objects.active = armature_obj
    bpy.ops.object.mode_set(mode='EDIT')
    bone_objs = [None for _ in bones]
    for bone in bones:
        new_bone = armature.edit_bones.new(name=bone.name)
        new_bone.select = True
        bone_objs[bone.bone_idx] = new_bone

        if bone.name != 'root':
            new_bone.parent = bone_objs[bone.parent_bone.bone_idx]
            new_bone.use_connect = False
            new_bone.use_inherit_rotation = True
            new_bone.use_inherit_scale = True

            new_bone.head = (bone.abs_x, 0, bone.abs_y)
            new_bone.tail = (new_bone.head[0] + bone.dx,
                             0,
                             new_bone.head[-1] + bone.dy)

            # bone cannot have zero length
            if bone.length == 0:
                new_bone.tail += mathutils.Vector((math.cos(bone.abs_rotation), 0, math.sin(bone.abs_rotation)))
            # roll = bone.abs_rotation % (2 * math.pi)
            # new_bone.roll = - roll if roll < math.pi else (math.pi - (roll % math.pi))
            new_bone.roll = bone.roll

        else:
            new_bone.head = (0, 0, 0)
            new_bone.tail = (1, 0, 0)

        bone_objs[bone.bone_idx] = new_bone

    for ik in iks:
        # disable deform on vertex for ik bones
        bone_objs[ik.target_bone.bone_idx].use_deform = False if ik.child_bone else True

    # create spline ik constraints
    # in Spine they are not connected
    for path in paths:
        parent = armature.edit_bones[armature.edit_bones.find(path.bones_list[0].name)]
        for path_node in path.bones_list[1:]:
            idx = armature.edit_bones.find(path_node.name)
            armature.edit_bones[idx].parent = parent
            parent = armature.edit_bones[idx]

    bpy.ops.object.mode_set(mode='OBJECT')

    # apply transformation for each bone
    for bone in bones:
        bone_control_obj = armature_control_obj.pose.bones[bone.name + '_Control']
        bone_control_obj.rotation_mode = 'XYZ'
        bone_control_obj.rotation_euler = (bone.rotation, 0, 0)
        bone_control_obj.scale = (1, bone.scaleX, bone.scaleY)
        bone_control_obj.location = (0, bone.x, bone.y)

        bone_obj = armature_obj.pose.bones[bone.name]
        copy_constraint = bone_obj.constraints.new('COPY_TRANSFORMS')
        copy_constraint.target = armature_control_obj
        copy_constraint.subtarget = bone_obj.name + '_Control'

    parts = data['skins'][0]['attachments']
    for slot_name, slot_attachment in parts.items():
        # attachment = v[k]
        for k, attachment in slot_attachment.items():

            attachment_type = attachment.get('type', 'region')
            if attachment_type == 'mesh':

                vertices = load_vertex(attachment['vertices'], slots[slot_name].bone_obj.bone_idx if attachment['hull'] * 2 == len(attachment['vertices']) or not str(attachment['vertices'][0]).isdigit() else None)

                triangles = load_triangle(attachment['triangles'])
                mesh_object = bpy.data.meshes.new(k)
                mesh = bpy.data.objects.new(slot_name, mesh_object)

                bpy.context.scene.collection.objects.link(mesh)

                vertices_list = [vertex.global_pos(bones) for vertex in vertices]

                mesh_object.from_pydata(vertices_list, [], triangles)
                # adjust layer order
                mesh.location.y = slots[slot_name].slot_idx * layer_gap

                vertex_group = [None for _ in bones]
                for idx, vertex in enumerate(vertices):
                    for vertex_bone_idx in range(len(vertex.bone_idx)):
                        if not vertex_group[vertex.bone_idx[vertex_bone_idx]]:
                            vertex_group[vertex.bone_idx[vertex_bone_idx]] = mesh.vertex_groups.new(name=bones[vertex.bone_idx[vertex_bone_idx]].name)
                        vertex_group[vertex.bone_idx[vertex_bone_idx]].add([idx], vertex.bone_weight[vertex_bone_idx], 'REPLACE')

                mesh_object.update()
                mesh.modifiers.new('Armature', 'ARMATURE').object = armature_obj

                if alternative_mesh:
                    mesh_control_object = bpy.data.meshes.new(k + '_Control')
                    mesh_control = bpy.data.objects.new(slot_name + '_Control', mesh_control_object)
                    alt_collection.objects.link(mesh_control)
                    vertices_control_list = [vertex.local_pos() for vertex in vertices]
                    mesh_control_object.from_pydata(vertices_control_list, [], triangles)
                    mesh_control.location.y = slots[slot_name].slot_idx * layer_gap

                    vertex_group = [None for _ in bones]
                    for idx, vertex in enumerate(vertices):
                        for vertex_bone_idx in range(len(vertex.bone_idx)):
                            if not vertex_group[vertex.bone_idx[vertex_bone_idx]]:
                                vertex_group[vertex.bone_idx[vertex_bone_idx]] = mesh_control.vertex_groups.new(name=bones[vertex.bone_idx[vertex_bone_idx]].name + '_Control')
                            vertex_group[vertex.bone_idx[vertex_bone_idx]].add([idx], vertex.bone_weight[vertex_bone_idx], 'REPLACE')

                    mesh_control_object.update()
                    mesh_control.modifiers.new('Armature', 'ARMATURE').object = armature_control_obj

                uv_data = attachment['uvs']
                uvs = []
                atlas = atlas_dict[slot_name] if slot_name in atlas_dict else atlas_dict[k]
                for idx in range(len(uv_data)//2):
                    x, y = uv_data[idx*2:idx*2+2]
                    x = x * atlas.size[0] / atlas.atlas_image.size_x
                    y = y * atlas.size[1] / atlas.atlas_image.size_y
                    if atlas.rotate:

                        # Assume rotate is 90 for now
                        if atlas.rotate != 90:
                            MDST_LOGGER.error('Unsupported atlas rotation: %s' % atlas.rotate)

                        x, y = y, x
                        x += atlas.xy[0] / atlas.atlas_image.size_x
                        y += 1 - ((atlas.size[0] + atlas.xy[1]) / atlas.atlas_image.size_x)

                    else:
                        x += atlas.xy[0] / atlas.atlas_image.size_x
                        y += atlas.xy[1] / atlas.atlas_image.size_y
                        y = 1 - y
                    uvs.append((x, y))

            elif attachment_type == 'path':
                # already handled in path / spline ik constraint
                continue

            elif attachment_type == 'boundingbox':
                MDST_LOGGER.warning('Unsupported attachment type: boundingbox')
                continue

            elif attachment_type == 'point':
                MDST_LOGGER.warning('Unsupported attachment type: point')
                continue

            elif attachment_type == 'clipping':
                mesh_object = bpy.data.meshes.new(k)

                # create polygon
                # FIXME do mask has multiple vertex group?
                vertices = load_vertex(attachment['vertices'], slots[slot_name].bone_obj.bone_idx)

                masked_slot = list(slots.values())[list(slots.keys()).index(k) + 1:list(slots.keys()).index(attachment['end']) + 1]

                bm = bmesh.new()
                for v in vertices:
                    x, _, y = v.global_pos(bones)

                    # prevent backface culling
                    if layer_gap < 0:
                        bm.verts.new([x, layer_gap * (len(masked_slot) + 1), y])
                        bm.verts.new([x, 0, y])
                    else:
                        bm.verts.new([x, 0, y])
                        bm.verts.new([x, layer_gap * (len(masked_slot) + 1), y])

                # extrude mask
                verts = list(bm.verts)
                bm.faces.new(verts[::2])
                for i in range(attachment['vertexCount'] - 1):
                    bm.faces.new([verts[i * 2], verts[i * 2 + 1], verts[i * 2 + 3], verts[i * 2 + 2]])
                bm.faces.new([verts[-2], verts[-1], verts[1], verts[0]])
                bm.faces.new(verts[::-2])

                bm.normal_update()
                bm.to_mesh(mesh_object)

                mesh = bpy.data.objects.new(k, mesh_object)
                bpy.context.scene.collection.objects.link(mesh)
                mesh.location.y = slots[slot_name].slot_idx * layer_gap
                mesh_object.update()

                mesh.vertex_groups.new(name=slots[k].bone).add(list(range(attachment['vertexCount'] * 2)), 1, 'REPLACE')
                mesh.modifiers.new('Armature', 'ARMATURE').object = armature_obj

                if not mask_material:
                    mask_material = bpy.data.materials.new('Mask')
                    mask_material.use_nodes = True
                    mask_material.blend_method = 'BLEND'
                    mask_material.shadow_method = 'CLIP'

                    bsdf_node = mask_material.node_tree.nodes.get('Principled BSDF')
                    value = mask_material.node_tree.nodes.new('ShaderNodeValue')

                    mask_material.node_tree.links.new(value.outputs['Value'], bsdf_node.inputs['Alpha'])

                mesh.data.materials.append(mask_material)

                for slot in masked_slot:
                    boolean = bpy.data.objects[slot.name].modifiers.new('Boolean', 'BOOLEAN')
                    boolean.object = mesh
                    boolean.operation = 'INTERSECT'

            elif attachment_type == 'linkedmesh':
                MDST_LOGGER.warning('Unsupported attachment type: linkedmesh')
                continue

            elif attachment.get('type', 'region') == 'region':
                mesh_object = bpy.data.meshes.new(k)
                mesh = bpy.data.objects.new(k, mesh_object)

                bpy.context.scene.collection.objects.link(mesh)

                bone = slots[slot_name].bone_obj
                abs_rotation = bone.abs_rotation + math.radians(attachment['rotation']) if 'rotation' in attachment else bone.abs_rotation

                dx = attachment.get('x', 0) * math.cos(bone.abs_rotation) - attachment.get('y', 0) * math.sin(bone.abs_rotation)
                dy = attachment.get('x', 0) * math.sin(bone.abs_rotation) + attachment.get('y', 0) * math.cos(bone.abs_rotation)

                vertices_list = [
                    (
                        ((-attachment['width'] / 2) * math.cos(abs_rotation) - (attachment['height'] / 2) * math.sin(abs_rotation) + dx) * bone.abs_scale_x + bone.abs_x,
                        0,
                        ((-attachment['width'] / 2) * math.sin(abs_rotation) + (attachment['height'] / 2) * math.cos(abs_rotation) + dy) * bone.abs_scale_y + bone.abs_y
                    ), (
                        ((attachment['width'] / 2) * math.cos(abs_rotation) - (attachment['height'] / 2) * math.sin(abs_rotation) + dx) * bone.abs_scale_x + bone.abs_x,
                        0,
                        ((attachment['width'] / 2) * math.sin(abs_rotation) + (attachment['height'] / 2) * math.cos(abs_rotation) + dy) * bone.abs_scale_y + bone.abs_y
                    ), (
                        ((-attachment['width'] / 2) * math.cos(abs_rotation) - (-attachment['height'] / 2) * math.sin(abs_rotation) + dx) * bone.abs_scale_x + bone.abs_x,
                        0,
                        ((-attachment['width'] / 2) * math.sin(abs_rotation) + (-attachment['height'] / 2) * math.cos(abs_rotation) + dy) * bone.abs_scale_y + bone.abs_y
                    ), (
                        ((attachment['width'] / 2) * math.cos(abs_rotation) - (-attachment['height'] / 2) * math.sin(abs_rotation) + dx) * bone.abs_scale_x + bone.abs_x,
                        0,
                        ((attachment['width'] / 2) * math.sin(abs_rotation) + (-attachment['height'] / 2) * math.cos(abs_rotation) + dy) * bone.abs_scale_y + bone.abs_y
                    )
                ]

                mesh_object.from_pydata(vertices_list, [], [[0, 1, 2], [1, 3, 2]])
                mesh.location.y = slots[slot_name].slot_idx * layer_gap

                mesh_object.update()

                mesh.vertex_groups.new(name=slots[k].bone).add([0, 1, 2, 3], 1, 'REPLACE')
                mesh.modifiers.new('Armature', 'ARMATURE').object = armature_obj

                atlas = atlas_dict[slot_name] if slot_name in atlas_dict else atlas_dict[k]
                uvs = [(x / atlas.atlas_image.size_x, 1 - y / atlas.atlas_image.size_y) for x, y in ([
                    (atlas.xy[0], atlas.xy[1] + atlas.size[0]),
                    (atlas.xy[0], atlas.xy[1]),
                    (atlas.xy[0] + atlas.size[1], atlas.xy[1] + atlas.size[0]),
                    (atlas.xy[0] + atlas.size[1], atlas.xy[1]),
                ] if atlas.rotate else [
                    (atlas.xy[0], atlas.xy[1]),
                    (atlas.xy[0] + atlas.size[0], atlas.xy[1]),
                    (atlas.xy[0], atlas.xy[1] + atlas.size[1]),
                    (atlas.xy[0] + atlas.size[0], atlas.xy[1] + atlas.size[1]),
                ])]

            else:
                MDST_LOGGER.error('Unknown attachment type: ' + attachment_type)
                # raise Exception('Unknown attachment type: ' + attachment_type)
                continue

            if attachment_type in ['region', 'mesh']:
                # uv = mesh.data.uv_layers.new(name=k)
                uv = mesh.data.uv_layers.new(name=atlas.atlas_image.image)
                for idx, loop in enumerate(mesh.data.loops):
                    uv.data[idx].uv = uvs[loop.vertex_index]

                # assign material
                material = create_material(materials[atlas.atlas_image.image], k, filepath)
                mesh.data.materials.append(material if separate_material else materials[atlas.atlas_image.image])

                if attachment_type == 'mesh' and alternative_mesh:
                    uv = mesh_control.data.uv_layers.new(name=atlas.atlas_image.image)
                    for idx, loop in enumerate(mesh_control.data.loops):
                        uv.data[idx].uv = uvs[loop.vertex_index]
                    mesh_control.data.materials.append(material if separate_material else materials[atlas.atlas_image.image])

    create_constrains(bones, armature_control_obj, iks, tks, paths, True)
    create_constrains(bones, armature_obj, iks, tks, paths, False)

    if alternative_mesh:
        # bpy.context.view_layer.layer_collection.children.get('AlternativeMesh').hide_viewport = True
        [obj.hide_set(True) for obj in bpy.data.collections['AlternativeMesh'].objects]


    # legacy load fix
    # for bone in pose_bones:
    #     # create ik constraints modifier
    #     bone.rotation_mode = 'XYZ'
    #
    #     # for experimental purpose, record the original rotation
    #     bone['_parent_roll'] = 0 if bone.name == 'root' else bone_dict[bone.name].parent_bone.roll
    #     bone['_parent_rotation'] = 0 if bone.name == 'root' else bone_dict[bone.name].parent_bone.abs_rotation
    #     bone['_parent_local_rotation'] = 0 if bone.name == 'root' else bone_dict[bone.name].parent_bone.rotation
    #     bone['_roll'] = 0 if bone.name == 'root' else bone_dict[bone.name].roll
    #     bone['_rotation'] = 0 if bone.name == 'root' else bone_dict[bone.name].abs_rotation
    #     bone['_local_rotation'] = 0 if bone.name == 'root' else bone_dict[bone.name].rotation
    #     bone['_scale_x'] = 1 if bone.name == 'root' else bone_dict[bone.name].parent_bone.abs_scale_x
    #     bone['_scale_y'] = 1 if bone.name == 'root' else bone_dict[bone.name].parent_bone.abs_scale_y

    for a in bpy.context.screen.areas:
        if a.type == 'VIEW_3D':
            for s in a.spaces:
                if s.type == 'VIEW_3D':
                    s.clip_end = 100000
                    s.shading.type = 'MATERIAL'
                    s.region_3d.view_matrix = mathutils.Matrix(((1, 0, 0, 0), (0, 0, 1, 0), (0, -1, -1, 0), (0, 0, 0, 1)))
                    s.region_3d.view_distance = 3000
                    s.region_3d.view_location = mathutils.Vector((0, -100, 0))
                    s.region_3d.view_rotation = mathutils.Euler((0.001 + math.pi / 2, 0, 0), 'XYZ').to_quaternion()
                    s.region_3d.view_perspective = 'ORTHO'
    bpy.context.view_layer.update()
    return data


def load_animation(mdst_spine):

    data = load_json(mdst_spine.spine_ref.as_string())
    animation_name = mdst_spine.animation

    bones = bpy.data.objects['rootControl'].pose.bones
    if bpy.context.object.name != 'rootControl':
        bpy.context.view_layer.objects.active = bpy.data.objects['rootControl']
        bpy.data.objects['rootControl'].select_set(True)

    if not bpy.context.object.animation_data:
        bpy.context.object.animation_data_create()

    if mdst_spine.chk_create_static_action:

        MDST_LOGGER.info('Create static action')
        if not bpy.data.actions.get('staticAction'):
            bpy.data.actions.new('staticAction')
        bpy.context.object.animation_data.action = bpy.data.actions['staticAction']

        for bone in bpy.data.objects['rootControl'].pose.bones:
            bone.keyframe_insert('location', frame=0)
            bone.keyframe_insert('rotation_euler', frame=0)
            bone.keyframe_insert('scale', frame=0)

    action_name = 'rootControlAction'
    if not bpy.data.actions.get(action_name):
        bpy.data.actions.new(action_name)
    bpy.context.object.animation_data.action = bpy.data.actions[action_name]
    bpy.data.objects['root'].animation_data_clear()

    separate_material = mdst_spine.chk_separate_material
    layer_gap = mdst_spine.layer_gap

    fps = data['skeleton'].get('fps', 30)
    bpy.context.scene.render.fps = fps
    frame_end = 0

    # create animation
    # for animation_name, animation in data['animations'].items():
    animation = data['animations'][animation_name if animation_name else list(data['animations'].keys())[-1]]
    for slot_name, slot in animation.get('slots', {}).items():
        if not separate_material:
            break
        try:
            slot_obj = bpy.data.objects[slot_name]
        except KeyError:
            if any([obj_name.startswith(slot_name) for obj_name in list(bpy.data.objects.keys())]):
                MDST_LOGGER.warning('Slot {} not found, it could be a curve'.format(slot_name))
            else:
                MDST_LOGGER.error('Slot {} not found'.format(slot_name))
            continue

        # what version does spine use color instead of rgba?
        handle_left = []
        for keyframe in slot.get('rgba', slot.get('color', [])):
            material_node = slot_obj.material_slots[0].material.node_tree

            # skip mask material (for now)
            if 'Mix' not in material_node.nodes:
                continue

            color = RGBA(keyframe['color'])
            curve = keyframe.get('curve', 'LINEAR')

            # alpha keyframe
            material_node.nodes['Mix'].inputs[0].default_value = 1 - color.a
            material_node.nodes['Mix'].inputs[0].keyframe_insert('default_value', frame=keyframe.get('time', 0) * fps)

            if handle_left:
                material_node.animation_data.action.fcurves[-1].keyframe_points[-1].handle_left_type = 'FREE'
                material_node.animation_data.action.fcurves[-1].keyframe_points[-1].handle_left = handle_left[-1]

            # set f-curve
            if type(curve) == list:
                curve = [(t * fps, v) for t, v in zip(keyframe['curve'][0::2], keyframe['curve'][1::2])]

                # set bezier curve handle for alpha
                material_node.animation_data.action.fcurves[-1].keyframe_points[-1].handle_right_type = 'FREE'
                material_node.animation_data.action.fcurves[-1].keyframe_points[-1].handle_right = curve[-2]
                handle_left = curve[1::2]
            else:
                curve_type = curve if curve == 'LINEAR' else 'CONSTANT'
                material_node.animation_data.action.fcurves[-1].keyframe_points[-1].interpolation = curve_type
                handle_left = []

            frame_end = max(frame_end, keyframe.get('time', 0) * fps)

        for attachment in slot.get('attachment', []):
            slot_obj.hide_render = slot_obj.hide_viewport = 'name' not in attachment
            slot_obj.keyframe_insert('hide_viewport', frame=attachment.get('time', 0) * fps)
            slot_obj.keyframe_insert('hide_render', frame=attachment.get('time', 0) * fps)

    for bone_name, bone in animation.get('bones', {}).items():
        try:
            bone_obj = bones[bone_name + '_Control']
        except KeyError:
            MDST_LOGGER.error('Bone {} not found'.format(bone_name))
            continue
        _, static_x, static_y = bone_obj.location
        static_rotation = bone_obj.rotation_euler[0]
        _, static_scale_x, static_scale_y = bone_obj.scale

        handle_left = []
        for translate in bone.get('translate', []):

            # legacy approach
            # if legacy_load_fix:
            #     # roll = bone_obj['_roll']
            #     # if bone_obj.name.startswith('IK_'):
            #     # MDST_LOGGER.info(bone_obj.name)
            #     # MDST_LOGGER.info(math.degrees(bone_obj.rotation_euler.x))
            #     if scene.frame_current != int(translate.get('time', 0) * fps):
            #         scene.frame_set(int(translate.get('time', 0) * fps))
            #     # roll = context_bones[bone_obj.name].matrix.to_euler().x
            #     # roll = context_bones[bone_obj.name].rotation_euler.x - bone_obj['_parent_local_rotation']
            #     roll = bone_obj['_parent_local_rotation'] - context_bones[bone_obj.name].rotation_euler.x
            #     # roll = context_bones[bone_obj.name].rotation_euler.x - bone_obj['_parent_local_rotation']
            #     # roll = bone_obj['_parent_rotation'] - context_bones[bone_obj.name].rotation_euler.x
            #     # roll = bone_obj['_rotation']
            #     # MDST_LOGGER.info(math.degrees(roll))
            #     tr_x = (translate.get('x', 0) * math.cos(roll) - translate.get('y', 0) * math.sin(roll)) * 1
            #     tr_y = (translate.get('x', 0) * math.sin(roll) + translate.get('y', 0) * math.cos(roll)) * 1
            # else:
            #     tr_x = translate.get('x', 0)
            #     tr_y = translate.get('y', 0)

            tr_x = static_x + translate.get('x', 0)
            tr_y = static_y + translate.get('y', 0)

            # but why x, 0, y become 0, x, y?
            bone_obj.location = mathutils.Vector((0, tr_x, tr_y))
            bone_obj.keyframe_insert('location', frame=translate.get('time', 0) * fps)
            curve = translate.get('curve', 'LINEAR')

            if handle_left:
                bpy.data.actions[action_name].fcurves[-2].keyframe_points[-1].handle_left_type = 'FREE'
                bpy.data.actions[action_name].fcurves[-1].keyframe_points[-1].handle_left_type = 'FREE'

                # if legacy_load_fix:
                #     tr_x = (handle_left[0][1] * math.cos(roll) - handle_left[1][1] * math.sin(roll)) * 1
                #     tr_y = (handle_left[0][1] * math.sin(roll) + handle_left[1][1] * math.cos(roll)) * 1
                # else:
                #     tr_x = handle_left[0][1]
                #     tr_y = handle_left[1][1]

                tr_x = static_x + handle_left[0][1]
                tr_y = static_y + handle_left[1][1]
                bpy.data.actions[action_name].fcurves[-2].keyframe_points[-1].handle_left = (handle_left[0][0], tr_x)
                bpy.data.actions[action_name].fcurves[-1].keyframe_points[-1].handle_left = (handle_left[1][0], tr_y)

            if type(curve) == list:
                curve = [(t * fps, v) for t, v in zip(translate['curve'][::2], translate['curve'][1::2])]

                bpy.data.actions[action_name].fcurves[-2].keyframe_points[-1].handle_right_type = 'FREE'
                bpy.data.actions[action_name].fcurves[-1].keyframe_points[-1].handle_right_type = 'FREE'

                # if legacy_load_fix:
                #     tr_x = (curve[0][1] * math.cos(roll) - curve[2][1] * math.sin(roll)) * 1
                #     tr_y = (curve[0][1] * math.sin(roll) + curve[2][1] * math.cos(roll)) * 1
                # else:
                #     tr_x = curve[0][1]
                #     tr_y = curve[2][1]

                tr_x = static_x + curve[0][1]
                tr_y = static_y + curve[2][1]

                bpy.data.actions[action_name].fcurves[-2].keyframe_points[-1].handle_right = (curve[0][0], tr_x)
                bpy.data.actions[action_name].fcurves[-1].keyframe_points[-1].handle_right = (curve[2][0], tr_y)
                handle_left = curve[1], curve[3]
            else:
                curve_type = curve if curve == 'LINEAR' else 'CONSTANT'
                bpy.data.actions[action_name].fcurves[-2].keyframe_points[-1].interpolation = curve_type
                bpy.data.actions[action_name].fcurves[-1].keyframe_points[-1].interpolation = curve_type
                handle_left = []

        handle_left = []
        for rotate in bone.get('rotate', []):
            # bone_obj.rotation_euler[0] = math.radians(rotate.get('value', 0))

            bone_obj.rotation_euler[0] = static_rotation + math.radians(rotate.get('value', 0))
            bone_obj.keyframe_insert('rotation_euler', frame=rotate.get('time', 0) * fps)
            curve = rotate.get('curve', 'LINEAR')

            if handle_left:
                bpy.data.actions[action_name].fcurves[-3].keyframe_points[-1].handle_right_type = 'FREE'
                bpy.data.actions[action_name].fcurves[-3].keyframe_points[-1].handle_left = handle_left

            if type(curve) == list:

                # curve = [(t * fps, math.radians(v)) for t, v in zip(rotate['curve'][::2], rotate['curve'][1::2])]

                curve = [(t * fps, static_rotation + math.radians(v)) for t, v in zip(rotate['curve'][::2], rotate['curve'][1::2])]

                bpy.data.actions[action_name].fcurves[-3].keyframe_points[-1].interpolation = 'BEZIER'
                bpy.data.actions[action_name].fcurves[-3].keyframe_points[-1].handle_right_type = 'FREE'
                bpy.data.actions[action_name].fcurves[-3].keyframe_points[-1].handle_right = curve[0]
                handle_left = curve[1]
            else:
                curve_type = curve if curve == 'LINEAR' else 'CONSTANT'
                bpy.data.actions[action_name].fcurves[-3].keyframe_points[-1].interpolation = curve_type
                handle_left = []

        handle_left = []
        for scale in bone.get('scale', []):
            # bone_obj.scale = (1, scale.get('x', 1), scale.get('y', 1))

            bone_obj.scale = (1, static_scale_x * scale.get('x', 1), static_scale_y * scale.get('y', 1))
            bone_obj.keyframe_insert('scale', frame=scale.get('time', 0) * fps)
            curve = scale.get('curve', 'LINEAR')

            if handle_left:
                bpy.data.actions[action_name].fcurves[-2].keyframe_points[-1].handle_right_type = 'FREE'
                bpy.data.actions[action_name].fcurves[-1].keyframe_points[-1].handle_right_type = 'FREE'
                bpy.data.actions[action_name].fcurves[-2].keyframe_points[-1].handle_left = handle_left[0]
                bpy.data.actions[action_name].fcurves[-1].keyframe_points[-1].handle_left = handle_left[1]

            if type(curve) == list:
                curve = [(t * fps, v) for t, v in zip(scale['curve'][::2], scale['curve'][1::2])]
                bpy.data.actions[action_name].fcurves[-2].keyframe_points[-1].handle_right_type = 'FREE'
                bpy.data.actions[action_name].fcurves[-1].keyframe_points[-1].handle_right_type = 'FREE'
                bpy.data.actions[action_name].fcurves[-2].keyframe_points[-1].handle_right = curve[0]
                bpy.data.actions[action_name].fcurves[-1].keyframe_points[-1].handle_right = curve[2]
                handle_left = [curve[1], curve[3]]
            else:
                curve_type = curve if curve == 'LINEAR' else 'CONSTANT'
                bpy.data.actions[action_name].fcurves[-2].keyframe_points[-1].interpolation = curve_type
                bpy.data.actions[action_name].fcurves[-1].keyframe_points[-1].interpolation = curve_type
                handle_left = []

        for _ in bone.get('shear', []):
            ...

    for _, _ in animation.get('attachments', {}).items():
        ...

    for _, _ in animation.get('deform', {}).items():
        ...

    offset_dict = {}
    for draw_order in animation.get('drawOrder', []):
        time = draw_order.get('time', 0) * fps
        offsets = {slot['slot']: slot['offset'] for slot in draw_order.get('offsets', [])}

        for slot_name in offsets.keys():
            if slot_name not in offset_dict:
                offset_dict[slot_name] = bpy.data.objects[slot_name].location[1]

        for slot_name, orig_offset in offset_dict.items():
            offset = offset_dict[slot_name] + offsets[slot_name] * layer_gap if slot_name in offsets else offset_dict[slot_name]

            bpy.data.objects[slot_name].location = (0, offset, 0)
            bpy.data.objects[slot_name].keyframe_insert('location', frame=time)

            bpy.data.actions[slot_name + 'Action'].fcurves[-2].keyframe_points[-1].interpolation = 'CONSTANT'

            if slot_name + '_Control' in bpy.data.objects:
                bpy.data.objects[slot_name + '_Control'].location = (0, offset, 0)
                bpy.data.objects[slot_name + '_Control'].keyframe_insert('location', frame=time)

                bpy.data.actions[slot_name + '_ControlAction'].fcurves[-2].keyframe_points[-1].interpolation = 'CONSTANT'

    bpy.context.scene.frame_end = round(frame_end)
    bpy.context.view_layer.update()


def apply_pose():
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    for obj in bpy.context.scene.objects:
        obj.select_set(obj.type == 'MESH')
        if obj.type == 'MESH':
            bpy.context.view_layer.objects.active = obj
            obj.select_set(obj.select_get())

    bpy.ops.object.convert(target='MESH', keep_original=False)

    bpy.ops.object.select_all(action='DESELECT')
    root = bpy.data.objects['root']
    root.select_set(True)
    bpy.context.view_layer.objects.active = root
    bpy.ops.object.mode_set(mode='POSE')

    bpy.ops.pose.armature_apply(selected=False)

    bpy.ops.object.mode_set(mode='OBJECT')
    for obj in bpy.data.objects:
        if obj.type == 'MESH':
            obj.modifiers.new('Armature', 'ARMATURE').object = bpy.data.objects['root']


def toggle_armature_constrain(mdst_spine):
    bpy.ops.object.mode_set(mode='OBJECT')
    for bone in bpy.data.objects['root'].pose.bones:
        for constrains in bone.constraints:
            if constrains.type == 'COPY_TRANSFORMS':
                constrains.enabled = mdst_spine.armature_constrain
            if constrains.name in ['COPY_LOCATION', 'COPY_ROTATION', 'IK']:
                constrains.enabled = not mdst_spine.armature_constrain
