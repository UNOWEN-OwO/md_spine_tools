import bpy
from bpy.types import Operator, Panel, PropertyGroup, Text, UIList, Scene
from bpy.props import StringProperty, IntProperty, BoolProperty, CollectionProperty, FloatProperty, PointerProperty, EnumProperty
from bpy_extras.io_utils import ImportHelper

from . import MDST_LOGGER, MDST_SETTINGS
from .mdst_io import load_json, load_spine, load_animation, apply_pose, toggle_armature_constrain


# ['objects', 'armatures', 'meshes', 'curves', 'materials', 'actions']
def delete_helper(objs):
    try:
        bpy.ops.object.mode_set(mode="OBJECT")
    except:
        pass

    for obj in objs:
        for data in getattr(bpy.data, obj):
            getattr(bpy.data, obj).remove(data)


def animation_list_callback(self, context):
    return context.scene.mdst_spine.animation_list


def attachment_list_callback(self, context):
    return context.scene.mdst_spine.attachment_list


class MDSTSpine(PropertyGroup):
    spine_ref: PointerProperty(type=Text)
    atlas_ref: PointerProperty(type=Text)
    layer_gap: FloatProperty(name='Layer Gap', default=-0.01)
    chk_auto_load_animation: BoolProperty(name='Auto Load Animation', default=True)
    chk_alternative_mesh: BoolProperty(name='Create Alternative Mesh', default=True)
    chk_separate_material: BoolProperty(name='Separate Material', default=True)
    chk_generate_ik_pole: BoolProperty(name='Generate IK Pole', default=True)
    chk_create_static_action: BoolProperty(name='Create Static Action', default=True)

    spine_loaded: BoolProperty(name='Spine Loaded', default=False)
    armature_constrain: BoolProperty(name='Spine Loaded', default=True)

    animation_list = []
    animation: EnumProperty(items=animation_list_callback)
    attachment_list = []
    attachment: EnumProperty(items=attachment_list_callback)


class MDST_OT_ImportSpine(Operator, ImportHelper):
    bl_idname = 'md_spine_tools.import_spine'
    bl_description = bl_label = 'Import MD Spine json'
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = '*.json;.*.spine-json;*.asset'
    filter_glob: StringProperty(default='*.json;.*.spine-json;*.asset', options={'HIDDEN'})
    filepath: StringProperty(name='File Path', maxlen=1024)

    def execute(self, context):
        self.report({'INFO'}, f'[md_spine_tools] Finished importing {self.filepath}')
        MDST_SETTINGS.last_import = self.filepath
        scene = context.scene
        scene.mdst_spine.spine_ref = bpy.data.texts.load(self.filepath)
        data = load_json(scene.mdst_spine.spine_ref.as_string())
        scene.mdst_spine.animation_list.clear()
        for i, animation in enumerate(data.get('animations', {}).items()):
            scene.mdst_spine.animation_list.append((animation[0], animation[0], '', i))
            # select the last animation
            scene.mdst_spine.animation = animation[0]
        return {'FINISHED'}

    def invoke(self, context, event):
        self.filepath = MDST_SETTINGS.last_import or ''
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


class MDST_OT_ImportAtlas(Operator, ImportHelper):
    bl_idname = 'md_spine_tools.import_atlas'
    bl_description = bl_label = 'Import MD Spine Atlas'
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = '*.txt;.*.atlas.txt;*.asset'
    filter_glob: StringProperty(default='*.txt;.*.atlas.txt;*.asset', options={'HIDDEN'})
    filepath: StringProperty(name='File Path', maxlen=1024)

    def execute(self, context):
        self.report({'INFO'}, f'[md_spine_tools] Finished importing {self.filepath}')
        MDST_SETTINGS.last_import = self.filepath
        context.scene.mdst_spine.atlas_ref = bpy.data.texts.load(self.filepath)
        return {'FINISHED'}

    def invoke(self, context, event):
        self.filepath = MDST_SETTINGS.last_import or ''
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


class MDST_OT_LoadSpine(Operator):
    bl_idname = 'md_spine_tools.load_spine'
    bl_description = bl_label = 'Load MD Spine'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        delete_helper(['objects', 'armatures', 'meshes', 'curves', 'materials', 'actions', 'collections'])
        mdst_spine = context.scene.mdst_spine
        load_spine(mdst_spine)
        if mdst_spine.chk_auto_load_animation:
            load_animation(mdst_spine)
        mdst_spine.spine_loaded = True
        return {'FINISHED'}


class MDST_OT_ApplyPose(Operator):
    bl_idname = 'md_spine_tools.apply_pose'
    bl_description = bl_label = 'Apply Pose & Reassign Armature Modifier'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, _):
        apply_pose()
        return {'FINISHED'}


class MDST_OT_ToggleArmatureConstrain(Operator):
    bl_idname = 'md_spine_tools.toggle_armature_constrain'
    bl_description = bl_label = 'Toggle Transform Constrain from root to rootContol'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        mdst_spine = context.scene.mdst_spine
        mdst_spine.armature_constrain = not mdst_spine.armature_constrain
        toggle_armature_constrain(mdst_spine)
        return {'FINISHED'}


class MDST_OT_LoadAnimation(Operator):
    bl_idname = 'md_spine_tools.load_animation'
    bl_description = bl_label = 'Load MD Spine Animation'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        delete_helper(['actions'])
        mdst_spine = context.scene.mdst_spine
        load_animation(mdst_spine)
        return {'FINISHED'}


class MDST_OT_ClearAnimation(Operator):
    bl_idname = 'md_spine_tools.clear_animation'
    bl_description = bl_label = 'Clear MD Spine Animation'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, _):
        delete_helper(['actions'])
        return {'FINISHED'}


class MDST_PT_Tools(Panel):
    bl_category = 'MDST Spine Tools'
    bl_label = 'Import'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'

    def draw(self, context):
        self.layout.label(text='Spine:', icon='MESH_CUBE')
        row = self.layout.row(align=True)
        spine = context.scene.mdst_spine
        row.template_ID(spine, 'spine_ref', open='md_spine_tools.import_spine')
        self.layout.label(text='Atlas:', icon='UV')
        row = self.layout.row(align=True)
        row.template_ID(spine, 'atlas_ref', open='md_spine_tools.import_atlas')
        self.layout.label(text='Settings:', icon='TOOL_SETTINGS')
        row = self.layout.row(align=True)
        row.prop(spine, 'layer_gap')
        row = self.layout.row(align=True)
        row.prop(spine, 'chk_auto_load_animation')
        row = self.layout.row(align=True)
        row.prop(spine, 'chk_alternative_mesh')
        row = self.layout.row(align=True)
        row.prop(spine, 'chk_separate_material')
        row = self.layout.row(align=True)
        row.prop(spine, 'chk_generate_ik_pole')
        self.layout.label(text='Load:', icon='IMPORT')
        row = self.layout.row(align=True)
        row.operator('md_spine_tools.load_spine', icon='MESH_CUBE', text='Load Spine')
        if not spine.spine_ref or not spine.atlas_ref:
            row.enabled = False
        row = self.layout.row(align=True)
        row.operator('md_spine_tools.toggle_armature_constrain', icon='MODIFIER_ON' if spine.armature_constrain else 'MODIFIER_OFF', text='Toggle Armature Constrain')
        if not context.scene.mdst_spine.spine_loaded:
            row.enabled = False


class MDST_PT_Animation(Panel):
    bl_category = 'MDST Spine Tools'
    bl_label = 'Animation'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'

    def draw(self, context):
        if context.scene.mdst_spine.spine_ref and context.scene.mdst_spine.chk_separate_material:
            if context.scene.mdst_spine.animation_list:
                row = self.layout.row(align=True)
                row.prop(context.scene.mdst_spine, 'chk_create_static_action')
                row = self.layout.row(align=True)
                row.prop(context.scene.mdst_spine, 'animation', text='', icon='RENDER_ANIMATION')
                row = self.layout.row(align=True)
                row.operator('md_spine_tools.load_animation', icon='ANIM', text='Load Animation')
                if not context.scene.mdst_spine.spine_loaded:
                    row.enabled = False
                row = self.layout.row(align=True)
                row.operator('md_spine_tools.clear_animation', icon='BRUSH_DATA', text='Clear All Animation')
            else:
                self.layout.label(text='No Data', icon='ERROR')


classes = [MDSTSpine, MDST_OT_ImportSpine, MDST_OT_ImportAtlas, MDST_OT_LoadSpine, MDST_OT_ApplyPose, MDST_OT_ToggleArmatureConstrain, MDST_OT_LoadAnimation, MDST_OT_ClearAnimation, MDST_PT_Tools, MDST_PT_Animation]


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    # bpy.types.Scene.mdst_settings = bpy.props.PointerProperty(type=MDSTSettings)
    Scene.mdst_spine = PointerProperty(type=MDSTSpine)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

    # del bpy.types.Scene.mdst_settings
    del Scene.mdst_spine
