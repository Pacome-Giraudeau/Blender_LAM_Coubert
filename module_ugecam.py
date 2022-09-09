bl_info = {
    'name': 'UGECAM Module',
    'author': 'Omar GALARRAGA, Mordjane SAHRANE',
    'category': 'Animation',
    'location': 'View 3D > Tool Shelf > UGECAM',
    'description': 'Import an avatar and retarget it'
}

import bpy
from bpy.props import EnumProperty
from bpy.types import Operator, Panel
from bpy.utils import register_class, unregister_class


class LAM_PT_panel(Panel):
    bl_idname = 'LAM_PT_panel'
    bl_label = 'LAM Coubert'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'UGECAM'

    def draw(self, context):
        layout = self.layout
        layout.operator('lam.cga_anim', text='Clear scene').action = 'CLEAR'
        # layout.operator('test.test_op', text='Load and retarget bvh').action = 'LOAD_RETARGET_BVH'
        layout.operator('lam.cga_anim', text='Load default avatar').action = 'LOAD_MHX2'
        #layout.operator("import_scene.makehuman_mhx2")

        if hasattr(bpy.types, 'MCP_OT_load_and_retarget'):
            layout.operator("mcp.load_and_retarget")
        else:
            layout.operator('test.test_op', text="BVH retargeter not found")

        if hasattr(bpy.types, 'MHX_PT_Setup'):
            layout.operator("import_scene.makehuman_mhx2")
        else:
            layout.operator('test.test_op', text="MHX not found")


class LAM_OT_CGA_animation(Operator):
    bl_idname = 'lam.cga_anim'
    bl_label = 'LAM Coubert'
    bl_description = 'LAM Coubert CGA animation'
    bl_options = {'REGISTER', 'UNDO'}

    action: EnumProperty(
        items=[
            ('CLEAR', 'clear scene', 'clear scene'),
            ('LOAD_RETARGET_BVH', 'add cube', 'add cube'),
            ('LOAD_MHX2', 'add sphere', 'add sphere')
        ]
    )

    def execute(self, context):
        if self.action == 'CLEAR':
            self.clear_scene(context=context)
        elif self.action == 'LOAD_RETARGET_BVH':
            self.add_cube(context=context)
        elif self.action == 'LOAD_MHX2':
            self.add_sphere(context=context)
        return {'FINISHED'}

    @staticmethod
    def clear_scene(context):
        for obj in bpy.data.objects:
            bpy.data.objects.remove(obj)

    @staticmethod
    def add_cube(context):
        # bpy.ops.import_anim.bvh(filepath="C:/Users/mordj/Documents/Projet_stage_UGECAM/c3dtoBVH/SautCorde_IMS_20211123_2021_11_23_052_2_test.bvh")
        # bpy.ops.mcp.verify_target_rig()
        # bpy.ops.mcp.verify_source_rig()
        # bpy.ops.mcp.load_bvh(filepath="C:/Users/mordj/Documents/Projet_stage_UGECAM/c3dtoBVH/SautCorde_IMS_20211123_2021_11_23_052_2_test.bvh")
        myproblems = ProblemsString()
        print(myproblems.problems)
        bpy.ops.mcp.load_and_retarget()

    @staticmethod
    def add_sphere(context):
        bpy.ops.import_scene.makehuman_mhx2(filepath="D:/BLENDER_MAKEHUMAN/BVH_doc_test/avatar1.mhx2")


def register():
    register_class(LAM_OT_CGA_animation)
    register_class(LAM_PT_panel)


def unregister():
    unregister_class(LAM_OT_CGA_animation)
    unregister_class(LAM_PT_panel)


if __name__ == '__main__':
    registe