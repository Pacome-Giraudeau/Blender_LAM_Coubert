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
from os import path, listdir
from re import match, compile

class LAM_PT_panel(Panel):
    bl_idname = 'LAM_PT_panel'
    bl_label = 'LAM Coubert'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'UGECAM'

    def draw(self, context):
        layout = self.layout
        layout.operator('lam.cga_anim', text='Clear scene').action = 'CLEAR'
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


        bpy.ops.mcp.load_and_retarget()

    @staticmethod
    def add_sphere(context):
        def dossier_plus_grand(filepath):
            """
            Trouve le dossier ayant la plus grande valeur numérique dans un dossier donné.
            Les dossiers sont nommés sous la forme "x.y".

            Args:
                filepath (str): Le chemin du dossier à explorer.

            Returns:
                str: Le nom du dossier avec la plus grande valeur numérique, ou une chaîne vide si aucun dossier n'est trouvé.
            """
            pattern = compile(r'^(\d+)\.(\d+)$')
            plus_grand = None

            try:
                for nom_dossier in listdir(filepath):
                    chemin_complet = path.join(filepath, nom_dossier)
                    if path.isdir(chemin_complet):
                        match = pattern.match(nom_dossier)
                        if match:
                            x, y = map(int, match.groups())
                            if plus_grand is None or (x, y) > plus_grand:
                                plus_grand = (x, y)
                                plus_grand_nom = nom_dossier
                                
            except Exception as e:
                print(f"Erreur lors de l'accès au dossier : {e}")
                return ""

            return plus_grand_nom if plus_grand else ""
        
        filepath_before_start = path.expanduser('~')
        filepath_start = r"\AppData\Roaming\Blender Foundation\Blender"
        filepath_mid = dossier_plus_grand( filepath_before_start + filepath_start)
        filepath_end =  r"\scripts\addons\modele.mhx2"
        filepath = filepath_before_start+filepath_start+"\\"+filepath_mid+filepath_end
        bpy.ops.import_scene.makehuman_mhx2(filepath=filepath)


def register():
    register_class(LAM_OT_CGA_animation)
    register_class(LAM_PT_panel)


def unregister():
    unregister_class(LAM_OT_CGA_animation)
    unregister_class(LAM_PT_panel)


if __name__ == '__main__':
    register()