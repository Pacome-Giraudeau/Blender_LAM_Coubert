bl_info = {
    'name': 'UGECAM Module',
    'author': 'Omar GALARRAGA, Mordjane SAHRANE',
    'category': 'Animation',
    'location': 'View 3D > Tool Shelf > UGECAM',
    'description': 'Import an avatar and retarget it'
}

import bpy
from bpy.props import EnumProperty, FloatProperty, BoolProperty
from bpy.types import Operator, Panel
from bpy.utils import register_class, unregister_class
from os import path, listdir
from re import match, compile
import math

class LAM_PT_panel(Panel):
    bl_idname = 'LAM_PT_panel'
    bl_label = 'LAM Coubert'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'UGECAM'

    def draw(self, context):
        layout = self.layout
        layout.operator('lam.cga_anim', text='Clear scene').action = 'CLEAR'
            
class MKH2_import(Panel):
    bl_idname = 'MKH2_import'
    bl_label = 'import avatar'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'UGECAM'

    def draw(self, context):
        layout = self.layout
        layout.operator('lam.cga_anim', text='Load default avatar').action = 'LOAD_MHX2'
        if hasattr(bpy.types, 'MHX_PT_Setup'):
            layout.operator("import_scene.makehuman_mhx2")
        else:
            layout.operator('test.test_op', text="MHX not found")

class BVH_import(Panel):
    bl_idname = 'BVH_import'
    bl_label = 'import bvh'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'UGECAM'

    def draw(self, context):
        layout = self.layout
        layout.operator('lam.cga_anim', text='preparation retargating bvh').action = 'PREP_BVH'

        if hasattr(bpy.types, 'MCP_OT_load_and_retarget'):
            layout.operator("mcp.load_and_retarget")
        else:
            layout.operator('test.test_op', text="BVH retargeter not found")

        layout.operator('lam.cga_anim', text='orient bvh toward x').action = 'ORI_BVH'

class Modify_positions_panel(Panel):
    bl_idname = 'Modify_position_panel'
    bl_label = 'Modify position panel'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'UGECAM'

    def draw(self, context):
        layout = self.layout    
        row = layout.row()
        layout.prop(context.scene, "lam_cga_rotation_angle1", text="Rotation Angle phalanx 1")
        layout.prop(context.scene, "lam_cga_rotation_angle2", text="Rotation Angle phalanx 2")
        layout.prop(context.scene, "lam_cga_rotation_angle3", text="Rotation Angle phalanx 3")
        layout.prop(context.scene, "bool_thumb", text="Change only Thumb ?")
        layout.operator('lam.cga_anim', text='change the hands').action = 'HANDS'


class Default_positions_panel(Panel):
    bl_idname = 'Default_position_panel'
    bl_label = 'Default position panel'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'UGECAM'

    def draw(self, context):
        layout = self.layout    
        row = layout.row()
        layout.operator('lam.cga_anim', text='close hands').action = 'HANDS_CLOSE'
        layout.operator('lam.cga_anim', text='half-open hands').action = 'HANDS_HALF'
        layout.operator('lam.cga_anim', text='open hands').action = 'HANDS_OPEN'
        

class LAM_OT_CGA_animation(Operator):
    bl_idname = 'lam.cga_anim'
    bl_label = 'LAM Coubert'
    bl_description = 'LAM Coubert CGA animation'
    bl_options = {'REGISTER', 'UNDO'}

    action: EnumProperty(
        items=[
            ('CLEAR', 'clear scene', 'clear scene'),
            ('ORI_BVH', 'ori_bvh', 'ori_bvh'),
            ('PREP_BVH', 'prep_bvh', 'prep_bvh'),
            ('LOAD_MHX2', 'import_mkh2', 'import_mkh2'),
            ('HANDS', 'hands', 'hands'),
            ('HANDS_CLOSE', 'hands_close', 'hands_close'),
            ('HANDS_OPEN', 'hands_open', 'hands_open'),
            ('HANDS_HALF', 'hands_half', 'hands_half')
        ]
    )
    
    rotation_angle: FloatProperty(
        name="Rotation Angle",
        description="Angle to rotate bones (in degrees)",
        default=45.0,
        min=-360.0,
        max=360.0
    )

    def execute(self, context):
        if self.action == 'CLEAR':
            self.clear_scene(context=context)
        elif self.action == 'ORI_BVH':
            self.ori_bvh(context=context)
        elif self.action == 'LOAD_MHX2':
            self.import_mkh2(context=context)
        elif self.action == 'PREP_BVH':
            self.prep_bvh(context=context)
        elif self.action == 'HANDS':
            self.hands(context=context)
        elif self.action == 'HANDS_OPEN':
            self.hands_open(context=context)
        elif self.action == 'HANDS_HALF':
            self.hands_half(context=context)
        elif self.action == 'HANDS_CLOSE':
            self.hands_close(context=context)
        return {'FINISHED'}

    @staticmethod
    def clear_scene(context):
        
        for obj in bpy.data.objects:
            bpy.data.objects.remove(obj)
            
    def hands_open(self, context):
        bpy.context.scene.bool_thumb = False
        bpy.context.scene.lam_cga_rotation_angle1 = 0
        bpy.context.scene.lam_cga_rotation_angle2 = 0
        bpy.context.scene.lam_cga_rotation_angle3 = 0
        self.hands(context)
        bpy.context.scene.bool_thumb = True
        bpy.context.scene.lam_cga_rotation_angle1 = 0
        bpy.context.scene.lam_cga_rotation_angle2 = 0
        bpy.context.scene.lam_cga_rotation_angle3 = 0
        self.hands(context=context)
        
    def hands_half(self, context):
        bpy.context.scene.bool_thumb = False
        bpy.context.scene.lam_cga_rotation_angle1 = 10
        bpy.context.scene.lam_cga_rotation_angle2 = 30
        bpy.context.scene.lam_cga_rotation_angle3 = 20
        self.hands(context)
        bpy.context.scene.bool_thumb = True
        bpy.context.scene.lam_cga_rotation_angle1 = 20
        bpy.context.scene.lam_cga_rotation_angle2 = 0
        bpy.context.scene.lam_cga_rotation_angle3 = 0
        self.hands(context)
        
    def hands_close(self, context):
        bpy.context.scene.bool_thumb = False
        bpy.context.scene.lam_cga_rotation_angle1 = 60
        bpy.context.scene.lam_cga_rotation_angle2 = 60
        bpy.context.scene.lam_cga_rotation_angle3 = 60
        self.hands(context)
        bpy.context.scene.bool_thumb = True
        bpy.context.scene.lam_cga_rotation_angle1 = 20
        bpy.context.scene.lam_cga_rotation_angle2 = 20
        bpy.context.scene.lam_cga_rotation_angle3 = 20
        self.hands(context)
        
    @staticmethod     
    def hands(context):
        # Passe en mode Pose pour appliquer les rotations
        bpy.ops.object.mode_set(mode='POSE')
        bpy.context.scene.tool_settings.use_keyframe_insert_auto = True
        armature = context.object
        if armature and armature.type == 'ARMATURE':  # Vérifie que l'objet sélectionné est bien une armature# Liste des os à faire pivoter
            bone_names = [
                "pinky_01_r", "pinky_01_l", 
                "index_01_r", "index_01_l", 
                "ring_01_r", "ring_01_l", 
                "middle_01_r", "middle_01_l"
            ]
            if bpy.context.scene.bool_thumb:
                bone_names = [
                "thumb_01_r", "thumb_01_l"
                ]
                
                
            bone_names_2 = [
                "pinky_02_r", "pinky_02_l", 
                "index_02_r", "index_02_l", 
                "ring_02_r", "ring_02_l", 
                "middle_02_r", "middle_02_l"
            ]
            if bpy.context.scene.bool_thumb:
                bone_names_2 = [
                "thumb_02_r", "thumb_02_l"
                ]
            
            bone_names_3 = [
                "pinky_03_r", "pinky_03_l", 
                "index_03_r", "index_03_l", 
                "ring_03_r", "ring_03_l", 
                "middle_03_r", "middle_03_l",
            ]
            if bpy.context.scene.bool_thumb:
                bone_names_3 = [
                "thumb_03_r", "thumb_03_l"
                ]
                
             
            angle1= bpy.context.scene.lam_cga_rotation_angle1
            angle2= bpy.context.scene.lam_cga_rotation_angle2
            angle3= bpy.context.scene.lam_cga_rotation_angle3

            # Applique la rotation à chaque os
            for bone_name in bone_names:
                bone = armature.pose.bones.get(bone_name)
                if bone:
                    # Pour sélectionner un os en mode pose, on passe par l'API des objets
                    bpy.ops.object.mode_set(mode='POSE')
                    armature.data.bones[bone.name].select = True
                    # Applique une rotation autour de l'axe Z (par exemple, angle1 degrés)
                    bone.rotation_mode = 'XYZ'  # Définit le mode de rotation (ex. 'XYZ', 'QUATERNION', etc.)
                    bone.rotation_euler.x = math.radians(angle1)  # Ajoute angle1 degrés à la rotation autour de l'axe Z

                else:
                    print(f"Bone '{bone_name}' not found")
                    
             # Applique la rotation à chaque os
            for bone_name in bone_names_2:
                bone = armature.pose.bones.get(bone_name)
                if bone:
                    # Pour sélectionner un os en mode pose, on passe par l'API des objets
                    bpy.ops.object.mode_set(mode='POSE')
                    armature.data.bones[bone.name].select = True
                    # Applique une rotation autour de l'axe Z (par exemple, angle2 degrés)
                    bone.rotation_mode = 'XYZ'  # Définit le mode de rotation (ex. 'XYZ', 'QUATERNION', etc.)
                    bone.rotation_euler.x = math.radians(angle2)  # Ajoute angle2 degrés à la rotation autour de l'axe Z

                else:
                    print(f"Bone '{bone_name}' not found")
                    
             # Applique la rotation à chaque os
            for bone_name in bone_names_3:
                bone = armature.pose.bones.get(bone_name)
                if bone:
                    # Pour sélectionner un os en mode pose, on passe par l'API des objets
                    bpy.ops.object.mode_set(mode='POSE')
                    armature.data.bones[bone.name].select = True
                    # Applique une rotation autour de l'axe Z (par exemple, angle3 degrés)
                    bone.rotation_mode = 'XYZ'  # Définit le mode de rotation (ex. 'XYZ', 'QUATERNION', etc.)
                    bone.rotation_euler.x = math.radians(angle3)  # Ajoute angle3 degrés à la rotation autour de l'axe Z

                else:
                    print(f"Bone '{bone_name}' not found")
        else:
            print("Active object is not an armature") 
                   
    @staticmethod
    def prep_bvh(context):
        # Rotation Ad-Hoc
        context = bpy.context
        ob = context.object
        ob.rotation_euler = (0, 0, 0)
      

    @staticmethod
    def ori_bvh(context):
        # Rotation Ad-Hoc
        context = bpy.context
        ob = context.object
        ob.rotation_euler = (0, 0, math.pi/2)
        return
        
        
    @staticmethod
    def import_mkh2(context):
        def dossier_plus_grand(filepath):
            """
            Trouve le dossier ayant la plus grande valeur lexicographique dans un dossier donné.
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
        filepath_end =  r"\scripts\addons\modele2.mhx2"
        filepath = filepath_before_start+filepath_start+"\\"+filepath_mid+filepath_end
        bpy.ops.import_scene.makehuman_mhx2(filepath=filepath)


def register():
    register_class(LAM_OT_CGA_animation)
    register_class(LAM_PT_panel)
    register_class(MKH2_import)
    register_class(BVH_import)
    register_class(Modify_positions_panel)
    register_class(Default_positions_panel)
    bpy.types.Scene.lam_cga_rotation_angle1 = FloatProperty(
        name="Rotation Angle phalanx 1",
        description="Angle to rotate bones (in degrees)",
        default=45.0,
        min=-360.0,
        max=360.0
    )
    bpy.types.Scene.lam_cga_rotation_angle2 = FloatProperty(
        name="Rotation Angle phalanx 2",
        description="Angle to rotate bones (in degrees)",
        default=45.0,
        min=-360.0,
        max=360.0
    )
    bpy.types.Scene.lam_cga_rotation_angle3 = FloatProperty(
        name="Rotation Angle phalanx 3",
        description="Angle to rotate bones (in degrees)",
        default=45.0,
        min=-360.0,
        max=360.0
    )
    bpy.types.Scene.bool_thumb = BoolProperty(
        name="Thumb ?",
        description="A boolean value to determine if the thunm is modified ?",
        default=False
    )


def unregister():
    unregister_class(LAM_OT_CGA_animation)
    unregister_class(LAM_PT_panel)
    unregister_class(BVH_import)
    unregister_class(MKH2_import)
    unregister_class(Modify_positions_panel)
    unregister_class(Default_positions_panel)
    del bpy.types.Scene.lam_cga_rotation_angle1
    del bpy.types.Scene.lam_cga_rotation_angle2
    del bpy.types.Scene.lam_cga_rotation_angle3
    del bpy.types.Scene.bool_thumb



if __name__ == '__main__':
    register()