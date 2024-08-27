# Copyright (c) 2019-2024, Thomas Larsson
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# The views and conclusions contained in the software and documentation are those
# of the authors and should not be interpreted as representing official policies,
# either expressed or implied, of the FreeBSD Project.

import bpy
from bpy.props import *
from bpy_extras.io_utils import ImportHelper, ExportHelper

import os
from math import sqrt, pi
from mathutils import Quaternion, Matrix
from .utils import *

#------------------------------------------------------------------
#   Classes
#------------------------------------------------------------------

class JsonFile:
    filename_ext = ".json"
    filter_glob : StringProperty(default="*.json", options={'HIDDEN'})
    filepath : StringProperty(name="File Path", description="Filepath to json file", maxlen=1024, default="")


class Rigger:
    autoRig : BoolProperty(
        name = "Auto Rig",
        description = "Find rig automatically",
        default = True)

    def draw(self, context):
        self.layout.prop(self, "autoRig")
        if not self.autoRig:
            scn = context.scene
            rig = context.object
            if self.isSourceRig:
                self.layout.prop(scn, "McpSourceRig")
                self.layout.prop(scn, "McpSourceTPose")
            else:
                self.layout.prop(scn, "McpTargetRig")
                self.layout.prop(scn, "McpTargetTPose")


    def initRig(self, context):
        from .target import findTargetArmature
        from .source import findSourceArmature

        rig = context.object
        pose = [(pb, pb.matrix_basis.copy()) for pb in rig.pose.bones]

        if self.isSourceRig:
            findSourceArmature(context, rig, self.autoRig)
        else:
            findTargetArmature(context, rig, self.autoRig)

        for pb,mat in pose:
            pb.matrix_basis = mat

        if isRigify(rig):
            setRigifyFKIK(rig, 0.0)
        elif isRigify2(rig):
            setRigify2FKIK(rig, 1.0)

        return rig

#------------------------------------------------------------------
#   Set FK/IK
#------------------------------------------------------------------

def setMhxIk(rig, useArms, useLegs, value):
    if isMhxRig(rig):
        ikLayers = []
        fkLayers = []
        if useArms:
            rig["MhaArmIk_L"] = value
            rig["MhaArmIk_R"] = value
            ikLayers += [(2,"IK Arm Left"), (18,"IK Arm Right")]
            fkLayers += [(3,"FK Arm Left"), (19,"FK Arm Right")]
        if useLegs:
            rig["MhaLegIk_L"] = value
            rig["MhaLegIk_R"] = value
            ikLayers += [(4,"IK Leg Left"), (20,"IK Leg Right")]
            fkLayers += [(5,"FK Leg Left"), (21,"FK Leg Right")]
        if value:
            first = ikLayers
            second = fkLayers
        else:
            first = fkLayers
            second = ikLayers
        for n,cname in first:
            enableRigLayer(rig, n, cname, True)
        for n,cname in second:
            enableRigLayer(rig, n, cname, False)


def setRigifyFKIK(rig, value):
    rig.pose.bones["hand.ik.L"]["ikfk_switch"] = value
    rig.pose.bones["hand.ik.R"]["ikfk_switch"] = value
    rig.pose.bones["foot.ik.L"]["ikfk_switch"] = value
    rig.pose.bones["foot.ik.R"]["ikfk_switch"] = value
    enableRigifyLayers(rig, value)


def enableRigifyLayers(rig, value):
    on = (value > 0.5)
    for n,cname in [
        (7, "Arm.L (IK)"),
        (10, "Arm.R (IK)"),
        (13, "Leg.L (IK)"),
        (16, "Leg.R (IK)")]:
        enableRigLayer(rig, n, cname, on)
    for n,cname in [
        (8, "Arm.L (FK)"),
        (11, "Arm.R (FK)"),
        (14, "Leg.L (FK)"),
        (17, "Leg.R (FK)")]:
        enableRigLayer(rig, n, cname, (not on))


def setRigify2FKIK(rig, value):
    rig.pose.bones["upper_arm_parent.L"]["IK_FK"] = value
    rig.pose.bones["upper_arm_parent.R"]["IK_FK"] = value
    rig.pose.bones["thigh_parent.L"]["IK_FK"] = value
    rig.pose.bones["thigh_parent.R"]["IK_FK"] = value
    enableRigifyLayers(rig, 1-value)
    torso = rig.pose.bones["torso"]
    torso["head_follow"] = 1.0
    torso["neck_follow"] = 1.0


def setRigToFK(rig):
    setMhxIk(rig, True, True, 0.0)
    if isRigify(rig):
        setRigifyFKIK(rig, 0.0)
    elif isRigify2(rig):
        setRigify2FKIK(rig, 1.0)

#------------------------------------------------------------------
#   Define current pose as rest pose
#------------------------------------------------------------------

class MCP_OT_RestCurrentPose(BvhOperator, IsArmature):
    bl_idname = "mcp.rest_current_pose"
    bl_label = "Current Pose => Rest Pose"
    bl_description = "Change rest pose to current pose"
    bl_options = {'UNDO'}

    def run(self, context):
        rig = context.object
        children = []
        for ob in context.view_layer.objects:
            if ob.type != 'MESH':
                continue

            setActiveObject(context, ob)
            if ob != context.object:
                raise MocapError("Context switch did not take:\nob = %s\nc.ob = %s\nc.aob = %s" %
                    (ob, context.object, context.active_object))

            if (ob.McpArmatureName == rig.name and
                ob.McpArmatureModifier != ""):
                mod = ob.modifiers[ob.McpArmatureModifier]
                ob.modifiers.remove(mod)
                ob.data.shape_keys.key_blocks[ob.McpArmatureModifier].value = 1.0
                children.append(ob)
            else:
                for mod in ob.modifiers:
                    if (mod.type == 'ARMATURE' and
                        mod.object == rig):
                        children.append(ob)
                        if bpy.app.version < (2,90,0):
                            bpy.ops.object.modifier_apply(apply_as='SHAPE', modifier=mod.name)
                        else:
                            bpy.ops.object.modifier_apply_as_shapekey(modifier=mod.name)
                        ob.data.shape_keys.key_blocks[mod.name].value = 1.0
                        ob.McpArmatureName = rig.name
                        ob.McpArmatureModifier = mod.name
                        break

        setActiveObject(context, rig)
        bpy.ops.object.mode_set(mode='POSE')
        try:
            bpy.ops.pose.armature_apply()
        except RuntimeError as err:
            raise MocapError("Error when applying armature:   \n%s" % err)

        for pb in rig.pose.bones:
            pb.McpQuat = (1,0,0,0)

        bpy.ops.object.mode_set(mode='OBJECT')
        for ob in children:
            name = ob.McpArmatureModifier
            setActiveObject(context, ob)
            mod = ob.modifiers.new(name, 'ARMATURE')
            mod.object = rig
            mod.use_vertex_groups = True
            bpy.ops.object.modifier_move_up(modifier=name)
            if False and ob.data.shape_keys:
                skey = ob.data.shape_keys.key_blocks[name]
                skey.value = 1.0

        setActiveObject(context, rig)
        raise MocapMessage("Applied pose as rest pose")

#------------------------------------------------------------------
#   Automatic T-Pose
#------------------------------------------------------------------

TPose = {
    #"shoulder.L" : (0, 0, -90*D),
    "upper_arm.L" : (0, 0, -90*D),
    "forearm.L" :   (0, 0, -90*D),
    "hand.L" :      (0, 0, -90*D),

    #"shoulder.R" : (0, 0, 90*D),
    "upper_arm.R" : (0, 0, 90*D),
    "forearm.R" :   (0, 0, 90*D),
    "hand.R" :      (0, 0, 90*D),

    "thigh.L" :     (-90*D, 0, 0),
    "shin.L" :      (-90*D, 0, 0),
    #"foot.L" :      (None, 0, 0),
    #"toe.L" :       (pi, 0, 0),

    "thigh.R" :     (-90*D, 0, 0),
    "shin.R" :      (-90*D, 0, 0),
    #"foot.R" :      (None, 0, 0),
    #"toe.R" :       (pi, 0, 0),

    #"f_thumb.01.L": (0, 0, -135*D),
    #"f_thumb.02.L": (0, 0, -135*D),
    #"f_thumb.03.L": (0, 0, -135*D),
    "f_index.01.L": (0, 0, -90*D),
    "f_index.02.L": (0, 0, -90*D),
    #"f_index.03.L": (0, 0, -90*D),
    "f_middle.01.L": (0, 0, -90*D),
    "f_middle.02.L": (0, 0, -90*D),
    #"f_middle.03.L": (0, 0, -90*D),
    "f_ring.01.L": (0, 0, -90*D),
    "f_ring.02.L": (0, 0, -90*D),
    #"f_ring.03.L": (0, 0, -90*D),
    "f_pinky.01.L": (0, 0, -90*D),
    "f_pinky.02.L": (0, 0, -90*D),
    #"f_pinky.03.L": (0, 0, -90*D),

    #"f_thumb.01.R": (0, 0, 135*D),
    #"f_thumb.02.R": (0, 0, 135*D),
    #"f_thumb.03.R": (0, 0, 135*D),
    "f_index.01.R": (0, 0, 90*D),
    "f_index.02.R": (0, 0, 90*D),
    #"f_index.03.R": (0, 0, 90*D),
    "f_middle.01.R": (0, 0, 90*D),
    "f_middle.02.R": (0, 0, 90*D),
    #"f_middle.03.R": (0, 0, 90*D),
    "f_ring.01.R": (0, 0, 90*D),
    "f_ring.02.R": (0, 0, 90*D),
    #"f_ring.03.R": (0, 0, 90*D),
    "f_pinky.01.R": (0, 0, 90*D),
    "f_pinky.02.R": (0, 0, 90*D),
    #"f_pinky.03.R": (0, 0, 90*D),
}

def autoTPose(context, rig):
    print("Auto T-pose", rig.name)
    scn = context.scene
    putInRestPose(context, rig, True)
    for pb in BS.sortBones(rig):
        if pb.McpBone not in TPose.keys():
            continue
        euler = Euler(TPose[pb.McpBone], 'XYZ')
        M1 = euler.to_matrix().to_4x4()
        R1 = pb.bone.matrix_local
        par = rig.pose.bones.get(pb.McpParent)
        if par:
            M0 = par.matrix
            R0 = par.bone.matrix_local
            L1 = R1.inverted() @ R0 @ M0.inverted() @ M1
        else:
            L1 = R1.inverted() @ M1
        euler = L1.to_euler('YZX')
        euler.y = 0
        pb.matrix_basis = euler.to_matrix().to_4x4()
        setKeys(pb)
        updateScene(context)

#------------------------------------------------------------------
#   Put in rest and T pose
#------------------------------------------------------------------

def putInRestPose(context, rig, useSetKeys):
    for pb in rig.pose.bones:
        pb.matrix_basis = Matrix()
        if useSetKeys:
            setKeys(pb)
    updateScene(context)


def putInRightPose(context, rig, tpose):
    if tpose != "Default":
        tinfo = BS.tposeInfos.get(tpose)
        if tinfo:
            tinfo.addTPose(rig)
            putInTPose(context, rig, tpose)
            return True
    else:
        putInRestPose(context, rig, True)
    return False


def getStoredTPose(context, rig, useSetKeys):
    for pb in rig.pose.bones:
        quat = Quaternion(pb.McpQuat)
        pb.matrix_basis = quat.to_matrix().to_4x4()
        if useSetKeys:
            setKeys(pb)
    updateScene(context)


def setKeys(pb):
    if pb.rotation_mode == "QUATERNION":
        pb.keyframe_insert("rotation_quaternion", group=pb.name)
    elif pb.rotation_mode == "AXIS_ANGLE":
        pb.keyframe_insert("rotation_axis_angle", group=pb.name)
    else:
        pb.keyframe_insert("rotation_euler", group=pb.name)


def putInTPose(context, rig, name):
    scn = context.scene
    if False and rig.McpTPoseDefined:
        getStoredTPose(context, rig, True)
    elif name == "Default":
        autoTPose(context, rig)
        print("Put %s in automatic T-pose" % (rig.name))
    else:
        info = BS.tposeInfos.get(name)
        if info is None:
            raise MocapError("T-pose %s not found" % name)
        info.addTPose(rig)
        getStoredTPose(context, rig, True)
        print("Put %s in T-pose %s" % (rig.name, name))
    updateScene(context)


class MCP_OT_PutInSrcTPose(BvhPropsOperator, IsArmature, Rigger):
    bl_idname = "mcp.put_in_src_t_pose"
    bl_label = "Put In T-pose (Source)"
    bl_description = "Put the character into T-pose"
    bl_options = {'UNDO'}

    isSourceRig = True

    def run(self, context):
        rig = self.initRig(context)
        putInTPose(context, rig, context.scene.McpSourceTPose)
        print("Pose set to source T-pose")

    def invoke(self, context, event):
        BS.ensureSourceInited(context.scene)
        return BvhPropsOperator.invoke(self, context, event)


class MCP_OT_PutInTrgTPose(BvhPropsOperator, IsArmature, Rigger):
    bl_idname = "mcp.put_in_trg_t_pose"
    bl_label = "Put In T-pose (Target)"
    bl_description = "Put the character into T-pose"
    bl_options = {'UNDO'}

    isSourceRig = False

    def run(self, context):
        rig = self.initRig(context)
        putInTPose(context, rig, context.scene.McpTargetTPose)
        print("Pose set to target T-pose")

    def invoke(self, context, event):
        BS.ensureTargetInited(context.scene)
        return BvhPropsOperator.invoke(self, context, event)

#------------------------------------------------------------------
#   Define and undefine T-Pose
#------------------------------------------------------------------

class MCP_OT_DefineTPose(BvhOperator, IsArmature):
    bl_idname = "mcp.define_t_pose"
    bl_label = "Define T-pose"
    bl_description = "Define T-pose as current pose"
    bl_options = {'UNDO'}

    def run(self, context):
        rig = context.object
        for pb in rig.pose.bones:
            pb.McpQuat = pb.matrix_basis.to_quaternion()
        rig.McpTPoseDefined = True
        print("T-pose defined as current pose")


class MCP_OT_UndefineTPose(BvhOperator, IsArmature):
    bl_idname = "mcp.undefine_t_pose"
    bl_label = "Undefine T-pose"
    bl_description = "Remove definition of T-pose"
    bl_options = {'UNDO'}

    def run(self, context):
        rig = context.object
        rig.McpTPoseDefined = False
        quat = Quaternion()
        for pb in rig.pose.bones:
            pb.McpQuat = quat
        print("Undefined T-pose")

#------------------------------------------------------------------
#   Load T-pose from file
#------------------------------------------------------------------

def getBoneName(rig, name):
    if rig.McpIsSourceRig:
        return name
    else:
        pb = getTrgBone(name, rig)
        if pb:
            return pb.name
        else:
            return ""


class MCP_OT_LoadTPose(BvhOperator, IsArmature, ExportHelper, JsonFile):
    bl_idname = "mcp.load_t_pose"
    bl_label = "Load T-Pose"
    bl_description = "Load T-pose from file"
    bl_options = {'UNDO'}

    isSourceRig = True

    def run(self, context):
        from .io_json import loadJson
        rig = context.object
        print("Loading %s" % self.filepath)
        struct = loadJson(self.filepath)
        rig.McpTPoseFile = self.filepath
        if "t-pose" in struct.keys():
            self.setTPose(context, rig, struct["t-pose"])
        else:
            raise MocapError("File does not define a T-pose:\n%s" % self.filepath)

    def setTPose(self, context, rig, struct):
        putInRestPose(context, rig, True)
        for bname,value in struct.items():
            if bname in rig.pose.bones.keys():
                pb = rig.pose.bones[bname]
                rotmode = ('XYZ' if pb.rotation_mode in ('QUATERNION', 'AXIS_ANGLE') else pb.rotation_mode)
                euler = Euler(Vector(value)*D, rotmode)
                quat = pb.McpQuat = euler.to_quaternion()
                pb.matrix_basis = quat.to_matrix().to_4x4()
                setKeys(pb)

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

#------------------------------------------------------------------
#   Save current pose to file
#------------------------------------------------------------------

class MCP_OT_SaveTPose(BvhOperator, IsArmature, ExportHelper, JsonFile):
    bl_idname = "mcp.save_t_pose"
    bl_label = "Save T-Pose"
    bl_description = "Save current pose as .json file"
    bl_options = {'UNDO'}

    onlyMcpBones : BoolProperty(
        name = "Only Mcp Bones",
        default = False,
    )

    def draw(self, context):
        self.layout.prop(self, "onlyMcpBones")

    def run(self, context):
        from collections import OrderedDict
        from .io_json import saveJson
        rig = context.object
        tstruct = OrderedDict()
        struct = OrderedDict()
        fname = os.path.splitext(os.path.basename(self.filepath))[0]
        words = [word.capitalize() for word in fname.split("_")]
        struct["name"] = " ".join(words)
        struct["t-pose"] = tstruct
        for pb in rig.pose.bones:
            bmat = pb.matrix
            rmat = pb.bone.matrix_local
            if pb.parent:
                bmat = pb.parent.matrix.inverted() @ bmat
                rmat = pb.parent.bone.matrix_local.inverted() @ rmat
            mat = rmat.inverted() @ bmat
            q = mat.to_quaternion()
            magn = math.sqrt( (q.w-1)*(q.w-1) + q.x*q.x + q.y*q.y + q.z*q.z )
            if magn > -1e-4:
                if pb.McpBone or not self.onlyMcpBones:
                    euler = Vector(mat.to_euler())/D
                    tstruct[pb.name] = [int(round(ex)) for ex in euler]

        if os.path.splitext(self.filepath)[-1] != ".json":
            filepath = self.filepath + ".json"
        else:
            filepath = self.filepath
        filepath = os.path.join(os.path.dirname(__file__), filepath)
        print("Saving %s" % filepath)
        saveJson(struct, filepath)
        print("Saved current pose")

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

#----------------------------------------------------------
#   Global T-pose
#----------------------------------------------------------

from .source import CRigInfo

class CTPoseInfo(CRigInfo):
    verboseString = "Read T-pose file"

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    MCP_OT_RestCurrentPose,
    MCP_OT_PutInSrcTPose,
    MCP_OT_PutInTrgTPose,
    #MCP_OT_DefineTPose,
    #MCP_OT_UndefineTPose,
    MCP_OT_LoadTPose,
    MCP_OT_SaveTPose,
]

def register():
    bpy.types.Object.McpTPoseDefined = BoolProperty(default = False)
    bpy.types.Object.McpTPoseFile = StringProperty(default = "")
    bpy.types.Object.McpArmatureName = StringProperty(default = "")
    bpy.types.Object.McpArmatureModifier = StringProperty(default = "")
    bpy.types.PoseBone.McpQuat = FloatVectorProperty(size=4, default=(1,0,0,0))
    bpy.types.Object.McpIsSourceRig = BoolProperty(default=False)

    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)
