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
from bpy_extras.io_utils import ExportHelper
import math
import os

from .utils import *
from .armature import CArmature
from .source import CRigInfo

#----------------------------------------------------------
#   Target classes
#----------------------------------------------------------

class CTargetInfo(CArmature, CRigInfo):
    verboseString = "Read target file"

    def __init__(self, scn, name):
        CArmature.__init__(self, scn)
        CRigInfo.__init__(self, scn, name)


class Target:
    useAutoTarget : BoolProperty(
        name = "Auto Target",
        description = "Find target rig automatically",
        default = True)

    def draw(self, context):
        self.layout.prop(self, "useAutoTarget")
        if not self.useAutoTarget:
            scn = context.scene
            self.layout.prop(scn, "McpTargetRig")
            self.layout.prop(scn, "McpTargetTPose")

    def findTarget(self, context, rig):
        return findTargetArmature(context, rig, self.useAutoTarget)

#
#   findTargetArmature(context, rig, auto):
#

def findTargetArmature(context, rig, auto):
    from .t_pose import autoTPose

    scn = context.scene
    BS.ensureTargetInited(scn)

    if auto:
        scn.McpTargetRig, scn.McpTargetTPose = guessArmatureFromList(rig, scn, BS.targetInfos)

    if scn.McpTargetRig == "Automatic":
        info = CTargetInfo(scn, "Automatic")
        tposed = info.identifyRig(context, rig, scn.McpTargetTPose)
        if not tposed:
            autoTPose(context, rig)
        BS.targetInfos["Automatic"] = info
        info.display("Target")
    else:
        info = BS.targetInfos[scn.McpTargetRig]
        info.addManualBones(rig)
        tinfo = BS.tposeInfos.get(scn.McpTargetTPose)
        if tinfo:
            tinfo.addTPose(rig)
        else:
            scn.McpTargetTPose = "Default"

    rig.McpArmature = info.name
    print("Using target armature %s." % rig.McpArmature)
    return info


def guessArmatureFromList(rig, scn, infos):
    print("Identifying rig")
    for name,info in infos.items():
        if name == "Automatic":
            continue
        elif matchAllBones(rig, info, scn):
            if info.t_pose_file:
                return name, info.t_pose_file
            else:
                return name, "Default"
    else:
        return "Automatic", "Default"


def matchAllBones(rig, info, scn):
    if not hasAllBones(info.fingerprint, rig):
        if BS.verbose:
            print(info.name, ": Fingerprint failed")
        return False
    if hasSomeBones(info.illegal, rig):
        if BS.verbose:
            print(info.name, ": Illegal bone")
        return False
    for bname,mhx in info.bones:
        if bname in info.optional:
            continue
        elif bname not in rig.data.bones.keys():
            if BS.verbose:
                print(info.name, ": Missing bone:", bname)
            return False
    return True

#-------------------------------------------------------------
#    Target initialization
#-------------------------------------------------------------

class MCP_OT_IdentifyTargetRig(BvhOperator, IsArmature):
    bl_idname = "mcp.identify_target_rig"
    bl_label = "Identify Target Rig"
    bl_description = "Identify the target rig type of the active armature."
    bl_options = {'UNDO'}

    def prequel(self, context):
        from .retarget import changeTargetData
        return changeTargetData(context.object, context.scene)

    def run(self, context):
        scn = context.scene
        scn.McpTargetRig = "Automatic"
        findTargetArmature(context, context.object, True)
        print("Identified rig %s" % scn.McpTargetRig)

    def sequel(self, context, data):
        from .retarget import restoreTargetData
        restoreTargetData(data)

#----------------------------------------------------------
#   List Rig
#----------------------------------------------------------

from .source import ListRig

class MCP_OT_ListTargetRig(BvhOperator, ListRig):
    bl_idname = "mcp.list_target_rig"
    bl_label = "List Target Rig"
    bl_description = "List the bone associations of the active target rig"
    bl_options = {'UNDO'}

    @classmethod
    def poll(self, context):
        return context.scene.McpTargetRig

    def getInfos(self, context):
        scn = context.scene
        info = BS.targetInfos.get(scn.McpTargetRig)
        tinfo = BS.tposeInfos.get(scn.McpTargetTPose)
        return info, tinfo


class MCP_OT_VerifyTargetRig(BvhOperator):
    bl_idname = "mcp.verify_target_rig"
    bl_label = "Verify Target Rig"
    bl_description = "Verify the target rig type of the active armature"
    bl_options = {'UNDO'}

    @classmethod
    def poll(self, context):
        ob = context.object
        return (context.scene.McpTargetRig and ob and ob.type == 'ARMATURE')

    def run(self, context):
        rigtype = context.scene.McpTargetRig
        info = BS.targetInfos[rigtype]
        info.testRig(rigtype, context.object, context.scene)
        raise MocapMessage("Target armature %s verified" % rigtype)

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    MCP_OT_IdentifyTargetRig,
    MCP_OT_ListTargetRig,
    MCP_OT_VerifyTargetRig,
]

def register():
    bpy.types.Scene.McpTargetRig = EnumProperty(
        items = [("Automatic", "Automatic", "Automatic")],
        name = "Target Rig",
        default = "Automatic")

    bpy.types.Scene.McpTargetTPose = EnumProperty(
        items = [("Default", "Default", "Default")],
        name = "TPose Target",
        default = "Default")

    bpy.types.Object.McpReverseHip = BoolProperty(
        name = "Reverse Hip",
        description = "The rig has a reverse hip",
        default = False)

    bpy.types.PoseBone.McpBone = StringProperty(
        name = "Canonical Bone Name",
        description = "Canonical bone corresponding to this bone",
        default = "")

    bpy.types.PoseBone.McpParent = StringProperty(
        name = "Parent",
        description = "Parent of this bone for retargeting purposes",
        default = "")

    bpy.types.Object.DazRig = bpy.props.StringProperty(
        name = "Rig Type",
        default = "")

    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)
