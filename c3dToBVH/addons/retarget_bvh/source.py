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
import os
from collections import OrderedDict
from math import pi
from mathutils import *
from bpy.props import *

from .armature import CArmature
from .utils import *

#----------------------------------------------------------
#   Source classes
#----------------------------------------------------------

class CRigInfo:
    def __init__(self, scn, name):
        self.name = name
        self.filepath = "None"
        self.bones = []
        self.boneNames = {}
        self.parents = {}
        self.optional = []
        self.fingerprint = []
        self.illegal = []
        self.t_pose = {}
        self.t_pose_file = None


    def readFile(self, filepath):
        from .io_json import loadJson
        if BS.verbose:
            print(self.verboseString, filepath)
        self.filepath = filepath
        struct = loadJson(filepath)
        if "name" in struct.keys():
            self.name = struct["name"]
        else:
            self.name = os.path.splitext(os.path.basename(filepath))[0]
        if "bones" in struct.keys():
            self.bones = [(key, nameOrNone(value)) for key,value in struct["bones"].items()]
            self.boneNames = dict([(canonicalName(key), value) for key,value in self.bones])
        if "parents" in struct.keys():
            self.parents = struct["parents"]
        if "optional" in struct.keys():
            self.optional = struct["optional"]
        if "fingerprint" in struct.keys():
            self.fingerprint = struct["fingerprint"]
        if "illegal" in struct.keys():
            self.illegal = struct["illegal"]
        if "t-pose" in struct.keys():
            self.t_pose = struct["t-pose"]
        if "t-pose-file" in struct.keys():
            self.t_pose_file = struct["t-pose-file"]


    def identifyRig(self, context, rig, tpose):
        from .t_pose import putInRightPose
        tposed = putInRightPose(context, rig, tpose)
        self.findArmature(rig)
        self.addAutoBones(rig)
        return tposed


    def getHip(self):
        for bname,mhx in self.boneNames.items():
            if mhx == "hips":
                return bname
        return None


    def clearMcpBones(self, rig):
        for pb in rig.pose.bones:
            pb.McpBone = ""
            pb.McpParent = ""


    def addAutoBones(self, rig):
        self.bones = []
        for pb in rig.pose.bones:
            if pb.McpBone:
                self.bones.append( (pb.name, pb.McpBone) )
        self.addParents(rig)
        rig.McpTPoseDefined = False


    def addManualBones(self, rig):
        for pb in rig.pose.bones:
            pb.McpBone = ""
        for bname,mhx in self.bones:
            if bname in rig.pose.bones.keys():
                pb = rig.pose.bones[bname]
                pb.McpBone = mhx
            else:
                print("  Missing:", bname)
        rig.McpTPoseDefined = False
        self.addParents(rig)


    def addTPose(self, rig):
        for bname in self.t_pose.keys():
            if bname in rig.pose.bones.keys():
                pb = rig.pose.bones[bname]
                rotmode = ('XYZ' if pb.rotation_mode in ('QUATERNION', 'AXIS_ANGLE') else pb.rotation_mode)
                euler = Euler(Vector(self.t_pose[bname])*D, rotmode)
                pb.McpQuat = euler.to_quaternion()
        rig.McpTPoseDefined = True


    def getParent(self, rig, bname, pname):
        if pname in rig.pose.bones.keys():
            return pname
        elif pname in self.parents.keys():
            return self.getParent(rig, pname, self.parents[pname])
        else:
            return ""


    def addParents(self, rig):
        for pb in rig.pose.bones:
            if pb.McpBone:
                pb.McpParent = ""
                par = pb.parent
                while par:
                    if par.McpBone:
                        pb.McpParent = par.name
                        break
                    par = par.parent
        for bname,pname in self.parents.items():
            if bname in rig.pose.bones.keys():
                pname = self.getParent(rig, bname, pname)
                pb = rig.pose.bones[bname]
                pb.McpParent = pname

        if BS.verbose:
            print("Parents")
            for pb in rig.pose.bones:
                if pb.McpBone:
                    print("  ", pb.name, pb.McpParent)


    def testRig(self, name, rig, scn):
        from .armature import validBone
        if not self.bones:
            raise MocapError("Cannot verify after rig identification failed")
        print("Testing %s" % name)
        bname = hasSomeBones(self.illegal, rig)
        if bname:
            raise MocapError(
                    "Armature %s does not\n" % rig.name +
                    "match armature %s.\n" % name +
                    "Has illegal bone %s     " % bname)

        pbones = dict([(pb.name,pb) for pb in rig.pose.bones])
        for pb in rig.pose.bones:
            pbones[pb.name.lower()] = pb
        for (bname, mhxname) in self.bones:
            if bname in self.optional:
                continue
            if bname in pbones.keys():
                pb = pbones[bname]
            else:
                pb = None
            if pb is None or not validBone(pb):
                print("  Did not find bone %s (%s)" % (bname, mhxname))
                print("Bones:")
                for pair in self.bones:
                    print("  %s : %s" % pair)
                raise MocapError(
                    "Armature %s does not\n" % rig.name +
                    "match armature %s.\n" % name +
                    "Did not find bone %s     " % bname)


class CSourceInfo(CArmature, CRigInfo):
    verboseString = "Read source file"

    def __init__(self, scn, name):
        CArmature.__init__(self, scn)
        CRigInfo.__init__(self, scn, name)

#
#   findSourceArmature(context, rig, auto):
#

def findSourceArmature(context, rig, auto):
    from .t_pose import autoTPose, putInRestPose
    scn = context.scene

    BS.ensureSourceInited(scn)
    if auto:
        from .target import guessArmatureFromList
        scn.McpSourceRig, scn.McpSourceTPose = guessArmatureFromList(rig, scn, BS.sourceInfos)

    if scn.McpSourceRig == "Automatic":
        info = CSourceInfo(scn, "Automatic")
        tposed = info.identifyRig(context, rig, scn.McpSourceTPose)
        if not tposed:
            autoTPose(context, rig)
            scn.McpSourceTPose = "Default"
        BS.sourceInfos["Automatic"] = info
        BS.activeSrcInfo = info
        info.display("Source")
    else:
        info = BS.sourceInfos[scn.McpSourceRig]
        BS.activeSrcInfo = info
        info.addManualBones(rig)
        tinfo = BS.tposeInfos.get(scn.McpSourceTPose)
        if tinfo:
            tinfo.addTPose(rig)
        else:
            scn.McpSourceTPose = "Default"

    rig.McpArmature = BS.activeSrcInfo.name
    print("Using source armature %s." % rig.McpArmature)

#
#    setSourceArmature(rig, scn)
#

def setSourceArmature(rig, scn):
    name = rig.McpArmature
    if name:
        scn.McpSourceRig = name
    else:
        raise MocapError("No source armature set")
    BS.activeSrcInfo = BS.sourceInfos[name]
    print("Set source armature to %s" % name)


#----------------------------------------------------------
#   Class
#----------------------------------------------------------

class Source:
    useAutoSource : BoolProperty(
        name = "Auto Source",
        description = "Find source rig automatically",
        default = True)

    def draw(self, context):
        self.layout.prop(self, "useAutoSource")
        if not self.useAutoSource:
            scn = context.scene
            self.layout.prop(scn, "McpSourceRig")
            self.layout.prop(scn, "McpSourceTPose")

    def findSource(self, context, rig):
        return findSourceArmature(context, rig, self.useAutoSource)

#----------------------------------------------------------
#   Source initialization
#----------------------------------------------------------

class MCP_OT_InitKnownRigs(bpy.types.Operator):
    bl_idname = "mcp.init_known_rigs"
    bl_label = "Init Known Rigs"
    bl_description = "(Re)load all json files in the known_rigs directory."
    bl_options = {'UNDO'}

    def execute(self, context):
        BS.initSources(context.scene)
        BS.initTargets(context.scene)
        return{'FINISHED'}

#----------------------------------------------------------
#   List Rig
#
#   (mhx bone, text)
#----------------------------------------------------------

class ListRig:
    def draw(self, context):
        info,tinfo = self.getInfos(context)
        mcpbones = dict([(mcpname, bname) for bname,mcpname in info.bones])
        if not mcpbones:
            return
        bonelist = []
        for mcpname,longname in BS.BoneNames:
            if mcpname is None:
                column = []
                bonelist.append(column)
            else:
                bname = mcpbones.get(mcpname, "")
                column.append((longname, bname))
        nrows = max([len(column) for column in bonelist])
        for column in bonelist:
            while len(column) < nrows:
                column.append(None)
        box = self.layout.box()
        for m in range(nrows):
            row = box.row()
            for column in bonelist:
                if column[m]:
                    string = "%-20s: %20s" % column[m]
                else:
                    string = ""
                row.label(text=string)


    def invoke(self, context, event):
        clearErrorMessage()
        wm = context.window_manager
        return wm.invoke_props_dialog(self, width=1100)


class MCP_OT_ListSourceRig(BvhOperator, ListRig):
    bl_idname = "mcp.list_source_rig"
    bl_label = "List Source Rig"
    bl_description = "List the bone associations of the active source rig"
    bl_options = {'UNDO'}

    @classmethod
    def poll(self, context):
        return context.scene.McpSourceRig

    def getInfos(self, context):
        scn = context.scene
        info = BS.sourceInfos.get(scn.McpSourceRig)
        tinfo = BS.tposeInfos.get(scn.McpSourceTPose)
        return info, tinfo


class MCP_OT_VerifySourceRig(BvhOperator):
    bl_idname = "mcp.verify_source_rig"
    bl_label = "Verify Source Rig"
    bl_description = "Verify the source rig type of the active armature"
    bl_options = {'UNDO'}

    @classmethod
    def poll(self, context):
        ob = context.object
        return (context.scene.McpSourceRig and ob and ob.type == 'ARMATURE')

    def run(self, context):
        rigtype = context.scene.McpSourceRig
        info = BS.sourceInfos[rigtype]
        info.testRig(rigtype, context.object, context.scene)
        raise MocapMessage("Source armature %s verified" % rigtype)


class MCP_OT_IdentifySourceRig(BvhOperator):
    bl_idname = "mcp.identify_source_rig"
    bl_label = "Identify Source Rig"
    bl_description = "Identify the source rig type of the active armature"
    bl_options = {'UNDO'}

    @classmethod
    def poll(self, context):
        ob = context.object
        return (ob and ob.type == 'ARMATURE')

    def run(self, context):
        from .target import guessArmatureFromList
        scn = context.scene
        rig = context.object
        scn.McpSourceRig,scn.McpSourceTPose = guessArmatureFromList(rig, scn, BS.sourceInfos)
        info = BS.sourceInfos[scn.McpSourceRig]
        if scn.McpSourceRig == "Automatic":
            info.identifyRig(context, rig, scn.McpSourceTPose)
            info.addAutoBones(rig)
        else:
            info.addManualBones(rig)
            tinfo = BS.tposeInfos.get(info.t_pose_file)
            if tinfo:
                scn.McpSourceTPose = tinfo.name
                tinfo.addTPose(rig)
        print("Identified rig %s" % scn.McpSourceRig)

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    MCP_OT_InitKnownRigs,
    MCP_OT_ListSourceRig,
    MCP_OT_VerifySourceRig,
    MCP_OT_IdentifySourceRig,
]

def register():
    bpy.types.Scene.McpSourceRig = EnumProperty(
        items = [("Automatic", "Automatic", "Automatic")],
        name = "Source Rig",
        default = "Automatic")

    bpy.types.Scene.McpSourceTPose = EnumProperty(
        items = [("Default", "Default", "Default")],
        name = "TPose Source",
        default = "Default")

    bpy.types.Object.McpArmature = StringProperty()

    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)
