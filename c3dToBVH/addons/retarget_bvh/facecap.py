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
from bpy_extras.io_utils import ImportHelper
import os
from time import perf_counter
from mathutils import Vector, Euler, Matrix
from .utils import *

def propRef(prop):
    return '["%s"]' % prop

#------------------------------------------------------------------
#   Utility class BoneHandler
#------------------------------------------------------------------

class BoneHandler:
    def setRotation(self, pb, euler, frame, fraction=None):
        if fraction == 0 or pb is None:
            return
        elif fraction is not None:
            euler = Euler(fraction*Vector(euler))
        mat = euler.to_matrix()
        if pb.rotation_mode == 'QUATERNION':
            pb.rotation_quaternion = mat.to_quaternion()
            pb.keyframe_insert("rotation_quaternion", frame=frame, group=pb.name)
        else:
            pb.rotation_euler = mat.to_euler(pb.rotation_mode)
            pb.keyframe_insert("rotation_euler", frame=frame, group=pb.name)


    def getBones(self, bnames, rig):
        def getBone(bname, rig):
            if bname not in rig.pose.bones.keys():
                return None
            pb = rig.pose.bones[bname]
            if rig.animation_data and not self.useShapekeys:
                msg = ("Bone %s is driven.\nMake bones posable first" % bname)
                datapath = 'pose.bones["%s"].rotation_euler' % bname
                for fcu in rig.animation_data.drivers:
                    if fcu.data_path == datapath:
                        raise MocapError(msg)
            return pb

        for bname in bnames:
            pb = getBone(bname, rig)
            if pb:
                return pb
        print("Did not find bones: %s" % bnames)
        return None

#------------------------------------------------------------------
#   Head User
#------------------------------------------------------------------

class HeadUser:
    useHeadLoc : BoolProperty(
        name = "Head Location",
        description = "Include head location animation",
        default = False)

    useHeadRot : BoolProperty(
        name = "Head Rotation",
        description = "Include head rotation animation",
        default = True)

    headDist : FloatProperty(
        name = "Head",
        description = "Fraction of head rotation that affects head",
        min = 0.0, max = 1.0,
        default = 0.15)

    neckUpperDist : FloatProperty(
        name = "Upper Neck",
        description = "Fraction of head rotation that affects upper neck",
        min = 0.0, max = 1.0,
        default = 0.4)

    neckLowerDist : FloatProperty(
        name = "Lower Neck",
        description = "Fraction of head rotation that affects lower neck",
        min = 0.0, max = 1.0,
        default = 0.4)

    abdomenDist : FloatProperty(
        name = "Abdomen",
        description = "Fraction of head rotation that affects abdomen",
        min = 0.0, max = 1.0,
        default = 0.05)

    def draw(self, context):
        self.layout.prop(self, "useHeadLoc")
        self.layout.prop(self, "useHeadRot")
        if self.useHeadRot:
            box = self.layout.box()
            box.prop(self, "headDist")
            box.prop(self, "neckUpperDist")
            box.prop(self, "neckLowerDist")
            box.prop(self, "abdomenDist")

    def setupHead(self, rig):
        self.head = self.getBones(["head"], rig)
        self.neckUpper = self.getBones(["neckUpper", "neck2", "neck-1"], rig)
        self.neckLower = self.getBones(["neckLower", "neck1", "neck"], rig)
        self.abdomen = self.getBones(["abdomenUpper", "spine2", "spine-1", "spine_fk.002"], rig)
        self.hip = self.getBones(["hip", "torso"], rig)
        if self.head is None:
            self.headDist = 0
        if self.neckUpper is None:
            self.neckUpperDist = 0
        if self.neckLower is None:
            self.neckLowerDist = 0
        if self.abdomen is None:
            self.abdomenDist = 0
        distsum = self.headDist + self.neckUpperDist + self.neckLowerDist + self.abdomenDist
        self.headDist /= distsum
        self.neckUpperDist /= distsum
        self.neckLowerDist /= distsum
        self.abdomenDist /= distsum

#------------------------------------------------------------------
#   Generic FACS importer
#------------------------------------------------------------------

class FACSImporter(BoneHandler):

    useShapekeys : BoolProperty(
        name = "Load To Shapekeys",
        description = "Load morphs to mesh shapekeys instead of rig properties",
        default = False)

    useEyes : BoolProperty(
        name = "Eyes",
        description = "Include eyes animation",
        default = False)

    useTongue : BoolProperty(
        name = "Tongue",
        description = "Include tongue animation",
        default = False)

    filepath : StringProperty(
        name="File Path",
        description="Filepath used for importing the file",
        maxlen=1024,
        default="")

    makeNewAction : BoolProperty(
        name = "New Action",
        description = "Unlink current action and make a new one",
        default = True)

    actionName : StringProperty(
        name = "Action Name",
        description = "Name of loaded action.\nUse file name if blank",
        default = "")

    fps : FloatProperty(
        name = "Frame Rate",
        description = "Animation FPS in FaceCap/LiveLink file.\nFPS = 0 means one frame per step",
        min = 0,
        default = 0)

    def draw(self, context):
        self.layout.prop(self, "fps")
        self.layout.prop(self, "makeNewAction")
        if self.makeNewAction:
            self.layout.prop(self, "actionName")
        #self.layout.prop(self, "useShapekeys")
        #self.layout.prop(self, "useEyes")
        #self.layout.prop(self, "useTongue")


    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


    def run(self, context):
        rig = context.object
        rig["MhaGaze_L"] = 0.0
        rig["MhaGaze_R"] = 0.0
        self.getSource(context, rig)
        self.setupFacsTable(rig)
        self.bshapes = []
        self.bskeys = {}
        self.hlockeys = {}
        self.hrotkeys = {}
        self.leyekeys = {}
        self.reyekeys = {}
        self.shapekeys = {}
        if self.useShapekeys:
            for ob in getShapeChildren(rig):
                for skey in ob.data.shape_keys.key_blocks:
                    self.shapekeys[skey.name] = True
        self.parse(context)
        first = list(self.bskeys.values())[0]
        print("Blendshapes: %d\nKeys: %d" % (len(self.bshapes), len(first)))
        if self.makeNewAction and rig.animation_data:
            rig.animation_data.action = None
        if self.makeNewAction and self.useShapekeys:
            for ob in getShapeChildren(rig):
                if ob.data.shape_keys.animation_data:
                    ob.data.shape_keys.animation_data.action = None
        self.build(rig, context)
        if self.makeNewAction and rig.animation_data:
            act = rig.animation_data.action
            if act:
                if self.actionName:
                    act.name = self.actionName
                else:
                    act.name = os.path.splitext(os.path.basename(self.filepath))[0]
                print("NEW", act.name)


    def getSource(self, context, rig):
        pass


    def setupFacsTable(self, rig):
        def copyTable(char):
            print("Setting up FACS table for %s" % char)
            miss = []
            for key,data in struct["facs"].items():
                prop, weight = data
                self.facstable[key.lower()] = {prop : weight}
                if prop not in rig.keys():
                    miss.append(prop)
            print("Missing FACS morphs: %s" % miss)

        BS.ensureFacsInited()
        self.facstable = {}
        for prop,struct in BS.facsTables.items():
            if prop in rig.keys():
                copyTable(struct["name"])
                return


    def build(self, rig, context):
        def isMatch(string, bases):
            for base in bases:
                if string in base:
                    return True
            return False

        missing = []
        for bshape in self.bshapes:
            if bshape not in self.facstable.keys():
                missing.append(bshape)
        if missing:
            msg = "Missing blendshapes:     \n"
            for bshape in missing:
                msg += ("  %s\n" % bshape)
            print("Total %d blendshapes missing" % len(missing))
            raise MocapError(msg)

        self.setupBones(rig)
        self.skipped = {}
        if self.useShapekeys:
            self.facsShapes = {}
            for key,value in self.facstable.items():
                if key in self.shapekeys.keys():
                    self.facsShapes[key] = value
            self.facsProps = {}
        else:
            self.facsShapes = {}
            self.facsProps = self.facstable

        missingShapes = {}
        self.scale = rig.DazScale
        warned = []
        nframes = len(self.bskeys)
        t1 = perf_counter()
        for n,t in enumerate(self.bskeys.keys()):
            prev = {}
            if self.fps == 0:
                frame = n+1
            else:
                frame = self.getFrame(t)
            self.setBoneFrame(t, frame, context)
            for bshape,value in zip(self.bshapes, self.bskeys[t]):
                formulas = self.facsShapes.get(bshape, {})
                for prop,factor in formulas.items():
                    for ob in getShapeChildren(rig):
                        if prop in ob.data.shape_keys.key_blocks.keys():
                            skey = ob.data.shape_keys.key_blocks[prop]
                            prev[prop] = skey.value = value*factor + prev.get(prop, 0)
                            skey.keyframe_insert("value", frame=frame)
                        else:
                            if ob.name not in missingShapes.keys():
                                missingShapes[ob.name] = {}
                            missingShapes[ob.name][prop] = True

                formulas = self.facsProps.get(bshape, {})
                for prop,factor in formulas.items():
                    prev[prop] = rig[prop] = value*factor + prev.get(prop, 0)
                    rig.keyframe_insert(propRef(prop), frame=frame, group="FACS")
                if formulas:
                    continue

                if bshape not in warned and bshape not in self.skipped.keys():
                    print("MISS", bshape)
                    warned.append(bshape)
        t2 = perf_counter()
        print("%d frames loaded in %g seconds" % (nframes, t2-t1))
        if missingShapes:
            msg = "The following objects are missing shapekeys:\n"
            for obname in missingShapes.keys():
                msg += "  %s\n" % obname
            raise MocapError(msg, warning=True)


    def setupBones(self, rig):
        self.leye = self.getBones(["lEye", "l_eye", "eye.L"], rig)
        self.reye = self.getBones(["rEye", "r_eye", "eye.R"], rig)
        self.setupHead(rig)


    def setupHead(self, rig):
        pass


    def setBoneFrame(self, t, frame, context):
        if self.useHeadLoc:
            self.hip.location = self.scale*self.hlockeys[t]
            self.hip.keyframe_insert("location", frame=frame, group="hip")
        if self.useHeadRot:
            self.setRotation(self.head, self.hrotkeys[t], frame, self.headDist)
            self.setRotation(self.neckUpper, self.hrotkeys[t], frame, self.neckUpperDist)
            self.setRotation(self.neckLower, self.hrotkeys[t], frame, self.neckLowerDist)
            self.setRotation(self.abdomen, self.hrotkeys[t], frame, self.abdomenDist)
        if self.useEyes:
            self.setRotation(self.leye, self.leyekeys[t], frame)
            self.setRotation(self.reye, self.reyekeys[t], frame)

#------------------------------------------------------------------
#   FaceCap
#------------------------------------------------------------------

class MCP_OT_ImportFaceCap(HeadUser, FACSImporter, BvhOperator, IsArmature):
    bl_idname = "mcp.import_facecap"
    bl_label = "Import FaceCap File"
    bl_description = "Import a text file with facecap data"
    bl_options = {'UNDO'}

    filename_ext = ".txt"
    filter_glob : StringProperty(default="*.txt", options={'HIDDEN'})

    def draw(self, context):
        FACSImporter.draw(self, context)
        HeadUser.draw(self, context)

    def getFrame(self, t):
        return self.fps * 1e-3 * t

    # timestamp in milli seconds (file says nano),
    # head position xyz,
    # head eulerAngles xyz,
    # left-eye eulerAngles xy,
    # right-eye eulerAngles xy,
    # blendshapes
    def parse(self, context):
        with open(self.filepath, "r", encoding="utf-8-sig") as fp:
            for line in fp:
                line = line.strip()
                if line[0:3] == "bs,":
                    self.bshapes = [bshape.lower() for bshape in line.split(",")[1:]]
                elif line[0:2] == "k,":
                    words = line.split(",")
                    t = int(words[1])
                    self.hlockeys[t] = Vector((float(words[2]), -float(words[3]), -float(words[4])))
                    self.hrotkeys[t] = Euler((D*float(words[5]), D*float(words[6]), D*float(words[7])))
                    self.leyekeys[t] = Euler((D*float(words[9]), 0.0, D*float(words[8])))
                    self.reyekeys[t] = Euler((D*float(words[11]), 0.0, D*float(words[10])))
                    self.bskeys[t] = [float(word) for word in words[12:]]
                elif line[0:5] == "info,":
                    pass
                else:
                    raise MocapError("Illegal syntax:\%s     " % line)

#------------------------------------------------------------------
#   Unreal Live Link
#------------------------------------------------------------------

class MCP_OT_ImportLiveLink(HeadUser, FACSImporter, BvhOperator, IsArmature):
    bl_idname = "mcp.import_livelink"
    bl_label = "Import Live Link File"
    bl_description = "Import a csv file with Unreal's Live Link data"
    bl_options = {'UNDO'}

    filename_ext = ".csv"
    filter_glob : StringProperty(default="*.csv", options={'HIDDEN'})

    def draw(self, context):
        FACSImporter.draw(self, context)
        HeadUser.draw(self, context)

    def getFrame(self, t):
        return t+1

    def parse(self, context):
        from csv import reader
        with open(self.filepath, newline='', encoding="utf-8-sig") as fp:
            lines = list(reader(fp))
        if len(lines) < 2:
            raise MocapError("Found no keyframes")

        self.bshapes = [bshape.lower() for bshape in lines[0][2:-9]]
        for t,line in enumerate(lines[1:]):
            nums = [float(word) for word in line[2:]]
            self.bskeys[t] = nums[0:-9]
            self.hlockeys[t] = Vector((0,0,0))
            yaw,pitch,roll = nums[-9:-6]
            self.hrotkeys[t] = Euler((-pitch, -yaw, roll))
            yaw,pitch,roll = nums[-6:-3]
            self.leyekeys[t] = Euler((-pitch, roll, yaw))
            yaw,pitch,roll = nums[-3:]
            self.reyekeys[t] = Euler((-pitch, roll, yaw))

        for key in self.bshapes:
            if key not in self.facstable.keys():
                print(key)

#------------------------------------------------------------------
#   Copy FACS animation
#------------------------------------------------------------------

class FACSCopier:
    useHeadLoc = False
    useHeadRot = False

    def getFcurves(self, act):
        fcus = []
        tmin = 99999
        tmax = -99999
        for fcu in act.fcurves:
            sname,channel = getShapeChannel(fcu)
            if sname and channel == "value":
                fcus.append((sname, fcu))
                times = [kp.co[0] for kp in fcu.keyframe_points]
                t0 = int(min(times))
                t1 = int(max(times))
                if t0 < tmin:
                    tmin = t0
                if t1 > tmax:
                    tmax = t1
        for t in range(tmin, tmax+1):
            self.bskeys[t] = []
            self.hlockeys[t] = Vector((0,0,0))
            self.hrotkeys[t] = Euler((0,0,0))
            self.leyekeys[t] = Euler((0,0,0))
            self.reyekeys[t] = Euler((0,0,0))
        for sname,fcu in fcus:
            self.bshapes.append(sname.lower())
            for t in range(tmin, tmax+1):
                self.bskeys[t].append(fcu.evaluate(t))


    def getFrame(self, t):
        return t+1


class MCP_OT_CopyFacsAnimation(BvhPropsOperator, FACSImporter, FACSCopier):
    bl_idname = "mcp.copy_facs_animation"
    bl_label = "Copy FACS Animation"
    bl_description = "Copy FACS animation from selected mesh to active character"
    bl_options = {'UNDO'}

    def getSource(self, context, rig):
        self.action = None
        for ob in getSelectedMeshes(context):
            if ob != rig:
                skeys = ob.data.shape_keys
                if skeys and skeys.animation_data and skeys.animation_data.action:
                    self.action = skeys.animation_data.action
                    return
        raise MocapError("No source mesh found")

    def parse(self, context):
        self.getFcurves(self.action)

#------------------------------------------------------------------
#   BVH
#------------------------------------------------------------------

class MCP_OT_ImportFbxFacs(FACSImporter, BvhOperator, FACSCopier, IsArmature):
    bl_idname = "mcp.import_fbx_facs"
    bl_label = "Import FACS From FBX File"
    bl_description = "Import a fbx file with FACS animation"
    bl_options = {'UNDO'}

    filename_ext = ".fbx"
    filter_glob : StringProperty(default="*.fbx", options={'HIDDEN'})

    def parse(self, context):
        from .load import deleteObjects
        print("Importing FBX file")
        existing_objects = set(context.scene.objects)
        try:
            bpy.ops.import_scene.fbx(filepath = self.filepath, automatic_bone_orientation=True, ignore_leaf_bones=True)
        except AttributeError:
            raise MocapError("Blender's built-in FBX importer must be enabled")
        imported_objects = set(context.scene.objects) - existing_objects
        print("Temporary FBX objects imported: %s" % imported_objects)
        actions = []
        for ob in imported_objects:
            if ob and ob.animation_data and ob.animation_data.action:
                actions.append(ob.animation_data.action)
            if ob.type == 'MESH':
                skeys = ob.data.shape_keys
                if skeys and skeys.animation_data:
                    act = skeys.animation_data.action
                    if act:
                        print("FBX MESH:", ob.name, skeys.name)
                        actions.append(act)
                        self.getFcurves(act)
        print("Deleting temporary FBX objects")
        for act in actions:
            bpy.data.actions.remove(act)
        deleteObjects(context, imported_objects)


#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    MCP_OT_ImportFaceCap,
    MCP_OT_ImportLiveLink,
    MCP_OT_CopyFacsAnimation,
    MCP_OT_ImportFbxFacs,
]

def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)