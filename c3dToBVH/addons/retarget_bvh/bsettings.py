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

import os
import sys
import bpy
from bpy.props import EnumProperty

class BvhSettings:
    def __init__(self):
        if sys.platform == 'win32':
            self.defaultDir = "~/Documents/DAZ Importer"
        elif sys.platform == 'darwin':
            self.defaultDir = "~/DAZ Importer"
        else:
            self.defaultDir = "~/DAZ Importer"
        self.settingsPath = "%s/%s" % (self.defaultDir, "retarget_bvh_settings.json")

        # Settings
        self.verbose = False
        self.useLimits = True
        self.useUnlock = False
        self.useBlenderBvh = False
        self.useNativeFbx = False

        # Global variables
        self.sourceInfos = {}
        self.activeSrcInfo = None
        self.targetInfos = {}
        self.tposeInfos = {}
        self.activeTPoseInfo = None
        self.facsTables = {}

        self.markers = []
        self.editLoc = None
        self.editRot = None


    def fromDialog(self, btn):
        for attr in dir(self):
            if attr[0] != "_" and hasattr(btn, attr):
                setattr(self, attr, getattr(btn, attr))


    def toDialog(self, btn):
        for attr in dir(self):
            if attr[0] != "_":
                try:
                    setattr(btn, attr, getattr(self, attr))
                except:
                    pass


    def loadSettings(self):
        from .io_json import loadJson
        filepath = os.path.expanduser(self.settingsPath)
        if not os.path.exists(filepath):
            print('Did not find settings file: "%s"' % filepath)
            return
        struct = loadJson(filepath)
        if struct and "bvh-settings" in struct.keys():
            print('Load settings from "%s"' % filepath)
            settings = struct["bvh-settings"]
            for attr,value in settings.items():
                if hasattr(self, attr) and isinstance(value, (float, int, bool, str)):
                    setattr(self, attr, value)


    def saveSettings(self):
        from .io_json import saveJson
        struct = {}
        for attr in dir(self):
            value = getattr(self, attr)
            if attr[0] != "_" and isinstance(value, (int, float, bool, str)):
                struct[attr] = value
        filepath = os.path.expanduser(self.settingsPath)
        saveJson({"bvh-settings" : struct}, filepath)
        print('Save settings file "%s"' % filepath)


    def readJsonFiles(self, scn, cinfo, infos, subdir, name):
        keys = []
        folder = os.path.join(os.path.dirname(__file__), subdir)
        for fname in os.listdir(folder):
            filepath = os.path.join(folder, fname)
            if os.path.splitext(fname)[-1] == ".json":
                info = cinfo(scn, name)
                info.readFile(filepath)
                infos[info.name] = info
                keys.append(info.name)
        keys.sort()
        return keys


    def initTPoses(self, scn):
        from .t_pose import CTPoseInfo
        self.tposeInfos = { "Default" : CTPoseInfo(scn, "Default") }
        keys = self.readJsonFiles(scn, CTPoseInfo, self.tposeInfos, "t_poses", "")
        enums = [(key,key,key) for key in ["Default"] + keys]

        bpy.types.Scene.McpSourceTPose = EnumProperty(
            items = enums,
            name = "TPose Source",
            default = 'Default')
        scn.McpSourceTPose = 'Default'

        bpy.types.Scene.McpTargetTPose = EnumProperty(
            items = enums,
            name = "TPose Target",
            default = 'Default')
        scn.McpTargetTPose = 'Default'
        print("T-poses initialized")


    def initSources(self, scn):
        from .source import CSourceInfo
        self.initTPoses(scn)
        self.sourceInfos = { "Automatic" : CSourceInfo(scn, "Automatic") }
        keys = self.readJsonFiles(scn, CSourceInfo, self.sourceInfos, "known_rigs", "")
        enums = [(key,key,key) for key in ["Automatic"] + keys]

        bpy.types.Scene.McpSourceRig = EnumProperty(
            items = enums,
            name = "Source rig",
            default = 'Automatic')
        scn.McpSourceRig = 'Automatic'
        print("Defined McpSourceRig")


    def initTargets(self, scn):
        from .target import CTargetInfo
        self.initTPoses(scn)
        self.targetInfos = { "Automatic" : CTargetInfo(scn, "Automatic") }
        keys = self.readJsonFiles(scn, CTargetInfo, self.targetInfos, "known_rigs", "Manual")
        enums = [(key,key,key) for key in ["Automatic"] + keys]

        bpy.types.Scene.McpTargetRig = EnumProperty(
            items = enums,
            name = "Target rig",
            default = 'Automatic')
        print("Defined McpTargetRig")


    def ensureSourceInited(self, scn):
        if not self.sourceInfos:
            self.initSources(scn)


    def ensureTargetInited(self, scn):
        if not self.targetInfos:
            self.initTargets(scn)


    def ensureInited(self, scn):
        self.ensureSourceInited(scn)
        self.ensureTargetInited(scn)


    def ensureFacsInited(self):
        if self.facsTables:
            return
        from .io_json import loadJson
        folder = os.path.join(os.path.dirname(__file__), "facs")
        for fname in os.listdir(folder):
            filepath = os.path.join(folder, fname)
            if os.path.splitext(fname)[-1] == ".json":
                struct = loadJson(filepath)
                self.facsTables[struct["fingerprint"]] = struct
                print("FACS %s %s" % (struct["name"], struct["fingerprint"]))


    BoneNames = [
        (None,           None),
        ("hips",         "Root bone"),
        ("spine",        "Lower spine"),
        ("spine-1",      "Lower spine 2"),
        ("chest",        "Upper spine"),
        ("chest-1",      "Upper spine 2"),
        ("neck",         "Neck"),
        ("head",         "Head"),
        ("",             ""),
        ("shoulder.L",   "L shoulder"),
        ("upper_arm.L",  "L upper arm"),
        ("forearm.L",    "L forearm"),
        ("hand.L",       "L hand"),
        ("",             ""),
        ("shoulder.R",   "R shoulder"),
        ("upper_arm.R",  "R upper arm"),
        ("forearm.R",    "R forearm"),
        ("hand.R",       "R hand"),

        (None,           None),
        ("hip.L",        "L hip"),
        ("thigh.L",      "L thigh"),
        ("shin.L",       "L shin"),
        ("foot.L",       "L foot"),
        ("toe.L",        "L toes"),
        ("",             ""),
        ("hip.R",        "R hip"),
        ("thigh.R",      "R thigh"),
        ("shin.R",       "R shin"),
        ("foot.R",       "R foot"),
        ("toe.R",        "R toes"),

        (None,           None),
        ("f_thumb.01.L",   "L thumb 1"),
        ("f_thumb.02.L",   "L thumb 2"),
        ("f_thumb.03.L",   "L thumb 3"),
        ("f_index.01.L",   "L index 1"),
        ("f_index.02.L",   "L index 2"),
        ("f_index.03.L",   "L index 3"),
        ("f_middle.01.L",   "L middle 1"),
        ("f_middle.02.L",   "L middle 2"),
        ("f_middle.03.L",   "L middle 3"),
        ("f_ring.01.L",   "L ring 1"),
        ("f_ring.02.L",   "L ring 2"),
        ("f_ring.03.L",   "L ring 3"),
        ("f_pinky.01.L",   "L pinky 1"),
        ("f_pinky.02.L",   "L pinky 2"),
        ("f_pinky.03.L",   "L pinky 3"),

        (None,           None),
        ("f_thumb.01.R",   "R thumb 1"),
        ("f_thumb.02.R",   "R thumb 2"),
        ("f_thumb.03.R",   "R thumb 3"),
        ("f_index.01.R",   "R index 1"),
        ("f_index.02.R",   "R index 2"),
        ("f_index.03.R",   "R index 3"),
        ("f_middle.01.R",   "R middle 1"),
        ("f_middle.02.R",   "R middle 2"),
        ("f_middle.03.R",   "R middle 3"),
        ("f_ring.01.R",   "R ring 1"),
        ("f_ring.02.R",   "R ring 2"),
        ("f_ring.03.R",   "R ring 3"),
        ("f_pinky.01.R",   "R pinky 1"),
        ("f_pinky.02.R",   "R pinky 2"),
        ("f_pinky.03.R",   "R pinky 3"),
    ]

    def getMcpBones(self, rig):
        return dict([(pb.McpBone,pb) for pb in rig.pose.bones if pb.McpBone])


    def sortBones(self, rig):
        mcpbones = self.getMcpBones(rig)
        bnames = [bname for bname,longname in self.BoneNames if bname]
        bones = [mcpbones[bname] for bname in bnames if bname in mcpbones.keys()]
        return bones


BS = BvhSettings()
