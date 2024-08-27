# OdinGenerated
# ODIN_SCRIPT_NATURE = GRAPH

#BVH export script
#Exports 3D data in bvh format for animation 
#Authors: Omar Galarraga and Pacome Giraudeau
#Created: 2024-08-21
#Modifications:
# 

import shutil
import numpy as np
import os
from OdinPy import *
from copy import deepcopy

inputs = {
# Marker position
"PelvisOriginFiltered" : {"channelname":"PelvisOriginBD", "locatortype":"ts", "groupname":"RefPointFilter", "calclevel":"Calc"}, 
"PelvisOrigin" : {"channelname":"PelvisOriginBD", "locatortype":"ts", "groupname":"RefPoint", "calclevel":"Calc"}, 


# Angles
"EA" : {"locatortype":"tsgroup", "groupname":"JointAngle", "calclevel":"Calc"},
"EAFilter" : {"locatortype":"tsgroup", "groupname":"JointAngleFilter", "calclevel":"Calc"},


# Trial Information
"Name": {"path":"UserInput/Subject/ID","locatortype":"xmove"},
"RepLabel": {"path":"UserInput/RepLabel","locatortype":"xmove"},
"Date" : { "path":"Acq/Date","locatortype":"xmove"},
"Condition": {"path":"UserInput/Trial/Condition","locatortype":"xmove"},
"Comment": {"path":"UserInput/Trial/Comment","locatortype":"xmove"},
#"Fields": {"path":"UserInput","locatortype":"xmove"},
}

outputs = {
"Value" : { "locatortype": "xmove", "path": "UserInput/Trial/BVH_Export"},
}


def compute(PelvisOrigin, PelvisOriginFiltered, EA, EAFilter, Date, Name, RepLabel, Condition, Comment):
   
    if EAFilter is not None:
        EA = deepcopy(EAFilter)
    if PelvisOriginFiltered is not None:
        PelvisOrigin = deepcopy(PelvisOriginFiltered)
        
    
    frame_time = 1/EA[0]['Rate']
    occlusionsEA = zeros_like(EA[0]['Data']['occluded'])
    for angle in EA:
        occlusionsEA += angle['Data']['occluded']
    
    champ = where(occlusionsEA == 0)
    first_frame = champ[0][0]
    last_frame = champ[0][-1]
    
    frames = last_frame - first_frame + 1 
    
    sens = PelvisOrigin["Data"]["value"][last_frame,0] - PelvisOrigin["Data"]["value"][first_frame,0]
    if sens > 0:
        PelvisOrigin["Data"]["value"][:,0] = -PelvisOrigin["Data"]["value"][:,0]
    
    labels = [ang['Channel'] for ang in EA]
    labels = np.array(labels)
    ipelvis = np.where(labels == "L.Pelvis")[0][0]
    ilhip = np.where(labels == "L.Hip")[0][0]
    ilknee = np.where(labels == "L.Knee")[0][0]        
    ilankle = np.where(labels == "L.Ankle")[0][0]
    irhip = np.where(labels == "R.Hip")[0][0]
    irknee = np.where(labels == "R.Knee")[0][0]    
    irankle = np.where(labels == "R.Ankle")[0][0]        

    ispine = np.where(labels == "L.Spine")[0][0]
    ineck = np.where(labels == "Neck")[0][0]
    
    ilshoulder = np.where(labels == "L.Shoulder")[0][0]
    ilelbow = np.where(labels == "L.Elbow")[0][0]
    ilwrist = np.where(labels == "L.Wrist")[0][0]

    irshoulder = np.where(labels == "R.Shoulder")[0][0]
    irelbow = np.where(labels == "R.Elbow")[0][0]
    irwrist = np.where(labels == "R.Wrist")[0][0]       

    file_bvh =  'C:\\Codamotion\\Odin_x64\\Data\\' + Name +'_' + str(Date['Year']) +'_' + str(Date['Month']) +'_' + str(Date['Day']) + '_' + Comment + '_' + Condition +'_' + str(RepLabel) + '.bvh'    
   
    shutil.copyfile(r"C:\Codamotion\Odin_x64\BVHTemplates\bvh_base5.bvh", file_bvh)
    with open(file_bvh, 'a') as f:

        f.write("\n\n\nFrames: " + str(frames))
        f.write("\nFrame Time: " + str(frame_time) + "\n")     
        
        for i in range(first_frame, last_frame+1):
            
            angles_pelvis = EA[ipelvis]['Data']['value'][i, :]
            
            angles_Hip_left = EA[ilhip]['Data']['value'][i, :]
            angles_Knee_left =  EA[ilknee]['Data']['value'][i, :]
            angles_Ankle_left =  EA[ilankle]['Data']['value'][i, :]
            
            angles_Hip_right =  EA[irhip]['Data']['value'][i, :]
            angles_Knee_right = EA[irknee]['Data']['value'][i, :]
            angles_Ankle_right = EA[irankle]['Data']['value'][i, :]
            
            LowerBack = "0 -10 0 "
            
            if 'L.Spine' not in labels: 
                Spine = (0, 0, 0)
            else:
                Spine = EA[ispine]['Data']['value'][i, :]
            
            Spine1 = "0 0 0 "
            
            if 'Neck' not in labels:
                Neck = (0, 0, 0)
            else:
                Neck = EA[ineck]['Data']['value'][i, :]
                
            Neck1 = "0 0 0 "
            Head = "0 0 0 "
            
            LClavicle = "0 0 0 "
         
            if 'L.Shoulder' not in labels:
                LShoulder = (0, 0, 0)
                LElbow = (0, 0, 0)   
                LWrist = (0, 0, 0)                
            else:
                LShoulder = EA[ilshoulder]['Data']['value'][i, :]
                LElbow = EA[ilelbow]['Data']['value'][i, :]            
                LWrist = EA[ilwrist]['Data']['value'][i, :]            

            RClavicle ="0 0 0 "

            if 'R.Shoulder' not in labels:
                RShoulder = (0, 0, 0)
                RElbow = (0, 0, 0)   
                RWrist = (0, 0, 0)                
            else:
                RShoulder = EA[irshoulder]['Data']['value'][i, :]
                RElbow = EA[irelbow]['Data']['value'][i, :]            
                RWrist = EA[irwrist]['Data']['value'][i, :]
            
            
            pelvis_str ="{} {} {} ".format(angles_pelvis[0], angles_pelvis[1], -angles_pelvis[2]) 
            Lhip_str = "{} {} {} ".format(-angles_Hip_left[0], -angles_Hip_left[1], -angles_Hip_left[2]) 
            Lknee_str = "{} {} {} ".format(-angles_Knee_left[0], angles_Knee_left[1], -angles_Knee_left[2])
            Lankle_str = "{} {} {} ".format(0, -angles_Ankle_left[1], -angles_Ankle_left[2])
            Rhip_str = "{} {} {} ".format(angles_Hip_right[0], -angles_Hip_right[1], angles_Hip_right[2])
            Rknee_str = "{} {} {} ".format(angles_Knee_right[0], angles_Knee_right[1], angles_Knee_right[2])
            Rankle_str = "{} {} {} ".format(0, -angles_Ankle_right[1], angles_Ankle_right[2])
            
            Spine_str = "{} {} {} ".format(-Spine[0], Spine[1], -Spine[2])
            Neck_str = "{} {} {} ".format(Neck[0], -Neck[1], Neck[2])
            LElbow_str = "{} {} {} ".format(LElbow[0], LElbow[2], -LElbow[1])
            RElbow_str = "{} {} {} ".format(-RElbow[0], RElbow[2], RElbow[1])
            LWrist_str = "{} {} {} ".format(-LWrist[1], LWrist[2], -LWrist[0])
            RWrist_str = "{} {} {} ".format(RWrist[1], RWrist[2], RWrist[0])
            RShoulder_str = "{} {} {} ".format(RShoulder[0]+90, RShoulder[1], RShoulder[2])
            LShoulder_str = "{} {} {} ".format(LShoulder[0]-90, LShoulder[1], LShoulder[2])
            
            positions_pelvis = "{} {} {} ".format(
                0,
                0,
                PelvisOrigin["Data"]["value"][i,0]/55,
                # data_points["V.PelvisOriginBD"][i][1]/100, 
                # data_points["V.PelvisOriginBD"][i][2]/100, 
                # -data_points["V.PelvisOriginBD"][i][0]/60
                )
            
            Lpelvis = "0 0 0 "
            Rplevis = "0 0 0 "
            ecrire = positions_pelvis + pelvis_str + Lpelvis + Lhip_str + Lknee_str + Lankle_str + Rplevis + Rhip_str + Rknee_str + Rankle_str + LowerBack + Spine1 + Spine_str + Neck_str + Neck1 + Head + LClavicle + LShoulder_str + LElbow_str + LWrist_str + RClavicle + RShoulder_str + RElbow_str + RWrist_str + "\n"
            f.write(ecrire.replace("nan","0"))
            
    return{"Value": None}