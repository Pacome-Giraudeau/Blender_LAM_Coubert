
import shutil
import numpy as np


def find_first_frame(EA):
    """
    Retourne la première frame non obstruée (i.e. aucun des points de la frame n'est obstrué )
    """
    f = 0
    obstruction = True
    while obstruction: 
        obstruction = False
        for point in EA:
            # si un des point est obstrué, on considère la frame obstruée 
            if np.isnan(EA[point]['Data']['occluded'][f]):
                obstruction = True
                break
        f+=1
    return f

def find_last_frame(EA, frames):
    """
    Retourne la première frame non obstruée (i.e. aucun des points de la frame n'est obstrué) en aprtant de la fin
    """
    f = frames
    obstruction = True
    while obstruction: 
        f-=1
        obstruction = False
        for point in EA:
            # si un des point est obstrué, on considère la frame obstruée 
            
            if np.isnan(EA[point]['Data']['occluded'][f]):
                obstruction = True
                break
    return f

def compute():
    
    frames = len(m["V.PelvisOriginBD"]["data"].shape[0])
    frame_time = 0.01
    first_frame = first_frame(EA)
    last_frame = last_frame(EA, frames)
    
    if EAFilter is not None:
        EA = deepcopy(EAFilter)
        
    Marker = deepcopy(MFilter)
        
    file_bvh = file[:-4] + ".bvh"

    shutil.copyfile("bvh_base5.bvh", file_bvh)
    with open(file_bvh, 'a') as f:

        f.write("\n\n\nFrames: " + str(frames))
        f.write("\nFrame Time: " + str(frame_time) + "\n") 
        
        list_of_points = EA.keys()
        
        for i in range(first_frame, last_frame):
                
                
            angles_pelvis = EA["L.Pelvis"]['Data']['value'][i, :]
            
            angles_Hip_left = EA["L.Hip"]['Data']['value'][i, :]
            angles_Knee_left =  EA["L.Knee"]['Data']['value'][i, :]
            angles_Ankle_left =  EA["L.Ankle"]['Data']['value'][i, :]
            
            angles_Hip_right =  EA["R.Hip"]['Data']['value'][i, :]
            angles_Knee_right = EA["R.Knee"]['Data']['value'][i, :]
            angles_Ankle_right = EA["R.Ankle"]['Data']['value'][i, :]
            
            
            LowerBack = "0 -10 0 "
            
            if 'L.Spine' not in list_of_points: 
                Spine = 0, 0, 0
            else:
                Spine = EA["L.Spine"]['Data']['value'][i, :]
                
            
            Spine1 = "0 0 0 "
            
            if 'Neck' not in list_of_points:
                Neck = 0, 0, 0
            else:
                Neck = EA["Neck"]['Data']['value'][i, :]
                
            Neck1 = "0 0 0 "
            Head = "0 0 0 "
            
            
            LClavicle = "0 0 0 "
            
            if 'L.Elbow' not in list_of_points:
                LElbow = 0, 0, 0
            else:
                LElbow = EA["L.Elbow"]['Data']['value'][i, :]
                
            if 'L.Wrist' not in list_of_points:
                LWrist = 0, 0, 0
            else:
                LWrist = EA["L.Wrist"]['Data']['value'][i, :]
            
            if 'L.Shoulder' not in list_of_points:
                LShoulder = 0, 0, 0
            else:
                LShoulder = EA["L.Shoulder"]['Data']['value'][i, :]
            
            
            RClavicle ="0 0 0 "

            if 'R.Wrist' not in list_of_points:
                RWrist =0, 0, 0
            else:
                RWrist =EA["R.Wrsit"]['Data']['value'][i, :]
                
            if 'R.Shoulder' not in list_of_points:
                RShoulder = 0, 0, 0
            else:
                RShoulder = EA["R.Shoulder"]['Data']['value'][i, :]
            if 'R.Elbow' not in list_of_points:
                RElbow = 0, 0, 0
            else:
                RElbow = EA["R.Elbow"]['Data']['value'][i, :]
            
            
            pelvis_str ="{} {} {} ".format(angles_pelvis[0], angles_pelvis[1], -angles_pelvis[2]) 
            Lhip_str = "{} {} {} ".format(-angles_Hip_left[0], -angles_Hip_left[1], -angles_Hip_left[2]) 
            Lknee_str = "{} {} {} ".format(-angles_Knee_left[0], angles_Knee_left[1], -angles_Knee_left[2])
            Lankle_str = "{} {} {} ".format(-angles_Ankle_left[0], -angles_Ankle_left[1], -angles_Ankle_left[2])
            Rhip_str = "{} {} {} ".format(angles_Hip_right[0], -angles_Hip_right[1], angles_Hip_right[2])
            Rknee_str = "{} {} {} ".format(angles_Knee_right[0], angles_Knee_right[1], angles_Knee_right[2])
            Rankle_str = "{} {} {} ".format(angles_Ankle_right[0], -angles_Ankle_right[1], angles_Ankle_right[2])
            
            
            
            
            Spine_str = "{} {} {} ".format(-Spine[0], Spine[1], -Spine[2])
            Neck_str = "{} {} {} ".format(Neck[0], -Neck[1], Neck[2])
            LElbow_str = "{} {} {} ".format(LElbow[0], LElbow[2], -LElbow[1])
            RElbow_str = "{} {} {} ".format(-RElbow[0], RElbow[2], RElbow[1])
            LWrist_str = "{} {} {} ".format(-LWrist[1], LWrist[2], -LWrist[0])
            RWrist_str = "{} {} {} ".format(RWrist[1], RWrist[2], RWrist[0])
            RShoulder_str = "{} {} {} ".format(RShoulder[0]+90, RShoulder[1], RShoulder[2])
            LShoulder_str = "{} {} {} ".format(LShoulder[0]-90, LShoulder[1], LShoulder[2])
            
            
            positions_pelvis = "{} {} {} ".format(
                Marker["V.PelvisOriginBD"]["data"][i][1]/55,
                Marker["V.PelvisOriginBD"]["data"][i][2]/55,
                Marker["V.PelvisOriginBD"]["data"][i][0]/55,
                # data_points["V.PelvisOriginBD"][i][1]/100, 
                # data_points["V.PelvisOriginBD"][i][2]/100, 
                # -data_points["V.PelvisOriginBD"][i][0]/60
                )
            
            Lpelvis = "0 0 0 "
            Rplevis = "0 0 0 "
            ecrire = positions_pelvis + pelvis_str + Lpelvis + Lhip_str + Lknee_str + Lankle_str + Rplevis + Rhip_str + Rknee_str + Rankle_str + LowerBack + Spine1 + Spine_str + Neck_str + Neck1 + Head + LClavicle + LShoulder_str + LElbow_str + LWrist_str + RClavicle + RShoulder_str + RElbow_str + RWrist_str + "\n"
            f.write(ecrire.replace("nan","0"))
            
