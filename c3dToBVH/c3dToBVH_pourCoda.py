
import shutil
import numpy as np


def find_first_frame(data_points):
    """
    Retourne la première frame non obstruée (i.e. aucun des points de la frame n'est obstrué )
    """
    points = data_points.keys()
    f = 0
    obstruction = True
    while obstruction: 
        obstruction = False
        for point in points:
            # si un des point est obstrué, on considère la frame obstruée 
            if np.isnan(#  (frame f)  de [point][f][0]):)
                obstruction = True
                break
        f+=1
    return f

def find_last_frame(data_points, frames):
    """
    Retourne la première frame non obstruée (i.e. aucun des points de la frame n'est obstrué) en aprtant de la fin
    """
    points = data_points.keys()
    f = frames
    obstruction = True
    while obstruction: 
        f-=1
        obstruction = False
        for point in points:
            # si un des point est obstrué, on considère la frame obstruée 
            
            if np.isnan(# angles (frame i)  de point][f][0]):
                obstruction = True
                break
    return f

def c3d_to_bvh_pour_coda(file):
    
    data_points, frames, frame_time, first_frame, last_frame, base_global = init(file)
    
    file_bvh = file[:-4] + ".bvh"

    shutil.copyfile("bvh_base5.bvh", file_bvh)
    f = open(file_bvh, 'a')

    f.write("\n\n\nFrames: " + str(frames))
    f.write("\nFrame Time: " + str(frame_time) + "\n") 
    
    list_of_points = # liste des points captés
    
    for i in range(first_frame, last_frame):
            
            
        angles_pelvis = # angles (frame i)  de "L.Pelvis"][i]
        
        angles_Hip_left = # angles (frame i)  de "L.Hip"][i]
        angles_Hip_left = # angles (frame i)  de "L.Knee"][i]
        angles_Ankle_left = # angles (frame i)  de "L.Ankle"][i]
        
        angles_Hip_right = # angles (frame i)  de "R.Hip"][i]
        angles_Knee_right = # angles (frame i)  de "R.Knee"][i]
        angles_Ankle_right = # angles (frame i)  de "R.Ankle"][i]
        
        
        LowerBack = "0 -10 0 "
        
        if 'L.Spine' not in list_of_points: 
            Spine = 0, 0, 0
        else:
            Spine = # angles (frame i)  de "L.Spine"][i] 
            
        
        Spine1 = "0 0 0 "
        
        if 'Neck' not in list_of_points:
            Neck = 0, 0, 0
        else:
            Neck = # angles (frame i)  de "Neck"][i]
            
        Neck1 = "0 0 0 "
        Head = "0 0 0 "
        
        
        LClavicle = "0 0 0 "
        
        if 'L.Elbow' not in list_of_points:
            LElbow = 0, 0, 0
        else:
            LElbow = # angles (frame i)  de "L.Elbow"][i]
            
        if 'L.Wrist' not in list_of_points:
            LWrist = 0, 0, 0
        else:
            LWrist = # angles (frame i)  de "L.Wrist"][i] 
        
        if 'L.Shoulder' not in list_of_points:
            LShoulder = 0, 0, 0
        else:
            LShoulder =  # angles (frame i)  de "L.Shoulder"][i]
        
        
        RClavicle ="0 0 0 "

        if 'R.Wrist' not in list_of_points:
            RWrist =0, 0, 0
        else:
            RWrist = # angles (frame i)  de "R.Wrist"][i] 
            
        if 'R.Shoulder' not in list_of_points:
            RShoulder = 0, 0, 0
        else:
            RShoulder =  # angles (frame i)  de "R.Shoulder"][i]
            
        if 'R.Elbow' not in list_of_points:
            RElbow = 0, 0, 0
        else:
            RElbow = # angles (frame i)  de "R.Elbow"][i]
        
        
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
        
        # positions_pelvis = "{} {} {} ".format(# angles (frame i)  de "V.PelvisOriginBD"][i][1]/1000, # angles (frame i)  de "V.PelvisOriginBD"][i][2]/1000, -# angles (frame i)  de "V.PelvisOriginBD"][i][0]/600)
        
        positions_pelvis = "1000 1000 1000 "
        Lpelvis = "0 0 0 "
        Rplevis = "0 0 0 "
        leftover = 14*"0 0 0 "
        ecrire = positions_pelvis + pelvis_str + Lpelvis + Lhip_str + Lknee_str + Lankle_str + Rplevis + Rhip_str + Rknee_str + Rankle_str + LowerBack + Spine1 + Spine_str + Neck_str + Neck1 + Head + LClavicle + LShoulder_str + LElbow_str + LWrist_str + RClavicle + RShoulder_str + RElbow_str + RWrist_str + "\n"
        f.write(ecrire.replace("nan","0"))
        
