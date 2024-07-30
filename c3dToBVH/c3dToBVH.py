import c3d
import numpy as np
import math
import vector
from scipy.spatial.transform import Rotation as R
import shutil
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d



def get_data(file):
    data_points = dict()
    with open(file, 'rb') as handle:
        reader = c3d.Reader(handle)
        
        for i, label in enumerate(reader.point_labels):
            print(label)
            data_points[label.replace(" ","")] = []
                
        for i, (n, points, analog) in enumerate(reader.read_frames()):
            for j, label in enumerate(reader.point_labels):
                data_points[label.replace(" ","")].append(points[j])
        
            
        for i, label in enumerate(reader.point_labels):
            data_points[label.replace(" ","")] = np.array(data_points[label.replace(" ","")])
            
    return data_points
    
    
    
    
    
    
    
    
    
    
def init_joint_angles():      
    joints = ["Pelvis", "LPelvis", "LHip", "LKnee", "LAnkle", "RPelvis", "RHip", "RKnee", "RAnkle"]
    #, "LowerBack", "Spine", "Spine1", "Neck", "Neck1", "Head", "LClavicle", "LShoulder", "LElbow", "LWrist"
    joints_angles = dict()
    for j in joints:
        joints_angles[joints[j]] = []
        return joints_angles
    
def norme(u):
    x = math.sqrt(u[0]**2 + u[1]**2 + u[2]**2)
    return x

def normalized_vector(u1, u2):
    uprime = (u2[0] - u1[0], u2[1]- u1[1], u2[2] - u1[2])
    return np.array([uprime[0]/norme(uprime), uprime[1]/norme(uprime), uprime[2]/norme(uprime)])


def calculate_base_system_right(u1_1, u1_2, u2_interieur, u2_exterieur):
    if u1_1[3] == -1 or u1_2[3] == -1 or u2_interieur[3] == -1 or u2_exterieur[3] == -1:
        return np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0])
    
    u1 = normalized_vector(u1_1, u1_2)
    u2prime = normalized_vector(u2_interieur, u2_exterieur)
    u3 = np.cross(u1, u2prime)
    u2 = np.cross(u3, u1)
    return u1, u2, u3

print(calculate_base_system_right([0,0,0,0], [1,0,0,0], [0,-1,0,0], [0,1,0,0]))

def calculate_base_system_left(u1_1, u1_2, u2_interieur, u2_exterieur):
    if u1_1[3] == -1 or u1_2[3] == -1 or u2_interieur[3] == -1 or u2_exterieur[3] == -1:
        return np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0])
    
    u1 = normalized_vector(u1_1, u1_2)
    u2prime = normalized_vector(u2_interieur, u2_exterieur)
    u3 = np.cross(u2prime, u1)
    u2 = np.cross(u1, u3)
    return u1, u2, u3




data_points = get_data("Corridor_050_avec_angles.c3d")

def calculate_base_bassin_left(f):
    return calculate_base_system_left(data_points['V.MidASIS'][f], data_points['V.Sacrum'][f], data_points['V.R.ASIS'][f], data_points['V.L.ASIS'][f])


def calculate_base_bassin_right(f):
    return calculate_base_system_right(data_points['V.MidASIS'][f], data_points['V.Sacrum'][f], data_points['V.R.ASIS'][f], data_points['V.L.ASIS'][f])


def calculate_base_cuisse_left(f):
    return calculate_base_system_left(data_points['V.L.Knee'][f], data_points['V.L.ASIS'][f], data_points['V.L.MedialFemoralEpicondyle'][f], data_points['V.L.LateralFemoralEpicondyle'][f])

def calculate_base_cuisse_right(f):
    return calculate_base_system_right(data_points['V.R.Knee'][f], data_points['V.R.ASIS'][f], data_points['V.R.MedialFemoralEpicondyle'][f], data_points['V.R.LateralFemoralEpicondyle'][f])



def calculate_base_jambe_left(f):
    return calculate_base_system_left(data_points['V.L.Ankle'][f], data_points['V.L.Knee'][f], data_points['V.L.MedialMalleolus'][f], data_points['V.L.LateralMalleolus'][f])

def calculate_base_jambe_right(f):
    return calculate_base_system_right(data_points['V.R.Ankle'][f], data_points['V.R.Knee'][f], data_points['V.R.MedialMalleolus'][f], data_points['V.R.LateralMalleolus'][f])



def calculate_base_pied_left(f):
    return calculate_base_system_left(data_points['V.L.Calcaneus'][f], data_points['V.L.MidMT'][f], data_points['V.L.FMT'][f], data_points['L.5MT'][f])

def calculate_base_pied_right(f):
    return calculate_base_system_right(data_points['V.R.Calcaneus'][f], data_points['V.R.MidMT'][f], data_points['V.L.FMT'][f], data_points['L.5MT'][f])





def matrice_de_passage(B1, B2):
    """
    Calcule la matrice de passage entre deux repères.
    
    :param B1: Un tuple de trois vecteurs numpy représentant les bases du premier repère.
    :param B2: Un tuple de trois vecteurs numpy représentant les bases du deuxième repère.
    :return: La matrice de passage de B1 à B2.
    """
    
    print("\n\n Bases : \n")
    print(B1)
    print(B2)
    
    # Créer les matrices B1 et B2
    M1 = np.column_stack(B1)
    M2 = np.column_stack(B2)
    print("\n\n Matrices : \n")
    print(M1)
    print(M2)
    
    # Calculer la matrice de passage P
    P = np.dot(M2, np.linalg.inv(M1))
    print("\n\n Matrice de passage : \n")
    print(P)
    
    return P


def angles_euler_from_matrice_passage(P, sequence='zyx'):
    """
    Calcule les angles d'Euler à partir d'une matrice de passage.
    
    :param P: La matrice de passage (ou de rotation).
    :param sequence: La séquence des axes d'Euler (par défaut 'zyx').
    :return: Les angles d'Euler (en radians).
    """
    # Créer un objet Rotation à partir de la matrice de passage
    rotation = R.from_matrix(P)
    
    # Extraire les angles d'Euler selon la séquence spécifiée
    angles_euler = rotation.as_euler(sequence, degrees=True)
    f
    return angles_euler

def get_angles_euler(B1, B2, sequence='yxz'):
    P = matrice_de_passage(B1, B2)
    angles = angles_euler_from_matrice_passage(P, sequence)
    angles_arrondis = np.array([round(angles[0], 4), round(angles[1], 4), round(angles[2], 4)])
    return angles_arrondis






# points = [
#             'V.MidASIS', 'V.Sacrum', 'V.R.ASIS', 'V.L.ASIS', 'V.L.Knee','V.L.MedialFemoralEpicondyle', 'V.L.LateralFemoralEpicondyle', 
#             'V.R.Knee', 'V.R.MedialFemoralEpicondyle', 'V.R.LateralFemoralEpicondyle','V.L.Ankle', 'V.L.MedialMalleolus', 
#             'V.L.LateralMalleolus', 'V.R.Ankle', 'V.R.MedialMalleolus', 'V.R.LateralMalleolus', 'V.L.Calcaneus', 'V.R.Calcaneus',
#             'V.L.MidMT', 'V.R.MidMT', 'V.L.FMT', 'V.L.FMT', 'L.5MT', 'L.5MT'
#         ]


points = data_points.keys()


def find_first_frame():
    f = 0
    est_first_frame = True
    while est_first_frame: 
        est_first_frame = False
        for point in points:
            if data_points[point][f][3] == -1:
                est_first_frame = True
        f+=1
    return f

def find_last_frame():
    f = frames
    est_last_frame = True
    while est_last_frame: 
        f-=1
        est_last_frame = False
        for point in points:
            if data_points[point][f][3] == -1:
                est_last_frame = True
    return f
    

with open('.\\Corridor_050_avec_angles.c3d', 'rb') as handle:
    reader = c3d.Reader(handle)
    frames = reader.frame_count
    frame_time = 0.01
    first_frame= find_first_frame()
    last_frame= find_last_frame()
    
    
def existing_or_not__indices(point, first_frame, last_frame):
    missing_indices = []
    existing_indices = []
    existing_points = []
    for f in range(first_frame, last_frame):
        if data_points[point][f][3] == -1:
            missing_indices.append(f)
        else:
            existing_indices.append(f)
            existing_points.append(data_points[point][f][0:3])
    return np.array(missing_indices), np.array(existing_indices), np.array(existing_points)

    


############  INTERPOLATION
    
for point in points:

    # Extraire les points existants
    missing_indices, existing_indices, existing_points = existing_or_not__indices(point, first_frame, last_frame)
    # Interpoler séparément pour x, y et z
    interp_func = interp1d(existing_indices, existing_points, axis=0, kind='linear', fill_value="extrapolate")
    for f in range(len(missing_indices)):
        
        p = interp_func(missing_indices)
        data_points[point][missing_indices[f]][0] = p[f][0]
        data_points[point][missing_indices[f]][1] = p[f][1]
        data_points[point][missing_indices[f]][3] = p[f][2]
    

############### C3D -> BVH 

#######  c3d sans angles -> bvh 

shutil.copyfile("bvh_base5.bvh", "Corridor_050_sans_angles.bvh")
f = open("Corridor_050_sans_angles.bvh", 'a')

f.write("\n\n\nFrames: " + str(frames))
f.write("\nFrame Time: " + str(frame_time) + "\n") 

ordre = [1,0,2]
for i in range(first_frame, last_frame):
    base_global = np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])
    base_bassin_left = calculate_base_bassin_left(i)
    base_bassin_right = calculate_base_bassin_right(i)
    base_cuisse_left = calculate_base_cuisse_left(i)
    base_cuisse_right = calculate_base_cuisse_right(i)
    base_jambe_left = calculate_base_jambe_left(i)
    base_jambe_right = calculate_base_jambe_right(i)
    base_pied_left = calculate_base_pied_left(i)
    base_pied_right = calculate_base_pied_right(i)
    
    angles_bassin = get_angles_euler(base_global, base_bassin_right)
    pelvis_str ="{} {} {} ".format(angles_bassin[ordre[0]], angles_bassin[ordre[1]], angles_bassin[ordre[2]]) 
    
    angles_cuisse_left = get_angles_euler(base_bassin_left, base_cuisse_left)
    Lhip_str = "{} {} {} ".format(angles_cuisse_left[ordre[0]], angles_cuisse_left[ordre[1]], angles_cuisse_left[ordre[2]]) 
    angles_jambe_left = get_angles_euler(base_cuisse_left, base_jambe_left)
    Lknee_str = "{} {} {} ".format(angles_jambe_left[ordre[0]], angles_jambe_left[ordre[1]], angles_jambe_left[ordre[2]])
    angles_pied_left = get_angles_euler(base_jambe_left, base_pied_left)
    Lankle_str = "{} {} {} ".format(angles_pied_left[ordre[0]], angles_pied_left[ordre[1]], angles_pied_left[ordre[2]])
    
    angles_cuisse_right = get_angles_euler(base_bassin_right, base_cuisse_right)
    Rhip_str = "{} {} {} ".format(angles_cuisse_right[ordre[0]], angles_cuisse_right[ordre[1]], angles_cuisse_right[ordre[2]])
    angles_jambe_right = get_angles_euler(base_cuisse_right, base_jambe_right)
    Rknee_str = "{} {} {} ".format(angles_jambe_right[ordre[0]], angles_jambe_right[ordre[1]], angles_jambe_right[ordre[2]])
    angles_pied_right = get_angles_euler(base_jambe_right, base_pied_right)
    Rankle_str = "{} {} {} ".format(angles_pied_right[ordre[0]], angles_pied_right[ordre[1]], angles_pied_right[ordre[2]])
    
    positions_bassin = "0 0 0 "
    Lpelvis = "0 0 0 "
    Rplevis = "0 0 0 "
    leftover = 14*"0 0 0 "
    
    f.write(positions_bassin + pelvis_str + Lpelvis + Lhip_str + Lknee_str + Lankle_str + Rplevis + Rhip_str + Rknee_str + Rankle_str + leftover + "\n")
    
    
    
    
#############  c3d avec angles -> bvh
    
    
# shutil.copyfile("bvh_base5.bvh", "Corridor_050_avec_angles.bvh")
# f = open(".\\Corridor_050_avec_angles.bvh", 'a')

# f.write("\n\n\nFrames: " + str(frames))
# f.write("\nFrame Time: " + str(frame_time) + "\n") 
# ordre = [0,1,2]
# for i in range(first_frame, last_frame):
    
#     
    
#     angles_bassin = data_points["L.Pelvis"][i]
#     pelvis_str ="{} {} {} ".format(angles_bassin[ordre[0]], angles_bassin[ordre[1]], -angles_bassin[ordre[2]]) 
    
#     angles_cuisse_left = data_points["L.Hip"][i]
#     Lhip_str = "{} {} {} ".format(-angles_cuisse_left[ordre[0]], -angles_cuisse_left[ordre[1]], -angles_cuisse_left[ordre[2]]) 
#     angles_jambe_left = data_points["L.Knee"][i]
#     Lknee_str = "{} {} {} ".format(-angles_jambe_left[ordre[0]], angles_jambe_left[ordre[1]], -angles_jambe_left[ordre[2]])
#     angles_pied_left = data_points["L.Ankle"][i]
#     Lankle_str = "{} {} {} ".format(-angles_pied_left[ordre[0]], -angles_pied_left[ordre[1]], -angles_pied_left[ordre[2]])
    
#     angles_cuisse_right = data_points["R.Hip"][i]
#     Rhip_str = "{} {} {} ".format(angles_cuisse_right[ordre[0]], -angles_cuisse_right[ordre[1]], angles_cuisse_right[ordre[2]])
#     angles_jambe_right = data_points["R.Knee"][i]
#     Rknee_str = "{} {} {} ".format(angles_jambe_right[ordre[0]], angles_jambe_right[ordre[1]], angles_jambe_right[ordre[2]])
#     angles_pied_right = data_points["R.Ankle"][i]
#     Rankle_str = "{} {} {} ".format(angles_pied_right[ordre[0]], -angles_pied_right[ordre[1]], angles_pied_right[ordre[2]])
    
#     positions_bassin = "0 0 0 "
#     Lpelvis = "0 0 0 "
#     Rplevis = "0 0 0 "
#     leftover = 14*"0 0 0 "
#     f.write(positions_bassin + pelvis_str + Lpelvis + Lhip_str + Lknee_str + Lankle_str + Rplevis + Rhip_str + Rknee_str + Rankle_str + leftover + "\n")
    
    


# for f in range(first_frame, last_frame):
#     base_bassin = calculate_base_bassin_left(f)
#     base_cuisse_left = calculate_base_cuisse_left(f)
#     angles_cuisse_left = get_angles_euler(base_bassin, base_cuisse_left)
#     Lhip_str = "{} {} {} ".format(angles_cuisse_left[0], angles_cuisse_left[1], angles_cuisse_left[2]) 
#     print(angles_cuisse_left, "  ---  ", data_points["L.Knee"][f])





# with open('Corridor_050_avec_angles.c3d', 'rb') as handle:
#     reader = c3d.Reader(handle)
    
#     for i, (n, points, analog) in enumerate(reader.read_frames()):
#         print("\n ----", i, n)
#         print(i, n, " - POINTS :")
#         ligne = 0
#         for label in enumerate(reader.point_labels):
#             print("     ", label, " : ", points[ligne][0:4])
#             ligne+=1
        
#         print(i, n, " - ANALOG :")
#         ligne = 0
#         for label in enumerate(reader.analog_labels):
#             print("     ", label, " : ", analog[ligne][0:4])
#             ligne+=1


        


def affiche(label, ff=0, lf=450):
    print(label, " : \n")
    for f in range(ff, lf):
        print ("  ", f, " -> ", data_points[label][f][0:5])
        
    points = []
    x = []
    y = []
    z = []
    print(ff, lf)
    for f in range(ff, lf):
        points.append(data_points[label][f][0:3])
        z.append(data_points[label][f][2])
        y.append(data_points[label][f][1])
        x.append(data_points[label][f][0])
    
    ax = plt.figure().add_subplot()
    ax.plot(range(ff, lf), x)
    ay = plt.figure().add_subplot()
    ay.plot(range(ff, lf), y)
    az = plt.figure().add_subplot()
    az.plot(range(ff, lf), z)
    axyz = plt.figure().add_subplot(projection='3d')
    axyz.plot(x, y, z)
    plt.show()
    
# affiche("L.Knee")


# with open('Corridor_050_avec_angles.c3d', 'rb') as handle:
#         reader = c3d.Reader(handle)
        
#         reader.read_frames