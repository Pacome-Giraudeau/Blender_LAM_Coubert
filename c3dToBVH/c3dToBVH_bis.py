import c3d
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import shutil
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import kineticstoolkit.lab as ktk



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
    
    
    
    
    
# def init_joint_angles():      
#     joints = ["LPelvis", "LHip", "LKnee", "LAnkle", "RPelvis", "RHip", "RKnee", "RAnkle"]
#     #, "LowerBack", "Spine", "Spine1", "Neck", "Neck1", "Head", "LClavicle", "LShoulder", "LElbow", "LWrist"
#     joints_angles = dict()
#     for j in joints:
#         joints_angles[joints[j]] = []
#         return joints_angles
    
def norme(u):
    x = math.sqrt(u[0]**2 + u[1]**2 + u[2]**2)
    return x

def normalized_vector(p1, p2):
    uprime = (p2[0] - p1[0], p2[1]- p1[1], p2[2] - p1[2])
    return np.array([uprime[0]/norme(uprime), uprime[1]/norme(uprime), uprime[2]/norme(uprime)])


def calculate_base_system_right(u1_1, u1_2, u2_interieur, u2_exterieur, ordre=[0, 1, 2]):
    if u1_1[3] == -1 or u1_2[3] == -1 or u2_interieur[3] == -1 or u2_exterieur[3] == -1:
        return np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0])
    
    u1 = normalized_vector(u1_1, u1_2)
    u2prime = normalized_vector(u2_interieur, u2_exterieur)
    u3 = np.cross(u1, u2prime)
    u2 = np.cross(u3, u1)
    base = [u1, u2, u3]
    
    return base[ordre[0]], base[ordre[1]], base[ordre[2]] 

  


def calculate_base_system_left(u1_1, u1_2, u2_interieur, u2_exterieur, ordre=[0, 1, 2]):
    if u1_1[3] == -1 or u1_2[3] == -1 or u2_interieur[3] == -1 or u2_exterieur[3] == -1:
        return np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0])
    
    u1 = normalized_vector(u1_1, u1_2)
    u2prime = normalized_vector(u2_interieur, u2_exterieur)
    u3 = np.cross(u2prime, u1)
    u2 = np.cross(u1, u3)
    base = [u1, u2, u3]
    return base[ordre[0]], base[ordre[1]], base[ordre[2]]   





def calculate_base_bassin_left(f):
    return calculate_base_system_left(data_points['V.Sacrum'][f], data_points['V.MidASIS'][f], data_points['V.R.ASIS'][f], data_points['V.L.ASIS'][f])


def calculate_base_bassin_right(f):
    return calculate_base_system_right(data_points['V.Sacrum'][f], data_points['V.MidASIS'][f], data_points['V.R.ASIS'][f], data_points['V.L.ASIS'][f])


def calculate_base_cuisse_left(f):
    return calculate_base_system_left(data_points['V.L.Knee'][f], data_points['V.L.Hip'][f], data_points['V.L.MedialFemoralEpicondyle'][f], data_points['V.L.LateralFemoralEpicondyle'][f], ordre=[2, 1, 0])


def calculate_base_cuisse_right(f):
    return calculate_base_system_right(data_points['V.R.Knee'][f], data_points['V.R.Hip'][f], data_points['V.R.MedialFemoralEpicondyle'][f], data_points['V.R.LateralFemoralEpicondyle'][f], ordre=[2, 1, 0])


def calculate_base_jambe_left(f):
    return calculate_base_system_left(data_points['V.L.Ankle'][f], data_points['V.L.Knee'][f], data_points['V.L.MedialMalleolus'][f], data_points['V.L.LateralMalleolus'][f], ordre=[2, 1, 0])


def calculate_base_jambe_right(f):
    return calculate_base_system_right(data_points['V.R.Ankle'][f], data_points['V.R.Knee'][f], data_points['V.R.MedialMalleolus'][f], data_points['V.R.LateralMalleolus'][f], ordre=[2, 1, 0])


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
    # Créer les matrices B1 et B2
    M1 = np.column_stack(B1)
    M2 = np.column_stack(B2)
    
    # Calculer la matrice de passage P
    P = np.dot(M2, M1.transpose())
    
    return P


def angles_euler_from_matrice_passage(P, sequence='yxz', polarity = (1,1,1)):
    """
    Calcule les angles d'Euler à partir d'une matrice de passage.
    
    :param P: La matrice de passage (ou de rotation).
    :param sequence: La séquence des axes d'Euler (par défaut 'yxz').
    :return: Les angles d'Euler (en degrés).
    """
    ordre = {'x': 0, 'y':1, 'z':2}
    # Créer un objet Rotation à partir de la matrice de passage
    rotation = R.from_matrix(P)
    
    # Extraire les angles d'Euler selon la séquence spécifiée
    angles_euler2 = rotation.as_euler(sequence, degrees=True)
    angles_euler = np.zeros_like(angles_euler2)   
    angles_euler[ordre[sequence[0]]] = polarity[0]*angles_euler2[0]
    angles_euler[ordre[sequence[1]]] = polarity[1]*angles_euler2[1]
    angles_euler[ordre[sequence[2]]] = polarity[2]*angles_euler2[2]       
    return angles_euler

def get_angles_euler(B1, B2, sequence='yxz', polarity=(1,1,1)):
    P = matrice_de_passage(B1, B2)
    angles = angles_euler_from_matrice_passage(P, sequence, polarity)
    angles_arrondis = np.array([round(angles[0], 4), round(angles[1], 4), round(angles[2], 4)])
    #angles_arrondis[0], angles_arrondis[1] = angles_arrondis[1], angles_arrondis[0]
    return angles_arrondis






# points = [
#             'V.MidASIS', 'V.Sacrum', 'V.R.ASIS', 'V.L.ASIS', 'V.L.Knee','V.L.MedialFemoralEpicondyle', 'V.L.LateralFemoralEpicondyle', 
#             'V.R.Knee', 'V.R.MedialFemoralEpicondyle', 'V.R.LateralFemoralEpicondyle','V.L.Ankle', 'V.L.MedialMalleolus', 
#             'V.L.LateralMalleolus', 'V.R.Ankle', 'V.R.MedialMalleolus', 'V.R.LateralMalleolus', 'V.L.Calcaneus', 'V.R.Calcaneus',
#             'V.L.MidMT', 'V.R.MidMT', 'V.L.FMT', 'V.L.FMT', 'L.5MT', 'L.5MT'
#         ]



def find_first_frame():
    f = 0
    obstruction = True
    while obstruction: 
        obstruction = False
        for point in points:
            if data_points[point][f][3] == -1:
                obstruction = True
                break
        f+=1
    return f

def find_last_frame():
    f = frames
    obstruction = True
    while obstruction: 
        f-=1
        obstruction = False
        for point in points:
            if data_points[point][f][3] == -1:
                obstruction = True
    return f
    

    
    
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

############### C3D -> BVH 

#
#       x -> rotation homme de vésuve
#       y -> rotation playmobil
#       z -> rotationautour de soi même
#

# #######  c3d sans angles -> bvh 

def c3d_sans_angles(file, data_points, first_frame, last_frame, ordre):
    file_bvh = file[:-3] + "bvh"
    shutil.copyfile("bvh_base5.bvh", file_bvh)
    f = open(file_bvh, 'a')

    f.write("\n\n\nFrames: " + str(frames))
    f.write("\nFrame Time: " + str(frame_time) + "\n") 

    for i in range(first_frame, last_frame+1):
        base_bassin_left = calculate_base_bassin_left(i)
        base_bassin_right = calculate_base_bassin_right(i)
        base_cuisse_left = calculate_base_cuisse_left(i)
        base_cuisse_right = calculate_base_cuisse_right(i)
        base_jambe_left = calculate_base_jambe_left(i)
        base_jambe_right = calculate_base_jambe_right(i)
        base_pied_left = calculate_base_pied_left(i)
        base_pied_right = calculate_base_pied_right(i)
        
        angles_bassin = get_angles_euler(base_global, base_bassin_right, sequence="zxy", polarity=(1, 1,-1)) # oki
        pelvis_str ="{} {} {} ".format(angles_bassin[0], angles_bassin[1], angles_bassin[2]) 
        
        angles_Hip_left = get_angles_euler(base_bassin_left, base_cuisse_left, "zxy")
        Lhip_str = "{} {} {} ".format(angles_Hip_left[0], angles_Hip_left[1], angles_Hip_left[2]) 
        angles_Knee_left = get_angles_euler(base_cuisse_left, base_jambe_left, "zxy")
        Lknee_str = "{} {} {} ".format(angles_Knee_left[0], angles_Hip_left[1], angles_Hip_left[2])
        angles_Ankle_left = get_angles_euler(base_jambe_left, base_pied_left, "xzy")
        Lankle_str = "{} {} {} ".format(angles_Ankle_left[0], angles_Ankle_left[1], angles_Ankle_left[2])
        
        angles_Hip_right = get_angles_euler(base_bassin_right, base_cuisse_right, "zxy")
        Rhip_str = "{} {} {} ".format(angles_Hip_right[0], angles_Hip_right[1], angles_Hip_right[2])
        angles_Knee_right = get_angles_euler(base_cuisse_right, base_jambe_right, "zxy")
        Rknee_str = "{} {} {} ".format(angles_Knee_right[0], angles_Knee_right[1], angles_Knee_right[2])
        angles_Ankle_right = get_angles_euler(base_jambe_right, base_pied_right, "xzy")
        Rankle_str = "{} {} {} ".format(angles_Ankle_right[0], angles_Ankle_right[1], angles_Ankle_right[2])
        
        positions_bassin = "0 0 0 "
        Lpelvis = "0 0 0 "
        Rplevis = "0 0 0 "
        leftover = 14*"0 0 0 "
        
        f.write(positions_bassin + pelvis_str + Lpelvis + Lhip_str + Lknee_str + Lankle_str + Rplevis + Rhip_str + Rknee_str + Rankle_str + leftover + "\n")

    
#############  c3d avec angles -> bvh

def c3d_avec_angles(file, data_points, first_frame, last_frame, ordre):
    
    file_bvh = file[:-3] + "bvh"

    shutil.copyfile("bvh_base5.bvh", file_bvh)
    f = open(file_bvh, 'a')

    f.write("\n\n\nFrames: " + str(frames))
    f.write("\nFrame Time: " + str(frame_time) + "\n") 
    ordre = [0,1,2]

    for i in range(first_frame, last_frame):
        
        angles_bassin = data_points["L.Pelvis"][i]
        pelvis_str ="{} {} {} ".format(angles_bassin[ordre[0]], angles_bassin[ordre[1]], -angles_bassin[ordre[2]]) 
        
        angles_Hip_left = data_points["L.Hip"][i]
        Lhip_str = "{} {} {} ".format(-angles_Hip_left[ordre[0]], -angles_Hip_left[ordre[1]], -angles_Hip_left[ordre[2]]) 
        angles_Hip_left = data_points["L.Knee"][i]
        Lknee_str = "{} {} {} ".format(-angles_Hip_left[ordre[0]], angles_Hip_left[ordre[1]], -angles_Hip_left[ordre[2]])
        angles_Ankle_left = data_points["L.Ankle"][i]
        Lankle_str = "{} {} {} ".format(-angles_Ankle_left[ordre[0]], -angles_Ankle_left[ordre[1]], -angles_Ankle_left[ordre[2]])
        
        angles_Hip_right = data_points["R.Hip"][i]
        Rhip_str = "{} {} {} ".format(angles_Hip_right[ordre[0]], -angles_Hip_right[ordre[1]], angles_Hip_right[ordre[2]])
        angles_Knee_right = data_points["R.Knee"][i]
        Rknee_str = "{} {} {} ".format(angles_Knee_right[ordre[0]], angles_Knee_right[ordre[1]], angles_Knee_right[ordre[2]])
        angles_Ankle_right = data_points["R.Ankle"][i]
        Rankle_str = "{} {} {} ".format(angles_Ankle_right[ordre[0]], -angles_Ankle_right[ordre[1]], angles_Ankle_right[ordre[2]])
        
        positions_bassin = "0 0 0 "
        Lpelvis = "0 0 0 "
        Rplevis = "0 0 0 "
        leftover = 14*"0 0 0 "
        f.write(positions_bassin + pelvis_str + Lpelvis + Lhip_str + Lknee_str + Lankle_str + Rplevis + Rhip_str + Rknee_str + Rankle_str + leftover + "\n")
        
    
    
def comparaison_angles():
    for i in range(first_frame, last_frame):
        base_bassin_left = calculate_base_bassin_left(i)
        base_bassin_right = calculate_base_bassin_right(i)
        base_cuisse_left = calculate_base_cuisse_left(i)
        base_cuisse_right = calculate_base_cuisse_right(i)
        base_jambe_left = calculate_base_jambe_left(i)
        base_jambe_right = calculate_base_jambe_right(i)
        base_pied_left = calculate_base_pied_left(i)
        base_pied_right = calculate_base_pied_right(i)
        
        angles_knee_left = get_angles_euler(base_global, base_bassin_right, sequence="zxy", polarity=(1, 1,-1))
        Lknee_str = "{} {} {} ".format(angles_knee_left[0], angles_knee_left[1], angles_knee_left[2]) 
        print(angles_knee_left, "  ---  ", data_points["R.Pelvis"][i])

def test(file):
    
    
    # Read the markers
    markers = ktk.read_c3d(
        ktk.doc.download(file)
    )["Points"]


    frames = ktk.TimeSeries(time=markers.time)
    origine = markers.data["V.L.Hip"]
    y = (0.5*(markers.data['V.L.MedialFemoralEpicondyle'] + markers.data['V.L.LateralFemoralEpicondyle']) - markers.data["V.L.Hip"])
    print(y)
    yz = markers.data['V.L.Hip'] - markers.data['V.L.Knee']
    frames.data["Cuisse"] = ktk.geometry.create_frames(origin=origine, y=y, yz=yz)   
    
    origine = markers.data["V.L.Knee"]
    y = (0.5*(markers.data['V.L.MedialMalleolus'] + markers.data['V.L.LateralMalleolus']) - markers.data["V.L.Knee"])
    yz = markers.data['V.L.Knee'] - markers.data['V.L.Ankle']
    frames.data["Jambe"] = ktk.geometry.create_frames(origin=origine, y=y, yz=yz) 
    
    angles_knee_left = ktk.geometry.get_local_coordinates(frames.data['Jambe'], frames.data['Cuisse'])
    Lknee_str = "{} {} {} ".format(angles_knee_left[0], angles_knee_left[1], angles_knee_left[2]) 
    print(angles_knee_left, "  ---  ", data_points["L.Knee"][i], "             écart : ", abs(abs(angles_knee_left)-abs(np.array(data_points["L.Knee"][i][:3]))))


def test2(f):
    print("\n", data_points['V.L.Knee'][f])
    print(data_points['V.L.Hip'][f])
    print(data_points['V.L.MedialFemoralEpicondyle'][f])
    print(data_points['V.L.LateralFemoralEpicondyle'][f], "\n")
    print(normalized_vector(data_points['V.L.Knee'][f], data_points['V.L.Hip'][f]))
    print(normalized_vector(data_points['V.L.MedialFemoralEpicondyle'][f],data_points['V.L.LateralFemoralEpicondyle'][f]), "\n")
    print(calculate_base_jambe_left(f))
    
    
    
def affiche_c3d(file):       
    with open(file, 'rb') as handle:
        reader = c3d.Reader(handle)

        for i, (n, points, analog) in enumerate(reader.read_frames()):
            print("\n ----", i, n)
            print(i, n, " - POINTS :")
            ligne = 0
            for label in enumerate(reader.point_labels):
                print("     ", label, " : ", points[ligne][0:4])
                ligne+=1
        
            print(i, n, " - ANALOG :")
            ligne = 0
            for label in enumerate(reader.analog_labels):
                print("     ", label, " : ", analog[ligne][0:4])
                ligne+=1


        


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


file=r"Corridor_050_avec_angles.c3d"

data_points = get_data(file)
ordre = [0,1,2]


points = data_points.keys()
with open(file, 'rb') as handle:
    reader = c3d.Reader(handle)
    frames = reader.frame_count
    frame_time = 0.01
    first_frame= find_first_frame()
    last_frame= find_last_frame()
    
    
if data_points["V.MidASIS"][last_frame][0] - data_points["V.MidASIS"][first_frame][0] >= 0:
    base_global = np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])
else:
    base_global = np.array([-1, 0, 0]), np.array([0, -1, 0]), np.array([0, 0, 1])    

    
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
    


c3d_sans_angles(file, data_points, first_frame, last_frame, ordre)
c3d_avec_angles(file, data_points, first_frame, last_frame, ordre)
comparaison_angles()
