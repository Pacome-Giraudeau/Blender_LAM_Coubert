import c3d
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import shutil
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d



def get_data(file):
    """
    entré : file
        file
        - lien d'un fichier c3d
    
    sortie : 
        data_point
        - dictionnaire où les clés sont les labels des différents points
        - aux labels des points sont associés les listes de leurs points dans le temps 
        - data_points['label'] -> liste de points
        - data_points['label'][f] -> liste de 5 valeurs correspondant au point 'label' à la frame f
            - data_points['label'][f][0] : coordonée x
            - data_points['label'][f][1] : coordonée y
            - data_points['label'][f][2] : coordonée z
            - data_points['label'][f][3] : occlusion (-1 si occulté, 0 sinon)
            - data_points['label'][f][4] : marge d'erreur
    """
    data_points = dict()
    with open(file, 'rb') as handle:
        reader = c3d.Reader(handle)
        
        for i, label in enumerate(reader.point_labels):
            data_points[label.replace(" ","")] = []
                
        for n, points, analog in reader.read_frames():
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
    """
    Entrée : vecteur u de dimension 3
    Sortie : norme euclidienne du vecteur u
    """
    return math.sqrt(u[0]**2 + u[1]**2 + u[2]**2)

def normalized_vector(p1, p2):
    """
    Entrée : 
        - deux points p1 et p2 de dimension au moins 3
    Sortie :
        - un vecteur de direction p1 vers p2 et de norme 1 sous forme de tableau numpy
    """
    uprime = (p2[0] - p1[0], p2[1]- p1[1], p2[2] - p1[2])
    return np.array([uprime[0]/norme(uprime), uprime[1]/norme(uprime), uprime[2]/norme(uprime)])


def calculate_base_system_right(u1_1, u1_2, u2_interieur, u2_exterieur, ordre=[0, 1, 2]):
    """
    Calcul d'une base orthonormée à partir de 4 points
    Entrée : 
        - u1_1, u1_2, u2_interieur, u2_exterieur sont des points
        - ordre : ordre dans lequel les vecteurs de la base sont calculés (x, y ou z).
        
    Procédé :
        - u1        : u1_1 -> u1_2 normalisé
        - u2prime   : u2_interieur -> u2_exterieur normalisé
        - u3        : u1^u2prime  
        - u2        : u3^u1 si ordre[0] == 0
                      u1^u3 si ordre[0] == 2
        
    Sortie :
        base [u1, u2, u3] orthonormale
        - u1 : dans le sens du mouvement
        - u2 : de la droite vers la gauche
        - u3 : vers le haut
        
    Erreur :
        - si un des points est occulté : renvoie d'une base nulle
    """
    if u1_1[3] == -1 or u1_2[3] == -1 or u2_interieur[3] == -1 or u2_exterieur[3] == -1:
        return np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0])
    
    u1 = normalized_vector(u1_1, u1_2)
    u2prime = normalized_vector(u2_interieur, u2_exterieur)
    u3 = np.cross(u1, u2prime)
    if ordre[0] == 0:
        u2 = np.cross(u3, u1)
    elif ordre[0] == 2:
        u2 = np.cross(u1, u3)
    base = [u1, u2, u3]
    
    return base[ordre[0]], base[ordre[1]], base[ordre[2]] 


def calculate_base_system_left(u1_1, u1_2, u2_interieur, u2_exterieur, ordre=[0, 1, 2]):
    """
    Calcul d'une base orthonormée à partir de 4 points
    Entrée : 
        - u1_1, u1_2, u2_interieur, u2_exterieur sont des points
        - ordre : ordre dans lequel les vecteurs de la base sont calculés (x, y ou z).
        
    Procédé :
        - u1        : u1_1 -> u1_2 normalisé
        - u2prime   : u2_interieur -> u2_exterieur normalisé
        - si ordre[0] == 0
            - u3        : u1^u2prime  
            - u2        : u3^u1 
        - si ordre[0] == 2
            - u3        : u2prime^u1  
            - u2        : u1^u3
        
    Sortie :
        base [u1, u2, u3] orthonormale
        - u1 : dans le sens du mouvement
        - u2 : de la droite vers la gauche
        - u3 : vers le haut
           
    Erreur :
        - si un des points est occulté : renvoie d'une base nulle
    """
    if u1_1[3] == -1 or u1_2[3] == -1 or u2_interieur[3] == -1 or u2_exterieur[3] == -1:
        return np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0])
    
    u1 = normalized_vector(u1_1, u1_2)
    u2prime = normalized_vector(u2_interieur, u2_exterieur)
    
    if ordre[0] == 0:
        u3 = np.cross(u1, u2prime)
        u2 = np.cross(u3, u1)
    elif ordre[0] == 2:
        u3 = np.cross(u2prime, u1)
        u2 = np.cross(u1, u3)
    
    base = [u1, u2, u3]
    return base[ordre[0]], base[ordre[1]], base[ordre[2]]   





def calculate_base_pelvis_left(f):
    """
    Calcul de la base lié au bassin 
    """
    return calculate_base_system_left(data_points['V.Sacrum'][f], data_points['V.MidASIS'][f], 
                                      data_points['V.R.ASIS'][f], data_points['V.L.ASIS'][f])


def calculate_base_pelvis_right(f):
    """
    Calcul de la base lié au bassin 
    """
    return calculate_base_system_right(data_points['V.Sacrum'][f], data_points['V.MidASIS'][f], 
                                       data_points['V.R.ASIS'][f], data_points['V.L.ASIS'][f])


def calculate_base_thigh_left(f):
    """
    Calcul de la base lié à la cuisse gauche 
    """
    return calculate_base_system_left(data_points['V.L.Knee'][f], data_points['V.L.Hip'][f], 
                                      data_points['V.L.MedialFemoralEpicondyle'][f], data_points['V.L.LateralFemoralEpicondyle'][f], 
                                      ordre=[2, 1, 0])


def calculate_base_thigh_right(f):
    """
    Calcul de la base lié à la cuisse droite 
    """
    return calculate_base_system_right(data_points['V.R.Knee'][f], data_points['V.R.Hip'][f], 
                                       data_points['V.R.MedialFemoralEpicondyle'][f], data_points['V.R.LateralFemoralEpicondyle'][f], 
                                       ordre=[2, 1, 0])


def calculate_base_leg_left(f):
    """
    Calcul de la base lié à la jambe gauche 
    """
    return calculate_base_system_left(data_points['V.L.Ankle'][f], data_points['V.L.Knee'][f], 
                                      data_points['V.L.MedialMalleolus'][f], data_points['V.L.LateralMalleolus'][f], ordre=[2, 1, 0])


def calculate_base_leg_right(f):
    """
    Calcul de la base lié à la jambe droite 
    """
    return calculate_base_system_right(data_points['V.R.Ankle'][f], data_points['V.R.Knee'][f], 
                                       data_points['V.R.MedialMalleolus'][f], data_points['V.R.LateralMalleolus'][f], ordre=[2, 1, 0])


def calculate_base_pied_left(f):
    """
    Calcul de la base lié au pied gauche
    """
    return calculate_base_system_left(data_points['V.L.Calcaneus'][f], data_points['V.L.MidMT'][f], 
                                      data_points['V.L.FMT'][f], data_points['L.5MT'][f])


def calculate_base_pied_right(f):
    """
    Calcul de la base lié à la pied droite 
    """
    return calculate_base_system_right(data_points['V.R.Calcaneus'][f], data_points['V.R.MidMT'][f], 
                                       data_points['R.5MT'][f], data_points['V.R.FMT'][f])





def matrice_de_passage(B1, B2):
    """
    Calcule la matrice de passage entre deux repères.
    
    :param B1: Un tuple de trois vecteurs numpy représentant les bases du premier repère.
    :param B2: Un tuple de trois vecteurs numpy représentant les bases du deuxième repère.
    :return: La matrice de passage de B1 à B2.
    """
    # Créer les matrices B1 et B2
    M1 = np.array(B1)
    M2 = np.array(B2)
    
    # Calculer la matrice de passage P
    P = np.dot(M2, M1.transpose())
    
    return P


def angles_euler_from_matrice_passage(P, sequence='yxz', polarity = (1,1,1)):
    """
    Calcule les angles d'Euler à partir d'une matrice de passage.
    
    :param P: La matrice de passage (ou de rotation).
    :param sequence: La séquence des axes d'Euler (par défaut 'yxz').
    :param polarity: polarité de chaque angle
    :return: Les angles d'Euler (en degrés).
    """
    ordre = {'x': 0, 'y':1, 'z':2}
    # Créer un objet Rotation à partir de la matrice de passage
    rotation = R.from_matrix(P)
    
    # Extraire les angles d'Euler selon la séquence spécifiée
    angles_euler2 = rotation.as_euler(sequence, degrees=True)
    
    # Réorganisation et application de la polarité
    angles_euler = np.zeros_like(angles_euler2)   
    angles_euler[ordre[sequence[0]]] = polarity[0]*angles_euler2[0]
    angles_euler[ordre[sequence[1]]] = polarity[1]*angles_euler2[1]
    angles_euler[ordre[sequence[2]]] = polarity[2]*angles_euler2[2]      
     
    return angles_euler

def get_angles_euler(B1, B2, sequence='yxz', polarity=(1,1,1)):
    """
    Entrées :
        - B1 : base orthonormée 
        - B2 : base orthonormée
        - séquence : séquence pour le calcul des angles d'euler
        - polarity : polarité des angles
    
    Sorties :
        - angles d'euleur arrondis à 4 décimales sous forme de triplet 
        - [0] -> rotation selon x
        - [1] -> rotation selon y
        - [2] -> rotation selon z
    """
    P = matrice_de_passage(B1, B2)
    angles = angles_euler_from_matrice_passage(P, sequence, polarity)
    angles_arrondis = np.array([round(angles[0], 4), round(angles[1], 4), round(angles[2], 4)])
    #angles_arrondis[0], angles_arrondis[1] = angles_arrondis[1], angles_arrondis[0]
    return angles_arrondis



def find_first_frame():
    """
    Retourne la première frame non obstruée (i.e. aucun des points de la frame n'est obstrué )
    """
    f = 0
    obstruction = True
    while obstruction: 
        obstruction = False
        for point in points:
            # si un des point est obstrué, on considère la frame obstruée 
            if data_points[point][f][3] == -1:
                obstruction = True
                break
        f+=1
    return f

def find_last_frame():
    """
    Retourne la première frame non obstruée (i.e. aucun des points de la frame n'est obstrué) en aprtant de la fin
    """
    f = frames
    obstruction = True
    while obstruction: 
        f-=1
        obstruction = False
        for point in points:
            # si un des point est obstrué, on considère la frame obstruée 
            if data_points[point][f][3] == -1:
                obstruction = True
    return f
    

    
    
def existing_or_not__indices(point, first_frame, last_frame):
    """
    Entrées : 
        - point (str) :label d'un des points
        - first_frame (int) : première frame considérée
        - last_frame (int) : dernière frame considérée
        
    Sorties : 
        trois tableux numpy 
        - missing_indices : indices des frames où le point est obstrué
        - existing_indices : indices des frames où le point N'est PAS obstrué
        - existing_points : coordonées des points non obstrués
                            existing_points[i] représente les coordonées du point à l'indice xisting_indices[i]
    """
    missing_indices = []
    existing_indices = []
    existing_points = []
    for f in range(first_frame, last_frame):
        # condition d'obstruction
        if data_points[point][f][3] == -1:
            missing_indices.append(f)
        else:
            existing_indices.append(f)
            existing_points.append(data_points[point][f][0:3])
    return np.array(missing_indices), np.array(existing_indices), np.array(existing_points)

    



############### C3D -> BVH 

#
#       x -> rotation homme de vésuve
#       y -> rotation playmobil
#       z -> rotationautour de soi même
#

# #######  c3d sans angles -> bvh 

def c3d_sans_angles(file, data_points, first_frame, last_frame):
    """
    Entrées :
        - file (str)    : nom du fichier traité
        - data_points   : données c3d liées au fichier traité selon le format donné par get_data()
        - first_frame   : première frame considérée, généralement la première frame non obstrué 
        - last_frame    : dernière frame considérée, généralement la première frame non obstrué en partant de la fin
        
    Sortie :
        - fichier bvh du même nom que file 
    
    
    """
    # création d'un fichier
    file_bvh = file[:-3] + "bvh"
    
    # squelette du fichier bvh
    shutil.copyfile("bvh_base5.bvh", file_bvh)
    f = open(file_bvh, 'a')
    f.write("\n\n\nFrames: " + str(frames))
    f.write("\nFrame Time: " + str(frame_time) + "\n") 
    
    
    # traitement frmae par frame
    for i in range(first_frame, last_frame+1):
        
        # calcul des bases des différents solides de la frame
        base_pelvis_left = calculate_base_pelvis_left(i)
        base_pelvis_right = calculate_base_pelvis_right(i)
        base_thigh_left = calculate_base_thigh_left(i)
        base_thigh_right = calculate_base_thigh_right(i)
        base_leg_left = calculate_base_leg_left(i)
        base_leg_right = calculate_base_leg_right(i)
        base_pied_left = calculate_base_pied_left(i)
        base_pied_right = calculate_base_pied_right(i)
        
        
        angles_pelvis = get_angles_euler(base_global, base_pelvis_right, sequence="yxz", polarity=(-1, 1, -1))
        pelvis_str ="{} {} {} ".format(-angles_pelvis[0], angles_pelvis[1], -angles_pelvis[2]) 
        
        angles_Hip_left = get_angles_euler(base_pelvis_left, base_thigh_left, sequence="yxz", polarity=(1, 1, 1))
        Lhip_str = "{} {} {} ".format(-angles_Hip_left[0], -angles_Hip_left[1], -angles_Hip_left[2]) 
        angles_Knee_left = get_angles_euler(base_thigh_left, base_leg_left, "yxz", polarity=(-1, 1, 1))
        Lknee_str = "{} {} {} ".format(-angles_Knee_left[0], angles_Knee_left[1], -angles_Knee_left[2])
        angles_Ankle_left = get_angles_euler(base_leg_left, base_pied_left, "xzy")
        Lankle_str = "{} {} {} ".format(-angles_Ankle_left[0], -angles_Ankle_left[1], -angles_Ankle_left[2])
        
        angles_Hip_right = get_angles_euler(base_pelvis_right, base_thigh_right, sequence="yxz", polarity=(1, -1, -1))
        Rhip_str = "{} {} {} ".format(angles_Hip_right[0], -angles_Hip_right[1], angles_Hip_right[2])
        angles_Knee_right = get_angles_euler(base_thigh_right, base_leg_right, sequence="yxz", polarity=(-1, -1, -1))
        Rknee_str = "{} {} {} ".format(angles_Knee_right[0], angles_Knee_right[1], angles_Knee_right[2])
        angles_Ankle_right = get_angles_euler(base_leg_right, base_pied_right, "zxy", polarity=(-1, -1, 1))
        Rankle_str = "{} {} {} ".format(angles_Ankle_right[0], -angles_Ankle_right[1], angles_Ankle_right[2])
        
        positions_pelvis = "{} {} {} ".format(data_points["V.PelvisOriginBD"][i][1]/100, data_points["V.PelvisOriginBD"][i][2]/100, -data_points["V.PelvisOriginBD"][i][0]/60)
        # positions_pelvis = "0 0 0 "
        Rpelvis = "0 0 0 "
        Lpelvis = "0 0 0 "
        leftover = 14*"0 0 0 "
        
        f.write(positions_pelvis + pelvis_str + Lpelvis + Lhip_str + Lknee_str + Lankle_str + Rpelvis + Rhip_str + Rknee_str + Rankle_str + leftover + "\n")

    
#############  c3d avec angles -> bvh

def c3d_avec_angles(file, data_points, first_frame, last_frame):
    
    file_bvh = file[:-3] + "bvh"

    shutil.copyfile("bvh_base5.bvh", file_bvh)
    f = open(file_bvh, 'a')

    f.write("\n\n\nFrames: " + str(frames))
    f.write("\nFrame Time: " + str(frame_time) + "\n") 

    for i in range(first_frame, last_frame):
        
        angles_pelvis = data_points["L.Pelvis"][i]
        pelvis_str ="{} {} {} ".format(angles_pelvis[0], angles_pelvis[1], -angles_pelvis[2]) 
        
        angles_Hip_left = data_points["L.Hip"][i]
        Lhip_str = "{} {} {} ".format(-angles_Hip_left[0], -angles_Hip_left[1], -angles_Hip_left[2]) 
        angles_Hip_left = data_points["L.Knee"][i]
        Lknee_str = "{} {} {} ".format(-angles_Hip_left[0], angles_Hip_left[1], -angles_Hip_left[2])
        angles_Ankle_left = data_points["L.Ankle"][i]
        Lankle_str = "{} {} {} ".format(-angles_Ankle_left[0], -angles_Ankle_left[1], -angles_Ankle_left[2])
        
        angles_Hip_right = data_points["R.Hip"][i]
        Rhip_str = "{} {} {} ".format(angles_Hip_right[0], -angles_Hip_right[1], angles_Hip_right[2])
        angles_Knee_right = data_points["R.Knee"][i]
        Rknee_str = "{} {} {} ".format(angles_Knee_right[0], angles_Knee_right[1], angles_Knee_right[2])
        angles_Ankle_right = data_points["R.Ankle"][i]
        Rankle_str = "{} {} {} ".format(angles_Ankle_right[0], -angles_Ankle_right[1], angles_Ankle_right[2])
        
        positions_pelvis = "0 0 0 "
        Lpelvis = "0 0 0 "
        Rplevis = "0 0 0 "
        leftover = 14*"0 0 0 "
        f.write(positions_pelvis + pelvis_str + Lpelvis + Lhip_str + Lknee_str + Lankle_str + Rplevis + Rhip_str + Rknee_str + Rankle_str + leftover + "\n")
        
        
############# full c3d avec angles -> bvh  

def c3d_avec_angles_complet(file, data_points, first_frame, last_frame):
    
    file_bvh = file[:-4] + "_complet.bvh"

    shutil.copyfile("bvh_base5.bvh", file_bvh)
    f = open(file_bvh, 'a')

    f.write("\n\n\nFrames: " + str(frames))
    f.write("\nFrame Time: " + str(frame_time) + "\n") 

    for i in range(first_frame, last_frame):
        
        angles_pelvis = data_points["L.Pelvis"][i]
        pelvis_str ="{} {} {} ".format(angles_pelvis[0], angles_pelvis[1], -angles_pelvis[2]) 
        
        angles_Hip_left = data_points["L.Hip"][i]
        Lhip_str = "{} {} {} ".format(-angles_Hip_left[0], -angles_Hip_left[1], -angles_Hip_left[2]) 
        angles_Hip_left = data_points["L.Knee"][i]
        Lknee_str = "{} {} {} ".format(-angles_Hip_left[0], angles_Hip_left[1], -angles_Hip_left[2])
        angles_Ankle_left = data_points["L.Ankle"][i]
        Lankle_str = "{} {} {} ".format(-angles_Ankle_left[0], -angles_Ankle_left[1], -angles_Ankle_left[2])
        
        angles_Hip_right = data_points["R.Hip"][i]
        Rhip_str = "{} {} {} ".format(angles_Hip_right[0], -angles_Hip_right[1], angles_Hip_right[2])
        angles_Knee_right = data_points["R.Knee"][i]
        Rknee_str = "{} {} {} ".format(angles_Knee_right[0], angles_Knee_right[1], angles_Knee_right[2])
        angles_Ankle_right = data_points["R.Ankle"][i]
        Rankle_str = "{} {} {} ".format(angles_Ankle_right[0], -angles_Ankle_right[1], angles_Ankle_right[2])
        
        # LowerBack = data_points[""][i] -> 0
        LowerBack = "0 0 0 "
        Spine = data_points["L.Spine"][i] 
        Spine_str = "{} {} {} ".format(-Spine[0], -Spine[1], Spine[2])
        # Spine1 = data_points[][i] -> 0
        Spine1 = "0 0 0 "
        # Neck = data_points[][i] -> 0 
        Neck = "0 0 0 "
        # Neck1 = data_points[][i] -> 0
        Neck1 = "0 0 0 "
        # Head = data_points[][i]-> 0
        Head = "0 0 0 "
        # LClavicle = data_points[][i] -> 0 
        LClavicle = "0 0 0 "
        LElbow = data_points["L.Elbow"][i]
        LElbow_str = "{} {} {} ".format(LElbow[0], LElbow[1], LElbow[2])
        print(LElbow_str)
        LWrist = data_points["L.Wrist"][i] 
        LWrist_str = "{} {} {} ".format(LWrist[0], LWrist[1], LWrist[2])
        # RClavicle = data_points[][i] -> 0
        RClavicle = "0 0 0 "
        # RElbow = data_points["R.Elbow"][i]
        # RElbow_str = "{} {} {} ".format(RElbow[0], RElbow[1], RElbow[2])
        RElbow_str = "0 0 0 "
        print(RElbow_str)
        RWrist = data_points["R.Wrist"][i] 
        RWrist_str = "{} {} {} ".format(RWrist[0], RWrist[1], RWrist[2])
        RShoulder =  data_points["R.Shoulder"][i]
        RShoulder_str = "{} 0 0 ".format(RShoulder[0], RShoulder[1], -RShoulder[1])
        # LShoulder =  data_points["L.Shoulder"][i]
        # LShoulder_str = "{} {} {} ".format(LShoulder[0], LShoulder[1], LShoulder[2])
        LShoulder_str = "0 0 0 "
        positions_pelvis = "0 0 0 "
        Lpelvis = "0 0 0 "
        Rplevis = "0 0 0 "
        leftover = 14*"0 0 0 "
        f.write(positions_pelvis + pelvis_str + 
                Lpelvis + Lhip_str + Lknee_str + Lankle_str + 
                Rplevis + Rhip_str + Rknee_str + Rankle_str + 
                LowerBack + Spine_str + Spine1 + Neck + Neck1 + Head + 
                LClavicle + LShoulder_str + LElbow_str + LWrist_str + RClavicle + 
                RShoulder_str + RElbow_str + RWrist_str + "\n")
        
    #LowerBack Spine Spine1 Neck Neck1 Head LClavicle LShoulder LElbow LWrist RClavicle RElbow RWrist

def comparaison_angles():
    for i in range(first_frame, last_frame):
        base_pelvis_left = calculate_base_pelvis_left(i)
        base_pelvis_right = calculate_base_pelvis_right(i)
        base_thigh_left = calculate_base_thigh_left(i)
        base_thigh_right = calculate_base_thigh_right(i)
        base_leg_left = calculate_base_leg_left(i)
        base_leg_right = calculate_base_leg_right(i)
        base_pied_left = calculate_base_pied_left(i)
        base_pied_right = calculate_base_pied_right(i)
        
        angles_knee_left = get_angles_euler(base_leg_right, base_pied_right, "zxy", polarity=(-1, -1, 1))
        Lknee_str = "{} {} {} ".format(angles_knee_left[0], angles_knee_left[1], angles_knee_left[2]) 
        print(angles_knee_left, "  ---  ", data_points["R.Ankle"][i])



def test(f):
    print(" POINTS : \n", data_points['V.L.Knee'][f])
    print(data_points['V.L.Hip'][f])
    print(data_points['V.L.MedialFemoralEpicondyle'][f])
    print(data_points['V.L.LateralFemoralEpicondyle'][f], "\n")
    print("vecteurs normalisés : \n", 
          normalized_vector(data_points['V.L.Knee'][f], data_points['V.L.Hip'][f]))
    print(normalized_vector(data_points['V.L.MedialFemoralEpicondyle'][f],data_points['V.L.LateralFemoralEpicondyle'][f]), "\n")
    print(normalized_vector(data_points['V.L.Ankle'][f], data_points['V.L.Knee'][f]))
    print(normalized_vector(data_points['V.L.MedialMalleolus'][f],data_points['V.L.LateralMalleolus'][f]), "\n")
    
    B1 = calculate_base_leg_left(f)
    B2 = calculate_base_thigh_left(f)
    print("B1 : \n", B1, "\n", np.array(B1), "\n", np.column_stack(B1))
    print("\n\nB2 : \n", B2, "\n", np.array(B2), "\n", np.column_stack(B2))
    print("\n\n")
    M = matrice_de_passage(B1, B2)
    print(M)
    print(np.dot(M, B1))
    print(B2, "\n\n")
    print("\n M*B1 : \n", np.dot(M, calculate_base_leg_left(f)))
    print("\n B2  : \n", calculate_base_thigh_left(f), "\n\n")
    rot = R.from_matrix(M)
    print("\n from matrix : ", rot.as_euler("yxz", degrees=True))
    print("\n from get_euler_angles: ", get_angles_euler(calculate_base_leg_left(f), calculate_base_thigh_left(f), sequence="xyz"))
    print("\n True angles: ", data_points['L.Knee'][f])
    print("\n", np.dot(B1, np.array(B1).transpose()))
    print("\n", np.dot(B2, np.array(B2).transpose()))
    
def comparaison_repères():
    for i in range(first_frame, first_frame+3):
        base_pelvis_left = calculate_base_pelvis_left(i)
        base_pelvis_right = calculate_base_pelvis_right(i)
        base_thigh_left = calculate_base_thigh_left(i)
        base_thigh_right = calculate_base_thigh_right(i)
        base_leg_left = calculate_base_leg_left(i)
        base_leg_right = calculate_base_leg_right(i)
        base_pied_left = calculate_base_pied_left(i)
        base_pied_right = calculate_base_pied_right(i)
        print(base_global[0])
        print(base_global[1])
        print(base_global[2])
        print("\n pelvis :")
        print(base_pelvis_left[0])
        print(base_pelvis_left[1])
        print(base_pelvis_left[2])
        print("\n thigh :")
        print(base_thigh_left[0])
        print(base_thigh_left[1])
        print(base_thigh_left[2])
        print("\n leg : ")
        print(base_leg_right[0])
        print(base_leg_right[1])
        print(base_leg_right[2])
        print("\n pied gauche : ")
        print(base_pied_left[0])
        print(base_pied_left[1])
        print(base_pied_left[2])
        print("\n")
        print("\n pied droiy : ")
        print(base_pied_right[0])
        print(base_pied_right[1])
        print(base_pied_right[2])
        print("\n")
    
    
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


file="Corridor_050_avec_angles.c3d"

data_points = get_data(file)

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
    


file="Corridor_050.c3d"
c3d_sans_angles(file, data_points, first_frame, last_frame)
file="Corridor_050_avec_angles.c3d"
c3d_avec_angles_complet(file, data_points, first_frame, last_frame)