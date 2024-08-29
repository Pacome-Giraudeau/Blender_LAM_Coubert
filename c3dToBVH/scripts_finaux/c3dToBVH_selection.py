import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import shutil
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from ezc3d import c3d as ezc3d

import tkinter as tk
from tkinter import filedialog



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
    reader = ezc3d(file)
    for i, label in enumerate(reader["parameters"]["POINT"]["LABELS"]["value"]):
        data_points[label.replace(" ","")] = []
        
        for f in range(len(reader["data"]["points"][0][i])):
            data_points[label.replace(" ","")].append([reader["data"]["points"][j][i][f] for j in range(4)])
            
        data_points[label.replace(" ","")] = np.array(data_points[label.replace(" ","")])
            
    return data_points
 
 
 
    
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


def calculate_base_pelvis_left(f, data_points):
    """
    Calcul de la base lié au bassin 
    """
    return calculate_base_system_left(data_points['V.Sacrum'][f], data_points['V.MidASIS'][f], 
                                      data_points['V.R.ASIS'][f], data_points['V.L.ASIS'][f])

def calculate_base_pelvis_right(f, data_points):
    """
    Calcul de la base lié au bassin 
    """
    return calculate_base_system_right(data_points['V.Sacrum'][f], data_points['V.MidASIS'][f], 
                                       data_points['V.R.ASIS'][f], data_points['V.L.ASIS'][f])

def calculate_base_thigh_left(f, data_points):
    """
    Calcul de la base lié à la cuisse gauche 
    """
    return calculate_base_system_left(data_points['V.L.Knee'][f], data_points['V.L.Hip'][f], 
                                      data_points['V.L.MedialFemoralEpicondyle'][f], data_points['V.L.LateralFemoralEpicondyle'][f], 
                                      ordre=[2, 1, 0])

def calculate_base_thigh_right(f, data_points):
    """
    Calcul de la base lié à la cuisse droite 
    """
    return calculate_base_system_right(data_points['V.R.Knee'][f], data_points['V.R.Hip'][f], 
                                       data_points['V.R.MedialFemoralEpicondyle'][f], data_points['V.R.LateralFemoralEpicondyle'][f], 
                                       ordre=[2, 1, 0])

def calculate_base_leg_left(f, data_points):
    """
    Calcul de la base lié à la jambe gauche 
    """
    return calculate_base_system_left(data_points['V.L.Ankle'][f], data_points['V.L.Knee'][f], 
                                      data_points['V.L.MedialMalleolus'][f], data_points['V.L.LateralMalleolus'][f], ordre=[2, 1, 0])

def calculate_base_leg_right(f, data_points):
    """
    Calcul de la base lié à la jambe droite 
    """
    return calculate_base_system_right(data_points['V.R.Ankle'][f], data_points['V.R.Knee'][f], 
                                       data_points['V.R.MedialMalleolus'][f], data_points['V.R.LateralMalleolus'][f], ordre=[2, 1, 0])

def calculate_base_foot_left(f, data_points):
    """
    Calcul de la base lié au foot gauche
    """
    return calculate_base_system_left(data_points['V.L.Calcaneus'][f], data_points['V.L.MidMT'][f], 
                                      data_points['V.L.FMT'][f], data_points['L.5MT'][f])

def calculate_base_foot_right(f, data_points):
    """
    Calcul de la base lié à la foot droite 
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
    return np.array([angles_arrondis[0], angles_arrondis[1], angles_arrondis[2],  0])


def find_occluded(data_points):
    occluded_points = []
    for p in data_points.keys():
        only_occlusion = True
        for f in data_points[p]:
            only_occlusion = only_occlusion and np.isnan(f[0])
        if only_occlusion:
            occluded_points.append(p)
    print(occluded_points)
    return occluded_points

def find_first_frame(data_points):
    """
    Retourne la première frame non obstruée (i.e. aucun des points de la frame n'est obstrué )
    """
    occluded_points = find_occluded(data_points)
    points = data_points.keys()
    f = 0
    obstruction = True
    while obstruction:  
        obstruction = False
        for point in points:
            # si un des point est obstrué, on considère la frame obstruée 
            if np.isnan(data_points[point][f][0]) and point not in occluded_points:
                obstruction = True
                print(point)
                break
        f+=1
    print(f)
    return f

def find_last_frame(data_points, frames):
    """
    Retourne la première frame non obstruée (i.e. aucun des points de la frame n'est obstrué) en aprtant de la fin
    """
    occluded_points=find_occluded(data_points)
    points = data_points.keys()
    f = frames
    obstruction = True
    while obstruction: 
        f-=1
        obstruction = False
        for point in points:
            # si un des point est obstrué, on considère la frame obstruée 
            
            if np.isnan(data_points[point][f][0]) and point not in occluded_points:
                obstruction = True
                break
    print(f)
    return f
    

    
    
def existing_or_not__indices(point, first_frame, last_frame, data_points):
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

def c3d_to_bvh(file):
    data_points, frames, frame_time, first_frame, last_frame, base_global = init(file)
    scale = 60
    
    sens = (data_points["V.PelvisOriginBD"][last_frame][0] - data_points["V.PelvisOriginBD"][first_frame][0]) / abs(data_points["V.PelvisOriginBD"][last_frame][0] - data_points["V.PelvisOriginBD"][first_frame][0])
    print("sens : ", sens)
    
    file_bvh = ".\\internal\\" + file[:-4] + ".bvh"

    shutil.copyfile(".\\_internal\\template\\bvh_base5.bvh", file_bvh)
    sens = (data_points["V.PelvisOriginBD"][last_frame][0] - data_points["V.PelvisOriginBD"][first_frame][0]) / abs(data_points["V.PelvisOriginBD"][last_frame][0] - data_points["V.PelvisOriginBD"][first_frame][0])
    print("sens : ", sens)
            
    with open(file_bvh, 'a') as f:

        f.write("\n\n\nFrames: " + str(last_frame-first_frame))
        f.write("\nFrame Time: " + str(frame_time) + "\n") 

        list_of_points = data_points.keys()
        
        for i in range(first_frame, last_frame):
            # calcul des bases des différents solides de la frame
            base_pelvis_left = calculate_base_pelvis_left(i, data_points)
            base_pelvis_right = calculate_base_pelvis_right(i, data_points)
            base_thigh_left = calculate_base_thigh_left(i, data_points)
            base_thigh_right = calculate_base_thigh_right(i, data_points)
            base_leg_left = calculate_base_leg_left(i, data_points)
            base_leg_right = calculate_base_leg_right(i, data_points)
            base_foot_left = calculate_base_foot_left(i, data_points)
            base_foot_right = calculate_base_foot_right(i, data_points)
            
            # Calcul des angles
            
            if 'L.Pelvis' not in list_of_points: 
                angles_pelvis = get_angles_euler(base_global, base_pelvis_left, sequence="yxz", polarity=(-1, -1, 1))
            else:
                angles_pelvis = data_points["L.Pelvis"][i]
                
            if 'L.Hip' not in list_of_points:
                angles_Hip_left = get_angles_euler(base_pelvis_left, base_thigh_left, sequence="yxz", polarity=(1, 1, 1))
            else:
                angles_Hip_left = data_points["L.Hip"][i]
                
            if 'L.Knee' not in list_of_points:
                angles_Knee_left = get_angles_euler(base_thigh_left, base_leg_left, "yxz", polarity=(-1, 1, 1))
            else:
                angles_Knee_left = data_points["L.Knee"][i]
                
            if 'L.Ankle' not in list_of_points:
                angles_Ankle_left = get_angles_euler(base_leg_left, base_foot_left, "xzy")
            else:
                angles_Ankle_left = data_points["L.Ankle"][i]
                
            if 'R.Hip' not in list_of_points:
                angles_Hip_right = get_angles_euler(base_pelvis_right, base_thigh_right, sequence="yxz", polarity=(1, -1, -1))
            else:
                angles_Hip_right = data_points["R.Hip"][i]
                
            if 'R.Knee' not in list_of_points:
                angles_Knee_right = get_angles_euler(base_thigh_right, base_leg_right, sequence="yxz", polarity=(-1, -1, -1))
            else:
                angles_Knee_right = data_points["R.Knee"][i]
                
            if 'R.Ankle' not in list_of_points:
                angles_Ankle_right = get_angles_euler(base_leg_right, base_foot_right, "zxy", polarity=(-1, -1, 1))
            else:
                angles_Ankle_right = data_points["R.Ankle"][i]
            
            pelvis_str ="{} {} {} ".format(angles_pelvis[0], angles_pelvis[1], -angles_pelvis[2]) 
            Lhip_str = "{} {} {} ".format(-angles_Hip_left[0], -angles_Hip_left[1], -angles_Hip_left[2]) 
            Lknee_str = "{} {} {} ".format(-angles_Knee_left[0], angles_Knee_left[1], -angles_Knee_left[2])
            Lankle_str = "{} {} {} ".format(0, -angles_Ankle_left[1], -angles_Ankle_left[2])
            Rhip_str = "{} {} {} ".format(angles_Hip_right[0], -angles_Hip_right[1], angles_Hip_right[2])
            Rknee_str = "{} {} {} ".format(angles_Knee_right[0], angles_Knee_right[1], angles_Knee_right[2])
            Rankle_str = "{} {} {} ".format(0, -angles_Ankle_right[1], angles_Ankle_right[2])
            
            LowerBack = "0 -10 0 "
            
            if 'L.Spine' not in list_of_points: 
                Spine = 0, 0, 0
            else:
                Spine = data_points["L.Spine"][i] 
            
            Spine1 = "0 0 0 "
            
            if 'Neck' not in list_of_points:
                Neck = 0, 0, 0
            else:
                Neck = data_points["Neck"][i]
                
            Neck1 = "0 0 0 "
            Head = "0 0 0 "
            
            LClavicle = "0 0 0 "
            if 'L.Elbow' not in list_of_points:
                LElbow = 0, 0, 0
            else:
                LElbow = data_points["L.Elbow"][i]
                
            if 'L.Wrist' not in list_of_points:
                LWrist = 0, 0, 0
            else:
                LWrist = data_points["L.Wrist"][i] 
            
            if 'L.Shoulder' not in list_of_points:
                LShoulder = 0, 0, 0
            else:
                LShoulder =  data_points["L.Shoulder"][i]
            
            RClavicle ="0 0 0 "

            if 'R.Wrist' not in list_of_points:
                RWrist =0, 0, 0
            else:
                RWrist = data_points["R.Wrist"][i] 
                
            if 'R.Shoulder' not in list_of_points:
                RShoulder = 0, 0, 0
            else:
                RShoulder =  data_points["R.Shoulder"][i]
                
            if 'R.Elbow' not in list_of_points:
                RElbow = 0, 0, 0
            else:
                RElbow = data_points["R.Elbow"][i]
            
            
            Spine_str = "{} {} {} ".format(-Spine[0], Spine[1], -Spine[2])
            Neck_str = "{} {} {} ".format(Neck[0], -Neck[1], Neck[2])
            LElbow_str = "{} {} {} ".format(LElbow[0], LElbow[2], -LElbow[1])
            RElbow_str = "{} {} {} ".format(-RElbow[0], RElbow[2], RElbow[1])
            LWrist_str = "{} {} {} ".format(-LWrist[1], LWrist[2], -LWrist[0])
            RWrist_str = "{} {} {} ".format(RWrist[1], RWrist[2], RWrist[0])
            RShoulder_str = "{} {} {} ".format(RShoulder[0]+90, RShoulder[1], RShoulder[2])
            LShoulder_str = "{} {} {} ".format(LShoulder[0]-90, LShoulder[1], LShoulder[2])
            
            positions_pelvis = "0 0 0 "
            positions_pelvis = "{} {} {} ".format( sens*data_points["V.PelvisOriginBD"][i][1]/scale, data_points["V.PelvisOriginBD"][i][2]/scale, sens * data_points["V.PelvisOriginBD"][i][0]/scale)
        
            Lpelvis = "0 0 0 "
            Rplevis = "0 0 0 "
            ecrire = positions_pelvis + pelvis_str + Lpelvis + Lhip_str + Lknee_str + Lankle_str + Rplevis + Rhip_str + Rknee_str + Rankle_str + LowerBack + Spine1 + Spine_str + Neck_str + Neck1 + Head + LClavicle + LShoulder_str + LElbow_str + LWrist_str + RClavicle + RShoulder_str + RElbow_str + RWrist_str + "\n"
            f.write(ecrire.replace("nan","0"))
            
            
        

def init(file):
    
    data_points = get_data(file)
    reader = ezc3d(file)
    frames = len(reader["data"]["points"][0][0])
    frame_time = 0.01
    first_frame= find_first_frame(data_points)
    last_frame= find_last_frame(data_points, frames)
    print(frames)
        
        
    if data_points["V.MidASIS"][last_frame][0] - data_points["V.MidASIS"][first_frame][0] >= 0:
        base_global = np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])
    else:
        base_global = np.array([-1, 0, 0]), np.array([0, -1, 0]), np.array([0, 0, 1])    

    data_points = interpolation(data_points, first_frame, last_frame)
    return data_points, frames, frame_time, first_frame, last_frame, base_global

############  INTERPOLATION
def interpolation(data_points, first_frame, last_frame):
    points = data_points.keys()
    for point in points:

        # Extraire les points existants
        missing_indices, existing_indices, existing_points = existing_or_not__indices(point, first_frame, last_frame, data_points)
        # Interpoler séparément pour x, y et z
        interp_func = interp1d(existing_indices, existing_points, axis=0, kind='linear', fill_value="extrapolate")
        for f in range(len(missing_indices)):
            
            p = interp_func(missing_indices)
            data_points[point][missing_indices[f]][0] = p[f][0]
            data_points[point][missing_indices[f]][1] = p[f][1]
            data_points[point][missing_indices[f]][2] = p[f][2]
        return data_points
        


class CustomFileDialog():
    
    def get_selection(self):
        # Retourne le fichier sélectionné
        return self.selection
        
    def show(self):
        # Vous pouvez personnaliser la boîte de dialogue ici
        self.filepath = filedialog.askopenfilename(
            title="Sélectionnez un fichier",
            filetypes=(("Fichiers c3d", "*.c3d"), ("Tous les fichiers", "*.*"))
        )
        print(self.filepath)
        self.handle_selection()
        
    def handle_selection(self):
        # Retourne le fichier sélectionné
        c3d_to_bvh(self.filepath, 60)


if __name__ == "__main__":
    root = tk.Tk()
    root.withdraw()  # On masque la fenêtre principale
    
    dialog = CustomFileDialog()
    dialog.show()  # Affiche la boîte de dialogue personnalisée