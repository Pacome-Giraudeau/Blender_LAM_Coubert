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


############# full c3d avec angles -> bvh  

def c3d_to_bvh(file):
    data_points, frames, frame_time, first_frame, last_frame, base_global = init(file)
    
    file_new = file[:-4] + "_new.c3d"

    c = ezc3d(file)
    del c['data']['meta_points']
    
    number_of_points = len(c["parameters"]["POINT"]["LABELS"]["value"])
    list_of_points = c["parameters"]["POINT"]["LABELS"]["value"]
    
    nouv_points = dict()
    if 'L.Pelvis' not in list_of_points:
        nouv_points['L.Pelvis'] = []
    if 'L.Hip' not in list_of_points:
        nouv_points['L.Hip'] = []
    if 'L.Knee' not in list_of_points:
        nouv_points['L.Knee'] = []
    if 'L.Ankle' not in list_of_points:
        nouv_points['L.Ankle'] = []
    if 'R.Hip' not in list_of_points:
        nouv_points['R.Hip'] = []
    if 'R.Knee' not in list_of_points:
        nouv_points['R.Knee'] = []
    if 'R.Ankle' not in list_of_points:
        nouv_points['R.Ankle'] = []
    
    
    c["parameters"]["POINT"]["LABELS"]["value"] = list_of_points + list(nouv_points.keys())
    l = [[], [], [], []]
    for j in range(4):
        for k in range(number_of_points):
            l[j].append(c["data"]["points"][j][k])
        for k in range(len(nouv_points.keys())):
            l[j].append(np.zeros(frames))
            
    c["data"]["points"] = np.array(l)
    

    for i in range(0, frames):
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
            nouv_points['L.Pelvis'].append(angles_pelvis)
            
        if 'L.Hip' not in list_of_points:
            angles_Hip_left = get_angles_euler(base_pelvis_left, base_thigh_left, sequence="yxz", polarity=(1, 1, 1))
            nouv_points['L.Hip'].append(angles_Hip_left)
            
        if 'L.Knee' not in list_of_points:
            angles_Knee_left = get_angles_euler(base_thigh_left, base_leg_left, "yxz", polarity=(-1, 1, 1))
            nouv_points['L.Knee'].append(angles_Knee_left)
            
        if 'L.Ankle' not in list_of_points:
            angles_Ankle_left = get_angles_euler(base_leg_left, base_foot_left, "xzy")
            nouv_points['L.Ankle'].append(angles_Ankle_left)
            
        if 'R.Hip' not in list_of_points:
            angles_Hip_right = get_angles_euler(base_pelvis_right, base_thigh_right, sequence="yxz", polarity=(1, -1, -1))
            nouv_points['R.Hip'].append(angles_Hip_right)
            
        if 'R.Knee' not in list_of_points:
            angles_Knee_right = get_angles_euler(base_thigh_right, base_leg_right, sequence="yxz", polarity=(-1, -1, -1))
            nouv_points['R.Knee'].append(angles_Knee_right)
            
        if 'R.Ankle' not in list_of_points:
            angles_Ankle_right = get_angles_euler(base_leg_right, base_foot_right, "zxy", polarity=(-1, -1, 1))
            nouv_points['R.Ankle'].append(angles_Ankle_right)
        
        k=number_of_points
        for point in nouv_points.keys():
            for j in range(4):
                c["data"]["points"][j][k][i] = nouv_points[point][i][j]
            k+=1
            
    print("j'écris sur les murs")
    c.write(file_new)
    




    









    
        


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
            if np.isnan(data_points[point][f][0]):
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
            
            if np.isnan(data_points[point][f][0]):
                obstruction = True
                break
    return f
    
        


def init(file):
    
    data_points = get_data(file)
    reader = ezc3d(file)
    frames = len(reader["data"]["points"][0][0])
    frame_time = 0.01
    first_frame= find_first_frame(data_points)
    last_frame= find_last_frame(data_points, frames)
        
        
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
            data_points[point][missing_indices[f]][3] = p[f][2]
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
        c3d_to_bvh(self.filepath)


if __name__ == "__main__":
    root = tk.Tk()
    root.withdraw()  # On masque la fenêtre principale
    
    dialog = CustomFileDialog()
    dialog.show()  # Affiche la boîte de dialogue personnalisée