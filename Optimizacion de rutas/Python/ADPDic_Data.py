# -*- coding: utf-8 -*-
"""
Created on Tue Apr  9 12:01:46 2024

@author: ccalderon
"""
import pandas as pd
import csv
import numpy as np
import json

def Nodo(i,j,F):
  return F*i+j

def VecinoSuperior(F,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i-1;
    s = j;
    k = Nodo(r,s,F);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20;
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def VecinoSuperiorDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i-1;
    s = j+1;
    k = Nodo(r,s,F);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20*np.sqrt(2);
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def VecinoDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i;
    s = j+1;
    k = Nodo(r,s,F);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20;
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def VecinoInferiorDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i+1;
    s = j+1;
    k = Nodo(r,s,F);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20*np.sqrt(2);
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def VecinoInferior(F,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i+1;
    s = j;
    k = Nodo(r,s,F);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20;
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def VecinoInferiorIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i+1;
    s = j-1;
    k = Nodo(r,s,F);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20*np.sqrt(2);
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod


def VecinoIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i;
    s = j-1;
    k = Nodo(r,s,F);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20;
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def VecinoSuperiorIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i-1;
    s = j-1;
    k = Nodo(r,s,F);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20*np.sqrt(2);
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def GenData_ADPDic(DataFile,DataDang):
    
    Data = pd.read_csv(DataFile,sep=',',comment='#').values; #Cargar matriz de datos
    Data_P = pd.read_csv(DataDang,sep=',',comment='#').values; #Cargar matriz peligro
    
    F = len(Data); #Cantidad de filas
    C = len(Data[1]); #Cantidad de columnas
    A = np.zeros((F*C,F*C)); #Matriz de adyacencia
    D = np.zeros((F*C,F*C)); #Matriz de distancias
    P = np.zeros((F*C,F*C)); #Matriz de peligrosidad
    Net = {}; #Crea el diccionario vacio
    
    for i in range(F):
        for j in range(C):
            if Data[i][j] != 0:
                n = Nodo(i,j,F); #Nodo principal en análisis
                Nod = {}; #Crea el primer nodo vacio (subdiccionario)
                if i==0:
                    if j==0: #Esquina superior izquierda
                        [A,D,P,Nod] = VecinoDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferiorDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferior(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    elif j==(C-1): #Esquina superior derecha
                        [A,D,P,Nod] = VecinoIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferiorIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferior(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    else: #Borde superior
                        [A,D,P,Nod] = VecinoIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferiorIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferior(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferiorDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                elif i==(F-1):
                    if j == 0: #Esquina inferior izquierda
                        [A,D,P,Nod] = VecinoSuperior(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoSuperiorDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    elif j == (C-1): #Esquina inferior derecha
                        [A,D,P,Nod] = VecinoSuperiorIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoSuperior(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    else: #Borde inferior
                        [A,D,P,Nod] = VecinoSuperiorIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoSuperior(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoSuperiorDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                elif j == 0 and i>0 and i<(F-1): #Borde ziquierdo
                    [A,D,P,Nod] = VecinoSuperior(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoSuperiorDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferior(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferiorDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                elif j ==(C-1) and i>0 and i<(F-1): #Borde derecho
                    [A,D,P,Nod] = VecinoSuperiorIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoSuperior(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferiorIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferior(F,i,j,n,A,D,P,Nod,Data,Data_P);
                else: #Elementos del centro
                    [A,D,P,Nod] = VecinoSuperiorIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoSuperior(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoSuperiorDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferiorIzquierdo(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferior(F,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferiorDerecho(F,i,j,n,A,D,P,Nod,Data,Data_P);
                Net = {**Net, n:Nod};
                Nod = {};
    
    with open('A.csv', 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(A)
        
    with open('D.csv', 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(D)
        
    with open('P.csv', 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(P)
        
    with open('Dic.json','w') as f:
        json.dump(Net, f, sort_keys=True)
        
    return
    

#Esto hay que comentarlo, es lo que se debe llamar desde otro archivo,
#enviando los nombres de los csv correspondientes y haciendo
#import ADPDic_Data
#ADPDic_Data.GenData_ADPDic(DataFile,DataDang)
#DataFile = 'DataPrueba.csv';
DataDang = 'MatrizPeligro.csv';

path_pruebas = "../Processing/mapeo/1_robot/5-"
archivo_matriz_dividida = "divided_color_matrix"
for x in range(1,11):
    while True:
        try:
            path = path_pruebas + str(x)
            GenData_ADPDic(DataFile,DataDang);
        except FileNotFoundError:
            break

#Matrices de prueba
# Data = [[0.0, 0.0, 0.0, 0.0, 0.0],
# [0.0, 0.0, 0.025, 0.39, 0.11],
# [0.0, 0.0, 0.41, 0.9025, 0.7675],
# [0.0, 0.0, 0.6525, 0.9025, 0.9025],
# [0.0, 0.0175, 0.87, 0.9025, 0.9025]];
# Data_P = [[0.0, 0.0, 0.0, 0.0, 0.0],
# [0.0, 0.0, 0.025, 0.39, 0.11],
# [0.0, 0.0, 0.41, 0.9025, 0.7675],
# [0.0, 0.0, 0.6525, 0.9025, 0.9025],
# [0.0, 0.0175, 0.87, 0.9025, 0.9025]];

#with open('Dic.json') as fid:
#    data = json.load(fid)