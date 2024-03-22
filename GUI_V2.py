# Importar bibliotecas

import customtkinter as ctk
from PIL import Image, ImageTk
import cv2
import serial
import time
import math
import numpy as np

#Inicia comunicação serial

porta_serial = serial.Serial('COM5', 9600)
    
time.sleep(2)

def forwardKinematics(ang1, ang2, ang3, ang4, ang5, ang6):

    # Sistemas de Coordenadas (SC)

    SC_O = np.array([0, 0, 0]) # Sistema de Coordenadas Origem
    SC_J1 = np.array([0, 0, 64]) # Localização do SC J1 em relação a SC Origem
    SC_J2 = np.array([0, 0, 111]) # Localização do SC J2 em relação a SC J1
    SC_J3 = np.array([0, 0, 163.619]) # Localização do SC J3 em relação ao SC J2
    SC_J4 = np.array([28.1, -104.765, 0]) # Localização do SC J4 em relação ao SC J3
    SC_J5 = np.array([0, 0, 246.15]) # Localização do SC J5 em relação ao SC J4
    SC_J6 = np.array([27.6, 0, 0]) # Localização do SC J6 em relação ao SC J5
    SC_E = np.array([0, 0+1.25, 97.5-3.7]) # Localização do Efetuador em relação ao SC J6 Vetor verdadeiro [0, 0, 97.5]

    # DH Parameters [alpha, theta, a, d]

    DH_J1 = np.array([math.radians(0), 0, 0, 64])
    DH_J2 = np.array([math.radians(-90), 0, 0, 111])
    DH_J3 = np.array([math.radians(0), 0, 163.619, 0])
    DH_J4 = np.array([math.radians(90), 0, 28.1, 104.765])
    DH_J5 = np.array([math.radians(-90), 0, 246.15, 0])
    DH_J6 = np.array([math.radians(90), 0, 27.6, 0])
    DH_E = np.array([math.radians(0), 0, 97.5, 0])

    ang1 = math.radians(ang1)
    ang2 = math.radians(ang2)
    ang3 = math.radians(ang3)
    ang4 = math.radians(ang4)
    ang5 = math.radians(ang5)
    ang6 = math.radians(ang6)


    # Matriz T_J1

    TransZ_J1 = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, DH_J1[3]],
                        [0, 0, 0, 1]])

    RotZ_J1 = np.array([[math.cos(ang1), -math.sin(ang1), 0, 0],
                        [math.sin(ang1), math.cos(ang1), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

    TransX_J1 = np.array([[1, 0, 0, DH_J1[2]],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

    RotX_J1 = np.array([[1, 0, 0, 0],
                        [0, math.cos(DH_J1[0]), -math.sin(DH_J1[0]), 0],
                        [0, math.sin(DH_J1[0]), math.cos(DH_J1[0]), 0],
                        [0, 0, 0, 1]])


    Z_J1 = np.matmul(TransZ_J1, RotZ_J1)

    X_J1 = np.matmul(TransX_J1, RotX_J1)

    T_J1 = np.matmul(Z_J1,X_J1)

    # Matriz T_J2 NÃO FAÇO A MENOR IDEIA, MAS FUNCIONA

    T_J2 = np.array([[math.cos(ang2), -math.sin(ang2), 0, math.cos(DH_J2[0])*DH_J2[2]],
                    [math.sin(ang2)*math.cos(DH_J2[0]), math.cos(ang2)*math.cos(DH_J2[0]), -math.sin(DH_J2[0]), math.sin(DH_J2[0])*DH_J2[2]],
                    [math.sin(ang2)*math.sin(DH_J2[0]), math.cos(ang2)*math.sin(DH_J2[0]), math.cos(DH_J2[0]), DH_J2[3]],
                    [0, 0, 0, 1]])


    # Matriz T_J3 - Foi no feeling 

    T_J3 = np.array([[math.cos(ang3), -math.sin(ang3), 0, 0],
                    [math.sin(ang3)*math.cos(DH_J3[0]), math.cos(ang3)*math.cos(DH_J3[0]), -math.sin(DH_J3[0]), -DH_J3[2]],
                    [math.sin(ang3)*math.sin(DH_J3[0]), math.cos(ang3)*math.sin(DH_J3[0]), math.cos(DH_J3[0]), DH_J3[3]],
                    [0, 0, 0, 1]])

    # Matriz T_J4 

    TransZ_J4 = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

    RotZ_J4 = np.array([[1, 0, 0, 0],
                        [0, math.cos(ang4), -math.sin(ang4), 0],
                        [0, math.sin(ang4), math.cos(ang4), 0],
                        [0, 0, 0, 1]])

    TransY_J4 = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

    RotY_J4 = np.array([[math.cos(DH_J4[0]), 0, math.sin(DH_J4[0]), 0],
                        [0, 1, 0, 0],
                        [-math.sin(DH_J4[0]), 0, math.cos(DH_J4[0]), 0],
                        [0, 0, 0, 1]])

    Z_J4 = np.matmul(TransZ_J4, RotZ_J4)

    Y_J4 = np.matmul(TransY_J4, RotY_J4)

    T_J4 = np.matmul(Z_J4,Y_J4)

    T_J4[(0,3)] = SC_J4[0]
    T_J4[(1,3)] = SC_J4[1]
    T_J4[(2,3)] = SC_J4[2]

    # Matriz T_J5

    RotZ_J5 = np.array([[1, 0, 0, 0],
                        [0, math.cos(ang5), -math.sin(ang5), 0],
                        [0, math.sin(ang5), math.cos(ang5), 0],
                        [0, 0, 0, 1]])

    RotY_J5 = np.array([[math.cos(DH_J5[0]), 0, math.sin(DH_J5[0]), 0],
                        [0, 1, 0, 0],
                        [-math.sin(DH_J5[0]), 0, math.cos(DH_J5[0]), 0],
                        [0, 0, 0, 1]])


    T_J5 = np.matmul(RotZ_J5, RotY_J5)

    T_J5[(0,3)] = SC_J5[0]
    T_J5[(1,3)] = SC_J5[1]
    T_J5[(2,3)] = SC_J5[2]


    # Matriz T_J6

    RotZ_J6 = np.array([[1, 0, 0, 0],
                        [0, math.cos(ang6), -math.sin(ang6), 0],
                        [0, math.sin(ang6), math.cos(ang6), 0],
                        [0, 0, 0, 1]])

    RotY_J6 = np.array([[math.cos(DH_J6[0]), 0, math.sin(DH_J6[0]), 0],
                        [0, 1, 0, 0],
                        [-math.sin(DH_J6[0]), 0, math.cos(DH_J6[0]), 0],
                        [0, 0, 0, 1]])


    T_J6 = np.matmul(RotZ_J6, RotY_J6)

    T_J6[(0,3)] = SC_J6[0]
    T_J6[(1,3)] = SC_J6[1]
    T_J6[(2,3)] = SC_J6[2]


    # Matriz Ef

    T_E = np.array([[1, 0, 0, SC_E[0]],
                    [0, 1, 0, SC_E[1]],
                    [0, 0, 1, SC_E[2]],
                    [0, 0, 0, 1]])


    T_12 = np.matmul(T_J1, T_J2)
    T_34 = np.matmul(T_J3, T_J4)
    T_56 = np.matmul(T_J5, T_J6)

    T = np.matmul(T_12, T_34)
    T = np.matmul(T, T_56)
    T = np.matmul(T, T_E)


    P_J = np.array([[0], [0], [0], [1]]) # Coordenadas do ponto em relação ao Sistema de Coordenadas J2

    P_O = np.matmul(T, P_J)

    np.set_printoptions(precision=3, suppress=True)


    return P_O


def collect(): #Função de coletar os dados inseridos pelo usuário

    J1 = int(ent14.get())
    J2 = int(ent15.get())
    J3 = int(ent16.get())
    J4 = int(ent17.get())
    J5 = int(ent18.get())
    J6 = int(ent19.get())

    J = [J1, J2, J3, J4, J5, J6]

    return J

def send(): # Função de enviar dados inseridos pelo usuário

    porta_serial.write(collect())
    
def receber_dados(): # Função de receber dados do Arduino

    if porta_serial.in_waiting > 0:

        arduino_data = porta_serial.readline().decode().strip()

        lbl20.configure(text=f"{arduino_data}")

def atualizar(): # Função de atualização da página
    receber_dados();
    janela.after(100, atualizar)


class CameraApp: # Classe que define as configurações da câmera
    def __init__(self, window, window_title, video_source=0):
        self.window = window
        self.window.title(window_title)
        
        self.vid = cv2.VideoCapture(video_source)
        
        self.canvas = ctk.CTkCanvas(window, width=self.vid.get(cv2.CAP_PROP_FRAME_WIDTH), height=self.vid.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.canvas.place(x=10, y=10)
                
        self.update()
        
        self.window.mainloop()
    
    def snapshot(self):
        ret, frame = self.vid.read()
        if ret:
            cv2.imwrite("snapshot.png", cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    
    def update(self):
        ret, frame = self.vid.read()
        if ret:
            self.photo = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
            self.canvas.create_image(0, 0, image=self.photo, anchor=ctk.NW)
        self.window.after(10, self.update)


janela = ctk.CTk() # Cria um objeto janela - GUI


# configurando a janela principal

janela.title("Manipulador Robótico")
janela.geometry("1050x700")
janela.resizable(width=True, height=True)

frm1 = ctk.CTkFrame(master = janela, width=500, height=390, bg_color="transparent", border_color="White", border_width=1)
frm1.place(x=530, y=10)

frm2 = ctk.CTkFrame(master = janela, width=500, height=100, bg_color="transparent", border_color="White", border_width=1)
frm2.place(x=530, y=410)

lbl1 = ctk.CTkLabel(master = frm1 ,text="Juntas do Manipulador")
lbl1.place(relx = 0.5, rely = 0.05, anchor='c')

lbl2 = ctk.CTkLabel(master = frm1 ,text="J1: ")
lbl2.place(relx = 0.1, rely = 0.12, anchor='c')
lbl2_2 = ctk.CTkLabel(master = frm1 ,text="360°")
lbl2_2.place(relx = 0.15, rely = 0.12, anchor='c')


lbl3 = ctk.CTkLabel(master = frm1 ,text="J2: ")
lbl3.place(relx = 0.25, rely = 0.12, anchor='c')
lbl3_2 = ctk.CTkLabel(master = frm1 ,text="360°")
lbl3_2.place(relx = 0.30, rely = 0.12, anchor='c')

lbl4 = ctk.CTkLabel(master = frm1 ,text="J3: ")
lbl4.place(relx = 0.4, rely = 0.12, anchor='c')
lbl4_2 = ctk.CTkLabel(master = frm1 ,text="360°")
lbl4_2.place(relx = 0.45, rely = 0.12, anchor='c')

lbl5 = ctk.CTkLabel(master = frm1 ,text="J4: ")
lbl5.place(relx = 0.55, rely = 0.12, anchor='c')
lbl5_2 = ctk.CTkLabel(master = frm1 ,text="360°")
lbl5_2.place(relx = 0.60, rely = 0.12, anchor='c')

lbl6 = ctk.CTkLabel(master = frm1 ,text="J5: ")
lbl6.place(relx = 0.7, rely = 0.12, anchor='c')
lbl6_2 = ctk.CTkLabel(master = frm1 ,text="360°")
lbl6_2.place(relx = 0.75, rely = 0.12, anchor='c')

lbl7 = ctk.CTkLabel(master = frm1 ,text="J6: ")
lbl7.place(relx = 0.85, rely = 0.12, anchor='c')
lbl7_2 = ctk.CTkLabel(master = frm1 ,text="360°")
lbl7_2.place(relx = 0.90, rely = 0.12, anchor='c')

lbl8 = ctk.CTkLabel(master = frm1 ,text="Posição do Manipulador")
lbl8.place(relx = 0.5, rely = 0.20, anchor='c')

lbl9 = ctk.CTkLabel(master = frm1 ,text="X: ")
lbl9.place(relx = 0.30, rely = 0.27, anchor='c')
lbl9_2 = ctk.CTkLabel(master = frm1 ,text="500")
lbl9_2.place(relx = 0.35, rely = 0.27, anchor='c')

lbl10 = ctk.CTkLabel(master = frm1 ,text="Y: ")
lbl10.place(relx = 0.45, rely = 0.27, anchor='c')
lbl10_2 = ctk.CTkLabel(master = frm1 ,text="500°")
lbl10_2.place(relx = 0.50, rely = 0.27, anchor='c')

lbl11 = ctk.CTkLabel(master = frm1 ,text="Z: ")
lbl11.place(relx = 0.60, rely = 0.27, anchor='c')
lbl12_2 = ctk.CTkLabel(master = frm1 ,text="500°")
lbl12_2.place(relx = 0.65, rely = 0.27, anchor='c')

lbl13 = ctk.CTkLabel(master = frm1 ,text="Cinemática Direta")
lbl13.place(relx = 0.50, rely = 0.35, anchor='c')

lbl14 = ctk.CTkLabel(master = frm1 ,text="J1: ")
lbl14.place(relx = 0.30, rely = 0.45, anchor='c')
ent14 = ctk.CTkEntry(master= frm1, width = 50, height = 10 ,placeholder_text="J1")
ent14.place(relx = 0.32, rely = 0.45, anchor='w')

lbl15 = ctk.CTkLabel(master = frm1 ,text="J2: ")
lbl15.place(relx = 0.50, rely = 0.45, anchor='c')
ent15 = ctk.CTkEntry(master= frm1, width = 50, height = 10 ,placeholder_text="J2")
ent15.place(relx = 0.52, rely = 0.45, anchor='w')

lbl16 = ctk.CTkLabel(master = frm1 ,text="J3: ")
lbl16.place(relx = 0.70, rely = 0.45, anchor='c')
ent16 = ctk.CTkEntry(master= frm1, width = 50, height = 10 ,placeholder_text="J3")
ent16.place(relx = 0.72, rely = 0.45, anchor='w')

lbl17 = ctk.CTkLabel(master = frm1 ,text="J4: ")
lbl17.place(relx = 0.30, rely = 0.60, anchor='c')
ent17 = ctk.CTkEntry(master= frm1, width = 50, height = 10 ,placeholder_text="J4")
ent17.place(relx = 0.32, rely = 0.60, anchor='w')

lbl18 = ctk.CTkLabel(master = frm1 ,text="J5: ")
lbl18.place(relx = 0.50, rely = 0.60, anchor='c')
ent18 = ctk.CTkEntry(master= frm1, width = 50, height = 10 ,placeholder_text="J5")
ent18.place(relx = 0.52, rely = 0.60, anchor='w')

lbl19 = ctk.CTkLabel(master = frm1 ,text="J6: ")
lbl19.place(relx = 0.70, rely = 0.60, anchor='c')
ent19 = ctk.CTkEntry(master= frm1, width = 50, height = 10 ,placeholder_text="J6")
ent19.place(relx = 0.72, rely = 0.60, anchor='w')

buttom = ctk.CTkButton(master = frm1, width= 100, height=20, text="Go", command=send)
buttom.place(relx = 0.50, rely = 0.80, anchor='c')

buttom1 = ctk.CTkButton(master = frm1, width= 100, height=20, text="Collect", command=collect)
buttom1.place(relx = 0.50, rely = 0.90, anchor='c')



lbl20 = ctk.CTkLabel(master = frm2 ,text="Comunicação Arduino")
lbl20.place(relx = 0.5, rely = 0.5, anchor='c')

atualizar()

app = CameraApp(janela, "Manipulador Robótico") # Função de iniciação da câmera. Qualquer código inserido abaixo não é lido


janela.mainloop()