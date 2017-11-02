#!/usr/bin/env python
# license removed for brevity
from vetor import Vetor
from reta import Reta
from bola import Bola
from campo import Campo
import math
import numpy
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32

class Robo():
    L = 0.063 # tamanho do eixo das rodas em cm
    r = 0.016 # raio das rodas em cm
    distCR = 0.0155 # distancia do centro de rotacao ao centro do robo em cm
    w = 10 # velocidade angular das rodas - determinar valor adequado
    correcaoTheta = 0

    # inicializa o objeto dadas as coordenadas da posicao, a orientacao e o numero do jogador
    def __init__(self):

        self.orientacao = Vetor(0, 0)
        self.pos = Vetor(0, 0)
        self.adversario = Vetor(0,0)

        rospy.Subscriber("/player1/pose", Pose, self.recebe_dados)
        rospy.Subscriber("/player2/pose", Pose, self.recebe_dados_adversario)

        self.vel = 0 #velocidade linear (inicializada como zero)
        self.omega = 0 #velocidade angular (inicializada como zero)

        #self.thetaDesejado = 0
        self.orientacaoDesejada = Vetor(0, 0)
        self.posDesejada = Vetor(0, 0)

        #self.thetaPrevisto = 0
        self.orientacaoPrevista = Vetor(0, 0)
        self.posPrevista = Vetor(0, 0)

        self.omegaD = 0 # velocidade angular da roda direita
        self.omegaE = 0 # velocidade angular da roda esquerda

        self.lado = 0

        self.envia_dados()


    # retorna a posicao do robo
    def get_pos(self):
        return self.pos


     # retorna a orientacao do robo
    def get_theta(self):
        return self.orientacao.angulo()


     # recebe posicao e orientacao da visao
    def recebe_dados(self, msg):
    	self.pos.x = msg.position.x
    	self.pos.y = msg.position.y

    	q = msg.orientation
    	yaw = math.atan2((2 * q.w * q.z ),(1 - 2 * q.z * q.z))

    	self.orientacao.x = math.cos(yaw)
    	self.orientacao.y = math.sin(yaw)

    #recebe posicao do adversario
    def recebe_dados_adversario(self, msg):
    	self.adversario.x = msg.position.x
    	self.adversario.y = msg.position.y


    def envia_dados(self):
        campo = Campo()
        bola = Bola()
    	self.encontra_posicao_goleiro(campo, bola)
    	self.encontra_velocidades()
        omegaE = self.omegaE
        omegaD = self.omegaD
    	pubR = rospy.Publisher('/player1__player__chassi_right_wheel/joint_vel', Float32, queue_size = 100)
    	pubL = rospy.Publisher('/player1__player__chassi_left_wheel/joint_vel', Float32, queue_size = 100)
    	rate = rospy.Rate(10)
    	while not rospy.is_shutdown():
            pubR.publish(omegaD)
            pubL.publish(omegaE)
            rate.sleep()


   #encontra posicao e orientacao desejadas para goleiro
    def encontra_posicao_goleiro(self, campo, bola):
        adversario_pose = self.adversario

        self.orientacaoDesejada = Vetor.unitario(math.pi/2) # orientacao paralela a linha do gol
        self.posDesejada.x = campo.get_gol(self.lado).centro().x # no eixo x, centro do gol

        reta = Reta(adversario_pose, bola.get_pos()) #reta que liga o adversario a bola
        if (reta.e_funcao_de_x()): # se a reta for funcao de x
            self.posDesejada.y = reta.encontra_y(self.posDesejada.x) # encontra o prolongamento da reta
        else: # se a reta for paralela a linha do gol
            self.posDesejada.y = self.pos.y # nao se move no eixo y

       #garante que o goleiro fique na frente do gol
        if (self.posDesejada.y < campo.minLgol):
            self.posDesejada.y = campo.minLgol
        if (self.posDesejada.y > campo.maxLgol):
            self.posDesejada.y = campo.maxLgol

    # calcula o raio e o sentido da trajetoria
    def encontra_tragetoria(self):
        direcaoDesejada = Vetor.subtrai(self.posDesejada, self.pos) #vetor que liga a posicao atual a posicao desejada

        sentido = self.encontra_sentido(direcaoDesejada);
        raio = self.encontra_raio(direcaoDesejada);

        return (sentido, raio)


 	# encontra sentido do movimento (usado para descobrir o sentido de rotacao dos motores)
    def encontra_sentido(self, direcaoDesejada):
        if  (direcaoDesejada.tamanho()!= 0): # se o robo nao esta na posicao desejada
            ro = self.orientacao.angulo_com(direcaoDesejada) # angulo entre a orientacao do robo e o vetor direcaoDesejada

            if (ro <= math.pi/2):
                sentido = 1 # robo move para frente
            else:
                sentido = -1 # robo move para tras

        else: # se o robo ja esta na posicao desejada
            deltaTheta = self.orientacao.delta_angulo_com(self.orientacaoDesejada)

            if (deltaTheta > 0):
                sentido = 1 # robo vira para a direita sem sair do lugar (sentido horario)
            elif (deltaTheta < 0):
                sentido = -1 # robo vira para a esquerda sem sair do lugar (sentido anti-horario)
            else:
                sentido = 0 # robo nao se move

        return sentido

    def encontra_raio(self, direcaoDesejada):
        # descobre se o robo vira para esquerda ou direita
        y = Reta.ponto_e_angulo(self.pos, self.orientacao.angulo()).encontra_y(self.posDesejada.x)
        theta = self.orientacao.delta_angulo_com(direcaoDesejada)

        if (theta < 0 and theta != None):
            normal = Vetor.gira(self.orientacao, -math.pi/2) # vira no sentido anti-horario / esquerda
        elif (theta > 0):
            normal = Vetor.gira(self.orientacao, math.pi/2) # vira no sentido horario / direita
        else:
            normal = None # linha reta ou vira sem sair do lugar

        # encontra raio de curvatura
        if (normal == None):
            if (direcaoDesejada.tamanho() == 0):
                raio = 0 # vira sem sair do lugar
            else:
                raio = -1 # linha reta
        else:
            phi = normal.angulo_com(direcaoDesejada)
            raio = direcaoDesejada.quadrado()/(2*direcaoDesejada.tamanho() * math.cos(phi))

            # encontra centro
            centro = Vetor.multiplica(normal, raio)
            centro = Vetor.soma(self.pos, centro)

            # corrige raio comparando com orientacao desejada
            if (raio > 0):
               raio = self.corrige_raio(raio, centro)

        return raio


    def corrige_raio(self, raio, centro):
         # encontra variacao do angulo ate a posicao desejada
         centroPos = Vetor.subtrai(self.pos, centro)
         centroPosDesejada = Vetor.subtrai(self.posDesejada, centro)
         deltaTheta = Vetor.delta_angulo_com(centroPos, centroPosDesejada)

         # encontra diferenca entre orientacao final prevista e desejada
         orientacaoPrevista = Vetor.gira(self.orientacao, deltaTheta)
         erroTheta = orientacaoPrevista.delta_angulo_com(self.orientacaoDesejada)

         if (deltaTheta < 0): # sentido anti-horario
             raio -= 0.5*raio * ( erroTheta / math.pi ) # fracao do raio * fracao do erro do angulo
         else: # sentido horario
             raio += 0.5*raio * ( erroTheta / math.pi )

         return raio


    # encontra velocidades linear e angular e velocidades das rodas
    def encontra_velocidades(self):
        sentido, raio = self.encontra_tragetoria();

        if (raio > 0): # curva
            razaoDE = (2*raio + Robo.L)/(2*raio - Robo.L) # omegaD / omegaE
            if (razaoDE > 1): # curva para a esquerda
                self.omegaD = sentido * Robo.w # roda direita a velocidade maxima
                self.omegaE = sentido * self.omegaD/razaoDE # roda esquerda a uma velocidade menor
            else: # curva para a direita
                self.omegaE = sentido * Robo.w # roda esquerda a velocidade maxima
                self.omegaD = sentido * self.omegaE*razaoDE # roda direita a uma velocidade menor

        elif (raio < 0): # reta
            # as duas rodas a velocidade maxima
            self.omegaD = sentido * Robo.w
            self.omegaE = sentido * Robo.w

        else: # roda sem sair do lugar
            self.omegaD = -1 * sentido * Robo.w
            self.omegaE = sentido * Robo.w

        self.vel = (self.omegaE + self.omegaD)*Robo.r/2
        self.omega = (self.omegaE - self.omegaD)*Robo.r/Robo.L
