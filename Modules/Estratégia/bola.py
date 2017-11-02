#!/usr/bin/env python
# license removed for brevity
from vetor import Vetor
import rospy
from geometry_msgs.msg import Pose

class Bola():
    diametro = 4.27 # diametro da bola em cm

    def __init__(self):
        self.pos = Vetor(0, 0)
        rospy.Subscriber("/ball/pose", Pose, self.recebe_dados)

    # recebe posicao da bola
    def recebe_dados(self, msg):
        self.pos.x = msg.position.x
        self.pos.y - msg.position.y

     # retorna posicao da bola
    def get_pos(self):
        return self.pos

     # imprime propriedades da bola
    def get_info(self):
        print 'Posicao: (' + repr(self.pos.x) + ', ' + repr(self.pos.y) + ')'
