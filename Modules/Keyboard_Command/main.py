#!/usr/bin/python
# -*- coding: utf-8 -*-

from PyQt4 import QtGui
from PyQt4 import QtCore

import sys
import numpy as np
import yaml

import rospy
from std_msgs.msg import Float32

class Window(QtGui.QWidget):
    def __init__(self, config):
        super(Window, self).__init__()

        self.loadConfig(config)

        self.velR = self.velL = 0
        self.keys = np.array([0, 0, 0, 0])
        self.initUI();

    def loadConfig(self, config):
        with open(config, 'r') as f:
            dados = yaml.load(f)

        self.vl = dados['VelLinha']
        self.vc = dados['VelCurva']

        self.initROS(dados)

    def initROS(self, dados):
        rospy.init_node('keyboardControl', anonymous=True)

        self.pubL = []
        for topic in dados['Esquerda']:
            self.pubL.append(rospy.Publisher(topic, Float32, queue_size = 100));

        self.pubR = []
        for topic in dados['Direita']:
            self.pubR.append(rospy.Publisher(topic, Float32, queue_size = 100));


    def initUI(self):
        self.displayVelL = QtGui.QLabel('Velocidade roda esquerda: ' + str(self.velL) + ' rads/s', self)
        self.displayVelL.move(20, 25)
        self.displayVelL.resize(300, 20)

        self.displayVelR = QtGui.QLabel('Velocidade roda direita ' + str(self.velR) + ' rads/s', self)
        self.displayVelR.move(20, 55)
        self.displayVelR.resize(300, 20)

    def keyMap(self, event, status):
        if event.key() == QtCore.Qt.Key_Up:
            self.keys[0] = status

        if event.key() == QtCore.Qt.Key_Down:
            self.keys[1] = status

        if event.key() == QtCore.Qt.Key_Left:
            self.keys[3] = status

        if event.key() == QtCore.Qt.Key_Right:
            self.keys[2] = status

        self.setVelocity()

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return

        self.keyMap(event, 1)
        event.accept()

    def setVelocity(self):
        direcao = -2*self.keys[1] + 1
        self.velL = np.dot(np.array([self.vl, -self.vl, self.vc*direcao, -self.vc*direcao]), self.keys);
        self.velR = np.dot(np.array([self.vl, -self.vl, -self.vc*direcao, self.vc*direcao]), self.keys);

        self.displayVelL.setText('Velocidade roda esquerda: ' + str(self.velL) + ' rads/s');
        self.displayVelR.setText('Velocidade roda direita: ' + str(self.velR) + ' rads/s');

        for topic in self.pubL:
            topic.publish(self.velL)

        for topic in self.pubR:
            topic.publish(self.velR)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return

        self.keyMap(event, 0)
        event.accept()

def main():
    app = QtGui.QApplication(sys.argv)

    if len(sys.argv) <= 1:
        print "Numero invalido de argumentos"
        quit()

    w = Window(sys.argv[1])
    w.resize(400, 100)
    w.move(300, 300)
    w.setWindowTitle('Simple')
    w.show()



    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
