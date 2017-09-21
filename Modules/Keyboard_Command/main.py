#!/usr/bin/python
# -*- coding: utf-8 -*-

from PyQt4 import QtGui
from PyQt4 import QtCore
import sys

import numpy as np

import yaml

import rospy
from std_msgs.msg import Float32

# --------------------- #

class Window(QtGui.QWidget):
    def __init__(self, config):
        super(Window, self).__init__()

        self.loadConfig(config)

        self.velR = self.velL = 0
        self.keys = np.array([0, 0, 0, 0]) # [up, down, left, rigth], 1 when pressed

        self.initUI();

    # load .yaml file and set configuration data (called from constructor)
    def loadConfig(self, config):
        try:
            with open(config, 'r') as f:
                data = yaml.load(f)
        except:
            print "Error: Invalid configuration file!"
            quit()

        self.velS = data['VelStraight']
        self.velC = data['VelCurve']

        self.initROS(data)

    # start initialize ros and create publisher for left and right sides (called from loadConfig)
    def initROS(self, data):
        rospy.init_node('keyboardControl', anonymous=True)

        self.pubL = []
        for topic in data['Left']:
            self.pubL.append(rospy.Publisher(topic, Float32, queue_size = 100));

        self.pubR = []
        for topic in data['Right']:
            self.pubR.append(rospy.Publisher(topic, Float32, queue_size = 100));

    # initialie interface elements (called from constructor)
    def initUI(self):
        self.displayVelL = QtGui.QLabel('Left wheel speed: ' + str(self.velL) + ' rads/s', self)
        self.displayVelL.move(20, 25)
        self.displayVelL.resize(250, 20)

        self.displayVelR = QtGui.QLabel('Velocidade roda direita ' + str(self.velR) + ' rads/s', self)
        self.displayVelR.move(20, 55)
        self.displayVelR.resize(250, 20)

    # called each time a key is pressed
    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return

        self.keyMap(event, 1)
        event.accept()

    # called each time a key is released
    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return

        self.keyMap(event, 0)
        event.accept()

    # save each arrow keys are pressed (called from keyPressEvent and keyReleaseEvent)
    def keyMap(self, event, status):
        if event.key() == QtCore.Qt.Key_Up:
            self.keys[0] = status
        elif event.key() == QtCore.Qt.Key_Down:
            self.keys[1] = status
        elif event.key() == QtCore.Qt.Key_Left:
            self.keys[3] = status
        elif event.key() == QtCore.Qt.Key_Right:
            self.keys[2] = status

        self.calcVelocity()

    # determine velocity of left an right wheels according to the keys being pressed (called form keyMap)
    def calcVelocity(self):
        direction = -2*self.keys[1] + 1 # used to deal with backwards motion, -1 when down is pressed 1 otherwise

        self.velL = np.dot(np.array([self.velS, -self.velS, self.velC*direction, -self.velC*direction]), self.keys);
        self.velR = np.dot(np.array([self.velS, -self.velS, -self.velC*direction, self.velC*direction]), self.keys);

        self.displayVelL.setText('Left wheel speed: ' + str(self.velL) + ' rads/s');
        self.displayVelR.setText('Right wheel speed: ' + str(self.velR) + ' rads/s');

        for topic in self.pubL:
            topic.publish(self.velL)

        for topic in self.pubR:
            topic.publish(self.velR)

# --------------------- #

def main():
    app = QtGui.QApplication(sys.argv)

    # quit if a config file is not sent
    if len(sys.argv) <= 1:
        print "Error: Invalid number of arguments!"
        quit()

    w = Window(sys.argv[1])
    w.resize(300, 100)
    w.move(300, 300)
    w.setWindowTitle('Keyboard Control')
    w.show()

    sys.exit(app.exec_())

# --------------------- #

if __name__ == '__main__':
    main()
