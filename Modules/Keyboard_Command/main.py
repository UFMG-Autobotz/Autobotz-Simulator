#!/usr/bin/python
# -*- coding: utf-8 -*-

from PyQt4 import QtGui
from PyQt4 import QtCore
import sys
import math
import numpy

class Window(QtGui.QWidget):

    def __init__(self, arg):
        super(Window, self).__init__()

        self.vl = arg[0]
        self.va = arg[1]

        self.velR = self.velL = 0
        self.keys = numpy.array([0, 0, 0, 0])
        self.initUI();

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
        self.velL = numpy.dot(numpy.array([self.vl, -self.vl, self.va*direcao, -self.va*direcao]), self.keys);
        self.velR = numpy.dot(numpy.array([self.vl, -self.vl, -self.va*direcao, self.va*direcao]), self.keys);

        self.displayVelL.setText('Velocidade roda esquerda: ' + str(self.velL) + ' rads/s');
        self.displayVelR.setText('Velocidade roda direita: ' + str(self.velR) + ' rads/s');

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return

        self.keyMap(event, 0)
        event.accept()

def argParser(args):
    lista = [0, 0]

    if len(args) > 1:
        lista[0] = float(args[1])
    else:
        lista[0] = 10

    if len(args) > 2:
        lista[1] = float(args[1])
    else:
        lista[1] = 0.25*lista[0]

    return lista

def main():
    app = QtGui.QApplication(sys.argv)

    w = Window(argParser(sys.argv))
    w.resize(400, 100)
    w.move(300, 300)
    w.setWindowTitle('Simple')
    w.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
