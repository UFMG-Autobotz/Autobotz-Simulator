#!/usr/bin/python
# -*- coding: utf-8 -*-

from PyQt4 import QtGui
from PyQt4 import QtCore
import sys

class Window(QtGui.QWidget):

    def __init__(self, arg):
        super(Window, self).__init__()

        self.velLinear = arg[0]
        self.velAngular = arg[1]

        self.velR = self.velL = 0

        self.initIU();

    def initIU(self):
        self.displayVelL = QtGui.QLabel('Velocidade roda esquerda: ' + str(self.velL) + ' rads/s', self)
        self.displayVelL.move(20, 25)
        self.displayVelL.resize(300, 20)

        self.displayVelR = QtGui.QLabel('Velocidade roda direita ' + str(self.velR) + ' rads/s', self)
        self.displayVelR.move(20, 55)
        self.displayVelR.resize(300, 20)


    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return

        key = event.key()
        left = key == QtCore.Qt.Key_Left
        right = key == QtCore.Qt.Key_Right
        up = key == QtCore.Qt.Key_Up
        down = key == QtCore.Qt.Key_Down

        if up:
            self.velR += self.velLinear
            self.velL += self.velLinear
        elif down:
            self.velR -= self.velLinear
            self.velL -= self.velLinear
        elif right:
            self.velR -= self.velAngular
            self.velL += self.velAngular
        elif left:
            self.velR += self.velAngular
            self.velL -= self.velAngular

        if (up or down or right or left):
            self.displayVelL.setText('Velocidade roda esquerda: ' + str(self.velL) + ' rads/s');
            self.displayVelR.setText('Velocidade roda direita: ' + str(self.velR) + ' rads/s');

        event.accept()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return

        key = event.key()
        left = key == QtCore.Qt.Key_Left
        right = key == QtCore.Qt.Key_Right
        up = key == QtCore.Qt.Key_Up
        down = key == QtCore.Qt.Key_Down

        if up:
            self.velR -= self.velLinear
            self.velL -= self.velLinear
        elif down:
            self.velR += self.velLinear
            self.velL += self.velLinear
        elif right:
            self.velR += self.velAngular
            self.velL -= self.velAngular
        elif left:
            self.velR -= self.velAngular
            self.velL += self.velAngular

        if (up or down or right or left):
            self.displayVelL.setText('Velocidade roda esquerda: ' + str(self.velL) + ' rads/s');
            self.displayVelR.setText('Velocidade roda direita: ' + str(self.velR) + ' rads/s');

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
