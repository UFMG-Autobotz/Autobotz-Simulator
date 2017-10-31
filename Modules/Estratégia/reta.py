import vetor
import math

# classe usada para armazenar equacoes de retas
class Reta():
    # inicializador, cria uma reta a partir de dois pontos
    def __init__(self, p1, p2):
        self.a, self.b, self.c = self.equacao(p1, p2)

    # cira uma reta a partir das coordenadas de dois pontos
    @classmethod
    def coordenadas(self, p1_x, p1_y, p2_x, p2_y):
        p1 = vetor.Vetor(p1_x, p1_y)
        p2 = vetor.Vetor(p2_x, p2_y)
        return Reta(p1, p2)

    # cria uma reta a partir de um ponto e um angulo
    @classmethod
    def ponto_e_angulo(self, p1, theta):
        p2 = vetor.Vetor(p1.x + math.cos(theta), p1.y + math.sin(theta))
        return Reta(p1, p2)

    # calcula a equacao da reta na forma: a*x + b*y + c = 0
    def equacao(self, p1, p2):
        if p1.x != p2.x:
            # y = m*x + q
            m = float(p2.y - p1.y) / (p2.x - p1.x)
            q = p1.y - p1.x*m
            return (m, -1., q)
        else:
            return (1, 0, -p1.x)

    # imprime equacao da reta
    def imprime_equacao(self):
        print '(' + repr(self.a) + ')*x + (' + repr(self.b) + ")*y + (" + repr(self.c) + ") = 0"

    # imprime informacoes sobre a reta
    def get_info(self):
        print 'a = ' + repr(self.a)
        print 'b = ' + repr(self.b)
        print 'c = ' + repr(self.c)

    # encontra y dado um x
    def encontra_y(self, x):
        if (self.e_funcao_de_x()):
            return self.a * x + self.c
        else:
            return None

    # verifica se a equacao da reta e uma funcao de x
    def e_funcao_de_x(self):
        if (self.b != 0):
            return True
        else:
            return False
