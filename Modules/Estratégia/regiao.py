from vetor import Vetor

#classe para armazenar canto superior esquerdo e canto inferior direito de regioes
class Regiao():
    # inicializa a regiao a partir de dois vetores
    def __init__(self, supE, infD):
        self.supE = supE
        self.infD = infD

    # cria regiao a partir das coordenadas dos vetores
    @classmethod
    def coordenadas(self, supEx, supEy, infDx, infDy):
        supE = Vetor(supEx, supEy)
        infD = Vetor(infDx, infDy)
        return Regiao(supE, infD)

    # verifica se um ponto esta dentro da regiao
    def esta_dentro(self, p):
        if p.x >= self.supE.x and p.x <= self.infD.x and p.y >= self.supE.y and p.y <= self.infD.y:
            return True
        else:
           return False

    # encontra o centro da regiao
    def centro(self):
        x = (self.infD.x + self.supE.x)/2.
        y = (self.infD.y + self.supE.y)/2.
        return Vetor(x, y)
