from regiao import Regiao
from vetor import Vetor

# classe para armazenar as regioes: campo, gol esquerdo e gol direito
# dimensoes em cm
class Campo():

    def __init__(self):
        self.campo_completo = Regiao.coordenadas(-0.695075, 0.585919, 0.695075, -0.585919)
        self.lado = [Regiao.coordenadas(-0.695075, 0.585919, -0.002881, -0.585919), Regiao.coordenadas(-0.002881, 0.585919, 0.695075, -0.585919)]
        self.areaGol = [Regiao.coordenadas(-0.705899, 0.327261, -0.5984889, -0.327261), Regiao.coordenadas(0, 0, 0, 0)]

        self.minLgol = -0.327261
        self.maxLgol = 0.327261

    # retorna gol dado o lado do campo (0 ou 1)
    def get_gol(self, lado):
        return self.areaGol[lado]
