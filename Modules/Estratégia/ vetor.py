import math

# classe usada para armazenar grandezas vetoriais e realizar operacoes envolvendo essas grandezas
class Vetor():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # cria um vetor unitario a partir de um angulo
    @classmethod
    def unitario(self, theta):
        return Vetor(math.cos(theta), math.sin(theta))

    # imprime informacoes sobre o vetor
    def get_info(self):
        print 'Coordenadas: (' + repr(self.x) + ', ' + repr(self.y) + ')'

    # modifica as coordenadas do vetor
    def set(self, x, y):
        self.x = x
        self.y = y

    # verifica se o vetor e diferente de um outor vetor
    def diferente(self, v):
        return self.x != v.x or self.y != v.y

    # calcula o modulo do vetor
    def tamanho(self):
        return math.sqrt(self.x**2 + self.y**2)

    # calcula o modulo do vetor ao quadrado
    def quadrado(self):
        return self.x**2 + self.y**2

    # calcula a distancia com relacao a outro vetor v
    def distancia(self, v):
        return math.sqrt((self.x - v.x)**2 + (self.y - v.y)**2)

    # retorna a orientacao do vetor
    def angulo(self):
        return math.atan2(self.y, self.x)

    # calcula produto escalar do vetor com outro vetor recebido
    def produto_escalar(self, v):
        return self.x*v.x + self.y*v.y

    # calcula o angulo que o vetor forma com outro vetor recebido
    def angulo_com(self, v):
        modulo_self = self.tamanho()
        modulo_v = v.tamanho()
        if (modulo_self != 0 and modulo_v != 0):
            return math.acos( self.produto_escalar(v) / (modulo_self*modulo_v) )
        else:
            return None

    # calcula a coordenada z do produto vetorial com outro vetor no plano xy
    def produto_vetorial(self, v):
        return self.x*v.y - self.y*v.x

    # calcula o angulo com outro vetor levando em conta se foi girado no sentido horario ou anti-horario
    def delta_angulo_com(self, v):
        modulo_self = self.tamanho()
        modulo_v = v.tamanho()
        produto = self.produto_vetorial(v)

        if (produto == 0 and modulo_self != 0 and modulo_v != 0):
            if (self.angulo() == v.angulo()): # paralelos, mesma direcao
                return 0    # angulo e zero
            return math.pi  # parelelos, direcao contraria, angulo e pi

        if (modulo_self != 0 and modulo_v != 0):
            return (produto / abs(produto)) * Vetor.angulo_com(self, v)

        return None

    # transforma o vetor em um vetor unitario
    def normaliza(self):
        modulo = self.tamanho()
        self.x /= modulo
        self.y /= modulo

    # retorna um vetor igual a soma de dois vetores
    @classmethod
    def soma(self, v1, v2):
        return Vetor(v1.x + v2.x, v1.y + v2.y)

    # retorna um vetor igual a subtracao de dois vetores
    @classmethod
    def subtrai(self, v1, v2):
        return Vetor(v1.x - v2.x, v1.y - v2.y)

    # multiplica um vetor por um escalar
    @classmethod
    def multiplica(self, v, escalar):
        return Vetor(escalar * v.x, escalar * v.y)

    # retorna um vetor igual ao vetor v girado de theta
    @classmethod
    def gira(self, v, theta):
        cosseno = math.cos(theta)
        seno = math.sin(theta)

        x = v.x*cosseno - v.y*seno
        y = v.x*seno + v.y*cosseno

        return Vetor(x, y)
