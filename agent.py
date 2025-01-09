'''
Diagnóstico 09/01/25

Fase 1 - Estático

O robô consegue desviar na maioria das vezes. O maior problema é que as vezes ele se desvia de um robô a sua frente, e logo após tem outro
e isso faz com que ele se perca e acabe batendo no obstáculo da frente, a mesma coisa acontece quando ele vai rápido até o alvo, chega um
momento em que ele se aproxima do robô, nem tenta desviar, e termina batendo.

Outra situação é que quando os ângulos chegam a um certo ponto de equidade, o robô começa a ir pra frente e voltar sequenciamente, e não escapa
desse ciclo, então é como se ele tentasse desviar, e depois fosse até o alvo. Também acontece quando a bola está junto com um robô oponente.

Ademais, os próximos objetivos são impedir que ele tenha qualquer contato com outro robô, e que foque sempre em chegar ao alvo, mesmo que outro robô esteja lá.
'''

import math
import random
from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point

class ExampleAgent(BaseAgent):
    MAX_VELOCITY = 1.5  # Definindo a velocidade máxima
    HIGH_VELOCITY = 3.0  # Velocidade alta para afastamento rápido
    REVERSE_VELOCITY = -1.0  # Velocidade de ré

    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.target = None  # Armazena o alvo escolhido

    def decision(self):
        if len(self.targets) == 0:
            return

        # Encontrar o alvo mais próximo ou o mais adequado com base nos obstáculos
        self.target = self.choose_target()

        if self.target is None:
            return

        # Calcular a velocidade desejada para atingir o ponto alvo
        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.target)

        # Verificar se há robôs no caminho
        avoidance_velocity = self.avoid_obstacles()

        # Se houver risco de colisão, ajusta a velocidade
        if avoidance_velocity:
            target_velocity = avoidance_velocity

        # Verificar se o robô está parado e, se estiver, dar ré
        if self.get_magnitude(target_velocity) < 0.1:  # Se a velocidade for muito baixa
            self.reverse_movement()
            return

        # Definir velocidades calculadas para o robô
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

    def get_magnitude(self, point):
        # Calcular a magnitude de um vetor (Pitagoras)
        return math.sqrt(point.x ** 2 + point.y ** 2)

    def choose_target(self):
        closest_target = None
        min_distance = float('inf')
        best_robot_count = float('inf')

        for target in self.targets:
            # Calcular a distância entre o agente e o alvo
            distance = self.pos.dist_to(target)

            # Contabilizar os obstáculos no caminho até o alvo
            obstacles_in_path = self.count_obstacles_in_path(target)

            # Escolher o alvo mais próximo ou, em caso de empate, o que tem menos obstáculos
            if distance < min_distance or (distance == min_distance and obstacles_in_path < best_robot_count):
                min_distance = distance
                best_robot_count = obstacles_in_path
                closest_target = target

        return closest_target

    def count_obstacles_in_path(self, target):
        # Contabilizar o número de obstáculos (robôs) no caminho entre o agente e o alvo
        obstacles_count = 0
        for opponent_id, opponent in self.opponents.items():
            if self.is_opponent_in_path(opponent, target):
                obstacles_count += 1
        return obstacles_count

    def is_opponent_in_path(self, opponent, target):
        # Calcular o ângulo do robô até o alvo
        angle_to_target = math.atan2(target.y - self.pos.y, target.x - self.pos.x)

        # Calcular o ângulo do robô até o oponente
        angle_to_opponent = math.atan2(opponent.y - self.pos.y, opponent.x - self.pos.x)

        # Calcular a diferença entre os ângulos
        angle_diff = abs(angle_to_target - angle_to_opponent)

        # Verificar se o oponente está no caminho direto entre o robô e o alvo
        return angle_diff < math.radians(40)  # Margem de 45 graus

    def avoid_obstacles(self):
        avoidance_velocity = None
        min_distance_to_obstacle = 0.5

        for opponent_id, opponent in self.opponents.items():
            # Calcular a distância entre o robô e o oponente
            distance = self.pos.dist_to(Point(opponent.x, opponent.y))

            # Se a distância for menor que o limite de segurança
            if distance <= min_distance_to_obstacle:
                # Se a distância for muito pequena, aumentar a velocidade de desvio
                if distance < 0.2:
                    avoidance_velocity = self.calculate_avoidance_vector(opponent, high_velocity=True)
                    self.reverse_movement()
                else:
                    # Verificar se o oponente está no caminho entre o robô e o alvo
                    if self.is_opponent_in_path(opponent, self.target):
                        # Calcular o vetor de desvio
                        avoidance_velocity = self.calculate_avoidance_vector(opponent)

                # Se um vetor de desvio for calculado, não precisamos verificar mais oponentes
                if avoidance_velocity:
                    break

        return avoidance_velocity

    def calculate_avoidance_vector(self, opponent, high_velocity=False):
        # Calcular o ângulo até o oponente
        angle_to_opponent = math.atan2(opponent.y - self.pos.y, opponent.x - self.pos.x)

        # Adicionar um desvio de 90 graus para evitar o oponente
        avoidance_angle = angle_to_opponent + (math.pi / 2 if self.pos.x < opponent.x else -math.pi / 2)

        # Se o afastamento for rápido, usar a alta velocidade
        velocity = self.HIGH_VELOCITY if high_velocity else self.MAX_VELOCITY

        # Criar o vetor de desvio com a velocidade desejada
        avoidance_velocity = Point(
            math.cos(avoidance_angle) * velocity,
            math.sin(avoidance_angle) * velocity
        )

        return avoidance_velocity

    def reverse_movement(self):
        # Aplicar o movimento de ré
        reverse_velocity = Point(self.REVERSE_VELOCITY, 0)  # Movimento de ré no eixo x
        self.set_vel(reverse_velocity)
        self.set_angle_vel(0)  # Não precisamos de rotação enquanto o robô dá ré

    def post_decision(self):
        pass
