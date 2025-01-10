'''
Diagnóstico 10/01/25

Fase 1 - Estático

O código aumentou um pouco. Com a pesquisa de documentos e artigos para auxílio, encontrei a tática de trajetórias bang bang.
Explicando resumidamente, o robô acelera até o máximo em direção ao alvo, mas quando ele se encontra próximo a unm alvo, começa a desacelerar.

No código anterior, nosso robô se encontrava nessa situação de bater em outros robôs quando estava em direção ao alvo.
Resolvi tornar a tática bang bang uma carta na manga, a aplicando no código. Conseguimos uma melhora bem notável na questão dos desvios.

Foi resolvido a questão dele ir e voltar na mesma linha pela questão de angulação notada anteriormente. Compensando o contra vetor que impedia o robô
de sair daquele ciclo infinito. Além disso, tentei adicionar uma saída pra situação em que o robô oponente compartilha coordenadas com o alvo, fazendo 
com que nosos robô tenha que colidir para chegar no alvo. Essa parte ainda está pouco desenvolvida ainda, mas espero que possamos resolvê-la.

Por fim, espero que a próxima atualização traga mais melhorias, ainda me queixo pela situação em que ele desvia de um robô, mas acaba esquecendo de outro
que também está próxima, situações como essas se fazem difíceis, já que não consegui pensar numa lógica que ele mapeie todos os robôs a sua frente, ao mesmo tempo.
'''

import math
from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point

class ExampleAgent(BaseAgent):
    MAX_VELOCITY = 1.5
    HIGH_VELOCITY = 3.0
    REVERSE_VELOCITY = -1.0
    SAFETY_DISTANCE = 0.6  # Distância de segurança

    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.target = None
        self.current_velocity = Point(0, 0)  # Velocidade atual do robô

    def decision(self):
        if len(self.targets) == 0:
            return

        self.target = self.choose_target()
        if self.target is None:
            return

        # Verificar se o alvo está bloqueado por um oponente
        if self.is_target_blocked():
            self.target = self.find_alternate_target()

        # Verificar se o alvo está muito próximo de um robô inimigo
        if self.is_target_near_opponent(self.target):
            self.collide_with_opponent()

        # Planejamento do caminho até o alvo
        if not self.is_path_clear(self.target):
            self.target = self.find_alternate_target()

        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.target)
        avoidance_velocity = self.avoid_obstacles()

        if avoidance_velocity:
            target_velocity = avoidance_velocity
        
        # Aplicar tática Bang-Bang para ajustar a velocidade
        self.apply_bang_bang_control(target_velocity)

        if self.get_magnitude(self.current_velocity) < 0.1:
            self.reverse_movement()
            return

        self.set_vel(self.current_velocity)
        self.set_angle_vel(target_angle_velocity)

    def is_path_clear(self, target):
        # Verifica se há obstáculos no caminho em direção ao alvo
        for opponent_id, opponent in self.opponents.items():
            distance_to_opponent = self.pos.dist_to(Point(opponent.x, opponent.y))
            distance_to_target = self.pos.dist_to(target)

            # Se um oponente estiver no caminho direto para o alvo
            if distance_to_opponent < distance_to_target and distance_to_opponent < self.SAFETY_DISTANCE:
                return False  # Caminho não está livre

        return True  # Caminho está livre

    def is_target_near_opponent(self, target):
        for opponent_id, opponent in self.opponents.items():
            distance_to_opponent = self.pos.dist_to(Point(opponent.x, opponent.y))
            distance_to_target = self.pos.dist_to(target)

            # Se o alvo estiver muito próximo de um robô inimigo
            if distance_to_opponent < distance_to_target and distance_to_opponent < self.SAFETY_DISTANCE:
                return True  # O alvo está próximo de um oponente

        return False  # O alvo não está próximo de nenhum oponente

    def collide_with_opponent(self):
        for opponent_id, opponent in self.opponents.items():
            distance_to_opponent = self.pos.dist_to(Point(opponent.x, opponent.y))

            # Se estiver próximo ao oponente, colidir intencionalmente
            if distance_to_opponent < self.SAFETY_DISTANCE:
                collision_vector = Point(opponent.x - self.pos.x, opponent.y - self.pos.y)
                collision_angle = math.atan2(collision_vector.y, collision_vector.x)

                collision_velocity = Point(
                    math.cos(collision_angle) * self.HIGH_VELOCITY,
                    math.sin(collision_angle) * self.HIGH_VELOCITY
                )

                # Definir a nova velocidade para colidir com o oponente
                self.set_vel(collision_velocity)
                break

    def apply_bang_bang_control(self, target_velocity):
        target_magnitude = self.get_magnitude(target_velocity)

        # Acelera até a velocidade máxima se ainda não atingiu
        if target_magnitude > self.get_magnitude(self.current_velocity):
            acceleration_vector = Point(
                (target_velocity.x - self.current_velocity.x) * 0.1,
                (target_velocity.y - self.current_velocity.y) * 0.1
            )
            new_velocity = Point(
                min(self.current_velocity.x + acceleration_vector.x, self.MAX_VELOCITY),
                min(self.current_velocity.y + acceleration_vector.y, self.MAX_VELOCITY)
            )
            self.current_velocity = new_velocity
            
        else:
            # Desacelera rapidamente se estiver muito próximo do alvo ou colidindo com um obstáculo
            deceleration_vector = Point(
                -0.2 * (self.current_velocity.x / max(1e-5, target_magnitude)),
                -0.2 * (self.current_velocity.y / max(1e-5, target_magnitude))
            )
            new_velocity = Point(
                max(self.current_velocity.x + deceleration_vector.x, 0),
                max(self.current_velocity.y + deceleration_vector.y, 0)
            )
            self.current_velocity = new_velocity

    def choose_target(self):
        closest_target = None
        min_distance = float('inf')

        for target in self.targets:
            distance = self.pos.dist_to(target)
            if not self.is_target_blocked_with_opponent(target):
                if distance < min_distance:
                    min_distance = distance
                    closest_target = target

        return closest_target

    def is_target_blocked(self):
        for opponent_id, opponent in self.opponents.items():
            if opponent.x == self.target.x and opponent.y == self.target.y:
                return True
        return False

    def find_alternate_target(self):
        for target in self.targets:
            if not self.is_target_blocked_with_opponent(target):
                return target
        return None

    def is_target_blocked_with_opponent(self, target):
        for opponent_id, opponent in self.opponents.items():
            if opponent.x == target.x and opponent.y == target.y:
                return True
        return False

    def avoid_obstacles(self):
        avoidance_velocity = None
        min_distance_to_obstacle = 0.5

        for opponent_id, opponent in self.opponents.items():
            distance = self.pos.dist_to(Point(opponent.x, opponent.y))

            if distance <= min_distance_to_obstacle:
                if distance < 0.2:
                    avoidance_velocity = self.calculate_avoidance_vector(opponent, high_velocity=True)
                    self.reverse_movement()
                else:
                    if self.is_opponent_in_path(opponent, self.target):
                        avoidance_velocity = self.calculate_avoidance_vector(opponent)

                if avoidance_velocity:
                    break

        return avoidance_velocity

    def calculate_avoidance_vector(self, opponent, high_velocity=False):
        angle_to_opponent = math.atan2(opponent.y - self.pos.y, opponent.x - self.pos.x)
        
        # Adicionar um desvio de 90 graus para evitar o oponente
        avoidance_angle = angle_to_opponent + (math.pi / 2 if self.pos.x < opponent.x else -math.pi / 2)
        
        velocity = self.HIGH_VELOCITY if high_velocity else self.MAX_VELOCITY
        
        avoidance_velocity = Point(
            math.cos(avoidance_angle) * velocity,
            math.sin(avoidance_angle) * velocity
        )

        return avoidance_velocity

    def reverse_movement(self):
        reverse_velocity = Point(self.REVERSE_VELOCITY, 0)
        self.set_vel(reverse_velocity)
        self.set_angle_vel(0)

    def get_magnitude(self, point):
        # Calcular a magnitude de um vetor (usando o teorema de Pitágoras)
        return math.sqrt(point.x ** 2 + point.y ** 2)

    def is_opponent_in_path(self, opponent, target):
        # Calcular o ângulo do robô até o alvo
        angle_to_target = math.atan2(target.y - self.pos.y, target.x - self.pos.x)

        # Calcular o ângulo do robô até o oponente
        angle_to_opponent = math.atan2(opponent.y - self.pos.y, opponent.x - self.pos.x)

        # Calcular a diferença entre os ângulos
        angle_diff = abs(angle_to_target - angle_to_opponent)

        # Verificar se o oponente está no caminho direto entre o robô e o alvo
        return angle_diff < math.radians(40)  # Margem de 40 graus

    def post_decision(self):
       pass 
