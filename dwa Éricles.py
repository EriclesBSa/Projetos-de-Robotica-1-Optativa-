# Algoritmo 2 - Éricles
import math
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def main():
    print("Conectando ao CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    print("Conectado! Buscando handles...")
    
    motorEsquerdo = sim.getObject("/Cuboid/MOTOR_ESQUERDO")
    motorDireito = sim.getObject("/Cuboid/MOTOR_DIREITO")
    
    sensores = [
        sim.getObject("/Cuboid/SENSOR_ESQUERDO"),
        sim.getObject("/Cuboid/SENSOR_DIAG_ESQUERDO"),
        sim.getObject("/Cuboid/SENSOR_MEIO"),
        sim.getObject("/Cuboid/SENSOR_DIAG_DIREITO"),
        sim.getObject("/Cuboid/SENSOR_DIREITO")
    ]
        
    goalHandle = sim.getObject("/Target")
    robotBase = sim.getObject("/Cuboid")
    
    R = 0.036
    L = 0.235
    
    predict_time = 0.5
    
    max_v = 0.2
    min_v = -0.05 
    max_w = 0.5
    max_accel_v = 0.5
    max_accel_w = 1.5
    
    alpha = 2.0
    beta = 1.5
    gamma = 0.5
    
    v_atual = 0.0
    w_atual = 0.0


    tempo_acumulado = 0.0
    intervalo_print = 2.0

    client.setStepping(True)
    sim.startSimulation()
    print(f"Simulação iniciada. Atualizando terminal a cada {intervalo_print}s.")

    try:
        while True:
            dt = sim.getSimulationTimeStep()
            
            # Acumula o tempo decorrido na simulação
            tempo_acumulado += dt
            
            pos = sim.getObjectPosition(robotBase, -1)
            goal_pos = sim.getObjectPosition(goalHandle, -1)
            
            # Verificação de Chegada
            dist_to_goal = math.hypot(goal_pos[0] - pos[0], goal_pos[1] - pos[1])
            if dist_to_goal < 0.2:
                print("\n[EVENTO] Alvo alcançado!")
                sim.setJointTargetVelocity(motorEsquerdo, 0.0)
                sim.setJointTargetVelocity(motorDireito, 0.0)
                client.step()
                break 
            
            distancias_obstaculos = []
            obstaculos_detectados = 0
            for i in range(5):
                res, dist, point, obj, n = sim.readProximitySensor(sensores[i])
                if res > 0 and dist < 0.4:
                    distancias_obstaculos.append(dist)
                    obstaculos_detectados += 1
                else:
                    distancias_obstaculos.append(0.4)
                    
            v_min_dw = max(min_v, v_atual - max_accel_v * dt)
            v_max_dw = min(max_v, v_atual + max_accel_v * dt)
            w_min_dw = max(-max_w, w_atual - max_accel_w * dt)
            w_max_dw = min(max_w, w_atual + max_accel_w * dt)
            
            melhor_v = 0.0
            melhor_w = 0.0
            max_score = -9999.0
            
            ori = sim.getObjectOrientation(robotBase, -1)
            theta = ori[2] + math.pi
            
            v_step = (v_max_dw - v_min_dw) / 5.0 if v_max_dw > v_min_dw else 0.1
            w_step = (w_max_dw - w_min_dw) / 10.0 if w_max_dw > w_min_dw else 0.1
            
            v = v_min_dw
            while v <= v_max_dw:
                w = w_min_dw
                while w <= w_max_dw:
                    theta_futuro = theta + w * predict_time
                    x_futuro = pos[0] + v * math.cos(theta_futuro) * predict_time
                    y_futuro = pos[1] + v * math.sin(theta_futuro) * predict_time
                    
                    angulo_alvo = math.atan2(goal_pos[1] - y_futuro, goal_pos[0] - x_futuro)
                    erro_heading = abs(math.atan2(math.sin(angulo_alvo - theta_futuro), math.cos(angulo_alvo - theta_futuro)))
                    score_heading = 1.0 - (erro_heading / math.pi)
                    
                    min_dist = min(distancias_obstaculos)
                    score_clearance = 1.0
                    
                    if min_dist < 0.4:
                        score_clearance = min_dist / 0.4
                        if (distancias_obstaculos[0] < 0.3 or distancias_obstaculos[1] < 0.3) and w > 0:
                            score_clearance -= 1.0
                        if (distancias_obstaculos[3] < 0.3 or distancias_obstaculos[4] < 0.3) and w < 0:
                            score_clearance -= 1.0
                        if distancias_obstaculos[2] < 0.25 and v > 0.1:
                            score_clearance -= 1.0
                            
                    score_vel = v / max_v if max_v > 0 else 0
                    score = (alpha * score_heading) + (beta * score_clearance) + (gamma * score_vel)
                    
                    if score > max_score:
                        max_score = score
                        melhor_v = v
                        melhor_w = w
                    w += w_step
                v += v_step
                
            dist_minima_geral = min(distancias_obstaculos)
            if dist_minima_geral < 0.18:
                melhor_v = 0.0 
                espaco_esquerda = distancias_obstaculos[0] + distancias_obstaculos[1]
                espaco_direita = distancias_obstaculos[3] + distancias_obstaculos[4]
                if espaco_esquerda > espaco_direita:
                    melhor_w = 0.8
                else:
                    melhor_w = -0.8
                        
            v_atual = melhor_v
            w_atual = melhor_w
            
            vel_esq = (v_atual - (w_atual * L / 2)) / R
            vel_dir = (v_atual + (w_atual * L / 2)) / R
            
            sim.setJointTargetVelocity(motorEsquerdo, vel_esq)
            sim.setJointTargetVelocity(motorDireito, vel_dir)

            if tempo_acumulado >= intervalo_print:
                print(f"[STATUS] Alvo: {dist_to_goal:.2f}m | Obstáculos: {obstaculos_detectados}/5 | Vel: {v_atual:.2f} m/s")
                tempo_acumulado = 0.0
            
            client.step()

    except KeyboardInterrupt:
        print("\nSimulação interrompida pelo VS Code.")
    finally:
        sim.setJointTargetVelocity(motorEsquerdo, 0.0)
        sim.setJointTargetVelocity(motorDireito, 0.0)
        sim.stopSimulation()
        print("Conexão encerrada.")

if __name__ == '__main__':
    main()