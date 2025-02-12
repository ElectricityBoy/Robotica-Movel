sim = require 'sim'

function sysCall_init()
    robot = sim.getObject('/PioneerP3DX')

    if robot == -1 then
        print("Erro: Objeto do robô não encontrado!")
    else
        print("Robô encontrado! Handle:", robot)
    end

    -- Criando a coleção de obstáculos
    obstacles = sim.createCollection(0)
    sim.addItemToCollection(obstacles, sim.handle_all, -1, 0)
    sim.addItemToCollection(obstacles, sim.handle_tree, robot, 1)

    -- Configuração dos sensores ultrassônicos
    usensors = {}
    for i = 1, 16 do
        usensors[i] = sim.getObject("../ultrasonicSensor", {index = i - 1})
        sim.setObjectInt32Param(usensors[i], sim.proxintparam_entity_to_detect, obstacles)
    end

    -- Motores
    motorLeft = sim.getObject("../leftMotor")
    motorRight = sim.getObject("../rightMotor")

    -- Parâmetros de Braitenberg
    noDetectionDist = 0.5
    maxDetectionDist = 0.2
    detect = {}
    braitenbergL = {-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0, 0, 0, 0, 0, 0, 0, 0}
    braitenbergR = {-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0, 0, 0, 0, 0, 0, 0, 0}
    v0 = 2.0

    -- Posição alvo
    goalX, goalY = 2.0, 0.8
    goalReached = false
    Kp = 1.3 -- Ganho proporcional
    stopThreshold = 0.1 -- Aumentado para evitar paradas prematuras
    stopAngleThreshold = 0.02  -- Limiar de parada (ângulo)

    -- Parâmetros do robô
    wheelRadius = 0.0975
    wheelBase = 0.331
end

function sysCall_actuation()
    if goalReached then
        sim.setJointTargetVelocity(motorLeft, 0)
        sim.setJointTargetVelocity(motorRight, 0)
        return
    end

    -- Leitura dos sensores
    obstacleDetected = false
    for i = 1, 16 do
        res, dist = sim.readProximitySensor(usensors[i])
        if res > 0 and dist < noDetectionDist then
            detect[i] = 1 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist))
            obstacleDetected = true
        else
            detect[i] = 0
        end
    end

    -- Posição e orientação do robô (Ground Truth)
    position = sim.getObjectPosition(robot, -1)
    orientation = sim.getObjectOrientation(robot, -1) 
    theta = orientation[3] 

    -- Depuração da posição
    print("Posição do robô:", position[1], position[2])

    -- Cálculo da distância ao objetivo
    dx, dy = goalX - position[1], goalY - position[2]
    distanceToGoal = math.sqrt(dx * dx + dy * dy)

    -- Determina o ângulo necessário para chegar ao objetivo
    angleToGoal = math.atan2(dy, dx)
    headingError = math.fmod(angleToGoal - theta + math.pi, 2 * math.pi) - math.pi

    -- Print erro angular
    print("Erro angular:", headingError)

    -- Se está perto o suficiente e o ângulo está dentro do limiar, PARA
    if distanceToGoal < stopThreshold and math.abs(headingError) < stopAngleThreshold then
        goalReached = true
        sim.setJointTargetVelocity(motorLeft, 0)
        sim.setJointTargetVelocity(motorRight, 0)
        return
    end

    -- Controle Braitenberg (somente se houver obstáculos)
    if obstacleDetected then
        vLeft, vRight = v0, v0
        for i = 1, 16 do
            vLeft = vLeft + braitenbergL[i] * detect[i]
            vRight = vRight + braitenbergR[i] * detect[i]
        end
    else
        -- Controle Proporcional para Navegação
        angularVel = Kp * headingError
        angularVel = math.max(math.min(angularVel, 1.0), -1.0)

        -- Reduz velocidade suavemente ao se aproximar do alvo
        v0 = math.max(0.2, 2.0 * (distanceToGoal / 2.0))  -- Garante que não fique muito lento

        -- Cálculo das velocidades das rodas
        vLeft = v0 - angularVel
        vRight = v0 + angularVel
    end

    -- Aplicar velocidades aos motores
    sim.setJointTargetVelocity(motorLeft, vLeft)
    sim.setJointTargetVelocity(motorRight, vRight)

end
