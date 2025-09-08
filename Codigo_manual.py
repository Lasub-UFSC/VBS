from machine import Pin, UART, PWM
from utime import sleep

"""
Para rodar este codigo, deve ser utilizado o botao 'run' no canto inferior esquerdo
com a raspberry pi pico conectada ao computador e nao pode estar escrito 'Pico Disconnected'
ao lado do botao

Este codigo permite o input manual das variaveis velocidade, aceleracao,
direcao e posicao a se mover. Ha comentarios no meio do codigo explicando o que cada parte faz.

Tomar muito cuidado pois este codigo nao utiliza os sensores de fim e inicio de curso,
ou seja, a movimentacao deve ser cuidadosa para que os fusos nao saiam das castanhas.
Alem disso, a fonte de alimentacao para a raspberry deve ser o computador e para o driver do motor
deve ser utilizada uma fonte de bancada separada entre 24v e 48v, utilizei a do LSE para tal.

!Caso haja algum problema, desligue imediatamente a fonte de bancada!

Os codigos comentados levam em conta o uso dos microrruptores, ou seja, descomentar somente se
tiver os microrruptores no local.
"""


"""
Seleciona a velocidade final em que o step é passado.

Lembrando que 3440 steps do motor sao 2mm de curso do pistão.
Entao na teoria uma velocidade de 3440 e equivalente a 2mm/s
"""
speed = int(input("set final speed to (between 500 and 8000): "))

"""
Seleciona a aceleracao que o motor terá no inicio (valores mais baixos
para uma aceleracao mais lenta caso tenho muito slip no motor)
"""
accel = int(input("initial acceleration (between 5 and 30): "))

"""
Definicao de pinos e frequencia inicial do pwm
"""
step_pin = PWM(Pin(3))
step_pin.freq(speed)
step_pin.duty_u16(0)
dir_pin = Pin(2, Pin.OUT)


"""
valores para transformacao de rotacao no-slip para axial.
"""
reduction = 17.2
steps_per_rev = 200
spindle = 2
motor_multiplier = steps_per_rev * reduction / spindle

max_pos_pin = Pin(4, Pin.IN, Pin.PULL_UP)
min_pos_pin = Pin(5, Pin.IN, Pin.PULL_UP)

max_pos_num = 0
cur_pos = 100


while True:
    try:
        
        #goes to minimal position
        dir_pin.value(1)
        i = 0
        while(not min_pos_pin.value()):
            step_pin.duty_u16(32768)
            sleep(sleep_time)
            i += 1
            if(step_pin.freq() < speed):
                step_pin.freq(500 + 20*i)

        step_pin.duty_u16(0)
        step_pin.freq(500)

        print("Setting initial position to 0\n")
        sleep(0.5)
        i = 0 

        #goes to maximum position
        dir_pin.value(0)
        while(max_pos_pin.value()):
            step_pin.duty_u16(32768)
            sleep(sleep_time)
            max_pos_num += 1
            i += 1
            if(step_pin.freq() < speed):
                step_pin.freq(500 + 20*i)

        step_pin.duty_u16(0)
        


        """
        Seleciona a direcao do movimento e a quantidade que ira se mover sem slip 
        (contando que o motor nao patina)

        0 o pistao desce (contrai) e 1 o pistao sobe (expande)
        """

        # set for direction
        while(True):
            direction = input("direction of movement ('contrair' for contract and 'expandir' for forward): ")
            if(direction == "expandir"):
                dir_pin.value(0)
                break
            elif(direction == "contrair"):
                dir_pin.value(1)
                break
            else:
                print("direcao invalida, insira novamente.\n")
        
        # define new position
        new_pos = float(input("movement in milimeters: "))
        new_pos = new_pos * motor_multiplier

        i = 0
        # go in that direction
        while(i < int(new_pos)):
            # test if it is in max pos or min pos
            if(not min_pos_pin.value() and (direction == "contrair")):
                step_pin.duty_u16(0)
                step_pin.freq(speed)
                print("pistao ja esta totalmente contraido\n")
                break
            elif(not max_pos_pin.value() and (direction == "expandir")):
                step_pin.duty_u16(0)
                step_pin.freq(speed)
                print("pistao ja esta totalmente expandido\n")
                break
            
            step_pin.duty_u16(32768) #starts the pwm to enable movement
            print("step number: ", i)
            if(step_pin.freq() < speed):
                step_pin.freq(speed + accel * i)
            sleep(1 / (1.1 * step_pin.freq())) # waits for almost the full pwm cycle
            i += 1

        #stops the movement and speed is set to minimum
        step_pin.duty_u16(0)
        step_pin.freq(speed)


    except Exception as e:
        print("Error: ", e)
        break

# close all pins and ports
step_pin.deinit()
dir_pin.off()
print("EOF.")
