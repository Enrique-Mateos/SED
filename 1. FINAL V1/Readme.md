
# Version final del proyecto

Desarrollado por:

Christopher Enga     
Enrique Mateos       

# Introducción
El presente trabajo tiene como objeto el desarrollo de un control de luminarias LED, el cual podría formar parte de un sistema domótica para el hogar.
Por lo general estos sistemas son capaces de crear diferentes ambientes y escenas controlados mediante una CPU, ya sea un PLC o un microcontrolador entre otros. Dicha centralita está encargada de la ejecución de la programación y de las comunicaciones con los sensores, luminarias y en algunos casos con aplicaciones móviles e incluso mediante la red.


# CONEXIONADO:


Pines:

-PA0:ADC1 PARA EL POTENCIOMETRO PARA REGULAR LA INTENSIDAD

-PA1:ADC2 PARA EL SENSOR LDR

-PA3:INTERRUPTOR TEMPORIZADO

-PBO,PB1,PB5,PB4:LEDS PARA EL SENSOR LDR

-PC6:TRIGGER DEL SENSOR  DE ULTTRASONIDOS

-PE9:ECHO PARA EL SENSOR DE ULTRASONIDOS(TIM1 INPUT CAPTURE 
MODE ,BOTH EDGES EN CONFIGURATION)
-PD15:TIM4_CH4 PMW LED POTENCIOMETR

-PD14:LED

-PD13:LED

-PD12:LED

-PD11:Para el interruptor del debounce

#     Pantalla LCD SPI

CS: PB6

RS: PA9

DC: PC7

SCK: PA5

MISO: PA6

MOSI: PA7



Los RCC los puse porque los tenia activados ,y el pin A13 tampoco lo he activado con serial wire.
Los demas valores los puedes com el preescaler,interrupciones activadas,etc  ver en configuration. 
