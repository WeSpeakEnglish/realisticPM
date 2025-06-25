from machine import Pin, Timer, DAC, ADC
 
led = Pin(18, Pin.OUT)
counter = 0
lightIntensity = 0 

dac = DAC(0) #DAC A0 output
adc = ADC(6) #ADC on A6

 
def fun(tim):
    global counter
    global lightIntensity
    counter = counter + 1 
    print(adc.read_u16()) 
    led.value(counter%2)
    dac.write(counter%1024)
 
tim = Timer(-1)
tim.init(period=1000, mode=Timer.PERIODIC, callback=fun)