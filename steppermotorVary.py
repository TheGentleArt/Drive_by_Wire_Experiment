#Jwilliams
#This script intends to move a stepper motor. It uses a RPi, a DRV8825 Stepper Motor Driver, a breadboard, a NEMA 17 Stepper Motor, a ADS1115 Analog to Digital Converter, and a power supply for now.
#A large portion of getting the stepper motor working correctly was achieved via watching a tutorial from ‘rdagger68’ on Youtube titled ‘Raspberry Pi Stepper Motor Tutorial’.
#Another portion of getting the ADS1115 to work correctly was done by watching a tutorial from 'Ravivarman Rajendiran' on Youtube titled 'Analog Sensors Interfacing with Raspberry Pi using ADS1115 ADC. Step by step guide.'


#DRV8825 Pin Layout (with pot screw in top left corner, pins facing down)
    ##ENABLE|VMOT#######
    ######M0|GND_MOT####
    ######M1|B2#########
    ######M2|B1#########
    ###RESET|A1#########
    ###SLEEP|A2#########
    ####STEP|FAULT######
    #####DIR|GND_LOGIC##

#DRV8825 Pin Assignment
    #Enable> Not Used
    #M0> (Optional) Pi GPIO 14 (Physical Pin 8)    (Without, operates in Full Step mode)
    #M1> (Optional) Pi GPIO 15 (Physical Pin 10)    (Without, operates in Full Step mode)
    #M2> (Optional) Pi GPIO 18 (Physical Pin 12)    (Without, operates in Full Step mode)
    #RESET>Pi 3.3V+ (Supposedly DRV8825 will not operate if this is not pulled to HIGH position)
    #SLEEP>Pi GPIO 17 (Physical Pin 11) (Could also be placed on 3.3V+ pin, but would always leave stepper motor engaged, which may result in unnecessary power draw)
    #STEP>Pi GPIO 21 (Physical Pin 40) (IIRC when pulsed, tells motor to take one step)
    #DIR>Pi GPIO20 (Physical Pin 38)
    #VMOT>Power Supply 12V+ (Be careful connecting anything over 3.3V as this may damage the Pi) (Also note that a 100 uF capacitor was used across the VMOT/GND_MOT pins in case of voltage spike issues)
    #GND_MOT>Power Supply 12V- (also connect to GND LOGIC Pin just in case the other ground circuit is bad)
    #B2> Motor Pole (Pair with B1) (Do not connect until DRV8825 Pot is adjusted)
    #B1> Motor Pole (Pair with B2) (Do not connect until DRV8825 Pot is adjusted)
    #A1> Motor Pole (Pair with A2) (Do not connect until DRV8825 Pot is adjusted)
    #A2> Motor Pole (Pair with A1) (Do not connect until DRV8825 Pot is adjusted)
    #FAULT> Not Used
    #GND_LOGIC>Pi GND (Physical Pin 39) (also connect to GND_MOT Pin just incase other ground circuit is bad)

#Description of how this was connected. This may be redundant information.
    #DRV8825 Logic_GND and Motor_GND pins tied to GND of breadboard. Logic GND and Motor GND were tied together at the GND on the breadboard, just to ensure a common ground.
    #Motor Ground tied to the power supply ground (negative terminal)
    #VMOT tied to the power supply (+ terminal)(Do NOT connect the power supply (+) to anything but the VMOT pin on the DRV8825, as this will be using more voltage than the Pi can handle and will damage it.)
    #100 uF capacitor used across the VMOT and GND pins for the DRV8825. This is done to have a capacitor in parallel so that any voltage spikes will be 'absorbed' by the capacitor.
    #SLEEP pin on the DRV8825 is connected to GPIO pin 17 on Pi. (Can cannect this to 3.3V+pin if do not need to put the motor to sleep when not turning (pedal not pressed) if want to save power.)
    #RESET Pin of DRV8825 to 3.3V (+)
    #STEP Pin to GPIO 21 of Pi
    #DIR Pin to GPIO 20 of Pi
    #3.3V RPi Pin (Physical Pin 1) to 3.3V (+) Pin on breadboard
    #RPi GND Pin (Physical Pin 6) to 3.3V (-) Pin on breadboard
    #Before connected motor, turn power supply on and set to 12V, then measure voltage on the Pot Screw on the DRV8825 and turn screw clockwise to reduce voltage (or ccw to increase) until around 0.5V, or whatever voltage is required.
    #For the DRV8825, approximate current limit in amps is twice the volts at the pot screw.
    #The DRV8825 steps the voltage down, so do not use power supply current as is for estimating motor current.
    #Once the voltage is regulated properly, ensure power supply is off and connect one pole pair of motor to A1 and A2, and the other to B1/B2 of DRV8825. If unsure which are paired, place an LED across leads with no power connected and turn motor by hand. If lights up, they are paired.
    #
    #The M0,M1,and M2 pins on the DRV8825 are used to change the stepping mode. With the default of these pins being low, full step is used. However, can have microstepping if using the pins.
    


#Library Import
from time import sleep
import RPi.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import board
import busio



#####ADS1115 chan.voltage input code:
#Create the I2C bus
i2c=busio.I2C(board.SCL, board.SDA)

#Create the ADC object using the I2C bus
ads=ADS.ADS1115(i2c)
ads.gain=1 #GAIN

#Create single ended input on channel 0 (A0)
chan=AnalogIn(ads, ADS.P0)

#now can call chan.voltage to get voltage from A0
#will have to see if can use A1, A2 from same board for other measurements
#####




###Setup###

#RPi Pin Assignments 
DIR=20 #Direction --- GPIO Pin Label
STEP=21 #Step --- GPIO Pin Label
MODE=(14,15,18) #Microstep Resolution Mode --GPIO Labels for M0, M1, and M2.
SLEEP=17 #Sleep --- GPIO Pin Label

#Motor Info
Step_deg=1.8 #degrees for each step

#Microstepping Resolution Setup for DRV8825 (From DRV8825 datasheet)
RESOLUTION={'Full': (0,0,0),
            'Half': (1,0,0),
            '1/4':  (0,1,0),
            '1/8':  (1,1,0),
            '1/16': (0,0,1),
            '1/32': (1,0,1),
            '1/64': (1,1,1)}

STEP_MODE='1/4' #Motor Stepping Mode. 'Full','Half','1/4',1/8','1/16','1/32',or '1/64'
CW=1 #Clockwise rotation
CCW=0 #Counterclockwise rotation
SPR_FullStep=int(360/Step_deg) #Steps per revolution ---may round and not be exact but could not get to work as float last i tried
delay_FullStep=.00125 #Time to delay between each step

#Setting/adjusting Steps per revolution for microstepping settings
if STEP_MODE=='Full':
    SPR=1*SPR_FullStep
    delay=delay_FullStep/1
elif STEP_MODE=='Half':
    SPR=2*SPR_FullStep
    delay=delay_FullStep/2
elif STEP_MODE=='1/4':
    SPR=4*SPR_FullStep
    delay=delay_FullStep/4
elif STEP_MODE=='1/8':
    SPR=8*SPR_FullStep
    delay=delay_FullStep/8
elif STEP_MODE=='1/16':
    SPR=16*SPR_FullStep
    delay=delay_FullStep/16
elif STEP_MODE=='1/32':
    SPR=32*SPR_FullStep
    delay=delay_FullStep/32
elif STEPMODE=='1/64':
    SPR=64*SPR_FullStep
    delay=delay_FullStep/64
#Now SPR (Steps per revolution) is set, as well as the delay
    

#GPIO Setup
GPIO.setmode(GPIO.BCM) #Set Pins to use the GPIO labels/broadcom labeling system instead of physical pin assignment
GPIO.setup(DIR, GPIO.OUT) #Direction Pin on Pi is set to an output pin
GPIO.setup(STEP, GPIO.OUT) #Step Pin on Pi is set to an output pin
GPIO.setup(MODE,GPIO.OUT) #Set mode pin on Pi as an output.
GPIO.setup(SLEEP,GPIO.OUT) #Set sleep pin on Pi as an output pin.
GPIO.output(DIR,CW) #Direction set to CW initially (Rpi pulls DIR pin high?)
GPIO.output(MODE,RESOLUTION[STEP_MODE]) #Sets full or half/micro stepping up. See 'RESLUTION' dictionary
GPIO.output(SLEEP,1) #enable stepper motor


###Program###

sleep(0.25) #wait some time for sleep pin to activate
SPD=SPR/360 #Steps per degree
act_deg=0 #say throttle always at 0 deg when starting program, should be accurate if stepper motor was not holding torque before program runs (throttle return spring pulls closed)
volt_per_deg=0.04 #measured value-----off 3.3V circuit and 330 ohm resistor to potentiometer


#enter a forever while loop, until 'CTRL+c' keyboard interrupt occurs, then cleanup GPIO pins
try:
    itr=0 #counter of iterations of while loop below
    while True:
        des_deg=round(chan.voltage/volt_per_deg,2) #Converts voltage reading to desired throttle position in degrees
        
        if des_deg<0: #adjusts for slight negative voltage reading i was getting
            des_deg=0
        else:
            pass
        
        if itr%75==0: #print desired and actual throttle position once every 75 iterations (to make easier to read) (modulo)
            print("Desired throttle pos:",des_deg,"deg"," Actual throttle pos:",act_deg,"deg")
        else:
            pass
        
        if des_deg>act_deg: #steps motor forward or backward by 1 step (or does nothing if desired=actual, unlikely)
            GPIO.output(DIR,CW)
            GPIO.output(STEP,GPIO.HIGH)
            GPIO.output(STEP,GPIO.LOW)
            act_deg=round(act_deg+1/SPD,2)
            sleep(delay)
        elif des_deg<act_deg:
            GPIO.output(DIR,CCW)
            GPIO.output(STEP,GPIO.HIGH)
            GPIO.output(STEP,GPIO.LOW)
            act_deg=round(act_deg-1/SPD,2)
            sleep(delay)
        else:
            pass
        
        itr+=1 #add 1 to while loop iteration counter
        
except KeyboardInterrupt:
    GPIO.output(SLEEP,GPIO.LOW) #disable stepper motor (to keep from getting hot unneccesarily)
    GPIO.cleanup() #reset GPIO pins to inputs to protect against shorting accidentally
    print("GPIO pins cleaned up") 