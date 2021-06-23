#J.Williams
#This script intends to move a stepper motor. It uses a RPi, a DRV8825 Stepper Motor Driver, a breadboard, a NEMA 17 Stepper Motor, a ADS1115 Analog to Digital Converter, and a power supply for now.
#A large portion of getting the stepper motor working correctly was achieved via watching a tutorial from ‘rdagger68’ on Youtube titled ‘Raspberry Pi Stepper Motor Tutorial’.
#Another portion of getting the ADS1115 to work correctly was done by watching a tutorial from 'Ravivarman Rajendiran' on Youtube titled 'Analog Sensors Interfacing with Raspberry Pi using ADS1115 ADC. Step by step guide.'


#####DRV8825 Info
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
#####


#####Description of how this was connected. This may be redundant information.
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
#####
    
#ADS1115 Pin Layout ---(and Placement)
    #VDD----Positive Voltage In (3.3 V from RPi)
    #GND----Ground
    #SCL----To SCL1 Pin (physical pin 5) on RPi
    #SDA----To SDA1 Pin (physical pin 7) on RPi
    #ADDR---(Not used)
    #ALRT---(Not used)
    #A0-----Signal Channel 0 (Pedal Position Sensor)
    #A1-----Signal Channel 1 (Axle Input Shaft Speed Sensor)
    #A2-----Signal Channel 2 (Throttle Position Sensor)
    #A3-----(Not Used)
    


#####Library Imports
import time
from time import sleep
import timeit
import RPi.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import board
import busio
import math
import numpy as np
#####



#####ADS1115 chan.voltage input code:
#Create the I2C bus
i2c=busio.I2C(board.SCL, board.SDA)
#Create the ADC object using the I2C bus
ads=ADS.ADS1115(i2c)
ads.gain=1 #GAIN
#Create single ended input on channel 0 (A0)
pps=AnalogIn(ads, ADS.P0) #Pedal Position Sensor
axle_spd_sens=AnalogIn(ads, ADS.P1) #Axle Shaft Speed Sensor
tps=AnalogIn(ads, ADS.P2) #Throttle Position Sensor
#####



#####Setup

#Motor Info
#NEMA17 Stepper Motor
Step_deg=1.8 #degrees for each step


###Setting up Microstepping---Only input to adjust here is "STEP_MODE" variable
#Microstepping Resolution Setup for DRV8825 (From DRV8825 datasheet)
RESOLUTION={'Full': (0,0,0),
            'Half': (1,0,0),
            '1/4':  (0,1,0),
            '1/8':  (1,1,0),
            '1/16': (0,0,1),
            '1/32': (1,0,1),
            '1/64': (1,1,1)}

STEP_MODE='1/8' #Motor Stepping Mode. 'Full','Half','1/4',1/8','1/16','1/32',or '1/64'
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
###Microstepping setup done   

#RPi Pin Assignments 
DIR=20 #Direction --- GPIO Pin Label
STEP=21 #Step --- GPIO Pin Label
MODE=(14,15,18) #Microstep Resolution Mode --GPIO Labels for M0, M1, and M2.
SLEEP=17 #Sleep --- GPIO Pin Label

#GPIO Setup
GPIO.setmode(GPIO.BCM) #Set Pins to use the GPIO labels/broadcom labeling system instead of physical pin assignment
GPIO.setup(DIR, GPIO.OUT) #Direction Pin on Pi is set to an output pin
GPIO.setup(STEP, GPIO.OUT) #Step Pin on Pi is set to an output pin
GPIO.setup(MODE,GPIO.OUT) #Set mode pin on Pi as an output.
GPIO.setup(SLEEP,GPIO.OUT) #Set sleep pin on Pi as an output pin.
GPIO.output(DIR,CW) #Direction set to CW initially (Rpi pulls DIR pin high?)
GPIO.output(MODE,RESOLUTION[STEP_MODE]) #Sets full/microstepping up. See 'RESLUTION' dictionary
GPIO.output(SLEEP,1) #enable stepper motor
#####



###Program###

def ax_spd_sens_v_to_veh_spd(axle_spd_sens_volt):
    axle_speed_sensor_v_in = 3.3 #Sensor Supply Voltage
    axle_spd_sens_volt_per_rpm = axle_speed_sensor_v_in/5500 #volts per 5500 rpm (25mph with 18" tires)
    axle_input_rpm = axle_spd_sens_volt/axle_spd_sens_volt_per_rpm
    axle_ratio = 11.47
    tire_dia = 18 #inch
    f_roll_rad = 0.965 #rolling radius factor
    tire_circ = tire_dia*math.pi*f_roll_rad 
    tire_rpm = (axle_input_rpm)/axle_ratio
    veh_spd_inchpermin = tire_rpm*tire_circ
    veh_spd = veh_spd_inchpermin*((60/1)*(1/12)*(1/5280))
    if veh_spd < 0:
        veh_spd = 0
    else:
        pass
    return veh_spd
    
def pps_v_to_des_spd(pedal_pos_sens_voltage):
    pps_v_max = 3.3 #3.3V max voltage for Pedal Position Sensor
    des_spd = (pedal_pos_sens_voltage/pps_v_max)*cruise_spd
    if des_spd < 0:
        des_spd = 0
    else:
        pass
    return des_spd

def tps_v_to_deg_throttle(tps_voltage):
    tps_v_in = 5 #volts --- Input either 3.3 or 5. On Vehicle, ECU delivers 5 V
    deg_throttle_min = 0
    deg_throttle_max = 80 #may be 82, see variation here from throttle body to throttle body
    
    if tps_v_in == 3.3: #3.3V Input to Throttle Body
        tps_v_min = 0.41 #Fully Closed throttle signal reading
        tps_v_max = 2.53 #Fully Open Throttle signal reading
    elif tps_v_in == 5: #5V Input to Throttle Body
        tps_v_min = 0.652 #Fully Closed throttle signal reading 
        tps_v_max = 3.865 #Fully Open Throttle signal reading
    else:
        print("Error with tps_v_to_deg_throttle function")
    
    if tps_voltage < tps_v_min: #in case tps signal is slightly lower from throttle body, round to tps_v_min for now
        tps_voltage = tps_v_min
    else:
        pass
    if tps_voltage > tps_v_max: #in case tps signal is slightly higher from throttle body to throttle body, round to tps_v_max value for now
        tps_voltage = tps_v_max
    else:
        pass
    
    deg_throttle = 0+(tps_voltage-tps_v_min)*(deg_throttle_max-deg_throttle_min)/(tps_v_max-tps_v_min) #linear interpolation
    if deg_throttle == -0: #was seeing negative zero sometimes, did not want to see negative zero. did not want to use abs() in case it goes below negative zero I would want to see it in testing.
        deg_throttle = 0
    else:
        pass
    
    return deg_throttle


#the following two lines are only required for reporting the actual degree moved
#SPD=SPR/360 #Steps per degree
#DPS=1/SPD #degrees per step - note this is already adjusted for microstepping

#$% believe I can remove this line -- act_deg=0 #say throttle always at 0 deg when starting program, should be accurate if stepper motor was not holding torque before program runs (throttle return spring pulls closed)
cruise_spd = 12 #desired cruising speed in mph

sleep(0.1) #wait some time to be sure sleep pin is activated

#enter a forever while loop, until 'CTRL+c' keyboard interrupt occurs, then cleanup GPIO pins
try:
    itr = 0 #counter of iterations of while loop below
    itr_reset_count = 80 #number of iterations before iterations reset, also controls how often print to screen
    veh_spd_list = [] #creates an empty list of vehicle speeds, to be used later to find average accel rate
    mov_avg_itr_window=120
    time_list=[] #creates an empty list of times, to be used later for caclulating accel rates
    while True:
        act_spd = round(ax_spd_sens_v_to_veh_spd(axle_spd_sens.voltage),3)#gets voltage reading from axle input shaft speed sensor and converts to vehicle speed in mph
        des_spd = round(pps_v_to_des_spd(pps.voltage),2) #Converts voltage reading to desired vehicle speed in mph
#$%believe can remove this line         des_deg=des_spd*80/12 ###need to find des_deg until can use TPS input
        tps_deg = tps_v_to_deg_throttle(tps.voltage) #throttle opening in degrees
                
#$% believe can remove this line        if act_deg<0:
#$% believe can remove this line            act_deg=0
#$% believe can remove this line        else:
#$% believe can remove this line            pass
        
        #Add the current speed to the speed list, then remove the oldest item in the list.
        #if program has just started and list size is small, keep oldest item in list
        veh_spd_list.append(act_spd) 
        if len(veh_spd_list) > mov_avg_itr_window: 
            veh_spd_list=veh_spd_list[1:]
        else:
            pass
        #Similarly to above speed, do so with time as well.
        time_list.append(timeit.default_timer()) #add current time to time list
#         if len(time_list) > mov_avg_itr_window:
#             time_old=time_list[0]
#             time_list=time_list-time_old #subtracts oldest time to keep time list starting with 0 always
#             time_list=time_list[1:] #remove oldest time
#         else:
#             pass
    #$% The above is having issues, because a list cannot be subtracted from. May try to use numpy arrays instead?
        

        
        #this next if statement for testing program only
        if itr >= itr_reset_count: #print desired and actual throttle position once every 75 iterations (to make easier to read) (modulo)
            print("tps_deg:",tps_deg,"deg  ","act_spd:",act_spd,"Des_veh_spd:",des_spd)
#             print(time_list)
            itr = 0 #reset iterations
        else:
            pass
        
        #if statement for moving stepper motor
        if des_spd>act_spd and tps_deg<80: #steps motor forward or backward by 1 step (or does nothing if delta beteen des_deg & act_deg are less than the degrees per step, as to eliminate cycling back and forth 1 step constantly)
            GPIO.output(DIR,CW)
            GPIO.output(STEP,GPIO.HIGH)
            GPIO.output(STEP,GPIO.LOW)
            #act_deg=round(act_deg+1/SPD,2)
            sleep(delay)
        elif des_spd<act_spd and tps_deg>0:
            GPIO.output(DIR,CCW)
            GPIO.output(STEP,GPIO.HIGH)
            GPIO.output(STEP,GPIO.LOW)
            #act_deg=round(act_deg-1/SPD,2)
            sleep(delay)
        else:
            pass
        
        itr+=1 #add 1 to while loop iteration counter
        
        #axle_rpm=axle_spd_sens.voltage/axle_spd_sens_volt_per_rpm
        #act_spd=ax_rpm_to_veh_spd(axle_rpm)
        
        

except KeyboardInterrupt:
    GPIO.output(SLEEP,GPIO.LOW) #disable stepper motor (to keep from getting hot unneccesarily)
    GPIO.cleanup() #reset GPIO pins to inputs to protect against shorting accidentally
    print("GPIO pins cleaned up")
#Perhaps limit the amount of throttle by linkingthe throttle 0-max to position of pedal (0-max), then once reaching 1/2 or 3/4 of desired speed, then ramp down.