#J.Williams
#This script intends to move a stepper motor. It uses a RPi, a DRV8825 Stepper Motor Driver, a breadboard, a NEMA 17 Stepper Motor, a ADS1115 Analog to Digital Converter, and a power supply for now.
#A large portion of getting the stepper motor working correctly was achieved via watching a tutorial from 'rdagger68' on Youtube titled 'Raspberry Pi Stepper Motor Tutorial'.
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
    #RESET> Pi 3.3V+ (Supposedly DRV8825 will not operate if this is not pulled to HIGH position)
    #SLEEP> Pi GPIO 17 (Physical Pin 11) (Could also be placed on 3.3V+ pin, but would always leave stepper motor engaged, which may result in unnecessary power draw)
    #STEP> Pi GPIO 21 (Physical Pin 40) (IIRC when pulsed, tells motor to take one step)
    #DIR> Pi GPIO20 (Physical Pin 38)
    #VMOT> Power Supply 12V+ (Be careful connecting anything over 3.3V as this may damage the Pi) (Also note that a 100 uF capacitor was used across the VMOT/GND_MOT pins in case of voltage spike issues)
    #GND_MOT> Power Supply 12V- (also connect to GND LOGIC Pin just in case the other ground circuit is bad)
    #B2> Motor Pole (Pair with B1) (Do not connect until DRV8825 Pot is adjusted)
    #B1> Motor Pole (Pair with B2) (Do not connect until DRV8825 Pot is adjusted)
    #A1> Motor Pole (Pair with A2) (Do not connect until DRV8825 Pot is adjusted)
    #A2> Motor Pole (Pair with A1) (Do not connect until DRV8825 Pot is adjusted)
    #FAULT> Not Used
    #GND_LOGIC> Pi GND (Physical Pin 39) (also connect to GND_MOT Pin just incase other ground circuit is bad)
#####


#####Description of how this was connected. This may be redundant information.
    #DRV8825 Logic_GND and Motor_GND pins tied to GND of breadboard. Logic GND and Motor GND were tied together at the GND on the breadboard, just to ensure a common ground.
    #Motor Ground tied to the power supply ground (negative terminal)
    #VMOT tied to the power supply (+ terminal)(Do NOT connect the power supply (+) to anything but the VMOT pin on the DRV8825, as this will be using more voltage than the Pi can handle and will damage it.)
    #100 uF capacitor used across the VMOT and GND pins for the DRV8825. This is done to have a capacitor in parallel so that any voltage spikes will be 'absorbed' by the capacitor.
    #SLEEP pin on the DRV8825 is connected to GPIO pin 17 on Pi. (Can connect this to 3.3V (+) pin if do not need to put the motor to sleep when not turning (pedal not pressed) if want to save power.)
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
    #
    #The HPC pedal setup was done by connecting wires to the Delphi 6 pin connector, filling the A, B, and C terminals.
    #Believe this is a Mouser PN 829-12162261 connector
    #Pin A(F) is signal out, pin B(E) is ground, and pin C(D) is 5V in.
    #
    #The axle input shaft speed sensor was setup using pin 1 as power in, pin 2 as output signal, and pin 4 as ground.
#####
    
#####ADS1115 Pin Layout ---(and Placement)
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
#####
    


#####Library Imports
import time
from time import sleep
import RPi.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import board
import busio
import math
#import timeit
#import numpy as np
#####



#####ADS1115 channel voltage input code (use 'channel_name.voltage' to get voltage):
#Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)
#Create the ADC object using the I2C bus
ads = ADS.ADS1115(i2c)
ads.gain = 1 #GAIN
#Create single ended input on channel 0 (A0)
pps = AnalogIn(ads, ADS.P0) #Pedal Position Sensor
axle_spd_sens = AnalogIn(ads, ADS.P1) #Axle Shaft Speed Sensor
tps = AnalogIn(ads, ADS.P2) #Throttle Position Sensor
#####



#####Setup

#Motor Info
#NEMA17 Stepper Motor (PN: 17HS19-2004S1)
Step_deg = 1.8 #degrees for each step

#Setting up Microstepping dictionary 
#Microstepping Resolution Setup for DRV8825 [From DRV8825 datasheet]
#This will later be used to allow motor to more finely tune vs move quickly, depending on speed conditions. See step_mode() function.
RESOLUTION = {'Full': (0,0,0),
             'Half': (1,0,0),
             '1/4':  (0,1,0),
             '1/8':  (1,1,0),
             '1/16': (0,0,1),
             '1/32': (1,0,1),
             '1/64': (1,1,1)}


#Setup for RPi pin being high/low depending on motor direction desired
CW = 1 #Clockwise rotation
CCW = 0 #Counterclockwise rotation

#RPi Pin Assignments 
DIR = 20 #Direction --- GPIO Pin Label
STEP = 21 #Step --- GPIO Pin Label
MODE = (14,15,18) #Microstep Resolution Mode --GPIO Labels for M0, M1, and M2.
SLEEP = 17 #Sleep --- GPIO Pin Label

#GPIO Setup
GPIO.setmode(GPIO.BCM) #Set Pins to use the GPIO labels/broadcom labeling system instead of physical pin assignment
GPIO.setup(DIR, GPIO.OUT) #Direction Pin on Pi is set to an output pin
GPIO.setup(STEP, GPIO.OUT) #Step Pin on Pi is set to an output pin
GPIO.setup(MODE, GPIO.OUT) #Set mode pin on Pi as an output.
GPIO.setup(SLEEP, GPIO.OUT) #Set sleep pin on Pi as an output pin.
GPIO.output(DIR, CW) #Direction set to CW initially (Rpi pulls DIR pin high?) 
GPIO.output(SLEEP, 1) #enable stepper motor
#####



###Program###
#Program will be written to move stepper motor forward (open throttle) if throttle is not already opened max
#also only if the desired speed is higher than actual speed, and also only if acceleration rate is not too high.

#Functions
def ax_spd_sens_v_to_veh_spd(): #convert axle speed sensor voltage to vehicle speed in mph
    #n_teeth = 32 #number of teeth per revolution of axle input shaft 
    axle_speed_sensor_v_in = 3.3 #Sensor Supply Voltage
    axle_spd_sens_volt_per_rpm = axle_speed_sensor_v_in/5500 #volts per 5500 rpm (25mph with 18" tires)
    axle_input_rpm = axle_spd_sens.voltage/axle_spd_sens_volt_per_rpm #This line takes the input from the speed sensor
    axle_ratio = 11.47
    tire_dia = 18 #inch
    f_roll_rad = 0.965 #rolling radius factor
    tire_circ = tire_dia*math.pi*f_roll_rad 
    tire_rpm = (axle_input_rpm)/axle_ratio
    veh_spd_inchpermin = tire_rpm*tire_circ
    veh_spd = veh_spd_inchpermin*((60/1)*(1/12)*(1/5280))
    if veh_spd < 0:
        veh_spd = 0
    return round(veh_spd,2)
    
def pps_v_to_des_spd(): #convert pedal position sensor voltage to desired speed, in mph
    pps_v_in = 3.3 #3.3 or 5 (ECU delivers 5V)
    pps_mode = "potentiometer" #"potentiometer" or "pedal"
    if pps_mode == "potentiometer":
        pps_v_min = 0
        if pps_v_in == 3.3:
            pps_v_max = 3.3
        elif tps_v_in == 5:
            pps_v_max = 5
    elif pps_mode == "pedal":
        if pps_v_in == 3.3:
            pps_v_min = 0.724 #measured value with HPC
            pps_v_max = 2.81 #measured value with HPC
        elif pps_v_in == 5:
            pps_v_min = 1.15 #measured value with HPC
            pps_v_max = 3.876 #measured value with HPC
    else:
        print ("Error with pps_v_to_des_spd function")
    #
    des_spd = cruise_spd*(pps.voltage-pps_v_min)/(pps_v_max-pps_v_min) #This line takes the input from the pps
    if des_spd < 0:
        des_spd = 0
    return round(des_spd,2)

def tps_v_to_deg_throttle(): #convert throttle position sensor voltage to deg of opening
    tps_voltage = tps.voltage #Storing TPS voltage as variable so that everything is evalauted from same value within this function
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
    #
    if tps_voltage < tps_v_min: #in case tps signal is slightly lower from throttle body, round to tps_v_min for now
        tps_voltage = tps_v_min
    elif tps_voltage > tps_v_max: #in case tps signal is slightly higher from throttle body to throttle body, round to tps_v_max value for now
        tps_voltage = tps_v_max
    #
    deg_throttle = 0+(tps_voltage-tps_v_min)*(deg_throttle_max-deg_throttle_min)/(tps_v_max-tps_v_min) #linear interpolation
    if deg_throttle == -0: #was seeing negative zero sometimes, did not want to see negative zero. did not want to use abs() in case it goes below negative zero I would want to see it in testing.
        deg_throttle = 0
    #
    return deg_throttle

def spd_error(): #calculates the difference between desired speed and actual speed
    diff = des_spd - act_spd #Positive Value indicates user commanding to go faster
    return diff

def step_mode(): #Allows the motor driver to control microstepping, depending upon how close actual and desired speeds are, in order for throttle control to be more precise vs quick
    diff = abs(spd_error())
    if diff < 0.25:
        step_mode = '1/64'
    elif diff < 0.5:
        step_mode = '1/32'
    elif diff < .75:
        step_mode = '1/16'
    elif diff < 1.25:
        step_mode = '1/8'
    elif diff < 1.75:
        step_mode = '1/4'
    elif diff < 2.5:
        step_mode = 'Half'
    else:
        step_mode = 'Full'
    return step_mode

def delay():
    delay_FullStep=0.00125 #Time to dealay between each step, if full stepping
    STEP_MODE = step_mode() #Assigning to variable so does not iterate through function each time below
    if STEP_MODE == 'Full':
        delay = delay_FullStep
    elif STEP_MODE == 'Half':
        delay = delay_FullStep/2
    elif STEP_MODE == '1/4':
        delay = delay_FullStep/4
    elif STEP_MODE == '1/8':
        delay = delay_FullStep/8
    elif STEP_MODE == '1/16':
        delay = delay_FullStep/16
    elif STEP_MODE == '1/32':
        delay = delay_FullStep/32
    elif STEP_MODE == '1/64':
        delay = delay_FullStep/64
    return delay

#End of functions



cruise_spd = 12 #desired cruising speed in mph
accel_rate_cap = 2.5 #mph/s --- This is the maximum acceleration rate desired. If going beyond this, throttle should be limited
sleep(0.1) #wait some time to be sure sleep pin is activated

#enter a forever while loop, until 'CTRL+c' keyboard interrupt occurs, then cleanup GPIO pins
try:
    print("Program Begun ; Press 'CNRL+c' to stop program and cleanup GPIO Pins")
    accel_rate = 0 # setting initial acceleration rate, in mph/s, set to 0 until enough data to change
    itr = 0 #counter of iterations of while loop below, setting to 0 initially
    print_itr_reset_count = 25 #number of iterations before iterations reset for print loop, controls how often values print to screen, if that section of code not commented out
    mov_avg_itr_window = 25
    veh_spd_list = [] #creates an empty list of vehicle speeds, to be used later to find average accel rate
    time_list = [] #creates an empty list of times, to be used later for caclulating accel rates

    while True:
        #get the current values of the sensors, and convert to useful numbers
        act_spd = ax_spd_sens_v_to_veh_spd()#gets voltage reading from axle input shaft speed sensor and converts to vehicle speed in mph
        des_spd = pps_v_to_des_spd() #Converts voltage reading to desired vehicle speed in mph
        tps_deg = tps_v_to_deg_throttle() #throttle opening in degrees, used to determine if throttle already at max/min limits before moving stepper motor further             
        
        #Calculate acceleration rate (the time period is controlled by the iterations 'mov_avg_itr_window' size now, but can change to wait for difference to be over some value...
        veh_spd_list.append(act_spd)  #add current vehicle speed to vehicle speed list
        time_list.append(time.perf_counter()) #add current time to time list
#         if len(veh_spd_list) != len(time_list): #warn if lists are not equal length
#             print("vehicle speed list length does not match time list length")
#             break
        if len(veh_spd_list) >= mov_avg_itr_window:  #allow list length to build before removing old data
            veh_spd_list = veh_spd_list[1:] #remove the oldest speed in list
            time_list = time_list[1:] #remove the oldest time in list        
            accel_rate = (veh_spd_list[-1]-veh_spd_list[0])/(time_list[-1]-time_list[0]) #calculate accel rate from oldest and newest speed and time values in lists
            # confirmed accel rate calculated correctly, but need to look into the time delta of this.
            # may want to have statement to only calculate if time difference is a certain delta or greater? perhaps 5 Hz?      
              
        GPIO.output(MODE, RESOLUTION[step_mode()]) #Sets full/microstepping up. See 'RESOLUTION' dictionary. This is used in order to change the speed at which the throttle will move, so that near cruising speeds it can be more fine tuned, etc.
        
        #This section for moving stepper motor --- steps motor forward or backward by 1 step (or does nothing if delta beteen des_deg & act_deg are less than the degrees per step, as to eliminate cycling back and forth 1 step constantly)
        if des_spd > act_spd and tps_deg < 79 and accel_rate < accel_rate_cap:  #want to go faster, not hitting max throttle or accel rate cap
            GPIO.output(DIR, CW)
            GPIO.output(STEP, GPIO.HIGH)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay())
        elif des_spd > act_spd and tps_deg > 0 and accel_rate > accel_rate_cap: #want to go faster, but hitting acccel rate cap
            GPIO.output(DIR, CCW)
            GPIO.output(STEP, GPIO.HIGH)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay())
        elif des_spd < act_spd and tps_deg > 0: #vehicle going faster than desired, close throttle
            GPIO.output(DIR, CCW)
            GPIO.output(STEP, GPIO.HIGH)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay())

        itr += 1 #add 1 to while loop iteration counter
        
        
#         #This next portion for testing only
#         print("speeds:",veh_spd_list[0:4],"...",veh_spd_list[-4:])
#         print("time:",time_list[0:4],"...",time_list[-4:])
#         print("accel_rate:",accel_rate)
#         print("")
#         if len(time_list) > 5:
#                print("time_delta:",(time_list[-1]-time_list[-2]))
                 
        #this next if statement/section for testing program only
        #may need to change above itr line or reset if removing this section
        if itr >= print_itr_reset_count: #print desired and actual throttle position once every 75 iterations (to make easier to read) (modulo)
            print("tps_deg:",tps_deg,"deg  ","act_spd:",act_spd,"Des_veh_spd:",des_spd,"  accel_rate:",accel_rate," mph/s")
            itr = 0 #reset iterations
            

except KeyboardInterrupt:
    GPIO.output(SLEEP, GPIO.LOW) #disable stepper motor (to keep from getting hot unneccesarily)
    GPIO.cleanup() #reset GPIO pins to inputs to protect against shorting accidentally
    print("Program Ended; GPIO pins cleaned up")

#General Comments on things to do:
#May want to look into accel rate time and make a minimum time delta before it overrides the accel rate calc? Since RPi time does not seem consistent...
#Nice to have a different ramp rate for different speed range. Say beginning is less sensitive, then as approaching desired speed, more sensitive. Perhaps a  then once reaching ~1/2 or 3/4 of desired speed, then ramp up sensitivity with max accel rate changes? Probably can control via PI/PID control similar though? Not sure if need that yet
