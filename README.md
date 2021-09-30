# Throttle_by_Wire_Experiment
My first 'useful' program, where I attempt to make a standalone makeshift throttle-by-wire system to put on a golf car.
This repo is not licensed, purposely. 

I will preface this with stating that I am new to coding. Ultimately, this program was initiated in order to learn. 
(If you have any constructive criticism, please share!)
This is the first program that I have made that I am actually using to accomplish some task. My hopes are that this program, being used for something I have wanted to do for a couple of years now, will keep me more motivated to learn than watching some tutorials online that I generally have little interest in.

The program ultimately takes four inputs, and uses these to control the throttle position.
With these four inputs, the program sends signals to a stepper motor driver in order for a stepper motor to control a throttle body.
The first input is the throttle position, which is controlled by the motor via a pulley/cable system. This is useful to know so the stepper motor does not step too far.
The second input is the pedal position, which shall be mapped to desired speed like most electric golf cars (contrary to most automobiles which use torque-based pedal mapping... may want to change this into a torque-based mapping eventually though).
The third input is the vehicle speed. This is gathered by means of a hall effect sensor and teeth (on a cogged ring pressed) on the axle input shaft. This should be a dual slope sensor if this is to be used by others besides myself one day, as that is an important safety feature, and will also need safeties built in if the two channels do not agree. For now, I am fine with running a single slope. In fact, all the other sensors could be improved by this, but I believe this one to be most critical.
The fourth input is the pedal switch. This is not necessary, but helpful for safety reasons. This is used to ensure that the program only attempts to accelerate the vehicle if both the pedal positions sensor value dictates so as well as confirming the pedal is indeed down.

The program works by a target cruising speed and maximum acceleration rate for the vehicle being set.
The program compares the two speeds (desired from the pedal ; actual from the hall sensor), adjusting throttle as needed.
The throttle is reduced if acceleration limits are approached. 
The throttle position sensor is used to determine if stepping further would put undue stress on the system. It is also used in order to not attempt to close the throttle once it reaches a limit (min throttle opening) in order to not have too much slack in the linkage (or putting too much stress in system if a rod linkage in lieu of a cable linkage), which may be undesirable as this would increase the amount of motor rotation required to result in throttle movement. 
The pedal position dictates the desired speed, and is non-linearly mapped to allow more granular control. This is done by making the pedal less sensitive at low speeds in order to make the user be able to more easily finely control vehicle movement at low speeds. 
(This is currently also done at high speeds as well, as to have a sideways 's' pattern of mapping, to allow the user to have a bit more fine control of speeds near the speeds of other (non-throttle-by-wire vehicles), as the traditional ground speed govenor systems are never an identical speed, and thus some vehicles may be governed slower and the user may want to match the speed of the leading vehicle more easily.)

PID control may be looekd at in the future. I'm not sure yet if PID is required, or if PI or PD etc. will work.
It may be the case that the response is fast enough that PID is not required. 

The program was being run on a table top setup. A vehicle is currently being modified now in order to transition to usage on a test vehicle. The pedal switch still needs to be wired into the pi circuit and I'm trying to figure out how to get measurements from the hall effect sensor over a period of time accurately while not pausing the other steps in the code to do so. I am considering getting another microcontroller to do that or attempt multithreading for this.

This is mainly just a 'fun' project for me to try to learn on, instead of just following along with tutorials online that put me to sleep, with no real deliverable end goal that is meaningful to me.

-JW

