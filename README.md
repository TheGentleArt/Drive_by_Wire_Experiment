# Drive_by_Wire_Experiment
My first 'useful' program, where I attempt to make a standalone makeshift throttle-by-wire system to put on a golf car.

I will preface this with stating that I am new to coding.
I recently got a Raspberry Pi and have been trying to do some physical computing with Python. After doing some Python tutorials online, I found that this method was not motivating me much, so I wanted to do something more useful. I have a desire to make a throttle-by-wire system operate on a golf car, and thus I am trying to learn.

My ultimate goal would be to have a motor control the position of the throttle (by means of pulley/cable is acceptable for now), having three inputs to do this.
The first input is the throttle position, which is controlled by the motor.
The second input is the pedal position.
The third input is the vehicle speed (axle input shaft speed or wheel speed etc.).

The program should work by being able to choose a target cruising speed and maximum acceleration rate for the vehicle. The program shall look at desired speed by means of pedal position (speed mapped, not torque mapped), and vehicle speed by means of a speed sensor mounted to a rotating part tied directly to the wheels (preferably with a higher angular speed than the wheels for more resolution), and compare the two speeds, adjusting throttle as needed.
The throttle shall be reduced if acceleration limits are approached. The throttle shall also not be attempted to be opened once it reaches a limit (max throttle opening) in order to not put undue stress on the system, and also shall not attempt to close the throttle once it reaches a limit (min throttle opening) in order to not have too much slack in the linkage (or putting too much stress in system if a rod linkage in lieu of a cable linkage), which may be undesirable as this would increase the amount of motor rotation required to result in throttle movement. 
The pedal position shall dictate the desired speed, but be non-linear to allow more granular control. This shall be done by making the pedal less sensitive at low speeds in order to make the user be able to more easily finely control vehicle movement at low speeds. 
I am currently debating doing this at high speeds as well, as to have a sideways 's' pattern of mapping, to allow the user to have a bit more fine control of speeds near the speeds of other non throttle-by-wire vehicles, as the traditional ground speed govenor systems are never an identical speed, and thus some vehicles may be governed slower and the user may want to match the speed of the leading vehicle more easily. 

A PID control will be nice. I'm not sure yet if PID is required, or if PI or PD etc. will work.
Currently, the program is being run on a table top setup, but I believe it is near ready to experiment on a vehicle.
It may be the case that the response is fast enough that PID is not required. Testing will have to be done soon.

This is mainly just a 'fun' project for me to try to learn on, instead of just following along with tutorials online that put me to sleep, with no real deliverable end goal that is meaningful to me.

I will update this once I believe I have a somewhat suitable program.
