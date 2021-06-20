# Drive_by_Wire_Experiment
My first 'useful' program, where I attempt to make a standalone makeshift drive by wire system to put on a golf car.

I will preface this with stating that I am new to coding.
I recently got a Raspberry Pi and have been trying to do some physical computing with Python. I have ran across some tutorials online and wanted to do something more useful. I have a desire to make a drive by wire system operate on a golf car, and thus I am trying to learn.

My ultimate goal would be to have a motor control the position of the throttle (by means of pulley/cable is acceptable for now), having three inputs to do this.
The first input is the throttle position, controlled by the motor.
The second input is the pedal position.
The third input is the vehicle speed (axle input shaft speed or wheel speed etc.).

I want to be able to choose a target cruising speed for the vehicle, say 12 mph. I would have a maximum acceleration rate, where I would monitor the wheel speed acceleration, and limit throttle accordingly. The pedal position will dictate the desired speed. Pedal 100% down would be cruising speed, Pedal 0% down (pedal up) would be 0 mph desired, and pedal 35% down would be cruising_speed*0.35 desired.

A PID control will be nice. I'm not sure yet if PID is required, or if PI or PD etc. will work. 

This is mainly just a 'fun' project for me to try to learn on, instead of just following along with tutorials online.

I will update this once I believe I have a somewhat suitable program.
