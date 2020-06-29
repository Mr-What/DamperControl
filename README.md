# DamperControl
Arduino library to control an electric damper

This is a damper unit designed to run from a 12v DC control signal.
Supply 12v DC to the center (common) pin.
Grund one pin to open damper.
Ground the other pin to close the damper.
The motor shuts it self down (mechanically?) when in the open
or closed position.
It is designed to be used in a bang-bang fashion.
This library allows users to command it to a partially open position.

The motion of this damper is a little complex.
It seems to have interlocks so that it returns to the closed state
under all but the proper input command.
It must be in closed position.
When grounding the open pin, it backs up a bit, I think this throws
some sort of switch allowing it to open.
It then opens.
Whenever it stops, no matter what pin is grounded, it returns to closed
position.

For this design, the user is expected to attach a SPDT relay to the
direction grounding pins.
The NC connection should be for the closing pin.
I found operations most reliable when another relay is used on the
high side to supply 12v DC power to the unit.
It draws only about 100mA when the motor is driving.

I added a 2.2 ohm shunt between the common pin of the direction
select relay and ground.
Current sensed over this shunt is very noisy when under drive,
so I added an analog LPF for the sensor pin.
I used a 2.2K resistor at the shunt node, to a 22uF cap to ground.
This gives a time constant of around 0.05 sec.
At this bandwidth, response to change of current is slightly underdamped
with my arduino's roughly 10Hz sampling rate.
I look at noise on this current signal to determine if the motor
is driving.
You do not want to eliminate this noise entirely.

One unfortunate part of this initial design is that the
user must supply an estimate of the current reading when the damper
is latched in one of the open or closed positions.
This should be easily determined by watching the serial port
during initialization.
If your guess is close enough, it will improve with use and
train itself on proper offsets.
Advanced users may wish to record these values, and store them
in EEPROM, so that system performance improves with use.

See DamperControl.h comments for more information.
