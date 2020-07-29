/*
Class to represent an electric damper unit.
Typical use case:
   DamperControl damper(enablePin, directionSelectPin, sensPin);
   ...
   damper.init();
   ...
   damper.cmd(byte pos);  // 0==closed, 255==open

enablePin OUTPUT enables damper to move.
directionSelectPin OUTPUT selects weather damper should open or close
sensPin INPUT reads a voltage proportional to current drive current

Most users should use ONLY these three calls
-----------------------------------------

Intended for use on a DC unit, where one pin is grounded to open damper,
a different pin is grounded to close damper.
12v  Power is supplied to the unit.
It tends to draw about 100mA  (1.2W) when moving

I had some trouble enabling with a FET.
Parasitic D-S capacatance on the large FET?  I dunno.
I changed to use a relay to enable motion.

A SPDT relay is used control weather the open or close pin will be grounded.

A shunt to ground should be attached to the direction select relay.
A LPF sensor of this voltage should be set up for the sensPin.
I used a 2.2K resistor, with a 22uF cap to ground, for RC~=0.5 sec
The LPF/shunt circuit will effect the expected sensed value when latched.
If you change these things, re-measure these values, and set them in
_latchedOpen and _latchClosed.

The relayPin should be in ground state to flip the relay to OPEN the damper.
The NC output (default.. high or no input) should be to close the damper.
The relay should only be "active" durring motion to OPEN the damper.

It appears that the damper must be closed and latched before
it can move toward opening.
If you stop half way, then command to move again, it will ALWAYS close.
If you wanted it to open, it will then turn around and open.

From closed and latched, when commanded to open, it seems
to back up a bit, then turn around and open.
This takes about 1.8 seconds.

There might be some backlash whenever motion changes direction.
The backlash seems to take about .2 sec.

Once motion starts, it takes about 9 seconds to go from one extreme
to the other.

It will be more straightforward, and model the operation of the damper
if you ALWAYS leave the "close" pin active, and ONLY activate
the "open" pin durring opening motion.

*/

#include <Arduino.h>

// This base class can be used for low-level test on the unit.
// It encapsulates use of the 3 control pins, and nothing more.
class DamperBase {
public:
  DamperBase(const int enPin, const int switchPin, const int sensPin)
  {
    _pinEnable = enPin;
    _pinSens = sensPin;
    _pinSwitch = switchPin;

    //pinMode(_pinEnable,OUTPUT);
    //pinMode(_pinSwitch,OUTPUT);
    //pinMode(_pinSens,INPUT);
  }

  // provides consistent interface weather or not logic is inverted
  //void move(bool s) { digitalWrite(_pinEnable,s?HIGH:LOW); }
  void move(bool s) {
    digitalWrite(_pinEnable,s?LOW:HIGH);  // ground to enable relay

    // after a stop, some sort of interlock happens, causing
    // damper to close when next enabled, even if you enable
    // open pin.
    // go with the flow.
    // Just set relay to close after motion stops, no matter
    // which direction
    if (!s)
      {
	delay(100);
        setOpen(false);  // always set to "close" mode after motion stops
      }
  }

  void begin() {
    pinMode(_pinEnable,OUTPUT);
    pinMode(_pinSwitch,OUTPUT);
    pinMode(_pinSens,INPUT);
    move(false);  // make sure not enabled
  }

  int sens() { return(analogRead(_pinSens)); }

  void setOpen(const bool open)
  {
    digitalWrite(_pinSwitch,open?LOW:HIGH);  // set logic invert or not 
  }
  
  int _pinEnable, _pinSens, _pinSwitch;
};



//class DamperControlPrivates;
class DamperControl : public DamperBase
{
 public:
  DamperControl();
  DamperControl(const int enablePin, const int relayPin, const int sensPin);
  void     init(const int enablePin, const int relayPin, const int sensPin);
  void init();  // re-initialize unit, in closed position
  void calibrate(const float trainingGain=0.0f);  // [0,1] --> 0==no training...blend...  1==ignore old values
  const byte getCmd(); // current desired position.  0==closed.  255==open
  const byte getPos(); // current estimated position, 0..255
  void cmd(const byte pos);  // request to move to this position 0..255
  void flush();  // force move to current requested position

  // NOTE:  int on AVR is 16 bit.  If sweep might get above 15 seconds,
  //        change these to long!!!
  int _msSweep; // msec to sweep open to close
  // Assert : this is same as time to get from close to open AFTER _msBacklashOpen

  int _msAccum;  // current state, as accumulated sweep msec
  int _msCmd;    // current desired position, 0..._msSweep


  int _msHyst;   // try to synch _msAccum to _msCmd if difference larter than this
  int _msBacklashOpen;  // time to start opening from closed (msec)
  int _msBacklash;  // this many msec to reverse direction from opening to closing

  // new circuit... nearly 0 counts when latched, as expected.
  // don't need this any more:
  //// expected sensor counts when latched, open and closed
  //long _latchedOpen, _latchedClosed;

  // Time needed to detect latch after sweep
  int _msLatch;

  int _latchThresh;  // less than this many counts when damper is latched open or closed

 protected:
  bool _opening;  // if last move was opening, set this so we know if we need to accomodate backlash
  void update();  // check command to see if position reset is necessary
  void setOpenFrac(const int openCount); // close, then set position
  void moveTowardClose(const int cmd); // move from position toward closed
  int runToLatch(const bool open = false, const byte verbose=0); // false to latch closed
  void setDefaultParameters();


  void forcePartialOpen(const int ms);
  void updateCal(const float trainingGain,
		 const int msPartialOpen, const int msPartialClose,
		 const int msOpen, const int msClose,
		 const byte verbose=0);

};

/* notes on failed attempt to enable and sense current over a FET

The drain of the enable FET should be sensed (through a large resistor, 
large enough to limit input current to << 1mA) to monitor damper motor current.
I found that a 22K current limiter, with a 22uF cap (sensor input to ground)
gave an overdamped, but reliable reading.

the enablePin should be connected to the FET gate.

*/
