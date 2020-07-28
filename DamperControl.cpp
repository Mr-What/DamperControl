/*

*/

#include <DamperControl.h>


//class DamperControlPrivates{};

DamperControl::DamperControl(const int enablePin, const int relayPin, const int sensPin) : DamperBase(enablePin,relayPin,sensPin) {
  move(false);

  _accum = 0;  // current state, as accumulated msec
  _cmd = 0;  // current desired state (msec into opening sweep)
  _hyst = 500;   // don't bother trying to move less than this much
  _hystToOpen = 1800;  // hysterisis to start opening from closed
  _toToggle = 9000; // total _accum for full range of position
  _backlash = 200;  // about this many msec to reverse backlash
  _opening = false;  // previous motion direction, to see if we need backlash adjustment
  _latchedOpen = 2; //100;  // sens counts when latched open
  _latchedClosed = 1; //60;  // sens counts when latched closed
  _toggleFuzz = 300;  // extra "fuzz" accum to latch
}

void printTab() { Serial.print('\t'); }
void printSpace() { Serial.print(' '); }
void printTime() {
  Serial.print(millis()*0.001,2);
  printTab();
}
void print4(const int v) {
  if (v < 1000) {
    printSpace();
    if (v < 100) {
      printSpace();
      if (v < 10) {
	printSpace();
      }
    }
  }
  Serial.print(v);
}

void printTo() { Serial.print(F(" to ")); }

long sweepTimeUpdate(const __FlashStringHelper *msg,
		     long sweepTime,
		     const long fullSweep, const long backlash)
{
  printTime();
  long sweepEst = fullSweep - backlash;
  if ( (sweepEst < (sweepTime>>1)) || (sweepEst > (sweepTime<<1)) )
    {
      Serial.print(msg);
      Serial.print(F(" time from "));
      Serial.print(sweepTime);  printTo();
      Serial.print(sweepEst);
      Serial.println(" unlikely.  NO UPDATE");
      return(sweepTime);
    }
  Serial.print(F(" changing "));
  Serial.print(msg);
  Serial.print(F(" time from "));
  Serial.print(sweepTime);  printTo();
  sweepTime = (sweepTime + sweepEst) / 2;   /* move halfway to new estimate */
  Serial.print(sweepTime); printTab();
  Serial.print(sweepEst);  Serial.print('+');
  Serial.print(backlash);  Serial.print(F("="));
  Serial.println(fullSweep);
  return(sweepTime);
}

void DamperControl::init()  // re-initialize unit, in closed position
{
  unsigned long t0,tClose0,tOpen1;

  // try to force to close position
  Serial.println(F("Make sure closed before starting"));
  runToLatch(false,3);  // close

  // should be closed now.
  _accum = 0;
  Serial.println(F("Check time to open"));
  long toOpen = runToLatch(true,3);  // estimated stall on timeout.

  _toToggle = sweepTimeUpdate(F("opening"),_toToggle,toOpen,_hystToOpen+_toggleFuzz);
  
  // measure time to close
  Serial.println(F("Checking time to close"));
  long toClose = runToLatch(false,3);  // estimated stall time on timeout

  _toToggle = sweepTimeUpdate(F("closing"),_toToggle,toClose,_backlash+_toggleFuzz);

  _cmd = 0;  // current desired state
  _opening = false; // false==closing
}

byte clamp8(long p)
{
  if (p>255) p=255;
  if (p < 0) p = 0;
  return((byte)p);
}

long cmd2ms(const byte cmdByte, const long msSweep)
{
  static byte prev = 111;

  // let command [0..255] be proportional to amount of surface area blocking duct
  float frac = ((float)cmdByte) * 0.003922f;  // (1/255)
  if (frac < 0.01f)
    frac = 0.0f;
  if (frac > .999f)
    frac = 0.999f;

  if (cmdByte != prev)
    {
      Serial.print(F("Command Byte "));
      Serial.print((int)cmdByte);printTab();
      Serial.print(frac,3);
      Serial.print(F(" of area --> "));
    }
  
  frac = asin(frac) / 1.57f;  // now fraction of sweep time

  
  if (cmdByte != prev)
    {
      Serial.print(frac,3);  Serial.print(F(" of sweep "));
    }
  
  long ms = (long)(frac * msSweep);
  if (ms < 20)
    ms = 0;
  if (ms > msSweep-30)
    ms = msSweep + 20;

  if (cmdByte != prev)
    {
      Serial.print(ms);Serial.print('/');Serial.println(msSweep);
    }
  prev = cmdByte;
  return(ms);
}
byte ms2cmd(const long ms, const long msSweep)
{
  static long prev = -99;
  
  //long p = (255 * ms) / msSweep;

  // make proportional to surface area, assuming equal angular velocity with time
  float ang = ( 1.571f * ((float)ms) ) / msSweep;

  byte cmdByte = (ang > 1.57f) ? ((byte)255) :
    ((ang < 0.002f) ? ((byte)0) : ((byte)((long)(255.0f * sin(ang)))) );

  if (ms != prev)
    {
      Serial.print(ms);Serial.print('/');Serial.print(msSweep);
      Serial.print(F(" --> "));
      Serial.print(ang,3);Serial.print(F("rad  Cmd Byte : "));
      Serial.println((int)cmdByte);
    }
  
  prev = ms;
  return(cmdByte);
}

// current commanded  position.  0==closed.  255==open
const byte DamperControl::getCmd()
{
  return(ms2cmd(_cmd,_toToggle));
}

// current estimated position.  0==closed.  255==open
const byte DamperControl::getPos()
{
  return(ms2cmd(_accum,_toToggle));
}

// command to try to move state to this level
void DamperControl::cmd(const byte pos)
{ 
  _cmd = cmd2ms(pos,_toToggle);
  update();
}

void DamperControl::update()
{
  Serial.print(F("%Update() _cmd="));
  Serial.print(_cmd);
  Serial.print(F("  _accum="));
  Serial.print(_accum);
  Serial.print(F("\t_toToggle="));
  Serial.print(_toToggle);
  Serial.print(F("\t_hyst="));
  Serial.println(_hyst);
  
  if ((_cmd < _toToggle/50) && (_accum > 0))  // nearly closed --> just close it
    {
      Serial.println(F("%request (nearly) closed, latch damper closed."));
      runToLatch(false,1);
      return;
    }
  if ((_cmd >= 19 * _toToggle / 20)  && (_accum < _cmd)) // nearly open --> just open it
    {
      Serial.println(F("%request (nearly) open, latch damper open."));
      runToLatch(true,1);
      return;
    }

  long dAccum = _cmd - _accum;
  Serial.print(F("Update position "));  Serial.println(dAccum);
  long a = (dAccum > 0) ? dAccum : -dAccum;
  if (a > _hyst)
    flush();
}

// reset to given open fraction (of _toToggle)
void DamperControl::setOpenFrac(long openCount)
{
  if (openCount < 10)
    {
      runToLatch(false);  // close
      return;
    }
  if (openCount >= _toToggle-20)
    {
      runToLatch(true); // open
      return;
    }
  Serial.println(F("Restart from closed before setting position"));
  long cmd = openCount;
  long toClose = runToLatch(false); // close

  // this and runToLatch(true) are the ONLY places we should EVER
  // enable the opening pin!!!  EVER!!!!
  setOpen(true);

  Serial.print(F("opening to "));
  Serial.print(cmd);
  Serial.print('/');
  Serial.println(_toToggle);
  cmd += _hystToOpen;  // add time to un-latch and start moving

  // I dunno why this is necessary, but it seems to be.
  // the hysterisis I see when doing this seems to be longer than observed in init.
  // perhaps relays changing too quickly?
  cmd += 1200;
  delay(10);

  unsigned long t0 = millis();
  move(true);
  long t = 0;
  while (t < cmd)
    {
      //printTime();
      //Serial.println(t);
      delay(2);
      t = millis() - t0;
    }
  move(false);
  _accum = openCount;//t - _hystToOpen;
  printTime();
  Serial.print(t);//_accum);
  Serial.print(F(" target "));
  Serial.println(openCount);
  _opening = true;
}

void DamperControl::moveTowardClose(const long cmd)
{
  if (cmd < 1)
    {
      runToLatch(false);  // close
      return;
    }
  setOpen(false);  // make sure closing pin is selected
  long dt = _accum - cmd;
  Serial.print(F("moving toward closed "));
  Serial.println(dt);
  unsigned long t0 = millis();
  move(true);
  long t = 0;
  if (_opening)
    dt += _backlash;  // add backlash adjustment to change direction
  _opening = false;
  while (t < dt)
    {
      //printTime(); printTab();
      //Serial.println(t);
      delay(1);
      t = millis() - t0;
    }
  move(false);
  _accum -= t;
}

void DamperControl::flush()  // force position synch
{
  long cmd = _cmd;
  if (cmd == _accum)
    return;  // no action necessary
  if (cmd >= _toToggle-50)
    {
      Serial.println(F("flush()->open"));
      runToLatch(true);  // open
      return;
    }
  if (cmd < 20)
    {
      Serial.println(F("flush()->closed"));
      runToLatch(false);  // open
      return;
    }

  if (cmd < _accum)
    { // will always move toward close after stop.  let it do so
      Serial.println(F("flush()->toward closed"));
      moveTowardClose(cmd);
      return;
    }
  // no matter what state was, need to close, latch, then move
  // to desired open fraction
  Serial.println(F("flush()->reset to partial open"));
  setOpenFrac(cmd);
}

bool latched(const long latchCount,
	     const long muEst,
	     const float errEst,
	     const long nReadings)
{
  /*
  Serial.print(F("latched "));
  Serial.print(latchCount); printSpace();
  Serial.print(muEst); printSpace();
  Serial.print(errEst,2); printSpace();
  Serial.println(nReadings);
  */
  
  if (nReadings < 4)
    return(false);

  // tends to have stable "leakage" current when latched,
  //   but oscillating/aliased drive current when moving
  if (errEst > 1.8)
    return(false);

  // error w.r.t. expected latched current reading
  long d = (muEst > latchCount) ? muEst - latchCount :
    latchCount - muEst;
  if (d > 8)
    return(false);

  return(true);
}

long DamperControl::runToLatch(const bool open, const byte verbose)
{
  if (open)
    {
      Serial.println(F("Reset damper to closed before attempting to open."));
      runToLatch(false,verbose);
    }

  long t;
  long latchTimeout = 3 * _hystToOpen;
  long timeout = latchTimeout + 2 * _toToggle;

  _opening = open;
  setOpen(open);
  if (verbose > 0)
    {
      Serial.print(open?F("%opening"):F("%closing"));
      Serial.println(F(" to latch"));
    }
  long nReadings = 0;
  long muEst = 9999;
  float errEst, errEst1, errEst2;
  //long muEst0 = muEst;
  //long varEst0 = varEst;

  // expected sensor count when latched
  int latchCount = (open) ? _latchedOpen : _latchedClosed;
  if (verbose > 1)
    {
      Serial.print(F("%Expected latched current "));
      Serial.print(latchCount);
      Serial.print(F(" timeout "));
      Serial.println(timeout);
    }

  move(true);
  unsigned long t0 = millis();
  long err = 9999;
  int nStep=0;  /* number of samples in a row detecting a step */

  if (verbose > 1)
    Serial.println(F("%sec nRead sens est err"));

  t = 0;
  // ignore latched() until current reading is reliable
  while( (t < 500) || (!latched(latchCount,muEst,errEst,nReadings)) )
    {
      long val = sens();

      if (verbose > 1)
	{
          printTime();
          print4(nReadings); printTab();
          print4(val);//       printSpace();
	}

      if (nReadings == 0)
	{
	  muEst = val;
	  errEst = errEst1 = errEst2 = 1;
	  nStep=0;
	}
      else
	{
	  // division on 8-bit processor is expensive.
	  // use bit-shift for this instead
	  //// let estimate be weighted (7/8)oldEst + (1/8)newReading
	  // let estimate be weighted (3/4)oldEst + (1/1)newReading
	  long prev = muEst;
	  muEst = (val + (muEst * 3)) >> 2;
	  if (muEst == prev)
	    { // make sure it converges, when est is close.
	      // problem when its a couple of counts off, and won't converge
	      if (val > muEst)
		muEst++;
	      else if (val < muEst)
		muEst--;
	    }
	  err = (val > muEst) ? val-muEst : muEst-val;  // abs err
	  //err = val - muEst;   err *= err;  // squared err
	  errEst = (err + (errEst * 4)) / 5;
	  if (errEst < 0.3f)
	    errEst = 0.3;  // min practical err
	}
      nReadings++;

      if (verbose > 1)
	{
          print4(muEst); printSpace();
          Serial.println(errEst,1);
	}

      // detect step in output, compare to error estimate from a few samples ago
      if ( (nReadings > 16) && (err > (errEst2*6)) )
	{
	  nStep++;
	  if (nStep > 1) // reset mu and err estimates
	    nReadings = 0;
	}
      else
	{
	  nStep = 0;
	  errEst2 = errEst1;
	  errEst1 = errEst;
	}  

      t = millis() - t0;
      if (t > timeout)
	{
	  t = -t;
	  printTime();
	  Serial.println(F("TIMEOUT"));
	  goto SHUTDOWN;
	}
    }
  _cmd = _accum = (open) ? _toToggle + _toggleFuzz : 0;
 SHUTDOWN:
  move(false);  // also sets direction relay to default : close
  if (verbose > 0)
    {
      printTime();
      Serial.print(t);
      Serial.print(F(" elapsed, latched at "));
      Serial.println(_cmd);
    }
  if (t > 1000)
    { // update latch level estimate
      long dLatch = muEst - latchCount;
      dLatch /= 2;  // damp correction a bit
      if (dLatch != 0)
	{
          printTime();
          Serial.print(open?F(" open"):F(" closed"));
          Serial.print(F(" latch count update "));
          Serial.print(latchCount);
          dLatch += latchCount;
          Serial.print(F(" to "));
          Serial.println(dLatch);
          if (open)
	    _latchedOpen = dLatch;
          else
	    _latchedClosed = dLatch;
	}
    }
  return(t);
}
