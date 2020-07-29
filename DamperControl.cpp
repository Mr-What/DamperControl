/*

*/

#include <DamperControl.h>


//class DamperControlPrivates{};

DamperControl::DamperControl(const int enablePin, const int relayPin, const int sensPin) : DamperBase(enablePin,relayPin,sensPin) {
  move(false);

  _msSweep = 9000; // time to sweep from open to closed (and vica-versa)
  _msAccum = 0;  // current state, as accumulated sweep msec
  _msCmd = 0;  // current desired state (msec into opening sweep)
  _msHyst = 500;   // don't bother trying to move less than this much
  _msBacklash = 200;  // about this many msec to start closing from opening
  _msBacklashOpen = 1800;  // ms to start open sweep from closed and latched
  _msLatch = 200;  // time after sweep to latch and detect latched state

  _latchThresh = 8;  // less than this many sens() counts when latched
  //_latchedOpen = 2; //100;  // sens counts when latched open
  //_latchedClosed = 1; //60;  // sens counts when latched closed

  _opening = false;  // previous motion direction, to see if we need backlash adjustment
}

void printTab() { Serial.print('\t'); }
void printSpace() { Serial.print(' '); }
void printSlash() { Serial.print('/'); }
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
void printParams(const int S, const int Bo,const int latch, const int B,
		 const __FlashStringHelper *note)
{
  Serial.print('%');
  Serial.print(S    ); printTab();
  Serial.print(Bo   ); printTab();
  Serial.print(latch); printTab();
  Serial.print(B    ); printTab();
  Serial.print('%');
  Serial.println(note);
}

void DamperControl::updateCal(
	    const float trainingGain,
	    const int msPartialOpen,
	    const int msPartialClose,
	    const int msOpen,
	    const int msClose)
{
  // _msLatch is usually small (kind of negligible) and fairly measurable
  // _msBacklash is also fairly small, and somewhat measurable by human experiment
  // may want to assert that these are (mostly) correct for updates from underdetermined
  // system

  // Assert _msBacklash is small, and (nearly) correct.
  float Bo = msOpen-msClose + _msBacklash;  // new estimate of _msBacklashOpen

  // estimate latch time.  Note that this is close to zero.
  // could be negative... check and warn
  float latch = Bo + (msPartialClose - msPartialOpen - _msBacklash);
  if (latch < 10.f)
    {
      Serial.print(F("%Low latch time estimate "));
      Serial.print(latch,1);
      Serial.println(F("ms.  Setting to minimum (10)"));
      latch = 10.f;
    }

  // update _msBacklash estimate.
  float B = Bo + (msClose - msOpen);
  if (B < 50.f)
    {
      Serial.print(F("%Low backlash time estimate "));
      Serial.print(latch);
      Serial.println(F("ms.  Setting to minimum (50)"));
      B = 50.f;
    }

  float So = msOpen - Bo - latch;
  float Sc = msClose - B - latch;
  Serial.print(F("%open  sweep est "));  Serial.println(So,1);
  Serial.print(F("%close sweep est "));  Serial.println(Sc,1);
  // time to close is more reliable than to open.  Open backlash is tricky.
  // weicht toward close
  float S = ( ((long)4) * ((long)Sc)+((long)So) ) / 5;

  Serial.println(F("% DamperControl Calibration Update :"));
  Serial.println(F("%sweep\ttoOpen\tlatch\tbacklash"));
  printParams(_msSweep,_msBacklashOpen,_msLatch,_msBacklash,F("current"));
  printParams((int)S,(int)Bo,(int)latch,(int)B,F("estimated"));
  
  float invGain = 1.0f - trainingGain;

  _msBacklashOpen = (int)( (invGain * ((float)_msBacklashOpen)) + (trainingGain * Bo   ) );
  _msSweep        = (int)( (invGain * ((float)_msSweep       )) + (trainingGain * S    ) );

  // _msBacklash and _msLatch are (somewhat) measurable, but chaotic to estimate.
  // slow their training a bit.
  float g = 0.5 * trainingGain;
  invGain = 1.0f - g;
  _msBacklash = (int)( (invGain * ((float)_msBacklash)) + (g * B    ) );
  _msLatch    = (int)( (invGain * ((float)_msLatch   )) + (g * latch) );
  
  printParams(_msSweep,_msBacklashOpen,_msLatch,_msBacklash,F("updated"));
}

/*
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
  sweepTime = (sweepTime + sweepEst) / 2;   // move halfway to new estimate
  Serial.print(sweepTime); printTab();
  Serial.print(sweepEst);  Serial.print('+');
  Serial.print(backlash);  Serial.print(F("="));
  Serial.println(fullSweep);
  return(sweepTime);
}
*/

void DamperControl::init()  // re-initialize unit, in closed position
{
  Serial.println(F("Make sure closed before starting"));
  runToLatch(false,3);  // close

  // should be closed now.
  _msAccum = _msCmd = 0;
  _opening = false;
}

// run calibration tests, and update parameters
void DamperControl::calibrate(const float trainingGain)
{
  // timing test results
  int msPartialOpen, msPartialClose, msOpen, msClose;

  init();  // make sure closed and reset

  msPartialOpen = _msBacklashOpen + _msSweep/3;
  Serial.print(F("% measure time to open part-way ("));
  Serial.print(msPartialOpen);
  Serial.println(F("ms), then close"));

  // OVERRIDE ENCAPSULATION : use low-level calls to open known ms.
  setOpen(true);
  delay(100);
  move(true);
  delay(msPartialOpen);
  move(false);
  delay(100);
  setOpen(false);

  Serial.println(F("%re-close to latch"));
  delay(100);
  msPartialClose = runToLatch(false,3);

  delay(100);
  msOpen = runToLatch(true,3);

  delay(100);
  msClose = runToLatch(false,3);

  updateCal(trainingGain, msPartialOpen, msPartialClose, msOpen, msClose);
}

byte clamp8(int p)
{
  if (p>255) p=255;
  if (p < 0) p = 0;
  return((byte)p);
}

int cmd2ms(const byte cmdByte, const int msSweep)
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
  
  int ms = (int)(frac * msSweep);
  if (ms < 20)
    ms = 0;
  if (ms > msSweep-30)
    ms = msSweep + 20;

  if (cmdByte != prev)
    {
      Serial.print(ms);printSlash();Serial.println(msSweep);
    }
  prev = cmdByte;
  return(ms);
}
byte ms2cmd(const int ms, const int msSweep)
{
  static int prev = -99;
  
  //long p = (255 * ms) / msSweep;

  // make proportional to surface area, assuming equal angular velocity with time
  float ang = ( 1.571f * ((float)ms) ) / msSweep;

  byte cmdByte = (ang > 1.57f) ? ((byte)255) :
    ((ang < 0.002f) ? ((byte)0) : ((byte)((int)(255.0f * sin(ang)))) );

  if (ms != prev)
    {
      Serial.print(ms);printSlash();Serial.print(msSweep);
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
  return(ms2cmd(_msCmd,_msSweep));
}

// current estimated position.  0==closed.  255==open
const byte DamperControl::getPos()
{
  return(ms2cmd(_msAccum,_msSweep));
}

// command to try to move state to this level
void DamperControl::cmd(const byte pos)
{ 
  _msCmd = cmd2ms(pos,_msSweep);
  update();
}

void DamperControl::update()
{
  Serial.print(F("%Update() cmd="));
  Serial.print(_msCmd);
  Serial.print(F("  accum="));
  Serial.print(_msAccum);
  Serial.print(F("\tsweep="));
  Serial.print(_msSweep);
  Serial.print(F("\thyst="));
  Serial.println(_msHyst);

  if ((_msCmd < _msSweep/50) && (_msAccum > 0))  // nearly closed --> just close it
    {
      Serial.println(F("%request (nearly) closed, latch damper closed."));
      runToLatch(false,1);
      return;
    }
  int nearlyOpen = (int)((((long)19) * ((long)_msSweep)) / 20);
  if ((_msCmd >= nearlyOpen)  && (_msAccum < _msCmd)) // nearly open --> just open it
    {
      Serial.println(F("%request (nearly) open, latch damper open."));
      runToLatch(true,1);
      return;
    }

  int dAccum = _msCmd - _msAccum;
  Serial.print(F("Update position "));  Serial.println(dAccum);
  int a = (dAccum > 0) ? dAccum : -dAccum;
  if (a > _msHyst)
    flush();
}

// reset to given open fraction (of _toToggle)
void DamperControl::setOpenFrac(const int openCount)
{
  if (openCount < 40)
    {
      runToLatch(false);  // close
      return;
    }
  if (openCount > _msSweep-50)
    {
      runToLatch(true); // open
      return;
    }
  Serial.println(F("Restart from closed before setting position"));
  int cmd = openCount;
  int toClose = runToLatch(false); // close

  // this, calibrate(), and runToLatch(true) are the ONLY places we should EVER
  // enable the opening pin!!!  EVER!!!!
  setOpen(true);

  Serial.print(F("opening to "));
  Serial.print(cmd);  printSlash();
  Serial.println(_msSweep);
  cmd += _msBacklashOpen;  // add time to un-latch and start moving

  // I dunno why this is necessary, but it seems to be.
  // the hysterisis I see when doing this seems to be longer than observed in init.
  // perhaps relays changing too quickly?
  //cmd += 1200;
  delay(100);

  unsigned long t0 = millis();
  move(true);

  int t = cmd;
#ifdef VERBOSE_PARTIAL_OPEN
  t = 0;
  while (t < cmd)
    {
      //printTime();
      //Serial.println(t);
      delay(2);
      t = millis() - t0;
    }
#else
  delay(cmd);
#endif
  move(false);

  _msAccum = t - _msBacklashOpen;
  
  Serial.print(F("%open "));
  printTime();
  Serial.print(_msAccum); printSlash();
  Serial.println(openCount);

  _opening = true;
}

void DamperControl::moveTowardClose(const int cmd)
{
  if (cmd < 50)
    {
      runToLatch(false);  // close
      return;
    }
  setOpen(false);  // make sure closing pin is selected
  int dtCmd = _msAccum - cmd;
  int dt = dtCmd;
  Serial.print(F("moving toward closed "));
  Serial.println(dt);
  if (_opening)
    dt += _msBacklash;  // add backlash adjustment to change direction
  _opening = false;

  long t = 0;
  unsigned long t0 = millis();
  move(true);
#ifdef VERBOSE_MOVETOWARDCLOSE
  while (t < dt)
    {
      //printTime(); printTab();
      //Serial.println(t);
      delay(1);
      t = millis() - t0;
    }
#else
  delay(dt);
#endif
  move(false);

#ifdef VERBOSE_MOVETOWARDCLOSE
  _msAccum -= (dtCmd + t-dt);
#else
  _msAccum -= dtCmd;
#endif
}

void DamperControl::flush()  // force position synch
{
  int cmd = _msCmd;
  if (cmd == _msAccum)
    return;  // no action necessary
  if (cmd >= _msSweep-50)
    {
      Serial.println(F("flush()->open"));
      runToLatch(true);  // open
      return;
    }
  if (cmd < 30)
    {
      Serial.println(F("flush()->closed"));
      runToLatch(false);  // open
      return;
    }

  if (cmd < _msAccum)
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

#if 0
bool latched(const int latchCount,
	     const int muEst,
	     const float errEst,
	     const int nReadings)
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
  int d = (muEst > latchCount) ? muEst - latchCount :
    latchCount - muEst;
  if (d > 8)
    return(false);

  return(true);
}
#endif


int DamperControl::runToLatch(const bool open, const byte verbose)
{
  if (open)
    {
      Serial.println(F("Reset damper to closed before attempting to open."));
      runToLatch(false,verbose);
    }

  long t;
  int latchTimeout = 3 * _msBacklashOpen;
  long timeout = latchTimeout + 2 * _msSweep;

  _opening = open;
  setOpen(open);
  if (verbose > 0)
    {
      Serial.print(open?F("%opening"):F("%closing"));
      Serial.println(F(" to latch"));
    }
  int nReadings = 0;
  int muEst = -1;
  //float errEst, errEst1, errEst2;

  //if (verbose > 1)
  //  {
  //    Serial.print(F("%Expected latched current "));
  //    Serial.print(latchCount);
  //    Serial.print(F(" timeout "));
  //    Serial.println(timeout);
  //  }

  unsigned long t0 = millis();
  move(true);
  //int err = 99;
  int nLatch=0;  /* number of samples in a row latched state */

  if (verbose > 1)
    Serial.println(F("%sec nRead sens est"));

  t = 0;
  // ignore latched() until current reading is reliable
  while( (t < 100) || (nLatch<4) ) //!latched(latchCount,muEst,errEst,nReadings)) )
    {
      int val = sens();
      nReadings++;

      if (verbose > 1)
	{
          printTime();
          print4(nReadings); printTab();
          print4(val);//       printSpace();
	}

      if (t < 50)
	muEst = val;  // wait for current sensor to settle
      else
	{
	  // division on 8-bit processor is expensive.
	  // use bit-shift for this instead
	  //// let estimate be weighted (7/8)oldEst + (1/8)newReading
	  // let estimate be weighted (3/4)oldEst + (1/1)newReading
	  int prev = muEst;
	  muEst = (val + (muEst * 3)) >> 2;
	  if (muEst == prev)
	    { // make sure it converges, when est is close.
	      // problem when its a couple of counts off, and won't converge
	      if (val > muEst)
		muEst++;
	      else if (val < muEst)
		muEst--;
	    }

	  // instead of depending on double LPF (muEst after analog LPF on sensor)
	  // Just counts samples in a row below latch current thresh,
	  // AFTER enough time has passed for reliable current reading
	  //    (It can take a few readings for val to have reliable reading)
	  if (val > _latchThresh) nLatch=0;
	  else                    nLatch++;
	}

      if (verbose > 1)
	{
          print4(muEst);
          Serial.println();
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
  _msCmd = _msAccum = (open) ? _msSweep : 0;
 SHUTDOWN:
  move(false);  // also sets direction relay to default : close
  if (verbose > 0)
    {
      printTime();
      Serial.print(t);
      Serial.print(F(" elapsed, latched at "));
      Serial.println(_msCmd);
    }

  //if (t > 1000)
  //  { // update latch threshold?
  //  }

  return(t);
}








#if 0
// older, more complex procedure.
// breadboard circuit had some significant current when latched.
// for some reason, soldered circuit showed near 0 counts when latched,
// as expected.
// hence, this procedure can be much less complex
int DamperControl::runToLatch(const bool open, const byte verbose)
{
  if (open)
    {
      Serial.println(F("Reset damper to closed before attempting to open."));
      runToLatch(false,verbose);
    }

  long t;
  int latchTimeout = 3 * _hystToOpen;
  long timeout = latchTimeout + 2 * _toToggle;

  _opening = open;
  setOpen(open);
  if (verbose > 0)
    {
      Serial.print(open?F("%opening"):F("%closing"));
      Serial.println(F(" to latch"));
    }
  int nReadings = 0;
  int muEst = 999;
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
  int err = 9999;
  int nStep=0;  /* number of samples in a row detecting a step */

  if (verbose > 1)
    Serial.println(F("%sec nRead sens est err"));

  t = 0;
  // ignore latched() until current reading is reliable
  while( (t < 500) || (!latched(latchCount,muEst,errEst,nReadings)) )
    {
      int val = sens();

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
	  int prev = muEst;
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
      int dLatch = muEst - latchCount;
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
#endif
