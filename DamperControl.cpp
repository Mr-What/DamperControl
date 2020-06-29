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
  _latchedOpen = 100;  // sens counts when latched open
  _latchedClosed = 60;  // sens counts when latched closed
  _toggleFuzz = 800;  // extra "fuzz" accum to latch
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

void DamperControl::init()  // re-initialize unit, in closed position
{
  unsigned long t0,tClose0,tOpen1;

  // try to force to close position
  Serial.println(F("Make sure closed before starting"));
  runToLatch(false);  // close

  // should be closed now.
  _accum = 0;
  Serial.println(F("Check time to open"));
  long toOpen = runToLatch(true);  // estimated stall on timeout.
  
  printTime();
  Serial.print(F(" changing opening time from "));
  Serial.print(_toToggle);
  Serial.print(F(" to "));
  //_toToggle = toOpen - _hystToOpen;
  Serial.print(_toToggle);
  Serial.print(F("\t+")); Serial.print(_hystToOpen);
  Serial.print(F("=")); Serial.println(toOpen);

  // measure time to close
  Serial.println(F("Checking time to close"));
  long toClose = runToLatch(false);  // estimated stall time on timeout

  printTime();
  Serial.print(F(" changing closing time from "));
  Serial.print(_toToggle);  Serial.print(F(" to "));
  //_toToggle = 0.5 * (_toToggle + toClose - _backlash);
  Serial.println(_toToggle);

  _cmd = 0;  // current desired state
  _opening = false; // false==closing
}


// current estimated position.  0==closed.  255==open
const byte DamperControl::getPos()
{
  long p = (255 * _accum) / _toToggle;
  if (p>255) p=255;
  if (p < 0) p = 0;
  return((byte)p);
}

// command to try to move state to this level
void DamperControl::cmd(const byte pos)
{  long toLatch = _toToggle + _toggleFuzz;

 
  _cmd = ( ((long)pos) * _toToggle ) / 255 ;
  if (_cmd < 0)
    _cmd = 0;
  update();
}

void DamperControl::update()
{
  if (_cmd < 1)
    {
      runToLatch(false);
      return;
    }
  if (_cmd >= _toToggle)
    {
      runToLatch(true);
      return;
    }
  long dAccum = _cmd - _accum;
  long a = (dAccum > 0) ? dAccum : -dAccum;
  if (a > _hyst)
    flush();
}

// reset to given open fraction (of _toToggle)
void DamperControl::setOpenFrac(long openCount)
{
  if (openCount < 1)
    {
      runToLatch(false);  // close
      return;
    }
  if (openCount >= _toToggle)
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
  unsigned long t0 = millis();
  move(true);
  long t = 0;
  while (t < cmd)
    {
      printTime();
      Serial.println(t);
      t = millis() - t0;
    }
  move(false);
  _accum = t - _hystToOpen;
  printTime();
  Serial.print(_accum);
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
      printTime(); printTab();
      Serial.println(t);
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
  if (cmd >= _toToggle)
    {
      runToLatch(true);  // open
      return;
    }
  if (cmd < 1)
    {
      runToLatch(false);  // open
      return;
    }

  if (cmd < _accum)
    { // will always move toward close after stop.  let it do so
      moveTowardClose(cmd);
      return;
    }
  // no matter what state was, need to close, latch, then move
  // to desired open fraction
  setOpenFrac(cmd);
}

bool latched(const long latchCount,
	     const long muEst,
	     const long errEst,
	     const long nReadings)
{
  if (nReadings < 16)
    return(false);

  // tends to have stable "leakage" current when latched,
  //   but oscillating/aliased drive current when moving
  if (errEst > 2)
    return(false);

  // error w.r.t. expected latched current reading
  long d = (muEst > latchCount) ? muEst - latchCount :
    latchCount - muEst;
  if (d > 20)
    return(false);

  return(true);
}

long DamperControl::runToLatch(const bool open)
{
  long t;
  long latchTimeout = 3 * _hystToOpen;
  long timeout = latchTimeout + 2 * _toToggle;

  _opening = open;
  setOpen(open);
  Serial.println(open?F("opening..."):F("closing..."));
  long nReadings = 0;
  long muEst = 9999;
  long errEst = 0;
  //long muEst0 = muEst;
  //long varEst0 = varEst;

  // expected sensor count when latched
  int latchCount = (open) ? _latchedOpen : _latchedClosed;

  move(true);
  unsigned long t0 = millis();
  long err = 9999;

  Serial.println(F("%sec nRead sens est err"));
  while( !latched(latchCount,muEst,errEst,nReadings) )
    {
      long val = sens();

      printTime();
      print4(nReadings); printTab();
      print4(val);//       printSpace();

      if (nReadings == 0)
	{
	  muEst = val;
	  errEst = 0;
	}
      else
	{
	  // division on 8-bit processor is expensive.
	  // use bit-shift for this instead
	  // let estimate be weighted (7/8)oldEst + (1/8)newReading
	  long prev = muEst;
	  muEst = (val + (muEst * 7)) >> 3;
	  if (muEst == prev)
	    { // make sure it converges, when est is close.
	      // problem when its a couple of counts off, and won't converge
	      if (val > muEst)
		muEst++;
	      else if (val < muEst)
		muEst--;
	    }
	  err = (val > muEst) ? val-muEst : muEst-val;
	  errEst = (err + (errEst * 7)) >> 3;
	  if (errEst < 1)
	    errEst = 1;  // min practical err
	}
      nReadings++;

      print4(muEst); printSpace();
      Serial.println(errEst);

      // detect step in output
      if ( (nReadings > 8) && (err > (errEst<<2)) )
	{  // reset mu and err estimates
	  nReadings = 0;
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
  _cmd = _accum = (open) ? _toToggle + 800 : 0;
 SHUTDOWN:
  move(false);  // also sets direction relay to default : close
  printTime();
  Serial.print(t);
  Serial.print(F(" elapsed, latched at "));
  Serial.println(_cmd);
  if (t > 100)
    { // update latch level estimate
      long dLatch = muEst - latchCount;
      printTime();
      Serial.print(open?F(" open"):F(" closed"));
      Serial.print(F(" latch count update "));
      Serial.print(latchCount);
      dLatch /= 2;  // damp correction a bit
      dLatch += latchCount;
      Serial.print(F(" to "));
      Serial.println(dLatch);
      if (open)
	_latchedOpen = dLatch;
      else
	_latchedClosed = dLatch;
    }
  return(t);
}
