# drumEcho
Record input from sensors and play back on a physical drum.

This uses the hardware from the [drummerduino](https://github.com/WCRSyyc/drummerduino) project, plus a momentary contact push button switch, and a pair of analog pressure sensors. It is used as a robotics demonstrater at WCRS events, to show the public what can be achieved with simple materials.

Although the pressure sensors provide full analog values, the code only uses the input to determine the time the sensor is pressed, not how hard.  Testing shows very little jitter in the raw readings.  To be safer, the code uses a dead band (hysteresis limit) to reduce the posibility that a single tap will trigger multiple input readings.  The concept is similar to debouncing switch inputs, though the implementation is considerably different.

To make this more interesting as a learning project, the sketch is setup to handle simultaneous beats with both sticks.  A hit is not instantaneous.  It requires a stick to be moved down then moved back up again.  At any point during playback, each stick could be waiting, swinging (down), or rebounding (up).  This is implemented with timers and a state machine.  Timers are used because there is no direct information about a stick position.  It has to be inferred from the motion command history.  This is simplified by setting the rebound time a little shorter than the swing time.  The strike on the drum head stops the swing, and the up stroke never gets too far away.

The main sketch is also a state machine.  It can be waiting for the start button, record input press times, or playing back the latest recorded sequence.  Each state machine (over all sketch, and each individual drum stick) is handled by enums.  State changes are mostly handled by timers.  The only exceptions, are that recording is started when the button is pressed, and playback ends when the queued beats run out.

It seems that the motor controller can not get quite as much power from USB, as it does when powered by the external power.  It is going to be 5 volts either way, but with the external power source, the motion is a bit faster / crisper.  The current the motors want must be a bit more than typical USB can supply.

## IDEA
As for the thoughts on expanding the original drummerduino, this could be refactored to move the mechanical control logic into a library (class).  The stick handling state machine could be moved into there.  The code is already broken into functions that handle fairly isolated operations.  It should be easy to turn those into class methods.
