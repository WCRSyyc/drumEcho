/* Drum Echo

  This sketch implements input record and playback for the WCRS drum machine

Using an Adafruit Motor Shield v2 ---->  http://www.adafruit.com/products/1438
controlling electric car door lock mechanisms as actuators for the drum sticks,
with a momentary push button to start recording, a pair of pressure sensors for
input, and 2 LEDs to show the recording and playback states.

  Modified: 2017/04/06
  By: H. Phil Duby

  sketch "pressure_sensor" used to examine beat profile, to pick thresholds
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Define to turn on debugging with serial print output
//#define SDEBUG 1
// Get compile errors that imply a missing ending brace (multiple functions not
// declared in this scope) when using #ifdef #endif and the macro is NOT defined
// Exactly the same code, with the #ifdef and #endif lines commented out works.

enum DrumState {
  WAITING_FOR_START,
  RECORDING_BEATS,
  PLAYING_BEATS
};
enum StickState {
  SWING,
  REBOUND,
  COAST
};

const unsigned int MAX_BEAT_COUNT = 50;
struct StickData {
  unsigned long track [ MAX_BEAT_COUNT ];
  Adafruit_DCMotor * motor;
  unsigned int trackLength;
  unsigned int nextBeat;
  boolean isInBeat;
  StickState state;
  unsigned long expireTime;
};

//#ifdef SDEBUG
unsigned const int SERIAL_SPEED = 57600;
//300, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
//#endif

// Setup the 'profiles' for the way the drum sticks move
const unsigned int DOWN_SPEED = 255;// full speed
const unsigned int DOWN_TIME = 50;// milliseconds
const unsigned int UP_SPEED = 255;// slower up?
const unsigned int UP_TIME = 30;// for less time: keep sticks close to drum head
// Setup for simplified beat detection based on threshold and dead band
// Setup hysteresis band to limit posibility of double detection of a single stroke
const unsigned int BEAT_MIN = 25; // Minimum reading to see beat
const unsigned int END_MAX = 10; // Maximum value to mark the end of a stroke

// The time to listen to inputs before playing them back
const unsigned int RECORD_TIME = 10000; // 10 seconds

// I2C address for the motor shield
// https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/faq
uint8_t const I2C_ADDR = 0x60; // default motor shield address
uint16_t const MAX_PWM_FREQ = 1600; // default (Hz) (max 4096)

// The motor shield 'ports' that the stick controlling motors are attached to
uint8_t const LEFT_PORT = 1;
uint8_t const RIGHT_PORT = 2;

// Arduino pin numbers used for sensors and status display
const int playStatePin = 12;
const int recordStatePin = 13;
const int startPin = 2;
const int leftSensorPin = A2;
const int rightSensorPin = A1;

// Create motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield ( I2C_ADDR );

// Setup global variables used by the sketch
DrumState echoState = WAITING_FOR_START;
unsigned long newStateTime; // Time reference point when cycling between states

// Storage for left and right beat tracks
StickData leftStick;
StickData rightStick;

bool showReady; // DEBUG
bool doingRoll;
unsigned int rollCount;
const unsigned ROLL_COUNT = 10;

void setup()
{
//#ifdef SDEBUG
  Serial.begin ( SERIAL_SPEED );
  Serial.println(F("Drum machine echo starting"));
//#endif
  // Specify the motor shield 'ports' each of the stick motors is attached to
  leftStick.motor = AFMS.getMotor ( LEFT_PORT );
  rightStick.motor = AFMS.getMotor ( RIGHT_PORT );

  AFMS.begin( MAX_PWM_FREQ );
  prepSticks ();
  leftStick.state = COAST;
  rightStick.state = COAST;

  // Initialize digital pins that need it
  pinMode( playStatePin, OUTPUT );
  digitalWrite( playStatePin, LOW );
  pinMode( recordStatePin, OUTPUT );
  digitalWrite( recordStatePin, LOW );
  pinMode( startPin, INPUT ); // could use INPUT_PULLUP, but circuit includes that

  delay ( 1000 );
  doingRoll = true;
  rollCount = 0;
  startSwing ( &leftStick );
  delay ( 40 );
}// ./void setup()


void loop()
{
  if ( doingRoll ) {
    drumRoll ();
    return;
  }

  if ( echoState == WAITING_FOR_START ) {
    handleStart();
  } else if ( echoState == RECORDING_BEATS ) {
    recordBeats();
  } else {// NOT ( WAITING_FOR_START || RECORDING_BEATS )
//    showBeats(); // DEBUG
    playBeats();
  }// ./else NOT ( WAITING_FOR_START || RECORDING_BEATS )
}// ./void loop()


void drumRoll ()
{
  checkSwingStates ();
  if ( leftStick.state == COAST && rollCount < ROLL_COUNT ) {
    startSwing ( &leftStick );
  }
  if ( rightStick.state == COAST && rollCount < ROLL_COUNT ) {
    startSwing ( &rightStick );
    rollCount++;
  }
  if( leftStick.state == COAST && rightStick.state == COAST &&
      rollCount >= ROLL_COUNT ) {
    doingRoll = false;
  }
}// ./void drumRoll ()


// Waiting for the start button to be pressed
void handleStart()
{
  if ( digitalRead ( startPin ) == LOW ) {
    // The start button was pushed: Setup to listen to the input sensors
    changeStateToRecording ();
  }// ./if ( digitalRead ( startPin ) == HIGH )
}// ./void handleStart()


void recordBeats()
{
  // not safe to use millis() - RECORD_TIME: for the initial RECORD_TIME, that
  // would be a negative value, but since using unsigned, it wraps to postive
  // maximum value, and the test passes immediately.  Oops.
  if ( millis() > newStateTime + RECORD_TIME ) {
    // Have recorded long enough: setup to play it back
    changeStateToPlaying ();
    return; // done here
  }// ./if ( millis() - RECORD_TIME > newStateTime )

  getStickSwingTime ( &leftStick, analogRead ( leftSensorPin ));
  getStickSwingTime ( &rightStick, analogRead ( rightSensorPin ));
}// ./void recordBeats()


void playBeats()
{
  if ( leftStick.nextBeat >= leftStick.trackLength &&
      rightStick.nextBeat >= rightStick.trackLength &&
      leftStick.state == COAST && rightStick.state == COAST ) {
    // Refuse to change to waiting state unless all of the sticks are in COAST state
    changeStateToWaiting ();
    return;
  }
  // handle stick state follow through
  checkSwingStates ();

  // This is the code to play the recorded beats
  checkBeatStart ( &leftStick, newStateTime );
  checkBeatStart ( &rightStick, newStateTime );
}// ./void playBeats()


void changeStateToRecording ()
{
  echoState = RECORDING_BEATS;// Move to the next state (recording)
  newStateTime = millis();// Remember the time started listening
  leftStick.trackLength = 0;
  leftStick.isInBeat = false;
  rightStick.trackLength = 0;
  rightStick.isInBeat = false;
  digitalWrite( recordStatePin, HIGH );// Set LED to show recording
}// ./void changeStateToRecording ()


void changeStateToPlaying ()
{
  echoState = PLAYING_BEATS;
  newStateTime = millis();// Remember the time started playing
  digitalWrite( recordStatePin, LOW );
  digitalWrite( playStatePin, HIGH );
  leftStick.nextBeat = 0;
  rightStick.nextBeat = 0;
  leftStick.track [ leftStick.trackLength ] = RECORD_TIME + 100; // Dummy time after end of all
  rightStick.track [ rightStick.trackLength ] = RECORD_TIME + 100;
  showReady = true;// DEBUG
}// ./void changeStateToPlaying ()


void changeStateToWaiting ()
{
  echoState = WAITING_FOR_START;
  digitalWrite( playStatePin, LOW );
}// ./void changeStateToWaiting ()


void getStickSwingTime ( struct StickData * stick, unsigned int sensorValue )
{
  if ( !stick -> isInBeat && sensorValue > BEAT_MIN ) {
    // Was not in a beat, but now reached the on detection threshold
    stick -> isInBeat = true;
    stick -> track [ stick -> trackLength ] = millis() - newStateTime; // time since start of track
    stick -> trackLength++;
    if ( stick -> trackLength >= MAX_BEAT_COUNT ) {
      // Make sure does not go past the MAX_BEAT_COUNT array size
      stick -> trackLength = MAX_BEAT_COUNT - 1;
    }
  }// ./if ( !stick -> isInBeat && sensorValue > BEAT_MIN )

  if ( stick -> isInBeat && sensorValue < END_MAX ) {
    // was in a beat, but now dropped back below the off threshold
    stick -> isInBeat = false;
  }
}// ./void getStickSwingTime()


void checkBeatStart ( struct StickData * stick, unsigned long baseTime )
{
  if( stick -> nextBeat < stick -> trackLength &&
      millis() > baseTime + stick -> track [ stick -> nextBeat ] ) {
    // Time to start a new beat for the stick
    startSwing ( stick );
    stick -> nextBeat++;
  }
}


void checkSwingStates ()
{
  checkStickState ( &leftStick );
  checkStickState ( &rightStick );
}// ./void checkSwingStates ()


/**
 * Start a drum stick forward/down swing
 *
 * @param stick control data structure for a drum stick
 */
void startSwing ( struct StickData * stick )
{
  stick -> motor -> run ( FORWARD );
  stick -> motor -> setSpeed( DOWN_SPEED );
  stick -> state = SWING;
  stick -> expireTime = millis () + DOWN_TIME;
}
void reboundSwing ( struct StickData * stick )
{
  stick -> motor -> run ( BACKWARD );
  stick -> state = REBOUND;
  stick -> expireTime = millis () + UP_TIME;
}
void endSwing ( struct StickData * stick )
{
  stick -> motor -> setSpeed( 0 );
  stick -> motor -> run ( RELEASE );
  stick -> state = COAST;
}
// May be able to use an extra state, to set the time point that another swing
// can start, even though the rebound/coast is not actually finished yet.  For
// now, that will be the start of COAST


/**
 * Handle follow through to complete a single drum beat after it has been started
 *
 * @param stick control data structure for a drum stick
 */
void checkStickState ( struct StickData * stick )
{
  if ( stick -> state == COAST ) { return; }

  if ( millis () >= stick -> expireTime ) {
    if ( stick -> state == SWING ) {
      reboundSwing ( stick );
    } else {
      endSwing ( stick );
    }
  }
}


/**
 * Hand tuned commands to get the drum sticks ready for real input.
 */
void prepSticks()
{
  commandSticks ( 0, 0, RELEASE );
  delay ( 100 );

  // move both of the sticks to the end (top) of the movement range, slowly
  commandSticks ( 180, 180, BACKWARD );// Jolt to get moving
  delay ( 50 );
  commandSticks ( 120, 90, BACKWARD );// keep moving
  delay ( 900 );
  commandSticks ( 255, 255, BACKWARD );// push to the end
  delay ( 50 );
  commandSticks ( 0, 0, RELEASE );
  delay ( 1000 );// pause at the top

  // move both of the sticks to the other end of the movement range, slowly
  commandSticks ( 180, 180, FORWARD );// Jolt to get moving
  delay ( 50 );
  commandSticks ( 110, 110, FORWARD );// keep moving
  delay ( 900 );
  commandSticks ( 200, 200, FORWARD );// push down
  delay ( 50 );
  commandSticks ( 0, 0, RELEASE );
  delay ( 1000 );// pause at the bottom

  // bring back up to the 'ready' position
  commandSticks ( DOWN_SPEED, DOWN_SPEED, BACKWARD );
  delay ( UP_TIME );
  commandSticks ( 0, 0, RELEASE );

  delay ( 200 );
  swingStick ( leftStick.motor );
  swingStick ( rightStick.motor );
}// ./void prepSticks()


/**
 * Send speed and direction commands to both left and right motors
 *
 * @param leftSpeed pwm speed setting to use for the left stick
 * @param rightSpeed pwm speed setting to use for the right stick
 * @param stickCommand Adafruit_DCMotor.run argument
 */
void commandSticks( unsigned int leftSpeed, unsigned int rightSpeed, unsigned int stickCommand )
{
  leftStick.motor -> setSpeed ( leftSpeed );
  rightStick.motor -> setSpeed ( rightSpeed );
  leftStick.motor -> run ( stickCommand );
  rightStick.motor -> run ( stickCommand );
}// ./void commandSticks()


// This has also been broken into multiple (sub) states, so that left and right
// strokes can be in progress at the same time, at different stages, but the
// prep function still uses the old code
void swingStick( Adafruit_DCMotor* stick )
{
  stick->run( FORWARD );
  stick->setSpeed( DOWN_SPEED );
  delay ( DOWN_TIME );
  stick->run( BACKWARD );
//  stick->setSpeed( UP_SPEED );
  delay ( UP_TIME );
  stick->setSpeed( 0 );
  stick->run( RELEASE );
}// ./ void swingStick()


/**
 * Report what has been recorded
 */
void showBeats()
{
  if ( !showReady ) { return; }
  Serial.print(F("Tracks: Left count is "));
  Serial.print( leftStick.trackLength );
  Serial.print(F(", Right count is " ));
  Serial.print( rightStick.trackLength );
  Serial.println(F(" beats" ));
  Serial.print(F("Left: " ));
  for ( unsigned int i = 0; i < leftStick.trackLength; i++ ) {
    if ( i > 0 ) { Serial.print(F(", ")); }
    Serial.print( leftStick.track [ i ]);
  }

  Serial.print(F("\nRight: " ));
  for ( unsigned int i = 0; i < rightStick.trackLength; i++ ) {
    if ( i > 0 ) { Serial.print(F(", ")); }
    Serial.print( rightStick.track [ i ]);
  }
  Serial.println();
  showReady = false;
}// ./showBeats()
