/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1-de-arduino-ed
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include "PID.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input,
         double* Output,
         double* Setpoint,
         PID::pid_tuning_t tuning,
         PID::pid_proportional_mode_t POn,
         PID::pid_direction_t ControllerDirection) {
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;

  PID::SetOutputLimits(0, 255);  //default output limit corresponds to
                                 //the arduino pwm limits

  PID::SetControllerDirection(ControllerDirection);
  PID::SetTunings(tuning, POn);
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(double* Input,
         double* Output,
         double* Setpoint,
         PID::pid_tuning_t tuning,
         PID::pid_direction_t ControllerDirection)
    : PID::PID(Input, Output, Setpoint, tuning, PID::pid_proportional_mode_t::on_error, ControllerDirection) {
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute() {
  if (!inAuto)
    return false;

  /*Compute all the working error variables*/
  double input = *myInput;
  double error = *mySetpoint - input;
  double dInput = (input - lastInput);
  outputSum += (tuning.Ki * error);

  /*Add Proportional on Measurement, if P_ON_M is specified*/
  if (!pOnE)
    outputSum -= tuning.Kp * dInput;

  if (outputSum > outMax)
    outputSum = outMax;
  else if (outputSum < outMin)
    outputSum = outMin;

  /*Add Proportional on Error, if P_ON_E is specified*/
  double output;
  if (pOnE)
    output = tuning.Kp * error;
  else
    output = 0;

  /*Compute Rest of PID Output*/
  output += outputSum - tuning.Kd * dInput;

  if (output > outMax)
    output = outMax;
  else if (output < outMin)
    output = outMin;
  *myOutput = output;

  /*Remember some variables for next time*/
  lastInput = input;
  return true;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(PID::pid_tuning_t tuning, PID::pid_proportional_mode_t POn) {
  if (tuning.Kp < 0 || tuning.Ki < 0 || tuning.Kd < 0)
    return;

  pOn = POn;
  pOnE = POn == PID::pid_proportional_mode_t::on_error;

  tuning = dispTuning = tuning;

  if (controllerDirection == REVERSE) {
    tuning.Kp = (0 - tuning.Kp);
    tuning.Ki = (0 - tuning.Ki);
    tuning.Kd = (0 - tuning.Kd);
  }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(PID::pid_tuning_t tuning) {
  SetTunings(tuning, pOn);
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max) {
  if (Min >= Max)
    return;
  outMin = Min;
  outMax = Max;

  if (inAuto) {
    if (*myOutput > outMax)
      *myOutput = outMax;
    else if (*myOutput < outMin)
      *myOutput = outMin;

    if (outputSum > outMax)
      outputSum = outMax;
    else if (outputSum < outMin)
      outputSum = outMin;
  }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(PID::pid_mode_t Mode) {
  bool newAuto = (Mode == PID::pid_mode_t::AUTOMATIC);
  if (newAuto && !inAuto) { /*we just went from manual to auto*/
    PID::Initialize();
  }
  inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize() {
  outputSum = *myOutput;
  lastInput = *myInput;
  if (outputSum > outMax)
    outputSum = outMax;
  else if (outputSum < outMin)
    outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(PID::pid_direction_t Direction) {
  if (inAuto && Direction != controllerDirection) {
    tuning.Kp = (0 - tuning.Kp);
    tuning.Ki = (0 - tuning.Ki);
    tuning.Kd = (0 - tuning.Kd);
  }
  controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
PID::pid_tuning_t PID::GetTunings() { return dispTuning; }
PID::pid_mode_t PID::GetMode() { return inAuto ? PID::pid_mode_t::AUTOMATIC : PID::pid_mode_t::MANUAL; }
PID::pid_direction_t PID::GetDirection() { return controllerDirection; }
