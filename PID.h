#ifndef v1_h
#define v1_h
#define LIBRARY_VERSION 1.2.1-de-arduino-ed

class PID {
  public:
//Constants used in some of the functions below
  typedef enum {
    MANUAL,
    AUTOMATIC,
  } mode_t;
  typedef enum {
    DIRECT,
    REVERSE,
  } direction_t;
  typedef enum {
    on_measurement,
    on_error,
  } proportional_mode_t;
  typedef struct {
    double Kp,
        Ki,
        Kd;
    proportional_mode_t prop_on;
  } tuning_t;

  //commonly used functions **************************************************************************
  PID(double *,
      double *,
      double *,  // * constructor.  links the PID to the Input, Output, and
      PID::tuning_t,
      PID::direction_t);  //   Setpoint.  Initial tuning parameters are also set here

  void SetMode(PID::mode_t Mode);  // * sets PID to either Manual (0) or Auto (non-0)

  bool Compute();  // * performs the PID calculation.  it should be
                   //   called every time loop() cycles. ON/OFF  can be set using SetMode

  void SetOutputLimits(double, double);  // * clamps the output to a specific range. 0-255 by default, but
                                         //   it's likely the user will want to change this depending on
                                         //   the application

  //available but not commonly used functions ********************************************************
  void SetTunings(PID::tuning_t);  // * While most users will set the tunings once in the
                                   //   constructor, this function gives the user the option
                                   //   of changing tunings during runtime for Adaptive control

  void SetControllerDirection(PID::direction_t);  // * Sets the Direction, or "Action" of the controller. DIRECT
                                     //   means the output will increase when error is positive. REVERSE
                                     //   means the opposite.  it's very unlikely that this will be needed
                                     //   once it is set in the constructor.

  //Display functions ****************************************************************
  PID::tuning_t GetTunings();  // These functions query the pid for interal values.
                                   // they were created mainly for the pid front-end,
                                   // where it's important to know what is actually
  PID::mode_t GetMode();       // inside the PID.
  PID::direction_t GetDirection();  //

  private:
  void Initialize();

  PID::tuning_t tuning;
  PID::direction_t controllerDirection;
  PID::mode_t mode;

  double *myInput;     // * Pointers to the Input, Output, and Setpoint variables
  double *myOutput;    //   This creates a hard link between the variables and the
  double *mySetpoint;  //   PID, freeing the user from having to constantly tell us
                       //   what these values are.  with pointers we'll just know.

  double outputSum, lastInput;

  double outMin, outMax;
};
#endif
