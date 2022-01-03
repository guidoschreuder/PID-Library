#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION 1.2.1-de-arduino-ed

class PID {
  public:
//Constants used in some of the functions below
  typedef enum {
    MANUAL,
    AUTOMATIC,
  } pid_mode_t;
  typedef enum {
    DIRECT,
    REVERSE,
  } pid_direction_t;
  typedef enum {
    on_measurement,
    on_error,
  } pid_proportional_mode_t;

  //commonly used functions **************************************************************************
  PID(double *, double *, double *,       // * constructor.  links the PID to the Input, Output, and
      double, double, double, PID::pid_proportional_mode_t, PID::pid_direction_t);  //   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

  PID(double *, double *, double *,  // * constructor.  links the PID to the Input, Output, and
      double, double, double, PID::pid_direction_t);  //   Setpoint.  Initial tuning parameters are also set here

  void SetMode(PID::pid_mode_t Mode);  // * sets PID to either Manual (0) or Auto (non-0)

  bool Compute();  // * performs the PID calculation.  it should be
                   //   called every time loop() cycles. ON/OFF  can be set using SetMode

  void SetOutputLimits(double, double);  // * clamps the output to a specific range. 0-255 by default, but
                                         //   it's likely the user will want to change this depending on
                                         //   the application

  //available but not commonly used functions ********************************************************
  void SetTunings(double, double,  // * While most users will set the tunings once in the
                  double);         //   constructor, this function gives the user the option
                                   //   of changing tunings during runtime for Adaptive control
  void SetTunings(double, double,  // * overload for specifying proportional mode
                  double, int);

  void SetControllerDirection(PID::pid_direction_t);  // * Sets the Direction, or "Action" of the controller. DIRECT
                                     //   means the output will increase when error is positive. REVERSE
                                     //   means the opposite.  it's very unlikely that this will be needed
                                     //   once it is set in the constructor.

  //Display functions ****************************************************************
  double GetKp();      // These functions query the pid for interal values.
  double GetKi();      //  they were created mainly for the pid front-end,
  double GetKd();      // where it's important to know what is actually
  PID::pid_mode_t GetMode();       //  inside the PID.
  PID::pid_direction_t GetDirection();  //

  private:
  void Initialize();

  double dispKp;  // * we'll hold on to the tuning parameters in user-entered
  double dispKi;  //   format for display purposes
  double dispKd;  //

  double kp;  // * (P)roportional Tuning Parameter
  double ki;  // * (I)ntegral Tuning Parameter
  double kd;  // * (D)erivative Tuning Parameter

  PID::pid_direction_t controllerDirection;
  int pOn;

  double *myInput;     // * Pointers to the Input, Output, and Setpoint variables
  double *myOutput;    //   This creates a hard link between the variables and the
  double *mySetpoint;  //   PID, freeing the user from having to constantly tell us
                       //   what these values are.  with pointers we'll just know.

  double outputSum, lastInput;

  double outMin, outMax;
  bool inAuto, pOnE;
};
#endif
