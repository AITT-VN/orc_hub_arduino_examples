#ifndef OHSTEM_ROBOTICS_CONSTANTS_H
#define OHSTEM_ROBOTICS_CONSTANTS_H

#include <Arduino.h>

namespace ohstem {
namespace robotics {

constexpr uint8_t ALL = 63;
constexpr uint8_t M1 = 1;
constexpr uint8_t M2 = 2;
constexpr uint8_t M3 = 4;
constexpr uint8_t M4 = 8;
constexpr uint8_t E1 = 16;
constexpr uint8_t E2 = 32;

constexpr uint8_t STEPPER1 = 0;
constexpr uint8_t STEPPER2 = 1;

constexpr int8_t DIR_CW = 1;
constexpr int8_t DIR_CCW = -1;

constexpr uint8_t S1 = 0;
constexpr uint8_t S2 = 1;
constexpr uint8_t S3 = 2;
constexpr uint8_t S4 = 3;

constexpr char BTN_UP[] = "U";
constexpr char BTN_DOWN[] = "D";
constexpr char BTN_LEFT[] = "L";
constexpr char BTN_RIGHT[] = "R";

constexpr char BTN_SQUARE[] = "SQ";
constexpr char BTN_TRIANGLE[] = "TR";
constexpr char BTN_CROSS[] = "CR";
constexpr char BTN_CIRCLE[] = "CI";

constexpr char BTN_L1[] = "L1";
constexpr char BTN_R1[] = "R1";
constexpr char BTN_L2[] = "L2";
constexpr char BTN_R2[] = "R2";

constexpr char BTN_M1[] = "M1";
constexpr char BTN_M2[] = "M2";
constexpr char BTN_THUMBL[] = "THUMBL";
constexpr char BTN_THUMBR[] = "THUMBR";

constexpr char AL[] = "AL";
constexpr char ALX[] = "ALX";
constexpr char ALY[] = "ALY";
constexpr char AL_DIR[] = "AL_DIR";
constexpr char AL_DISTANCE[] = "AL_DISTANCE";
constexpr char AR[] = "AR";
constexpr char ARX[] = "ARX";
constexpr char ARY[] = "ARY";
constexpr char AR_DIR[] = "AR_DIR";
constexpr char AR_DISTANCE[] = "AR_DISTANCE";

enum DriveMode : uint8_t {
  MODE_2WD = 0,
  MODE_4WD = 1,
  MODE_MECANUM = 2
};

enum StopAction : uint8_t {
  STOP = 0,
  BRAKE = 1,
  NONE_ACTION = 2
};

enum MoveUnit : uint8_t {
  SECOND = 0,
  DEGREE = 1,
  CM = 2,
  INCH = 3
};

enum Direction : int8_t {
  DIR_FW = 0,
  DIR_RF = 1,
  DIR_R = 2,
  DIR_RB = 3,
  DIR_BW = 4,
  DIR_LB = 5,
  DIR_L = 6,
  DIR_LF = 7,
  DIR_SL = 8,
  DIR_SR = 9
};

constexpr uint8_t DPAD = 1;
constexpr uint8_t JOYSTICK = 2;

enum class GamepadButton : uint8_t {
  Up = 0,
  Down,
  Left,
  Right,
  Square,
  Triangle,
  Cross,
  Circle,
  L1,
  R1,
  L2,
  R2,
  ThumbL,
  ThumbR,
  M1,
  M2,
  Count
};

enum LineState : int8_t {
  LINE_LEFT3 = -3,
  LINE_LEFT2 = -2,
  LINE_LEFT = -1,
  LINE_CENTER = 0,
  LINE_RIGHT = 1,
  LINE_RIGHT2 = 2,
  LINE_RIGHT3 = 3,
  LINE_CROSS = 4,
  LINE_END = 5
};

}  // namespace robotics
}  // namespace ohstem

#endif
