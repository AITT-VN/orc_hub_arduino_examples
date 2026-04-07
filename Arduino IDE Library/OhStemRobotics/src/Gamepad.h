#ifndef OHSTEM_GAMEPAD_H
#define OHSTEM_GAMEPAD_H

#include <Arduino.h>
#include <Wire.h>

#include "RoboticsConstants.h"

namespace ohstem {
namespace robotics {

struct GamepadState {
  bool up = false;
  bool down = false;
  bool left = false;
  bool right = false;
  bool square = false;
  bool triangle = false;
  bool cross = false;
  bool circle = false;
  bool l1 = false;
  bool r1 = false;
  bool l2 = false;
  bool r2 = false;
  bool thumbL = false;
  bool thumbR = false;
  bool m1 = false;
  bool m2 = false;

  int leftX = 0;
  int leftY = 0;
  int rightX = 0;
  int rightY = 0;
  int leftTriggerAnalog = 0;
  int rightTriggerAnalog = 0;
  int leftDirection = -1;
  int leftDistance = 0;
  int rightDirection = -1;
  int rightDistance = 0;

  void clear();
  bool pressed(GamepadButton button) const;
};

class PS4GamepadReceiver {
 public:
  explicit PS4GamepadReceiver(TwoWire& wire = Wire, uint8_t address = 0x55);

  bool begin();
  bool update();
  bool isConnected() const { return connected_; }
  const GamepadState& state() const { return state_; }

  void setLedColor(uint8_t red, uint8_t green, uint8_t blue);
  void setPlayerLed(uint8_t ledMask);
  void setRumble(uint8_t force, uint8_t duration);

 private:
  void convertData(const uint8_t* raw);

  TwoWire* wire_;
  uint8_t address_;
  bool connected_ = false;
  GamepadState state_;
};

class Gamepad {
 public:
  explicit Gamepad(PS4GamepadReceiver* receiver = nullptr);

  bool begin();
  void attachReceiver(PS4GamepadReceiver* receiver);
  bool update();

  bool applyNameValue(const String& name, int value);
  void setButton(GamepadButton button, bool pressed);
  void setLeftStick(int x, int y);
  void setRightStick(int x, int y);

  static void calculateJoystick(int x, int y, int& dir, int& distance);
  static int decodeSignedByte(int value);

  const GamepadState& state() const { return state_; }
  GamepadState& stateMutable() { return state_; }

 private:
  PS4GamepadReceiver* receiver_ = nullptr;
  GamepadState state_;
};

}  // namespace robotics
}  // namespace ohstem

#endif
