#include "Gamepad.h"

#include <math.h>

#include "RoboticsUtils.h"

namespace ohstem {
namespace robotics {

namespace {

constexpr uint8_t kRegLedColor = 0x01;
constexpr uint8_t kRegLedPlayer = 0x02;
constexpr uint8_t kRegRumble = 0x03;

constexpr uint8_t kDpadUp = 0;
constexpr uint8_t kDpadDown = 1;
constexpr uint8_t kDpadRight = 2;
constexpr uint8_t kDpadLeft = 3;

constexpr uint8_t kButtonA = 0;
constexpr uint8_t kButtonB = 1;
constexpr uint8_t kButtonX = 2;
constexpr uint8_t kButtonY = 3;
constexpr uint8_t kButtonShoulderL = 4;
constexpr uint8_t kButtonShoulderR = 5;
constexpr uint8_t kButtonTriggerL = 6;
constexpr uint8_t kButtonTriggerR = 7;
constexpr uint8_t kButtonThumbL = 8;
constexpr uint8_t kButtonThumbR = 9;

constexpr uint8_t kMiscButtonSystem = 0;
constexpr uint8_t kMiscButtonM1 = 1;
constexpr uint8_t kMiscButtonM2 = 2;

}  // namespace

void GamepadState::clear() {
  *this = GamepadState();
}

bool GamepadState::pressed(GamepadButton button) const {
  switch (button) {
    case GamepadButton::Up:
      return up;
    case GamepadButton::Down:
      return down;
    case GamepadButton::Left:
      return left;
    case GamepadButton::Right:
      return right;
    case GamepadButton::Square:
      return square;
    case GamepadButton::Triangle:
      return triangle;
    case GamepadButton::Cross:
      return cross;
    case GamepadButton::Circle:
      return circle;
    case GamepadButton::L1:
      return l1;
    case GamepadButton::R1:
      return r1;
    case GamepadButton::L2:
      return l2;
    case GamepadButton::R2:
      return r2;
    case GamepadButton::ThumbL:
      return thumbL;
    case GamepadButton::ThumbR:
      return thumbR;
    case GamepadButton::M1:
      return m1;
    case GamepadButton::M2:
      return m2;
    case GamepadButton::Count:
      return false;
  }
  return false;
}

PS4GamepadReceiver::PS4GamepadReceiver(TwoWire& wire, uint8_t address)
    : wire_(&wire), address_(address) {
  configureDefaultWirePins(wire);
}

bool PS4GamepadReceiver::begin() {
  configureDefaultWirePins(*wire_);
  wire_->begin();
  wire_->beginTransmission(address_);
  connected_ = wire_->endTransmission() == 0;
  return connected_;
}

bool PS4GamepadReceiver::update() {
  uint8_t raw[30] = {0};
  // The receiver always publishes a fixed-size packet.
  const size_t count = wire_->requestFrom(static_cast<int>(address_),
                                          static_cast<int>(sizeof(raw)));
  if (count != sizeof(raw)) {
    while (wire_->available()) {
      wire_->read();
    }
    connected_ = false;
    state_.clear();
    return false;
  }

  for (size_t i = 0; i < sizeof(raw); ++i) {
    raw[i] = wire_->read();
  }

  connected_ = raw[0] == 1;
  if (!connected_) {
    state_.clear();
    return false;
  }

  convertData(raw);
  return true;
}

void PS4GamepadReceiver::convertData(const uint8_t* raw) {
  // Unpack digital buttons and analog axes from the receiver packet.
  const uint8_t dpad = raw[1];
  const int32_t aLX = readInt32BE(raw + 2);
  const int32_t aLY = readInt32BE(raw + 6);
  const int32_t aRX = readInt32BE(raw + 10);
  const int32_t aRY = readInt32BE(raw + 14);
  const int32_t aL2 = readInt32BE(raw + 18);
  const int32_t aR2 = readInt32BE(raw + 22);
  const uint16_t buttons = static_cast<uint16_t>(readInt16BE(raw + 26));
  const uint16_t miscButtons = static_cast<uint16_t>(readInt16BE(raw + 28));

  state_.up = ((dpad >> kDpadUp) & 0x01) != 0;
  state_.down = ((dpad >> kDpadDown) & 0x01) != 0;
  state_.right = ((dpad >> kDpadRight) & 0x01) != 0;
  state_.left = ((dpad >> kDpadLeft) & 0x01) != 0;

  state_.cross = ((buttons >> kButtonA) & 0x01) != 0;
  state_.circle = ((buttons >> kButtonB) & 0x01) != 0;
  state_.square = ((buttons >> kButtonX) & 0x01) != 0;
  state_.triangle = ((buttons >> kButtonY) & 0x01) != 0;
  state_.l1 = ((buttons >> kButtonShoulderL) & 0x01) != 0;
  state_.r1 = ((buttons >> kButtonShoulderR) & 0x01) != 0;
  state_.l2 = ((buttons >> kButtonTriggerL) & 0x01) != 0;
  state_.r2 = ((buttons >> kButtonTriggerR) & 0x01) != 0;
  state_.thumbL = ((buttons >> kButtonThumbL) & 0x01) != 0;
  state_.thumbR = ((buttons >> kButtonThumbR) & 0x01) != 0;
  state_.m1 = ((miscButtons >> kMiscButtonM1) & 0x01) != 0;
  state_.m2 = ((miscButtons >> kMiscButtonM2) & 0x01) != 0;

  state_.leftTriggerAnalog = aL2;
  state_.rightTriggerAnalog = aR2;

  state_.leftX = static_cast<int>(translateValue(aLX, -508, 512, -100, 100));
  state_.leftY = static_cast<int>(translateValue(aLY, 512, -508, -100, 100));
  state_.rightX = static_cast<int>(translateValue(aRX, -508, 512, -100, 100));
  state_.rightY = static_cast<int>(translateValue(aRY, 512, -508, -100, 100));

  Gamepad::calculateJoystick(state_.leftX, state_.leftY, state_.leftDirection,
                             state_.leftDistance);
  Gamepad::calculateJoystick(state_.rightX, state_.rightY, state_.rightDirection,
                             state_.rightDistance);
}

void PS4GamepadReceiver::setLedColor(uint8_t red, uint8_t green, uint8_t blue) {
  wire_->beginTransmission(address_);
  wire_->write(kRegLedColor);
  wire_->write(red);
  wire_->write(green);
  wire_->write(blue);
  wire_->endTransmission();
}

void PS4GamepadReceiver::setPlayerLed(uint8_t ledMask) {
  wire_->beginTransmission(address_);
  wire_->write(kRegLedPlayer);
  wire_->write(ledMask);
  wire_->endTransmission();
}

void PS4GamepadReceiver::setRumble(uint8_t force, uint8_t duration) {
  wire_->beginTransmission(address_);
  wire_->write(kRegRumble);
  wire_->write(force);
  wire_->write(duration);
  wire_->endTransmission();
}

Gamepad::Gamepad(PS4GamepadReceiver* receiver) : receiver_(receiver) {}

bool Gamepad::begin() {
  if (receiver_ != nullptr) {
    return receiver_->begin();
  }
  return true;
}

void Gamepad::attachReceiver(PS4GamepadReceiver* receiver) { receiver_ = receiver; }

bool Gamepad::update() {
  if (receiver_ != nullptr && receiver_->update() && receiver_->isConnected()) {
    // Mirror the receiver state into the generic Gamepad wrapper.
    state_ = receiver_->state();
    return true;
  }
  return false;
}

int Gamepad::decodeSignedByte(int value) {
  int decoded = value & 0xFF;
  if (decoded > 127) {
    decoded -= 256;
  }
  return decoded;
}

void Gamepad::calculateJoystick(int x, int y, int& dir, int& distance) {
  dir = -1;
  distance = static_cast<int>(sqrtf(static_cast<float>(x * x + y * y)));
  if (distance < 15) {
    // Ignore small stick noise near the center.
    distance = 0;
    return;
  }
  if (distance > 100) {
    distance = 100;
  }

  int angle = static_cast<int>(atan2f(static_cast<float>(y), static_cast<float>(x)) *
                               180.0f / PI);
  if (angle < 0) {
    angle += 360;
  }

  // Convert a continuous joystick angle into one of the library's 8 directions.
  if ((angle >= 0 && angle < 10) || angle >= 350) {
    dir = DIR_R;
  } else if (angle >= 15 && angle < 75) {
    dir = DIR_RF;
  } else if (angle >= 80 && angle < 110) {
    dir = DIR_FW;
  } else if (angle >= 115 && angle < 165) {
    dir = DIR_LF;
  } else if (angle >= 170 && angle < 190) {
    dir = DIR_L;
  } else if (angle >= 195 && angle < 255) {
    dir = DIR_LB;
  } else if (angle >= 260 && angle < 280) {
    dir = DIR_BW;
  } else if (angle >= 285 && angle < 345) {
    dir = DIR_RB;
  }
}

bool Gamepad::applyNameValue(const String& name, int value) {
  // This path is useful when gamepad events arrive as key/value pairs from an app or BLE bridge.
  if (name == BTN_UP) {
    state_.up = value != 0;
  } else if (name == BTN_DOWN) {
    state_.down = value != 0;
  } else if (name == BTN_LEFT) {
    state_.left = value != 0;
  } else if (name == BTN_RIGHT) {
    state_.right = value != 0;
  } else if (name == BTN_SQUARE) {
    state_.square = value != 0;
  } else if (name == BTN_TRIANGLE) {
    state_.triangle = value != 0;
  } else if (name == BTN_CROSS) {
    state_.cross = value != 0;
  } else if (name == BTN_CIRCLE) {
    state_.circle = value != 0;
  } else if (name == BTN_L1) {
    state_.l1 = value != 0;
  } else if (name == BTN_R1) {
    state_.r1 = value != 0;
  } else if (name == BTN_L2) {
    state_.l2 = value != 0;
  } else if (name == BTN_R2) {
    state_.r2 = value != 0;
  } else if (name == BTN_THUMBL) {
    state_.thumbL = value != 0;
  } else if (name == BTN_THUMBR) {
    state_.thumbR = value != 0;
  } else if (name == BTN_M1) {
    state_.m1 = value != 0;
  } else if (name == BTN_M2) {
    state_.m2 = value != 0;
  } else if (name == AL) {
    const int x = (value >> 8) & 0xFF;
    const int y = decodeSignedByte(value);
    setLeftStick(x, y);
  } else if (name == AR) {
    const int x = (value >> 8) & 0xFF;
    const int y = decodeSignedByte(value);
    setRightStick(x, y);
  } else if (name == ALX) {
    state_.leftX = value;
    calculateJoystick(state_.leftX, state_.leftY, state_.leftDirection, state_.leftDistance);
  } else if (name == ALY) {
    state_.leftY = value;
    calculateJoystick(state_.leftX, state_.leftY, state_.leftDirection, state_.leftDistance);
  } else if (name == ARX) {
    state_.rightX = value;
    calculateJoystick(state_.rightX, state_.rightY, state_.rightDirection,
                      state_.rightDistance);
  } else if (name == ARY) {
    state_.rightY = value;
    calculateJoystick(state_.rightX, state_.rightY, state_.rightDirection,
                      state_.rightDistance);
  } else {
    return false;
  }
  return true;
}

void Gamepad::setButton(GamepadButton button, bool pressed) {
  switch (button) {
    case GamepadButton::Up:
      state_.up = pressed;
      break;
    case GamepadButton::Down:
      state_.down = pressed;
      break;
    case GamepadButton::Left:
      state_.left = pressed;
      break;
    case GamepadButton::Right:
      state_.right = pressed;
      break;
    case GamepadButton::Square:
      state_.square = pressed;
      break;
    case GamepadButton::Triangle:
      state_.triangle = pressed;
      break;
    case GamepadButton::Cross:
      state_.cross = pressed;
      break;
    case GamepadButton::Circle:
      state_.circle = pressed;
      break;
    case GamepadButton::L1:
      state_.l1 = pressed;
      break;
    case GamepadButton::R1:
      state_.r1 = pressed;
      break;
    case GamepadButton::L2:
      state_.l2 = pressed;
      break;
    case GamepadButton::R2:
      state_.r2 = pressed;
      break;
    case GamepadButton::ThumbL:
      state_.thumbL = pressed;
      break;
    case GamepadButton::ThumbR:
      state_.thumbR = pressed;
      break;
    case GamepadButton::M1:
      state_.m1 = pressed;
      break;
    case GamepadButton::M2:
      state_.m2 = pressed;
      break;
    case GamepadButton::Count:
      break;
  }
}

void Gamepad::setLeftStick(int x, int y) {
  state_.leftX = clampValue(x, -100, 100);
  state_.leftY = clampValue(y, -100, 100);
  calculateJoystick(state_.leftX, state_.leftY, state_.leftDirection, state_.leftDistance);
}

void Gamepad::setRightStick(int x, int y) {
  state_.rightX = clampValue(x, -100, 100);
  state_.rightY = clampValue(y, -100, 100);
  calculateJoystick(state_.rightX, state_.rightY, state_.rightDirection,
                    state_.rightDistance);
}

}  // namespace robotics
}  // namespace ohstem
