#include "DeterministicWalk.h"
#include "math.h"

DeterministicWalk::DeterministicWalk(float forward_force, int min_distance, int des_z)
: _forward_force(forward_force), _min_distance(min_distance), _des_z(des_z),
  _time_backward(2000), _TIME_ROTATE(2000), _des_yaw(radians(295)), _SWITCH_TIME(10000),
  _current_action(0), _time_elapse(0), _forward_zig_zag(1), _zz_counter(0),
  _STEP_ZIG_ZAG(radians(15)) {}

void DeterministicWalk::begin() {
  restart_timer();
}

void DeterministicWalk::execute(int distance_sensor, float yaw_sensor, float &force, int &z, float &yaw) {

  choose_action(distance_sensor, yaw_sensor);

  switch (_current_action) {
    case 0:
      action_move_forward(force, z, yaw);
      break;
    case 1:
      action_move_backward(force, z, yaw);
      break;
    case 2:
      action_rotate(yaw_sensor, force, z, yaw);
      break;
    case 3:
      action_wait(force, z, yaw);
      break;
    default:
      action_wait(force, z, yaw);
      break;
  }
}

void DeterministicWalk::setForwardForce(float forward_force) {
  _forward_force = forward_force;
}


void DeterministicWalk::set_SWITCH_TIME(float SWITCH_TIME) {
  _SWITCH_TIME = SWITCH_TIME;
}

void DeterministicWalk::set_TIME_ROTATE(float TIME_ROTATE) {
  _TIME_ROTATE = TIME_ROTATE;
}

void DeterministicWalk::set_STEP_ZIG_ZAG(float STEP_ZIG_ZAG) {
  _STEP_ZIG_ZAG = STEP_ZIG_ZAG;
}

void DeterministicWalk::setMinDistance(int min_distance) {
  _min_distance = min_distance;
}

void DeterministicWalk::setDesZ(int des_z) {
  _des_z = des_z;
}

void DeterministicWalk::choose_action(int distance_sensor, float yaw_sensor) {

  unsigned long time_elapsed = this->time_elapsed();

  if (_current_action == 0 && (distance_sensor < _min_distance || time_elapsed > _SWITCH_TIME)) {
    _current_action = 1;
    restart_timer();
    _zz_counter++;
    if (_zz_counter > 8) {
      _zz_counter = 0;
      _forward_zig_zag *= -1;
    }


  } else if (_current_action == 1 && time_elapsed > _time_backward) {
    _current_action = 2;
    restart_timer();
  } else if (_current_action == 2) {
    _current_action = 3;
  } else if (_current_action == 3 && (time_elapsed > _TIME_ROTATE) ) {// || yaw_sensor - yaw < 0.1
    _current_action = 0;
    restart_timer();
  }

    Serial.print("Fwd = ");
    Serial.print(_forward_force);
    Serial.print(" ZZ = ");
    Serial.print(_zz_counter);
    Serial.print(" action = ");
    Serial.println(_current_action);

//print("Fwd=", self.forward_zig_zag,
//"ZZ=", self.zz_counter, self.actions[self.current_action].__name__, degrees(abs(self._angle_bounded(feedback[1]-self.des_yaw))),self.des_yaw,self.yaw )
}

void DeterministicWalk::restart_timer() {
  _time_elapse = millis();
}

unsigned long DeterministicWalk::time_elapsed() {
  return millis() - _time_elapse;
}

void DeterministicWalk::action_move_forward(float &force, int &z, float &yaw) {
  force = _forward_force;
  z = _des_z;
  yaw = _des_yaw;
}

void DeterministicWalk::action_move_backward(float &force, int &z, float &yaw) {
  force = - 1.5 * _forward_force;
  z = _des_z;
  yaw = _des_yaw;
}

// Action method to rotate
void DeterministicWalk::action_rotate(float yaw_sensor, float &force, int &z, float &yaw) {
    // Increment the counter
//    _zz_counter++;

    // Check if the counter exceeds NUM_ZIGS
//    if (_zz_counter > NUM_ZIGS) {
//        _zz_counter = 0;
//        _forward_zig_zag *= -1;
//        _des_z += _forward_zig_zag * Z_LEVEL;
//    }

    // Normalize the yaw to be within -PI to PI
    float normalized_yaw = atan2(sin(yaw_sensor), cos(yaw_sensor));

    if (normalized_yaw > 0) {
        _des_yaw = radians(270 + ANGLE_OFFSET) + _STEP_ZIG_ZAG * _forward_zig_zag;
    } else {
        _des_yaw = radians(90 + ANGLE_OFFSET) - _STEP_ZIG_ZAG * _forward_zig_zag;
    }

    Serial.print("des_yaw");
    Serial.println(degrees(_des_yaw));

    force = 0;
    z = _des_z;
    yaw = _des_yaw;
}

//void DeterministicWalk::action_rotate(float yaw_sensor, float &force, int &z, float &yaw) {
//    // Normalize the yaw to be within -PI to PI
//    float normalized_yaw = atan2(sin(yaw_sensor), cos(yaw_sensor));
//
//    if (normalized_yaw > 0) {
//        _des_yaw = radians(270 + 8) + _step_zig_zag * _forward_zig_zag;
//    } else {
//        _des_yaw = radians(90 + 8) - _step_zig_zag * _forward_zig_zag;
//    }
//
//    force = 0;
//    z = _des_z;
//    yaw = _des_yaw;
//}

void DeterministicWalk::action_wait(float &force, int &z, float &yaw) {
    // In a wait action, typically you might set the force to 0
    // to indicate no movement, and maintain the current z and yaw values.
    force = 0;
    z = _des_z;
    yaw = _des_yaw;
}
