#ifndef DeterministicWalk_h
#define DeterministicWalk_h

#include "math.h"
#include "Arduino.h"

class DeterministicWalk {
  public:
    DeterministicWalk(float forward_force = 0.3, int min_distance = 500, int des_z = 7);

    void begin();
    void execute(int distance_sensor, float yaw_sensor, float &force, int &z, float &yaw);

    void setForwardForce(float forward_force);
    void setMinDistance(int min_distance);
    void setDesZ(int des_z);

    void set_SWITCH_TIME(float SWITCH_TIME);

    void set_TIME_ROTATE(float TIME_ROTATE);




  private:

    int _min_distance;
    int _des_z;
    int _time_backward;
//    int _time_rotate;
    float _des_yaw;
    int _current_action;
    unsigned long _time_elapse;
    int _forward_zig_zag;
    int _zz_counter;
    float _step_zig_zag;

    float _forward_force = 0.3;
    int NUM_ZIGS = 5;
    int Z_LEVEL = 3;
    float _SWITCH_TIME;
    float _TIME_ROTATE;
    float STEP_ZIG_ZAG;


    void choose_action(int distance_sensor, float yaw_sensor);
    void restart_timer();
    unsigned long time_elapsed();
    void action_move_forward(float &force, int &z, float &yaw);
    void action_move_backward(float &force, int &z, float &yaw);
    void action_wait(float &force, int &z, float &yaw);
    void action_rotate(float yaw_sensor, float &force, int &z, float &yaw);
//    float normalize_yaw(float yaw);
};


#endif
