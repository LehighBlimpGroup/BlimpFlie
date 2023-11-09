// Camera sensor related variables
float max_x = 240;
float max_y = 160;
float detection_x = 0;
float detection_y = 0; 
float detection_w = 0; 
float detection_h = 0;
float tracking_x = 0;
float tracking_y = 0;
float tracking_w = 0; 
float tracking_h = 0;
float last_detection_x = 0;
float last_detection_y = 0;
float last_detection_w = 0;
float last_detection_h = 0;
float last_tracking_x = 0;
float last_tracking_y = 0;
float last_tracking_w = 0;
float last_tracking_h = 0;
float tof_dist = 0;

float des_yaw, _yaw, Ky, _height;
float control_yaw = 0;
float control_height = 0;
float control_fx = 0;
float des_height, gamma2;
float height_diff = 0;

bool detected = false;

float random_spiral = 0;
float goal_act = 0;
float robot_to_goal = 0;

unsigned long last_front_goal_time = 0;
unsigned long last_back_goal_time = 0;
unsigned long last_noise_time = 0;

void addNiclaControl(controller_t *controls, sensors_t *sensors, ModBlimp *blimp, nicla_tuning_s *nicla_tuning, bool init_bool){
  /*
    This part of the code is meant to parse the nicla information in combination with the sensor values.
    There are 5 sensors: camera (nicla), tof (nicla), yaw, height, and time
      our goal is to use these sensors to figure out what to do using our sensors to transition our state machine

    1. determining what the nicla is really seeing
      a. back goal, front goal, or noise/high angle
    2. 

  */
  blimp->IBus.loop();
  int nicla_flag = (int)blimp->IBus.readChannel(0);
  tracking_x = (float)blimp->IBus.readChannel(1);
  tracking_y = (float)blimp->IBus.readChannel(2);
  tracking_w = (float)blimp->IBus.readChannel(3);
  tracking_h = (float)blimp->IBus.readChannel(4);
  detection_x = (float)blimp->IBus.readChannel(5);
  detection_y = (float)blimp->IBus.readChannel(6);
  detection_w = (float)blimp->IBus.readChannel(7);
  detection_h = (float)blimp->IBus.readChannel(8);
  tof_dist = (float)blimp->IBus.readChannel(9);
  _yaw = sensors->yaw;
  _height = sensors->estimatedZ - sensors->groundZ;

  if (init_bool) {
    des_height = controls->fz;
    height_diff = controls->fz - _height;
  }


  if (nicla_flag == 3) {
    // Some Detection
    detected = false;
  } else if (last_tracking_x != tracking_x || last_tracking_y != tracking_y || last_detection_w != detection_w || last_detection_h != detection_h) {
    float x_cal = tracking_x / max_x;
    float y_cal = tracking_y / max_y;
    des_yaw = ((x_cal - 0.5) * nicla_tuning->x_cal_weight);
    robot_to_goal = _yaw - des_yaw;
    if (nicla_flag == 1) {
      detected = true;
      float relative_to_goal = nicla_tuning->goal_theta_back - robot_to_goal;
      relative_to_goal = atan2(sin(relative_to_goal), cos(relative_to_goal));
      goal_act = nicla_tuning->goal_theta_back;
      if (abs(relative_to_goal) > 3*PI/4) { // seeing front goal
        goal_act = goal_act + PI;
        goal_act = atan2(sin(goal_act), cos(goal_act));
        control_yaw = robot_to_goal * nicla_tuning->goal_ratio - goal_act * (1 - nicla_tuning->goal_ratio);
        control_yaw = atan2(sin(control_yaw), cos(control_yaw));
        last_front_goal_time = millis();
        // changeHeight(y, _height, nicla_tuning);
      } else if (abs(relative_to_goal) > PI/2){ //seeing noise
        last_noise_time = millis();
      } else{ // seeing back goal
        last_back_goal_time = millis();
        goal_act = atan2(sin(goal_act), cos(goal_act));
        control_yaw = robot_to_goal * nicla_tuning->goal_ratio - goal_act * (1 - nicla_tuning->goal_ratio);
        control_yaw = atan2(sin(control_yaw), cos(control_yaw));
        // changeHeight(y, _height, nicla_tuning);
      }
    } else if (nicla_flag == 0) {
      float calibrated_x = (detection_x / max_x); // 0 < val < 1
      float calibrated_y = (detection_y / max_y); // 0 < val < 1
      float calibrated_area = (detection_w*detection_h)/(max_x*max_y);
      if (last_tracking_x != tracking_x || last_tracking_y != tracking_y || last_detection_w != detection_w || last_detection_h != detection_h) {
        des_yaw = ((calibrated_x - 0.5) * PI / 2);
        control_yaw = (_yaw - des_yaw) * (1-calibrated_area); //* Ky; 
        control_yaw = atan2(sin(control_yaw), cos(control_yaw));
        // control_height = (_height - (calibrated_y - 0.8)*0.5);
      }
      if (abs(control_yaw - _yaw) < .2) { //Proceed a little bit and change orientation
        controls->fx = 0.2;
      } else { //Hover in place and change orientation
        controls->fx = 0;
      } 
    }
  }

  // else {// nicla has seen something, but not right now}

  if (detected == false) {
    randomWalkGoal(controls,  nicla_tuning);
  } else {
    random_spiral = nicla_tuning->max_move_x; //reset spiral for random walk

    if (max(last_detection_w, last_detection_h) > nicla_tuning->goal_dist_thresh){
      fullCharge(controls, nicla_tuning);
    } else{

      chargeGoal(controls, goal_act, robot_to_goal, nicla_tuning);
    }
  }



  // controls->fz = des_height;
  controls->tz = control_yaw;
  controls->fx = control_fx;

  last_tracking_x = tracking_x;
  last_tracking_y = tracking_y;
  last_detection_w = detection_w;
  last_detection_h = detection_h;
}


void chargeGoal(controller_t *controls, float goal_act, float robot_to_goal, nicla_tuning_s *nicla_tuning){
  /*
    If I see a goal at the proper angle, I want to charge towards it and pass through
      If the goal is small in the image, I want to change height and adjust
        maybe we should include the centering command to the code, also make sure that last known location in yaw is used if we want to random search for it again.
      if the goal is large, the change in angle and height should be massively reduced or non exsistant, so that it can confidently move through the hole
        Once we have charged through, we should try to determine if we have passed through by turing around and detecting a large goal.
          If not true then we move away and try again

  */

  if (abs(control_yaw - _yaw) < nicla_tuning->yaw_move_threshold){
    control_fx = nicla_tuning->max_move_x;
  } else {
    control_fx = 0;
  }
}

void randomWalkGoal(controller_t *controls,  nicla_tuning_s *nicla_tuning){
  /*
    If I have seen a 'wall' recently time < 10 seconds, then I want to stop(move backwards) (3 seconds), turn around opposite to the wall found(2 seconds), then move foward for 5 seconds
    If I have not seen a wall recently > 10 seconds and I havent seen a goal recently > 10 seconds, then I want to perform a random walk for 10 seconds up and down the highbay,
              this procedure stops and resets if a goal has been seen or 10 more seconds have passed.
    If I have not seen anything in 20 seconds, reset by finding a wall again.
  */
  random_spiral -= .05 * ((float)dt)/1000000.0f;
  if (random_spiral < .1) {
    random_spiral = nicla_tuning->max_move_x;
  }
  control_yaw = control_yaw  + nicla_tuning->spiral_strength*((float)dt)/1000000.0f; //TODO add constant for speed of spin
  control_yaw = atan2(sin(control_yaw), cos(control_yaw));
  control_fx = random_spiral;
}

void fullCharge(controller_t *controls,  nicla_tuning_s *nicla_tuning) {
  control_fx = nicla_tuning->max_move_x;
  
}

void changeHeight(float y, float _height,  nicla_tuning_s *nicla_tuning) {
  float y_cal = (last_tracking_y / max_y); 
  float h_cal = (last_detection_h / max_y);
  if (max(last_detection_w, last_detection_h) < nicla_tuning->goal_dist_thresh && max(last_detection_w, last_detection_h) > 20){
    // if (y_cal - h_cal/2 > nicla_tuning->height_threshold || y_cal + h_cal/2 < nicla_tuning->height_threshold) {
    des_height = des_height - ((y_cal - nicla_tuning->height_threshold) * nicla_tuning->height_strength)*((float)dt)/1000000.0f;
  }
}