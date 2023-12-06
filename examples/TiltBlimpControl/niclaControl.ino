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

float _yaw, Ky, _height;
float control_height = 0;
float control_fx = 0;
float des_height, gamma2;
float height_diff = 0;

bool detected = false;

float random_spiral = 0;
float goal_act = 0;
float robot_to_goal = 0; //real angle to goal
float control_yaw = 0; //current heading
float goal_yaw = 0; // approximate(slower) angle to goal
// float des_yaw;

unsigned long last_front_goal_time = 0;
unsigned long last_back_goal_time = 0;
unsigned long last_noise_time = 0;
unsigned long detected_timer = 0;
unsigned long walk_timer = 0;

float rolling_dist = 0;
float dist_gamma = 0.8;
unsigned long near_time_dist = 0;
unsigned long full_charge_timer = 0;

float goal_wheel[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float state1yaw = 0;

int control_time = 1000;
float control_angle = 0;


bool fresh = false;
bool charged = false;

int state = 0;






/*states:
 0 = goal_walk
    in goal walk, we want to wander around until we are confident we are seeing the correct goal
 1 = goal_tracking
    in goal_tracking we want to move towards the goal and hopefully go into it.
    If we fail to go through, we go back to goal_walk

*/

int addNiclaControl(controller_t *controls, sensors_t *sensors, ModBlimp *blimp, nicla_tuning_s *nicla_tuning, float sonar, bool init_bool){
  /*
    This part of the code is meant to parse the nicla information in combination with the sensor values.
    There are 5 sensors: camera (nicla), tof (nicla), yaw, height, and time
      our goal is to use these sensors to figure out what to do using our sensors to transition our state machine

    1. determining what the nicla is really seeing
      a. back goal, front goal, or noise/high angle
    2. 

  */
  blimp->IBus.loop();
  /* 
  the 8-bit flag variable
  From MSB to LSB
  [7]: 1 if tracked, 0 if not
  [6:3] : Reserved
  [2] : 1 if solid, 0 if hole
  [1] : 1 if green OR BW, 0 if orange
  [0] : 1 if goal, 0 if balloon
  */
  int nicla_flag = (int)blimp->IBus.readChannel(0);
  bool tracked =  (bool) (nicla_flag & 0b10000000);
  bool solid =    (bool) (nicla_flag & 0b00000100);
  bool green =    (bool) (nicla_flag & 0b00000010);
  bool goal =     (bool) (nicla_flag & 0b00000001);
  //tracking is bounding box
  tracking_x = (float)blimp->IBus.readChannel(1); 
  tracking_y = (float)blimp->IBus.readChannel(2);
  tracking_w = (float)blimp->IBus.readChannel(3);
  tracking_h = (float)blimp->IBus.readChannel(4);
  //detection is the exact frame
  detection_x = (float)blimp->IBus.readChannel(5);
  detection_y = (float)blimp->IBus.readChannel(6);
  detection_w = (float)blimp->IBus.readChannel(7);
  detection_h = (float)blimp->IBus.readChannel(8);
  float size = max(detection_w,detection_h);
  tof_dist = (float)blimp->IBus.readChannel(9);
  _yaw = sensors->yaw;
  _height = sensors->estimatedZ - sensors->groundZ;

  if (init_bool) {
    // des_height = 8.5;
    height_diff = controls->fz - _height;
    control_yaw = controls->tz;
    goal_yaw = controls->tz;
    //nicla_tuning->goal_theta_back = _yaw;
    state2start(); // just track goal
    //state3start(); // go to correct goal side
  }

  if (!tracked){//!tracked || !solid || max(detection_h, detection_w) < 15) {
    des_height = des_height * .995 +  (8.5) * .005;
    detected = false;
    fresh = false;
  } else if (last_detection_x != detection_x || last_detection_y != detection_y || last_detection_w != detection_w || last_detection_h != detection_h) {
    rolling_dist = rolling_dist *dist_gamma + tof_dist * (1-dist_gamma);
    float x_cal = tracking_x / max_x;
    float des_yaw = ((x_cal - 0.5) * nicla_tuning->x_cal_weight);
    robot_to_goal =  atan2(sin(_yaw - des_yaw), cos(_yaw - des_yaw)); //real
    goal_yaw = atan2(sin(_yaw - des_yaw * nicla_tuning->goal_ratio), cos(_yaw - des_yaw * nicla_tuning->goal_ratio));//approximate
    detected = true;
    fresh = true;
    detected_timer = millis();
  } else {
    detected = true;
    fresh = false;
  }


  /*
      random walk until a goal is seen
      in goal walk, we want to see where the goals are relative to the robot, and try to figure out which direction we want to go. 
        if goal == orange
          if goal one direction(close to goal_theta_back)
            we are in the front quadrant behind yellow
            move to right wall, then move toward goal direction for 10 seconds, 
              then left 90 degrees, then back to goal direction and search that 180 degrees for a goal,
               then transition back to beginnig or goal tracking depending on detected goal
          if goal two directions()
            we are between goals
              face towards goal_theta_back closest goal, and go to goal tracking
          if goal one direction (opposite to goal_theta_back)
            we are in the back quadrant behind orange
              face towards the goal and go to goal tracking.     
    */

  if (state == 4) {
    if (millis() - walk_timer < control_time) {
      if (millis() - walk_timer > 5000 && sonar < 400){
        control_angle = control_angle + PI;
      }
      control_yaw = control_angle;
      control_fx = nicla_tuning->max_move_x;
      
    } else{
      state1start();
    }
  }
  if (state == 3){ // walk to wall
    
    if (sonar > 400  && abs(_height - des_height) < 2) {
      //walk to wall
      control_yaw= nicla_tuning->goal_theta_back- PI/2;// move towards the right wall
      control_fx = nicla_tuning->max_move_x;

    } else{
      state0start();
    }
  }
  if (state == 0) {
    if (millis() - walk_timer < 25000) {
      control_yaw= nicla_tuning->goal_theta_back+ PI/10;// move towards goal
      control_fx = nicla_tuning->max_move_x;
      if (millis() - walk_timer > 15000 && detected){
        state1start();
      } else if (millis() - walk_timer > 8000 && sonar < 400){
        float new_yaw = nicla_tuning->goal_theta_back+ PI/10 + PI;
        new_yaw = atan2(sin(new_yaw), cos(new_yaw));
        state4start(new_yaw, (int)((millis() - walk_timer)/2));
      }
    } else {
      state1start();
    }
  }
  if (state == 1) {
    // spin in place 360 and save directions of goals
    // then decide wether to go
    state1yaw += nicla_tuning->spiral_strength/2 * (float)dt/1000000.0f;
    if (state1yaw > 2*PI){
      // do logic here which reads the goal_wheel for determining what state to go to next
      //goal_wheel = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];//-pi - pi
      int index_goal_1 = -1;
      int index_goal_2 = -1; 
      float goal_angle = nicla_tuning->goal_theta_back;
      float array_step = 2*PI/20;
      for (int i = 0; i < 20; i++){
        float arr_yaw = i * array_step;
        float arr_relative_to_goal = atan2(sin(goal_angle + arr_yaw), cos(goal_angle + arr_yaw));
        if (abs(arr_relative_to_goal) < PI/2){ //close to the goal
          if (goal_wheel[i] != 0){
            if (index_goal_1 == -1){
              index_goal_1 = i;
            } else if(goal_wheel[i] > goal_wheel[index_goal_1]){
              index_goal_1 = i;
            }
          }
        } else {//far from the goal
          if (goal_wheel[i] != 0){
            if (index_goal_2 == -1){
              index_goal_2 = i;
            } else if(goal_wheel[i] > goal_wheel[index_goal_2]){
              index_goal_2 = i;
            }
          }
        }
      }
      if (index_goal_1 == -1 && index_goal_2 == -1){
        // no goal detections
        // need to random walk - do goal search anyway
        state2start();
      } else if (index_goal_1 != -1 && index_goal_2 == -1) {
        //wrong side
        //go to the middle under/ to the side of the goal
        state2start();
        // goal_yaw = index_goal_1 * array_step;
        // goal_yaw = atan2(sin(goal_yaw), cos(goal_yaw));
        // control_yaw = goal_yaw;
        // state4start(goal_yaw, 15000);
      } else if (index_goal_1 == -1 && index_goal_2 != -1) {
        //in corner in correct side
        // go to goal
        goal_yaw = index_goal_2 * array_step;
        goal_yaw = atan2(sin(goal_yaw), cos(goal_yaw));
        control_yaw = goal_yaw;
        state2start();
      } else { // detect 2 goals( could be between goals)
        float arr_goal_diff = atan2(sin((index_goal_1 + index_goal_2)* array_step), cos((index_goal_1 + index_goal_2)* array_step));
        if (abs(arr_goal_diff) > PI/2){
          
          goal_yaw = index_goal_1 * array_step;
          goal_yaw = atan2(sin(goal_yaw), cos(goal_yaw));
          control_yaw = goal_yaw;
          state2start();
          // head towards the  goal on the correct side
        } else{
          //random walk explore some more
          state2start();
        }
      }
      
    } else {
      control_yaw += nicla_tuning->spiral_strength/2 * (float)dt/1000000.0f;
      control_yaw = atan2(sin(control_yaw), cos(control_yaw));
      control_fx = 0;
    }
    if (detected){
      int goal_index = (int) (((robot_to_goal/3.14) + 1)*20);
      goal_wheel[goal_index] = max(goal_wheel[goal_index], size);
    }
  }
  if (state == 2){
    if (millis() - walk_timer < 3000){
      // initialization of state 2; wait to reach the desired yaw first
      control_yaw = control_yaw;
      control_fx = 0;
    }
    else if (millis() - full_charge_timer < 6000){
      random_spiral = nicla_tuning->max_move_x; //reset spiral for random walk
      fullCharge(controls, nicla_tuning);
    } else {
      if (fresh){
        changeHeight(tracking_y, size, _height, nicla_tuning);
      }
      charged = false;
      if (millis() - detected_timer > 3000) { // if there is no detection, random walk
        state6start();
        //detected_timer = millis();
      } else { // if there is a detection, try to position better relative to the goal
        //if (detected_timer > 3000 && )
        if (max(detection_h, detection_w) > nicla_tuning->goal_theta_front && abs((detection_y / max_y) - nicla_tuning->height_threshold) < .2){
          full_charge_timer = millis();
          fullCharge(controls, nicla_tuning);
        } else{
        
          chargeGoal(controls, goal_act, robot_to_goal, nicla_tuning);
        }

        random_spiral = nicla_tuning->max_move_x; //reset spiral for random walk
      }
    }
  }
  if (state == 5) {
    //balloon follow
    if (millis() - full_charge_timer < 9000){
      random_spiral = nicla_tuning->max_move_x; //reset spiral for random walk
      fullCharge(controls, nicla_tuning);
    } else {
      if (fresh){
        changeHeight(tracking_y, size, _height, nicla_tuning);
      }
      charged = false;
      if (detected == false) { // if there is no detection, random walk
        randomWalkGoal(controls,  nicla_tuning);

      } else { // if there is a detection, try to position better relative to the goal
        if (max(last_detection_w, last_detection_h) > nicla_tuning->goal_dist_thresh){
          full_charge_timer = millis();
          fullCharge(controls, nicla_tuning);
        } else{
          chargeBalloon(controls, goal_act, robot_to_goal, nicla_tuning);
        }
        random_spiral = nicla_tuning->max_move_x; //reset spiral for random walk
      }
    }
  }
  if (state == 6){
    randomLeviWalk();
  }
    
  

  controls->fz = des_height;
  controls->tz = control_yaw;
  controls->fx = control_fx;
  // Serial.print("des height: ");
  // Serial.println(des_height);
  // Serial.print("des yaw: ");
  // Serial.println(control_yaw);
  // Serial.print("des fx: ");
  // Serial.println(control_fx);

  last_detection_x = detection_x;
  last_detection_y = detection_y;
  last_detection_w = detection_w;
  last_detection_h = detection_h;
  return int(state);
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

  control_yaw = goal_yaw;
  // if (abs((detection_y / max_y) - nicla_tuning->height_threshold) < .2){
  //   control_fx = 0;
    
  // }
  if (abs(control_yaw - _yaw) < nicla_tuning->yaw_move_threshold ){
  
    control_fx = nicla_tuning->max_move_x/4;
  } else {
    control_fx = 0;
  }
}


void chargeBalloon(controller_t *controls, float goal_act, float robot_to_goal, nicla_tuning_s *nicla_tuning){
  /*
    If I see a goal at the proper angle, I want to charge towards it and pass through
      If the goal is small in the image, I want to change height and adjust
        maybe we should include the centering command to the code, also make sure that last known location in yaw is used if we want to random search for it again.
      if the goal is large, the change in angle and height should be massively reduced or non exsistant, so that it can confidently move through the hole
        Once we have charged through, we should try to determine if we have passed through by turing around and detecting a large goal.
          If not true then we move away and try again
  */

  control_yaw = goal_yaw;
  if (abs(control_yaw - _yaw) < nicla_tuning->yaw_move_threshold){
    control_fx = nicla_tuning->max_move_x/5;
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
  if (random_spiral < .02) {
    random_spiral = nicla_tuning->max_move_x;
  }
  control_yaw = control_yaw  + nicla_tuning->spiral_strength*((float)dt)/1000000.0f; //TODO add constant for speed of spin
  control_yaw = atan2(sin(control_yaw), cos(control_yaw));
  control_fx = random_spiral;
}

void fullCharge(controller_t *controls,  nicla_tuning_s *nicla_tuning) {
  // control_yaw = control_yaw;
  control_fx = nicla_tuning->max_move_x;
}

void changeHeight(float _y, float _h, float _height,  nicla_tuning_s *nicla_tuning) {
  float y_cal = (_y / max_y);

  if (_h < nicla_tuning->goal_dist_thresh && _h > 20){
    // if (y_cal - h_cal/2 > nicla_tuning->height_threshold || y_cal + h_cal/2 < nicla_tuning->height_threshold) {
    des_height = des_height  - (y_cal - nicla_tuning->height_threshold) * nicla_tuning->height_strength * (float)dt/1000000.0;
    if (des_height < 7){
      des_height = 7;
    }
    if (des_height > 11){
      des_height = 11;
    }
  }
}



void tooCloseGoal(nicla_tuning_s *nicla_tuning){
  des_height += 0.05*((float)dt)/1000000.0f; //+= nicla_tuning->spiral_strength*((float)dt)/1000000.0f;
  control_fx =  -nicla_tuning->max_move_x;
}

void state0start(){ //randomwalk
  changeNiclaTarget(0x81);
  state =0;
  walk_timer = millis();

}

void state1start(){ //spin search
  changeNiclaTarget(0x81);
  walk_timer = millis();
  state = 1;
  for (int i = 0; i < 20; i++){
    goal_wheel[i] = 0;
  }
  
  state1yaw = 0;

}

void state2start(){
  changeNiclaTarget(0x81);
  state = 2;
  walk_timer = millis();
}

void state3start(){
  changeNiclaTarget(0x81);
  state = 3;
  walk_timer = millis();
}


void state4start(float control_angle_4, int control_time_4){
  changeNiclaTarget(0x81);
  state = 4;
  walk_timer = millis();
  control_angle = control_angle_4;
  control_time = control_time_4;
}


void state5start(){
  changeNiclaTarget(0x80);
  des_height = des_height;
  state = 5;
  walk_timer = millis();
}

float walk_time_levi = 0;
float random_angle = 0;
float levi_height = 7;
void state6start(){
  
  walk_timer = millis();
  walk_time_levi = 7.0f*(pow((float)random(130, 1000)/1000.0f, -1.0f / 3.0f) - 1.0f);
  random_angle = ((float) random(0, 360)) / PI - PI;
  levi_height = random(7, 11);
  state = 6;
}

void randomLeviWalk(){
  if (millis() - walk_timer < walk_time_levi * 1000) {
    // Serial.print(millis() - walk_timer );
    // Serial.print("<" );
    // Serial.println(walk_time_levi * 1000 );
    des_height = levi_height;
    control_yaw = random_angle;
    control_fx = .15;
    if (detected){
      state2start();
      control_yaw = goal_yaw;
      control_fx = 0.0;
    }
    
  } else {
    state6start();
  }
}

void changeNiclaTarget(char state) {
  // 0x80 for balloon, 0x81 for goal
  Serial.println(int(state));
  blimp.sendMySerial(state);
}
// check if goal is in the proper position or if it is actually seeing a wall. 
      // if (abs(relative_to_goal) > 3*PI/4) { // seeing front goal
      //   goal_act = goal_act + PI;
      //   goal_act = atan2(sin(goal_act), cos(goal_act));
        
      //   goal_yaw = robot_to_goal - nicla_tuning->goal_theta_front*atan2(sin(goal_act - robot_to_goal), cos(goal_act - robot_to_goal))/
      //             (nicla_tuning->goal_ratio - (nicla_tuning->goal_ratio-1) * max(detection_h, max(detection_w,max_y))/max_y );
      //   goal_yaw = atan2(sin(goal_yaw), cos(goal_yaw));
      //   last_front_goal_time = millis();
      //   changeHeight(tracking_y, max(detection_h, detection_w), _height, nicla_tuning);
        
      //   detected = true;
      // }
      // else if (abs(relative_to_goal) > PI/2){ //seeing noise
      //   last_noise_time = millis();
      //   detected = false;
        
      // } else{ // seeing back goal
      //   last_back_goal_time = millis();
      //   goal_act = atan2(sin(goal_act), cos(goal_act));
      //   goal_yaw = robot_to_goal - nicla_tuning->goal_theta_front*atan2(sin(goal_act - robot_to_goal), cos(goal_act - robot_to_goal))/
      //             (nicla_tuning->goal_ratio - (nicla_tuning->goal_ratio-1) * max(detection_h, max(detection_w,max_y))/max_y );
      //   goal_yaw = atan2(sin(goal_yaw), cos(goal_yaw));
      //   changeHeight(tracking_y, size, _height, nicla_tuning);
        
      //   detected = true;
      // }


        
  // // do stuff with the information gathered.
  // if (rolling_dist > 300){
  //   near_time_dist = millis();// tof is not detecting something close by 
  // }
  
  // if (millis() - near_time_dist > 3000){//detect if time of flight is too near (stuck on a goal)
  // //TODO make a flag that turns this off
  //   tooCloseGoal(nicla_tuning);
  // } else