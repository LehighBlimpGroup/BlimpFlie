


float x_cal;
float calibrated_x, calibrated_y;
float max_x = 240;
float max_y = 160;
float _x, _y, _w, _h;
float tracking_x, tracking_y, tracking_w, tracking_h;
float detection_x, detection_y, detection_w, detection_h;
float des_yaw, _yaw, Ky, _height;
float last_x = 0;
float last_y = 0;
float last_w = 0;
float last_h = 0;
float control_yaw = 1;
float control_height = 1;
float des_height, gamma2;

void addNiclaControl(controller_t *controls, sensors_t *sensors, ModBlimp *blimp){
  
  blimp->IBus.loop();
  // Getting mode from Nicla
  int nicla_flag = (int)blimp->IBus.readChannel(0); 
  

  if (nicla_flag == 1) { // Goal Detection
    _x = (float)blimp->IBus.readChannel(1);
    _y = (float)blimp->IBus.readChannel(2);
    _w = (float)blimp->IBus.readChannel(7);
    _h = (float)blimp->IBus.readChannel(8);
    _yaw = sensors->yaw;
    _height = sensors->estimatedZ - sensors->groundZ;
    x_cal = (last_x / max_x); //- x_min_cal) / (x_max_cal - x_min_cal);

    if (_x == 0 && _y == 0 && _w == 0 && _h == 0) {
      //detected = false;
    } else if (last_x != _x || last_y != _y || last_w != _w || last_h != _h) {
      //detected = true;
      des_yaw = ((x_cal - 0.5) * PI / 2);
      control_yaw = _yaw - des_yaw; //* Ky; 
      // Removed * .2 + control_yaw* .8 as it seemed like a leftover code
      // des_height = des_height * gamma2 + ((y - max_y / 2) / max_y) * (1 - gamma2);
      // control_height = _height;
    }

    if (abs(control_yaw - _yaw) < .1) { //Proceed a little bit and change orientation
      controls->fx = 0.3;
    } else { //Hover in place and change orientation
      controls->fx = 0;
    } 

    controls->tz = control_yaw;
    last_x = _x;
    last_y = _y;
    last_w = _w;
    last_h = _h;

  } else if (nicla_flag == 0) { //Balloon Detection
    tracking_x = (float)blimp->IBus.readChannel(1);
    tracking_y = (float)blimp->IBus.readChannel(2);
    tracking_w = (float)blimp->IBus.readChannel(3);
    tracking_h = (float)blimp->IBus.readChannel(4);
    detection_x = (float)blimp->IBus.readChannel(5);
    detection_y = (float)blimp->IBus.readChannel(6);
    detection_w = (float)blimp->IBus.readChannel(7);
    detection_h = (float)blimp->IBus.readChannel(8);

    Serial.print("tracking_x: ");
    Serial.println(tracking_x);
    Serial.print("tracking_y: ");
    Serial.println(tracking_y);
    Serial.print("tracking_w: ");
    Serial.println(tracking_w);
    Serial.print("tracking_h: ");
    Serial.println(tracking_h);
    
    _yaw = sensors->yaw;
    _height = sensors->estimatedZ - sensors->groundZ;
    calibrated_x = (detection_x / max_x); // 0 < val < 1
    calibrated_y = (detection_y / max_y); // 0 < val < 1

    if (tracking_w == 0 && tracking_h == 0 && detection_w == 0 && detection_h == 0) {
      // No detection
    } else if (last_x != tracking_x || last_y != tracking_y || last_w != detection_w || last_h != detection_h) {
      des_yaw = ((calibrated_x - 0.5) * PI / 2);
      control_yaw = (_yaw - des_yaw) ; //* Ky; 
      control_yaw = atan2(sin(control_yaw), cos(control_yaw));
      control_height = (_height - (calibrated_y - 0.8)*0.5);
    }

   
    if (abs(control_yaw - _yaw) < .2) { //Proceed a little bit and change orientation
      controls->fx = 0.2;
    } else { //Hover in place and change orientation
      controls->fx = 0;
    } 

    controls->fz = control_height;
    controls->tz = control_yaw;
    last_x = tracking_x;
    last_y = tracking_y;
    last_w = detection_w;
    last_h = detection_h;
  }
}