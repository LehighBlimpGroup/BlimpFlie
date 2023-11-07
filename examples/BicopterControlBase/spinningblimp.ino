// creates the output values used for actuation from the control values
void SpinniggetOutputs(controller_t *controls, sensors_t *sensors, actuation_t *out)
{
  // set output to default if controls not ready
  if (controls->ready == false)
  {
    out->s1 = 0.5f; // angle  is based on 0 > PI and 0.5 is based on the normalized value.
    out->s2 = 0.5f; // this portion means that 0.5 should be centered
    // out->s1 = PI * 0.35;
    // out->s2 = PI * 0.35;
    out->m1 = 0;
    out->m2 = 0;
    out->ready = false;
    return;
  }
  out->ready = true;


  // Spinning Blimp periodic controller
  float yaw_calibrate = 0;
  float joytheta = raws.data[5];
  float joymag   = raws.data[0];

  // sensors->yaw is the raw senor reading without feedback
  s_yaw = sensors->yaw;

  if (0 <= ((s_yaw + joytheta)) && ((s_yaw + joytheta)) < PI){
    tau = joymag;
  } else{
    tau = -joymag;
  }

  // sf1 and sf2 variables associated with the spinning blimp motor values
  // Bang Bang motor configuration

  // // For CW spinning blimp
  // sf1 = controls->fz + (tau); // LHS motor
  // sf2 = controls->fz - (tau); // RHS motor

  // For CCW spinning blimp
  sf1 = controls->fz - (tau); // LHS motor
  sf2 = controls->fz + (tau); // RHS motor

  // Motor outputs
  out->m1 = clamp(sf1, 0, 1);
  out->m2 = clamp(sf2, 0, 1);
}
