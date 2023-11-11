


void Sonar_sensor(controller_t *controls, int sonar_sensor_enabled, int randomWalk_enabled){

    // if(sonar_sensor_enabled){
      sensorData.values[0] = sonar_sensor.readDistance();  // Read distance from sensor
      // if(randomWalk_enabled){
        // randomWalk.execute(sensorData.values[0], randomW_force, randomW_z, randomW_yaw);

        zigzag.execute(sensorData.values[0], controls->tz, randomW_force, randomW_z, randomW_yaw);

        controls->fx = randomW_force;
        // controls->fz = randomW_z;
        controls->tz += randomW_yaw;


      // }else{
        // }
    // }

}