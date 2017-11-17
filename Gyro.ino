void initGyro() {
  //Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    //Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.calibrateGyro();
  
  mpu.setThreshold(10);

}
