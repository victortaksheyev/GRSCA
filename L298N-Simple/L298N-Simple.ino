#include <L298N.h>
#include <Encoder.h>
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB

// gyro = degrees/sec
// rotation of the sensor clockwise (correct direction of text is upwards) results in postive gyroZ

// 480 ticks = 1 rev


MPU9250_DMP imu;
float degrees = 0;

//pin definition
#define EN 9
#define IN1 8
#define IN2 10
#define I_ROCKET 10     // i dont even know what units
#define I_RING 4        // i dont even know what units
#define DPS_MIN 0
#define DPS_MAX 2000
#define MOTOR_MIN 0     // 0, but can it actually spin at 0?
#define MOTOR_MAX 255

int curr_time = 0;

//create a motor instance
L298N motor(EN, IN1, IN2);
Encoder encoder(11,12);

void setup() {

  //used for display information
  SerialPort.begin(115200);

  motor.setSpeed(255); // an integer between 0 and 255

  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  imu.setSensors(INV_XYZ_GYRO); // Enable gyroscope only
  imu.setGyroFSR(DPS_MAX); // Set gyro to 2000 dps

  imu.dmpBegin(DMP_FEATURE_GYRO_CAL |   // Enable gyro cal
              DMP_FEATURE_SEND_CAL_GYRO,// Send cal'd gyro values
              10);                   // Set DMP rate to 10 Hz

  delay(8000);

}

float av_old  = 0;
float gyroZ;

void loop() {
    if (imu.fifoAvailable() ) {
      // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
      if (imu.dmpUpdateFifo() == INV_SUCCESS) {
//        SerialPort.print(calcCorrection(av_old));
        float correction_av = calcCorrection(av_old);

        performCorrection(correction_av);
        // if (calcCorrection(av_old) < 0){
        //   SerialPort.print("left"); 
        // } else {
        //   SerialPort.print("right"); 
        // }
      }
    }

    delay(10);

}

// reads gyroZ from the sensor and logs it to the serial 
float getGyroZ(bool log) {
  float gyroZ = imu.calcGyro(imu.gz);
  
  if (log)
    SerialPort.print(gyroZ);

  return gyroZ;
}

// takes old angular velocity (av_old)
// samples the new angular velocity (av_new)
// performs calculation
// updates old angular velocity
// returns the correction for the control wheel

float calcCorrection(float& av_old)
{  
  float av_new = getGyroZ(true);
  
  //  perform calculation
  float delta_av = av_new - av_old;
  float ring_av = (I_ROCKET / I_RING) * (delta_av);
  av_old = av_new;

  return ring_av;
}

float performCorrection(float correction_av) {
  float motor_av_t = map(abs(correction_av), DPS_LOW, DPS_HIGH, MOTOR_MIN, MOTOR_MAX);
  motor.setSpeed(motor_av); // set to computed gyroscope angular velocity
  if (correction_av > 0) {
    // while (motor_av_r < motor_av_t)
    motor.forward();        // continue moving forwards while motor_av_r(eal) != motor_av_t(heoretical)
  } else {
    // while (motor_av_r < motor_av_t)
    motor.backward();       // continue moving backwards while motor_av_r(eal) != motor_av_t(heoretical)
  }
  // verify that the motor has reached that angular velocity
  // control system needs to be implemented here

  if ()
  
  
  
}