#include "Wire.h"
#include "D:/Uyung/TA/Tsunami/Program/MPU-9250-AHRS-master/libs/I2Cdev.cpp"
#include "D:/Uyung/TA/Tsunami/Program/MPU-9250-AHRS-master/libs/MPU9250.cpp"
#include "SD.h"
#include "SPI.h"

MPU9250 mpu;
I2Cdev   I2C_M;
File sdCard;

int CS_pin = 53;

// offsets and correction matrix for accel and mag
float A_B[3]
{ 1529.76,  467.31,  -25.25};

float A_Ainv[3][3]
{ {  0.59100, -0.04303,  0.02990},
  { -0.04303,  0.63281, -0.00879},
  {  0.02990, -0.00879,  0.62263}
};

// mag offsets and correction matrix
float M_B[3]
{   -14.41,   21.43,   13.82};

float M_Ainv[3][3]
{ {  1.84445, -0.01730,  0.00046},
  { -0.01730,  1.90095, -0.01225},
  {  0.00046, -0.01225,  1.78335}
};

float G_off[3] = { -335.7, -86.1, 275.8}; //raw offsets, determined for gyro at rest

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.14159f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

//raw data and scaled as vector
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];

float Araw[3];
float Graw[3];
float Mraw[3];

float Acal[3];
float Gcal[3];
float Mcal[3];
#define gscale (250./32768.0)*(PI/180.0)  //gyro default 250 LSB per d/s -> rad/s

// globals for AHRS loop timing

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
float yaw, pitch, roll; //Euler angle output
float lin_acc[6] {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};  // linear acceleration (acceleration with gravity component subtracted)

unsigned long now = 0, last = 0; //micros() timers
float deltat = 0;  //loop time in seconds
unsigned long now_ms, last_ms = 0; //millis() timers
unsigned long print_ms = 20; //print every "print_ms" milliseconds

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  while (!Serial); //wait for connection

  // initialize device
  mpu.initialize();
  // verify connection
  Serial.println(mpu.testConnection() ? "MPU9250 OK" : "MPU9250 ??");
  last = micros();

  //---SD Card SetUp---//
/*  
  Serial.print("Initializing SD card...");
  if(!SD.begin(CS_pin)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  sdCard = SD.open("DATA.txt", FILE_WRITE);
  if (sdCard) {
    Serial.println("File opened ok");
    // print the headings for our data
    sdCard.println("Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz, q0, q1, q2, q3"); //Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz, LinAx, LinAy, LinAz, q0, q1, q2, q3
  }
  sdCard.close();*/
}

void loop() {
  // put your main code here, to run repeatedly:
  get_MPU_scaled();
  //get_MPU_cal();
  now = micros();
  deltat = (now - last) * 1.0e-6; //seconds since last update
  last = now;

  /* Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
   *  the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
   * We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
   * For the MPU9250+MS5637 Mini breakout the +x accel/gyro is North, then -y accel/gyro is East. So if we want te quaternions properly aligned
   * we need to feed into the Madgwick function Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity is by convention
   * positive down, we need to invert the accel data, so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the Madgwick
   * function to get North along the accel +x-axis, East along the accel -y-axis, and Down along the accel -z-axis.
   * This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
  */
  //filter.update(gyr.y(),gyr.x(),-gyr.z(),-acc.y(),-acc.x(),acc.z(),-hag.y(),-hag.x(),hag.z());
  //filter.update(Gxyz[1], Gxyz[0], -Gxyz[2], -Axyz[1], -Axyz[0], Axyz[2], -Mxyz[1], -Mxyz[0], Mxyz[2]);
  //MadgwickQuaternionUpdate(-ax, ay, az, gx * pi / 180.0f, -gy * pi / 180.0f, -gz * pi / 180.0f,  my,  -mx, mz);
  //MadgwickQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[1], Mxyz[0], -Mxyz[2]);
  MadgwickQuaternionUpdate(-Axyz[0], Axyz[1], Axyz[2], Gxyz[0], -Gxyz[1], -Gxyz[2], Mxyz[1], -Mxyz[0], Mxyz[2]);
  //MadgwickQuaternionUpdate(Gxyz[1], Gxyz[0], -Gxyz[2], -Axyz[1], -Axyz[0], Axyz[2], -Mxyz[1], -Mxyz[0], Mxyz[2]);
  
  //linear_acceleration(q[0], q[1], q[2], q[3]);
  /*roll = atan2f(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2]);
  pitch = asinf(-2.0f * (q[1]*q[3] - q[0]*q[2]));
  yaw = atan2f(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]);

  // to degrees
  yaw   *= 57.29578f;
  pitch *= 57.29578f;
  roll *= 57.29578f;
  
  // corrected for local magnetic declination
  yaw = yaw - 0.29;
  if(yaw <= 0) {
    yaw = 360 + yaw; 
  }*/
  /*if(yaw >= 180) {
    yaw = 360 - yaw;
  }
  if(yaw <= -180.0) {
    yaw = 360.0 + yaw;
  }*/
  
  /*roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
  
  // to degrees
  yaw   *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;

  // http://www.ngdc.noaa.gov/geomag-web/#declination
  //conventional nav, yaw increases CW from North, corrected for local magnetic declination

  yaw = -yaw + 0.29;
  if (yaw < 0) yaw += 360.0;
  if (yaw >= 360.0) yaw -= 360.0;*/
  
  now_ms = millis(); //time to print?
  if (now_ms - last_ms >= print_ms) {
    last_ms = now_ms;
    // print angles for serial plotter...
    Serial.print(now_ms*0.001); Serial.print(", ");

    //print_accRawCal(); Serial.println("");
    //print_gyrRawCal(); //Serial.println("");
    
    //Serial.print(Axyz[2], 2); Serial.println("");
    print_acc(); Serial.print(", "); 
    print_gyro(); Serial.print(", "); 
    print_mag(); Serial.print(", ");
    
    print_q(); Serial.println("");
    //Logging();

    //print_linAcc(); Serial.println("");
    //print_linAcc2(); Serial.println("");

    /*Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);*/
  }
}

void get_MPU_scaled(void) {
  float temp[3];
  int i;
  mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  Gxyz[0] = ((float) gx - G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
  Gxyz[1] = ((float) gy - G_off[1]) * gscale;
  Gxyz[2] = ((float) gz - G_off[2]) * gscale;

  Axyz[0] = (float) ax;
  Axyz[1] = (float) ay;
  Axyz[2] = (float) az;
  //apply offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  Mxyz[0] = (float) mx;
  Mxyz[1] = (float) my;
  Mxyz[2] = (float) mz;
  //apply offsets and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

void get_MPU_cal(void) {
  float temp[3];
  int i;
  mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  Graw[0] = (float) gx * gscale;
  Graw[1] = (float) gy * gscale;
  Graw[2] = (float) gz * gscale;

  Araw[0] = (float) ax;
  Araw[1] = (float) ay;
  Araw[2] = (float) az;
  //vector_normalize(Araw);
}

float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

/*void AccelEarthFrame() { //(float ax, float ay, float az) {
  //Acceleration earth reference
  //Rotation matrix
  q0 = q[0];
  q1 = q[1];
  q2 = q[2];
  q3 = q[3];

  a11 = (q0 * q0) + (q1 * q1) - (q2 * q2) - (q3 * q3); //a11 = -2.0f * (q2 * q2 + q3 * q3) + 1.0f;
  a21 = 2.0f * (q1 * q2 + q0 * q3);
  a31 = 2.0f * (q1 * q3 - q0 * q2);

  a12 = 2.0f * (q1 * q2 - q0 * q3);
  a22 = (q0 * q0) - (q1 * q1) + (q2 * q2) - (q3 * q3); //a22 = -2.0f * (q1 * q1 + q3 * q3) + 1.0f;
  a32 = 2.0f * (q0 * q1 + q2 * q3);

  a13 = 2.0f * (q0 * q2 + q1 * q3);
  a23 = 2.0f * (q2 * q3 - q0 * q1);
  a33 = (q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3); //a11 = -2.0f * (q1 * q1 + q2 * q2) + 1.0f;

  //Matrix multiplication aR = Rxa
  aRx = (a11 * Axyz[0]) + (a12 * Axyz[1]) + (a13 * Axyz[2]);
  aRy = (a21 * Axyz[0]) + (a22 * Axyz[1]) + (a23 * Axyz[2]);
  aRz = (a31 * Axyz[0]) + (a32 * Axyz[1]) + (a33 * Axyz[2]);
  //aRz = (aRz*0.98)-0.98;

  //Serial.print("lin aRx, lin aRy, lin aRz: ");
  Serial.print(aRx, 2); Serial.print(","); Serial.print(aRy, 2); Serial.print(","); Serial.println(aRz, 2);
}*/

void linear_acceleration(float qw, float qx, float qy, float qz)
{
  lin_acc[0] = Axyz[0] + (2.0f * (qw * qx + qy * qz));
  lin_acc[1] = Axyz[1] + (2.0f * (qx * qz - qw * qy));
  lin_acc[2] = Axyz[2] - (qw * qw - qx * qx - qy * qy + qz * qz);
  lin_acc[3] = Axyz[0] + (qw * qx + qy * qz);
  lin_acc[4] = Axyz[1] + (qw * qy - qx * qz);
  lin_acc[5] = Axyz[2] - (qx * qx + qy * qy);
}

void print_gyrRawCal() {
  Serial.print(Graw[0]); Serial.print(", "); Serial.print(Graw[1]); Serial.print(", "); Serial.print(Graw[2]); //Serial.print(", ");
  //Serial.print(Gcal[0]); Serial.print(", "); Serial.print(Gcal[1]); Serial.print(", "); Serial.print(Gcal[2]);
}

void print_accRawCal() {
  Serial.print(Araw[0]); Serial.print(", "); Serial.print(Araw[1]); Serial.print(", "); Serial.print(Araw[2]); //Serial.print(", ");
  //Serial.print(Acal[0]); Serial.print(", "); Serial.print(Acal[1]); Serial.print(", "); Serial.print(Acal[2]);
}

void print_acc() {
  Serial.print(Axyz[0], 2); Serial.print(", "); Serial.print(Axyz[1], 2); Serial.print(", "); Serial.print(Axyz[2], 2);
}

void print_gyro() {
  Serial.print(Gxyz[0], 2); Serial.print(", "); Serial.print(Gxyz[1], 2); Serial.print(", "); Serial.print(Gxyz[2], 2);
}

void print_mag() {
  Serial.print(Mxyz[0], 2); Serial.print(", "); Serial.print(Mxyz[1], 2); Serial.print(", "); Serial.print(Mxyz[2], 2);
}

void print_q() {
  Serial.print(q[0]); Serial.print(", "); Serial.print(q[1]); Serial.print(", "); Serial.print(q[2]); Serial.print(", "); Serial.print(q[3]);
  //Serial.print(q0); Serial.print(", "); Serial.print(q1); Serial.print(", "); Serial.print(q2); Serial.print(", "); Serial.print(q3);
}

void print_linAcc() {
  Serial.print(lin_acc[0], 2); Serial.print(", "); Serial.print(lin_acc[1], 2); Serial.print(", "); Serial.print(lin_acc[2], 2);
}

void print_linAcc2() {
  Serial.print(lin_acc[3], 2); Serial.print(", "); Serial.print(lin_acc[4], 2); Serial.print(", "); Serial.print(lin_acc[5], 2);
}

void Logging() {
  sdCard = SD.open("DATA.txt", FILE_WRITE);
  if(sdCard) {
    sdCard.print(now_ms*0.001); sdCard.print(", ");
    sdCard.print(Axyz[0], 2); sdCard.print(", "); sdCard.print(Axyz[1], 2); sdCard.print(", "); sdCard.print(Axyz[2], 2); sdCard.print(", ");
    sdCard.print(Gxyz[0], 2); sdCard.print(", "); sdCard.print(Gxyz[1], 2); sdCard.print(", "); sdCard.print(Gxyz[2], 2); sdCard.print(", ");
    sdCard.print(Mxyz[0], 2); sdCard.print(", "); sdCard.print(Mxyz[1], 2); sdCard.print(", "); sdCard.print(Mxyz[2], 2); sdCard.print(", ");
    sdCard.print(q[0]); sdCard.print(", "); sdCard.print(q[1]); sdCard.print(", "); sdCard.print(q[2]); sdCard.print(", "); sdCard.println(q[3]);
  }
  else {
    Serial.println("can't open sd card");
  }
  sdCard.close();
}
