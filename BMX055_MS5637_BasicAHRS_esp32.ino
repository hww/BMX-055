/*
 Forked from:
 BMX055_MS5637_ts Basic Example Code
 by: Kris Winer
 date: August 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

 Modifyed to:
 BMX055_MS5637_esp32 Basic Example Code
 by: Valeriya P. (hww)
 date: May 26, 2020

 Demonstrate basic BMX-055 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms. Sketch runs on ESP32-WROOM module.

 SDA and SCL should have external pull-up resistors 1K to 3.3V.

 Hardware setup:
 BMX055 Mini Add-On ------- ESP32-WROOM
 VDD ---------------------- 3.3V
 SDA ----------------------- 19
 SCL ----------------------- 18
 GND ---------------------- GND
 */

#include "Wire.h"
#include <SPI.h>

#define SerialBaudRate 115200
#define SerialDebug false  // set to true to get serial output friendly for human
#define SerialStream true  // set to true to get output friendly to serial plotter
#define SerialLogI2C true  // set true will log writing to sensor at the boot 

// Set the I2C pins
#define ESP32_SDA 19
#define ESP32_SCL 18

// =============================================================================
// Using the Teensy Mini Add-On board, SDO1 = SDO2 = CSB3 = GND as designed
// Seven-bit device addresses are ACC = 0x18, GYRO = 0x68, MAG = 0x10
// =============================================================================

#define BMX055_ACC_ADDRESS  0x19   // Address of BMX055 accelerometer
#define BMX055_GYRO_ADDRESS 0x69   // Address of BMX055 gyroscope
#define BMX055_MAG_ADDRESS  0x11   // Address of BMX055 magnetometer
#define MS5637_ADDRESS      0x76   // Address of altimeter

// =============================================================================
// The BMX055 is a conglomeration of three separate motion sensors packaged
// together but addressed and communicated with separately by design
// =============================================================================
// Accelerometer registers
#define BMX055_ACC_WHOAMI        0x00   // should return 0xFA
#define BMX055_ACC_D_X_LSB       0x02
#define BMX055_ACC_D_X_MSB       0x03
#define BMX055_ACC_D_Y_LSB       0x04
#define BMX055_ACC_D_Y_MSB       0x05
#define BMX055_ACC_D_Z_LSB       0x06
#define BMX055_ACC_D_Z_MSB       0x07
#define BMX055_ACC_D_TEMP        0x08
#define BMX055_ACC_INT_STATUS_0  0x09
#define BMX055_ACC_INT_STATUS_1  0x0A
#define BMX055_ACC_INT_STATUS_2  0x0B
#define BMX055_ACC_INT_STATUS_3  0x0C
#define BMX055_ACC_FIFO_STATUS   0x0E
#define BMX055_ACC_PMU_RANGE     0x0F
#define BMX055_ACC_PMU_BW        0x10
#define BMX055_ACC_PMU_LPW       0x11
#define BMX055_ACC_PMU_LOW_POWER 0x12
#define BMX055_ACC_D_HBW         0x13
#define BMX055_ACC_BGW_SOFTRESET 0x14
#define BMX055_ACC_INT_EN_0      0x16
#define BMX055_ACC_INT_EN_1      0x17
#define BMX055_ACC_INT_EN_2      0x18
#define BMX055_ACC_INT_MAP_0     0x19
#define BMX055_ACC_INT_MAP_1     0x1A
#define BMX055_ACC_INT_MAP_2     0x1B
#define BMX055_ACC_INT_SRC       0x1E
#define BMX055_ACC_INT_OUT_CTRL  0x20
#define BMX055_ACC_INT_RST_LATCH 0x21
#define BMX055_ACC_INT_0         0x22
#define BMX055_ACC_INT_1         0x23
#define BMX055_ACC_INT_2         0x24
#define BMX055_ACC_INT_3         0x25
#define BMX055_ACC_INT_4         0x26
#define BMX055_ACC_INT_5         0x27
#define BMX055_ACC_INT_6         0x28
#define BMX055_ACC_INT_7         0x29
#define BMX055_ACC_INT_8         0x2A
#define BMX055_ACC_INT_9         0x2B
#define BMX055_ACC_INT_A         0x2C
#define BMX055_ACC_INT_B         0x2D
#define BMX055_ACC_INT_C         0x2E
#define BMX055_ACC_INT_D         0x2F
#define BMX055_ACC_FIFO_CONFIG_0 0x30
#define BMX055_ACC_PMU_SELF_TEST 0x32
#define BMX055_ACC_TRIM_NVM_CTRL 0x33
#define BMX055_ACC_BGW_SPI3_WDT  0x34
#define BMX055_ACC_OFC_CTRL      0x36
#define BMX055_ACC_OFC_SETTING   0x37
#define BMX055_ACC_OFC_OFFSET_X  0x38
#define BMX055_ACC_OFC_OFFSET_Y  0x39
#define BMX055_ACC_OFC_OFFSET_Z  0x3A
#define BMX055_ACC_TRIM_GPO      0x3B
#define BMX055_ACC_TRIM_GP1      0x3C
#define BMX055_ACC_FIFO_CONFIG_1 0x3E
#define BMX055_ACC_FIFO_DATA     0x3F

// BMX055 Gyroscope Registers
#define BMX055_GYRO_WHOAMI        0x00  // should return 0x0F
#define BMX055_GYRO_RATE_X_LSB    0x02
#define BMX055_GYRO_RATE_X_MSB    0x03
#define BMX055_GYRO_RATE_Y_LSB    0x04
#define BMX055_GYRO_RATE_Y_MSB    0x05
#define BMX055_GYRO_RATE_Z_LSB    0x06
#define BMX055_GYRO_RATE_Z_MSB    0x07
#define BMX055_GYRO_INT_STATUS_0  0x09
#define BMX055_GYRO_INT_STATUS_1  0x0A
#define BMX055_GYRO_INT_STATUS_2  0x0B
#define BMX055_GYRO_INT_STATUS_3  0x0C
#define BMX055_GYRO_FIFO_STATUS   0x0E
#define BMX055_GYRO_RANGE         0x0F
#define BMX055_GYRO_BW            0x10
#define BMX055_GYRO_LPM1          0x11
#define BMX055_GYRO_LPM2          0x12
#define BMX055_GYRO_RATE_HBW      0x13
#define BMX055_GYRO_BGW_SOFTRESET 0x14
#define BMX055_GYRO_INT_EN_0      0x15
#define BMX055_GYRO_INT_EN_1      0x16
#define BMX055_GYRO_INT_MAP_0     0x17
#define BMX055_GYRO_INT_MAP_1     0x18
#define BMX055_GYRO_INT_MAP_2     0x19
#define BMX055_GYRO_INT_SRC_1     0x1A
#define BMX055_GYRO_INT_SRC_2     0x1B
#define BMX055_GYRO_INT_SRC_3     0x1C
#define BMX055_GYRO_FIFO_EN       0x1E
#define BMX055_GYRO_INT_RST_LATCH 0x21
#define BMX055_GYRO_HIGH_TH_X     0x22
#define BMX055_GYRO_HIGH_DUR_X    0x23
#define BMX055_GYRO_HIGH_TH_Y     0x24
#define BMX055_GYRO_HIGH_DUR_Y    0x25
#define BMX055_GYRO_HIGH_TH_Z     0x26
#define BMX055_GYRO_HIGH_DUR_Z    0x27
#define BMX055_GYRO_SOC           0x31
#define BMX055_GYRO_A_FOC         0x32
#define BMX055_GYRO_TRIM_NVM_CTRL 0x33
#define BMX055_GYRO_BGW_SPI3_WDT  0x34
#define BMX055_GYRO_OFC1          0x36
#define BMX055_GYRO_OFC2          0x37
#define BMX055_GYRO_OFC3          0x38
#define BMX055_GYRO_OFC4          0x39
#define BMX055_GYRO_TRIM_GP0      0x3A
#define BMX055_GYRO_TRIM_GP1      0x3B
#define BMX055_GYRO_BIST          0x3C
#define BMX055_GYRO_FIFO_CONFIG_0 0x3D
#define BMX055_GYRO_FIFO_CONFIG_1 0x3E

// BMX055 magnetometer registers
#define BMX055_MAG_WHOAMI         0x40  // should return 0x32
#define BMX055_MAG_Reserved       0x41
#define BMX055_MAG_XOUT_LSB       0x42
#define BMX055_MAG_XOUT_MSB       0x43
#define BMX055_MAG_YOUT_LSB       0x44
#define BMX055_MAG_YOUT_MSB       0x45
#define BMX055_MAG_ZOUT_LSB       0x46
#define BMX055_MAG_ZOUT_MSB       0x47
#define BMX055_MAG_ROUT_LSB       0x48
#define BMX055_MAG_ROUT_MSB       0x49
#define BMX055_MAG_INT_STATUS     0x4A
#define BMX055_MAG_PWR_CNTL1      0x4B
#define BMX055_MAG_PWR_CNTL2      0x4C
#define BMX055_MAG_INT_EN_1       0x4D
#define BMX055_MAG_INT_EN_2       0x4E
#define BMX055_MAG_LOW_THS        0x4F
#define BMX055_MAG_HIGH_THS       0x50
#define BMX055_MAG_REP_XY         0x51
#define BMX055_MAG_REP_Z          0x52

// Trim Extended Registers
#define BMM050_DIG_X1             0x5D // needed for magnetic field calculation
#define BMM050_DIG_Y1             0x5E
#define BMM050_DIG_Z4_LSB         0x62
#define BMM050_DIG_Z4_MSB         0x63
#define BMM050_DIG_X2             0x64
#define BMM050_DIG_Y2             0x65
#define BMM050_DIG_Z2_LSB         0x68
#define BMM050_DIG_Z2_MSB         0x69
#define BMM050_DIG_Z1_LSB         0x6A
#define BMM050_DIG_Z1_MSB         0x6B
#define BMM050_DIG_XYZ1_LSB       0x6C
#define BMM050_DIG_XYZ1_MSB       0x6D
#define BMM050_DIG_Z3_LSB         0x6E
#define BMM050_DIG_Z3_MSB         0x6F
#define BMM050_DIG_XY2            0x70
#define BMM050_DIG_XY1            0x71

// Set initial input parameters
// define X055 ACC full scale options
#define AFS_2G  0x03
#define AFS_4G  0x05
#define AFS_8G  0x08
#define AFS_16G 0x0C

enum ACCBW {    // define BMX055 accelerometer bandwidths
    ABW_8Hz,      // 7.81 Hz,  64 ms update time
    ABW_16Hz,     // 15.63 Hz, 32 ms update time
    ABW_31Hz,     // 31.25 Hz, 16 ms update time
    ABW_63Hz,     // 62.5  Hz,  8 ms update time
    ABW_125Hz,    // 125   Hz,  4 ms update time
    ABW_250Hz,    // 250   Hz,  2 ms update time
    ABW_500Hz,    // 500   Hz,  1 ms update time
    ABW_100Hz     // 1000  Hz,  0.5 ms update time
};

enum Gscale {
    GFS_2000DPS = 0,
    GFS_1000DPS,
    GFS_500DPS,
    GFS_250DPS,
    GFS_125DPS
};

enum GODRBW {
    G_2000Hz523Hz = 0, // 2000 Hz ODR and unfiltered (bandwidth 523Hz)
    G_2000Hz230Hz,
    G_1000Hz116Hz,
    G_400Hz47Hz,
    G_200Hz23Hz,
    G_100Hz12Hz,
    G_200Hz64Hz,
    G_100Hz32Hz  // 100 Hz ODR and 32 Hz bandwidth
};

enum MODR {
    MODR_10Hz = 0,   // 10 Hz ODR
    MODR_2Hz,        // 2 Hz ODR
    MODR_6Hz,        // 6 Hz ODR
    MODR_8Hz,        // 8 Hz ODR
    MODR_15Hz,       // 15 Hz ODR
    MODR_20Hz,       // 20 Hz ODR
    MODR_25Hz,       // 25 Hz ODR
    MODR_30Hz        // 30 Hz ODR
};

enum Mmode {
    lowPower         = 0,   // rms noise ~1.0 microTesla, 0.17 mA power
    Regular,                // rms noise ~0.6 microTesla, 0.5 mA power
    enhancedRegular,        // rms noise ~0.5 microTesla, 0.8 mA power
    highAccuracy            // rms noise ~0.3 microTesla, 4.9 mA power
};

// Specify sensor full scale
uint8_t Gscale = GFS_125DPS;       // set gyro full scale
uint8_t GODRBW = G_200Hz23Hz;      // set gyro ODR and bandwidth
uint8_t Ascale = AFS_2G;           // set accel full scale
uint8_t ACCBW  = 0x08 | ABW_16Hz;  // Choose bandwidth for accelerometer, need bit 3 = 1 to enable bandwidth choice in enum
uint8_t Mmode  = Regular;          // Choose magnetometer operation mode
uint8_t MODR   = MODR_10Hz;        // set magnetometer data rate
float aRes, gRes, mRes;            // scale resolutions per LSB for the sensors

// Parameters to hold BMX055 trim values
signed char   dig_x1;
signed char   dig_y1;
signed char   dig_x2;
signed char   dig_y2;
uint16_t      dig_z1;
int16_t       dig_z2;
int16_t       dig_z3;
int16_t       dig_z4;
unsigned char dig_xy1;
signed char   dig_xy2;
uint16_t      dig_xyz1;

// Pin definitions
// The BMX055 has three sensors, and two interrupt pins per device!
int intACC1   =  8;  // These are fixed on the BMX055 Mini Add-On for Teensy 3.1
int intACC2   =  9;
int intGYRO1  = 11;
int intGYRO2  = 10;
int intMAG1   = 12;
int intDRDYM  = 15;
int myLed     = 13;  // LED on the Teensy 3.1


// BMX055 variables
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 13/15-bit signed magnetometer sensor output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the BMX055 internal chip temperature in degrees Celsius
float SelfTest[6];            // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
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
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

bool isBoot;

// =============================================================================
// Initialization
// =============================================================================

void setup() 
{
    isBoot = true;
    Wire.begin(ESP32_SDA, ESP32_SCL);
    delay(5000);
    Serial.begin(SerialBaudRate);
    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intACC1,  INPUT);
    pinMode(intACC2,  INPUT);
    pinMode(intGYRO1, INPUT);
    pinMode(intGYRO2, INPUT);
    pinMode(intMAG1,  INPUT);
    pinMode(intDRDYM, INPUT);
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);
    // Read the BMX-055 WHO_AM_I registers, this is a good test of communication
    Serial.println("BMX055 accelerometer...");
    byte c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_WHOAMI);  // Read ACC WHO_AM_I register for BMX055
    Serial.print("BMX055 ACC");
    Serial.print(" I AM 0x");
    Serial.print(c, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0xFA, HEX);
    Serial.println("BMX055 gyroscope...");
    byte d = readByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_WHOAMI);  // Read GYRO WHO_AM_I register for BMX055
    Serial.print("BMX055 GYRO");
    Serial.print(" I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x0F, HEX);
    delay(1000);
    Serial.println("BMX055 magnetometer...");
    writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // wake up magnetometer first thing
    delay(100);
    byte e = readByte(BMX055_MAG_ADDRESS, BMX055_MAG_WHOAMI);  // Read MAG WHO_AM_I register for BMX055
    Serial.print("BMX055 MAG");
    Serial.print(" I AM 0x");
    Serial.print(e, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x32, HEX);
    delay(1000);

    if ((c == 0xFA) && (d == 0x0F) && (e == 0x32)) { // WHO_AM_I should always be ACC = 0xFA, GYRO = 0x0F, MAG = 0x32
        Serial.println("BMX055 is online...");
        initBMX055();
        Serial.println("BMX055 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        delay(1000);
        // get sensor resolutions, only need to do this once
        getAres();
        getGres();
        // magnetometer resolution is 1 microTesla/16 counts or 1/1.6 milliGauss/count
        mRes = 1. / 1.6;
        trimBMX055();  // read the magnetometer calibration data
        delay(1000);
        fastcompaccelBMX055(accelBias);
        Serial.println("accel biases (mg)");
        Serial.println(1000.*accelBias[0]);
        Serial.println(1000.*accelBias[1]);
        Serial.println(1000.*accelBias[2]);
        Serial.println("gyro biases (dps)");
        Serial.println(gyroBias[0]);
        Serial.println(gyroBias[1]);
        Serial.println(gyroBias[2]);
        magcalBMX055(magBias);
        Serial.println("mag biases (mG)");
        Serial.println(magBias[0]);
        Serial.println(magBias[1]);
        Serial.println(magBias[2]);
        isBoot = false;
    } else {
        Serial.print("Could not connect to BMX055: 0x");
        Serial.println(c, HEX);

        while (1) ; // Loop forever if communication doesn't happen
    }
}

void loop() {
    // If intPin goes high, all data registers have new data
//  if (digitalRead(intACC2)) {  // On interrupt, read data
    readAccelData(accelCount);  // Read the x/y/z adc values
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes; // + accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes; // + accelBias[1];
    az = (float)accelCount[2] * aRes; // + accelBias[2];
// }
//  if (digitalRead(intGYRO2)) {  // On interrupt, read data
    readGyroData(gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;
// }
// if (digitalRead(intDRDYM)) {  // On interrupt, read data
    readMagData(magCount);  // Read the x/y/z adc values
    // Calculate the magnetometer values in milliGauss
    // Temperature-compensated magnetic field is in 16 LSB/microTesla
    mx = (float)magCount[0] * mRes - magBias[0]; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes - magBias[1];
    mz = (float)magCount[2] * mRes - magBias[2];
//}
    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    sum += deltat; // sum for averaging filter update rate
    sumCount++;
    // Sensors x (y)-axis of the accelerometer is aligned with the -y (x)-axis of the magnetometer;
    // the magnetometer z-axis (+ up) is aligned with z-axis (+ up) of accelerometer and gyro!
    // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
    // For the BMX-055, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    // in the MPU9250 sensor. This rotation can be modified to allow any convenient orientation convention.
    // This is ok by aircraft orientation standards!
    // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,  -my,  mx, mz);
//  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -my, mx, mz);
    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;

    if (delt_t > 100) { // update LCD once per half-second independent of read rate
        if (SerialDebug) {
            Serial.print("ax = ");
            Serial.print((int)1000 * ax);
            Serial.print(" ay = ");
            Serial.print((int)1000 * ay);
            Serial.print(" az = ");
            Serial.print((int)1000 * az);
            Serial.println(" mg");
            Serial.print("gx = ");
            Serial.print( gx, 2);
            Serial.print(" gy = ");
            Serial.print( gy, 2);
            Serial.print(" gz = ");
            Serial.print( gz, 2);
            Serial.println(" deg/s");
            Serial.print("mx = ");
            Serial.print( (int)mx);
            Serial.print(" my = ");
            Serial.print( (int)my);
            Serial.print(" mz = ");
            Serial.print( (int)mz);
            Serial.println(" mG");
            Serial.print("q0 = ");
            Serial.print(q[0]);
            Serial.print(" qx = ");
            Serial.print(q[1]);
            Serial.print(" qy = ");
            Serial.print(q[2]);
            Serial.print(" qz = ");
            Serial.println(q[3]);
        }

        // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
        // In this coordinate system, the positive z-axis is down toward Earth.
        // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
        // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
        // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
        // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
        // applied in the correct order which for this configuration is yaw, pitch, and then roll.
        // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI;
        yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        roll  *= 180.0f / PI;

        // Or define output variable according to the Android system, where heading (0 to 260) is defined by the angle between the y-axis
        // and True North, pitch is rotation about the x-axis (-180 to +180), and roll is rotation about the y-axis (-90 to +90)
        // In this systen, the z-axis is pointing away from Earth, the +y-axis is at the "top" of the device (cellphone) and the +x-axis
        // points toward the right of the device.
        //

        if (SerialDebug) {
            Serial.print("Yaw, Pitch, Roll: ");
            Serial.print(yaw, 2);
            Serial.print(", ");
            Serial.print(pitch, 2);
            Serial.print(", ");
            Serial.println(roll, 2);
            Serial.print("rate = ");
            Serial.print((float)sumCount / sum, 2);
            Serial.println(" Hz");
        }
        if (SerialStream)
        {
            Serial.print(yaw, 2);
            Serial.print(",   ");
            Serial.print(pitch, 2);
            Serial.print(",   ");
            Serial.print(roll, 2);
            Serial.print(",   ");
            Serial.println((float)sumCount / sum, 2);
        }

        tempCount = readACCTempData();  // Read the gyro adc values
        temperature = ((float) tempCount) / 2.0 + 23.0; // Gyro chip temperature in degrees Centigrade
        // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
        // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
        // The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
        // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
        // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
        // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
        // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
        // This filter update rate should be fast enough to maintain accurate platform orientation for
        // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
        // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
        // The 3.3 V 8 MHz Pro Mini is doing pretty well!
        digitalWrite(myLed, !digitalRead(myLed));
        count = millis();
        sumCount = 0;
        sum = 0;
    }
}

// =============================================================================
// Set of useful function to access acceleration. gyroscope, magnetometer
// =============================================================================

/**
 * Possible gyro scales (and their register bit settings) are:
 * 125 DPS (100), 250 DPS (011), 500 DPS (010), 1000 DPS (001), and 2000 DPS (000).
 */
void getGres() {
    switch (Gscale) {
    case GFS_125DPS:
        gRes = 124.87 / 32768.0; // per data sheet, not exactly 125!?
        break;

    case GFS_250DPS:
        gRes = 249.75 / 32768.0;
        break;

    case GFS_500DPS:
        gRes = 499.5 / 32768.0;
        break;

    case GFS_1000DPS:
        gRes = 999.0 / 32768.0;
        break;

    case GFS_2000DPS:
        gRes = 1998.0 / 32768.0;
        break;
    }
}
/**
 * Possible accelerometer scales (and their register bit settings) are:
 * 2 Gs (0011), 4 Gs (0101), 8 Gs (1000), and 16 Gs  (1100).
 * BMX055 ACC data is signed 12 bit
 */
void getAres()
{
    switch (Ascale)
    {
    case AFS_2G:
        aRes = 2.0 / 2048.0;
        break;

    case AFS_4G:
        aRes = 4.0 / 2048.0;
        break;

    case AFS_8G:
        aRes = 8.0 / 2048.0;
        break;

    case AFS_16G:
        aRes = 16.0 / 2048.0;
        break;
    }
}

// =============================================================================
// Read data from sensor
// =============================================================================

void readAccelData(int16_t* destination) {
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(BMX055_ACC_ADDRESS, BMX055_ACC_D_X_LSB, 6, &rawData[0]);       // Read the six raw data registers into data array

    if ((rawData[0] & 0x01) && (rawData[2] & 0x01) && (rawData[4] & 0x01)) { // Check that all 3 axes have new data
        destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 4;  // Turn the MSB and LSB into a signed 12-bit value
        destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 4;
        destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 4;
    }
}

void readGyroData(int16_t* destination) {
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(BMX055_GYRO_ADDRESS, BMX055_GYRO_RATE_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]);
}

void readMagData(int16_t* magData) {
    int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
    uint16_t data_r = 0;
    uint8_t rawData[8];  // x/y/z hall magnetic field data, and Hall resistance data
    readBytes(BMX055_MAG_ADDRESS, BMX055_MAG_XOUT_LSB, 8, &rawData[0]);  // Read the eight raw data registers sequentially into data array

    if (rawData[6] & 0x01) { // Check if data ready status bit is set
        mdata_x = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 3;  // 13-bit signed integer for x-axis field
        mdata_y = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 3;  // 13-bit signed integer for y-axis field
        mdata_z = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 1;  // 15-bit signed integer for z-axis field
        data_r = (uint16_t) (((uint16_t)rawData[7] << 8) | rawData[6]) >> 2;  // 14-bit unsigned integer for Hall resistance
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // calculate temperature compensated 16-bit magnetic fields
        // TODO move to function
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14) / (data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
        magData[0] = ((int16_t)((((int32_t)mdata_x) *
                                 ((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
                                      (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
                                    ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_x2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
                     (((int16_t)dig_x1) << 3);
        temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14) / (data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
        magData[1] = ((int16_t)((((int32_t)mdata_y) *
                                 ((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
                                      (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
                                    ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_y2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
                     (((int16_t)dig_y1) << 3);
        magData[2] = (((((int32_t)(mdata_z - dig_z4)) << 15) - ((((int32_t)dig_z3) * ((int32_t)(((int16_t)data_r) -
                       ((int16_t)dig_xyz1)))) >> 2)) / (dig_z2 + ((int16_t)(((((int32_t)dig_z1) * ((((int16_t)data_r) << 1))) + (1 << 15)) >> 16))));
    }
}
/**
 * Read temperature from accelerometer
 */
int16_t readACCTempData() {
    uint8_t c =  readByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_TEMP);  // Read the raw data register
    return ((int16_t)((int16_t)c << 8)) >> 8 ;  // Turn the byte into a signed 8-bit integer
}

/**
 * get trim values for magnetometer sensitivity
 */
void trimBMX055()
{
    uint8_t rawData[2];  //placeholder for 2-byte trim data
    dig_x1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_X1);
    dig_x2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_X2);
    dig_y1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y1);
    dig_y2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y2);
    dig_xy1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY1);
    dig_xy2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY2);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z1_LSB, 2, &rawData[0]);
    dig_z1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z2_LSB, 2, &rawData[0]);
    dig_z2 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z3_LSB, 2, &rawData[0]);
    dig_z3 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z4_LSB, 2, &rawData[0]);
    dig_z4 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_XYZ1_LSB, 2, &rawData[0]);
    dig_xyz1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);
}

/**
 * Initialize BMX055
 */
void initBMX055()
{
    // start with all sensors in default mode with all registers reset
    writeByte(BMX055_ACC_ADDRESS,  BMX055_ACC_BGW_SOFTRESET, 0xB6);  // reset accelerometer
    delay(1000); // Wait for all registers to reset
    // ~~~~~~~~~~~~~~~~~~~~~~~
    // Configure accelerometer
    // ~~~~~~~~~~~~~~~~~~~~~~~
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_RANGE, Ascale & 0x0F); // Set accelerometer full range
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_BW, ACCBW & 0x0F);     // Set accelerometer bandwidth
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_HBW, 0x00);              // Use filtered data
    // ~~~~~~~~~~~~~~~~~~~~~~~
    // Configure Gyro
    // ~~~~~~~~~~~~~~~~~~~~~~~
    writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_RANGE, Gscale);  // set GYRO FS range
    writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BW, GODRBW);     // set GYRO ODR and Bandwidth
    // ~~~~~~~~~~~~~~~~~~~~~~~
    // Configure magnetometer
    // ~~~~~~~~~~~~~~~~~~~~~~~
    writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x82);  // Softreset magnetometer, ends up in sleep mode
    delay(100);
    writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // Wake up magnetometer
    delay(100);
    writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MODR << 3); // Normal mode

    // Set up four standard configurations for the magnetometer
    switch (Mmode) {
    case lowPower:
        // Low-power
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x01);  // 3 repetitions (oversampling)
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x02);  // 3 repetitions (oversampling)
        break;

    case Regular:
        // Regular
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x04);  //  9 repetitions (oversampling)
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x16);  // 15 repetitions (oversampling)
        break;

    case enhancedRegular:
        // Enhanced Regular
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x07);  // 15 repetitions (oversampling)
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x22);  // 27 repetitions (oversampling)
        break;

    case highAccuracy:
        // High Accuracy
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x17);  // 47 repetitions (oversampling)
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x51);  // 83 repetitions (oversampling)
        break;
    }
}

/**
 * Use sensor's feature "Fast Calibration" to calibrate accelerometer
 */
void fastcompaccelBMX055(float* dest1)
{
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x80); // set all accel offset compensation registers to zero
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_SETTING, 0x20);  // set offset targets to 0, 0, and +1 g for x, y, z axes
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x20); // calculate x-axis offset
    byte c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);

    while (!(c & 0x10)) {  // check if fast calibration complete
        c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
        delay(10);
    }

    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x40); // calculate y-axis offset
    c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);

    while (!(c & 0x10)) {  // check if fast calibration complete
        c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
        delay(10);
    }

    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x60); // calculate z-axis offset
    c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);

    while (!(c & 0x10)) {  // check if fast calibration complete
        c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
        delay(10);
    }

    int8_t compx = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_X);
    int8_t compy = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Y);
    int8_t compz = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Z);
    dest1[0] = (float) compx / 128.; // accleration bias in g
    dest1[1] = (float) compy / 128.; // accleration bias in g
    dest1[2] = (float) compz / 128.; // accleration bias in g
}

#ifdef COMPLEX_CALIBRATION
/**
 * UNUSED!
 * Function which accumulates gyro and accelerometer data after device
 * initialization. It calculates the average of the at-rest readings and then
 * loads the resulting offsets into accelerometer and gyro bias registers.
 */
void accelgyrocalBMX055(float * dest1, float * dest2)
{
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
    uint16_t samples, ii;

    Serial.println("Calibrating gyro...");

    // First get gyro bias
    byte c = readByte(BMX055G_ADDRESS, BMX055G_CTRL_REG5_G);
    writeByte(BMX055G_ADDRESS, BMX055G_CTRL_REG5_G, c | 0x40);     // Enable gyro FIFO
    delay(200);                                                       // Wait for change to take effect
    writeByte(BMX055G_ADDRESS, BMX055G_FIFO_CTRL_REG_G, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
    delay(1000);  // delay 1000 milliseconds to collect FIFO samples

    samples = (readByte(BMX055G_ADDRESS, BMX055G_FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples
    // Read the gyro data stored in the FIFO
    for(ii = 0; ii < samples ; ii++)
    {
        int16_t gyro_temp[3] = {0, 0, 0};
        readBytes(BMX055G_ADDRESS, BMX055G_OUT_X_L_G, 6, &data[0]);
        gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
        gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
        gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

        gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        gyro_bias[1] += (int32_t) gyro_temp[1];
        gyro_bias[2] += (int32_t) gyro_temp[2];
    }

    gyro_bias[0] /= samples; // average the data
    gyro_bias[1] /= samples;
    gyro_bias[2] /= samples;

    dest1[0] = (float)gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
    dest1[1] = (float)gyro_bias[1]*gRes;
    dest1[2] = (float)gyro_bias[2]*gRes;

    c = readByte(BMX055G_ADDRESS, BMX055G_CTRL_REG5_G);
    writeByte(BMX055G_ADDRESS, BMX055G_CTRL_REG5_G, c & ~0x40);   //Disable gyro FIFO
    delay(200);
    writeByte(BMX055G_ADDRESS, BMX055G_FIFO_CTRL_REG_G, 0x00);  // Enable gyro bypass mode

    Serial.println("Calibrating accel...");

    // now get the accelerometer bias
    c = readByte(BMX055XM_ADDRESS, BMX055XM_CTRL_REG0_XM);
    writeByte(BMX055XM_ADDRESS, BMX055XM_CTRL_REG0_XM, c | 0x40);     // Enable gyro FIFO
    delay(200);                                                       // Wait for change to take effect
    writeByte(BMX055XM_ADDRESS, BMX055XM_FIFO_CTRL_REG, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
    delay(1000);  // delay 1000 milliseconds to collect FIFO samples

    samples = (readByte(BMX055XM_ADDRESS, BMX055XM_FIFO_SRC_REG) & 0x1F); // Read number of stored samples

    for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
        int16_t accel_temp[3] = {0, 0, 0};
        readBytes(BMX055XM_ADDRESS, BMX055XM_OUT_X_L_A, 6, &data[0]);
        accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
        accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
    }

    accel_bias[0] /= samples; // average the data
    accel_bias[1] /= samples;
    accel_bias[2] /= samples;

    if(accel_bias[2] > 0L)
    {
        // Remove gravity from the z-axis accelerometer bias calculation
        accel_bias[2] -= (int32_t) (1.0/aRes);
    }
    else
    {
        accel_bias[2] += (int32_t) (1.0/aRes);
    }
    // Properly scale the data to get g
    dest2[0] = (float)accel_bias[0]*aRes;
    dest2[1] = (float)accel_bias[1]*aRes;
    dest2[2] = (float)accel_bias[2]*aRes;

    c = readByte(BMX055XM_ADDRESS, BMX055XM_CTRL_REG0_XM);
    writeByte(BMX055XM_ADDRESS, BMX055XM_CTRL_REG0_XM, c & ~0x40);   //Disable accel FIFO
    delay(200);
    writeByte(BMX055XM_ADDRESS, BMX055XM_FIFO_CTRL_REG, 0x00);  // Enable accel bypass mode
}
#endif


 /**
  * Calibrate magnetometer by multiple reading with computing min,max vector's
  * values and then use the center of this volume as a bias. But size as the
  * scale for each axis.
  */
void magcalBMX055(float* dest1) {
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0};
    int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};
    Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    delay(4000);
    sample_count = 128;

    for (ii = 0; ii < sample_count; ii++) {
        int16_t mag_temp[3] = {0, 0, 0};
        readMagData(mag_temp);

        for (int jj = 0; jj < 3; jj++) {
            if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];

            if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }

        delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
    }
    /*
    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);
    */
    mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts
    dest1[0] = (float) mag_bias[0] * mRes; // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1] * mRes;
    dest1[2] = (float) mag_bias[2] * mRes;
    /*
     // Write biases to accelerometermagnetometer offset registers as counts);
     writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
     writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
     writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
     writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
     writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
     writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);
    */
    Serial.println("Mag Calibration done!");
}

// =============================================================================
// I2C read/write functions
// =============================================================================

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    if (SerialLogI2C && isBoot)
    {
        Serial.printf("[I2C] addr: %02x reg: %02x data: %02x\n",address, subAddress, data);
    }
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
    // Put read results in the Rx buffer
    while (Wire.available()) {
        dest[i++] = Wire.read();
    }
}
