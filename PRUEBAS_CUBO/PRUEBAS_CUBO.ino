#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>



// MAPA DEREGISTROS DEL MPU //

// ----- MPU-9250 addresses
#define MPU9250_ADDRESS     0x68  // Device address when ADO = 0; Use 0x69 when AD0 = 1
#define AK8963_ADDRESS      0x0C  //  Address of magnetometer

// ----- MPU-9250 register map
#define AK8963_WHO_AM_I     0x00  // should return 0x48
#define AK8963_INFO         0x01
#define AK8963_ST1          0x02  // data ready status bit 0
#define AK8963_XOUT_L       0x03  // data
#define AK8963_XOUT_H       0x04
#define AK8963_YOUT_L       0x05
#define AK8963_YOUT_H       0x06
#define AK8963_ZOUT_L       0x07
#define AK8963_ZOUT_H       0x08
#define AK8963_ST2          0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL         0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC         0x0C  // Self test control
#define AK8963_I2CDIS       0x0F  // I2C disable
#define AK8963_ASAX         0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY         0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ         0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO    0x00
#define SELF_TEST_Y_GYRO    0x01
#define SELF_TEST_Z_GYRO    0x02

/*
  #define X_FINE_GAIN         0x03   // [7:0] fine gain
  #define Y_FINE_GAIN         0x04
  #define Z_FINE_GAIN         0x05
  #define XA_OFFSET_H         0x06   // User-defined trim values for accelerometer
  #define XA_OFFSET_L_TC      0x07
  #define YA_OFFSET_H         0x08
  #define YA_OFFSET_L_TC      0x09
  #define ZA_OFFSET_H         0x0A
  #define ZA_OFFSET_L_TC      0x0B
*/

#define SELF_TEST_X_ACCEL   0x0D
#define SELF_TEST_Y_ACCEL   0x0E
#define SELF_TEST_Z_ACCEL   0x0F

#define SELF_TEST_A         0x10

#define XG_OFFSET_H         0x13   // User-defined trim values for gyroscope
#define XG_OFFSET_L         0x14
#define YG_OFFSET_H         0x15
#define YG_OFFSET_L         0x16
#define ZG_OFFSET_H         0x17
#define ZG_OFFSET_L         0x18
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG2       0x1D
#define LP_ACCEL_ODR        0x1E
#define WOM_THR             0x1F

#define MOT_DUR             0x20   // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR            0x21   // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR           0x22   // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define DMP_INT_STATUS      0x39  // Check DMP interrupt
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define MOT_DETECT_STATUS   0x61
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A   // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1          0x6B   // Device defaults to the SLEEP mode
#define PWR_MGMT_2          0x6C
#define DMP_BANK            0x6D   // Activates a specific bank in the DMP
#define DMP_RW_PNT          0x6E   // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG             0x6F   // Register in DMP from which to read or to which to write
#define DMP_REG_1           0x70
#define DMP_REG_2           0x71
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I_MPU9250    0x75   // Should return 0x71; MPU9255 will return 0x73
#define XA_OFFSET_H         0x77
#define XA_OFFSET_L         0x78
#define YA_OFFSET_H         0x7A
#define YA_OFFSET_L         0x7B
#define ZA_OFFSET_H         0x7D
#define ZA_OFFSET_L         0x7E



// ----- FIN DEL MAPA DE REGISTROS -------------------------------------------------


float magCalibration[3] = {0, 0, 0},
                          magBias[3] = {0, 0, 0},
                                       magScale[3] = {0, 0, 0};

                                       
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
}; 

enum Mscale {
  MFS_14BITS = 0,               // 0.6 mG per LSB;
  MFS_16BITS                    // 0.15 mG per LSB
};

enum M_MODE {
  M_8HZ = 0x02,                 // 8 Hz ODR (output data rate) update
  M_100HZ = 0x06                // 100 Hz continuous magnetometer
};
byte Gscale = GFS_250DPS;
byte Ascale = AFS_2G;
byte Mscale = MFS_14BITS;                           // Choose either 14-bit or 16-bit magnetometer resolution (AK8963=14-bits)
byte Mmode = 0x02; 

float pitch, roll, yaw;
float ax, ay, az, gx, gy, gz, mx, my, mz;  
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; 
short accelCount[3];                                // Stores the 16-bit signed accelerometer sensor output
short gyroCount[3];                                 // Stores the 16-bit signed gyro sensor output
short magCount[3]; 

float aRes= 2.0 / 32768.0, gRes = 250.0 / 32768.0, mRes= 10.*4912. / 8190.; 

float gyroBias[3] = {0, 0, 0},
                    accelBias[3] = {0, 0, 0};

#define Kp 40.0f                                    // Kp proportional feedback parameter in Mahony filter and fusion scheme
#define Ki 0.0f                                     // Ki integral parameter in Mahony filter and fusion scheme
float eInt[3] = {0.0f, 0.0f, 0.0f};                 // vector to hold integral error for Mahony method


unsigned long delt_t = 0;                           // used to control display output rate
unsigned long count = 0, sumCount = 0;              // used to control display output rate

float deltat, sum = 0.0f;                    // integration interval for both filter schemes

unsigned long lastUpdate = 0, firstUpdate = 0;      // used to calculate integration interval
unsigned long Now = 0;   
unsigned long tiempo1 = 0;
unsigned long tiempo2 = 0;
int TASK = 2;
#define True_North true          // change this to "true" for True North                
float Declination = -5.2;     // substitute your magnetic declination


// ----- NZ offsets & scale-factors (TASK 2 ... 2nd run)
float
Mag_x_offset = -178.05,
Mag_y_offset = -43.15,
Mag_z_offset = -593.08,
Mag_x_scale = 1.10,
Mag_y_scale = 0.84,
Mag_z_scale = 1.12;





void setup(){
       Serial.begin(115200);

       Wire.begin(4,5); // D2(GPIO4)=SDA / D1(GPIO5)=SCL
      byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
//      Serial.println();
//      Serial.print("[ IMUSYSTEM - SETUP MPU] Dirección IMU --->");
//      Serial.println(MPU9250_ADDRESS, HEX);
//      Serial.println("[ IMUSYSTEM - SETUP MPU ] Calibrando giroscopio y acelerómetro");
      calibrateMPU9250(gyroBias, accelBias);
//      Serial.println("[ IMUSYSTEM - SETUP MPU ] Calibrados!!");
      delay(1000);
//      Serial.println("[ IMUSYSTEM  - SETUP MPU] Inicializando MPU");
      initMPU9250();
//      Serial.println("[ IMUSYSTEM - SETUP MPU ] Inicializado");

      byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I); 
//      Serial.print("[ IMUSYSTEM - SETUP MPU] Dirección Magnetómetro --> ");
//      Serial.println(AK8963_ADDRESS,HEX);

      if (d == 0x48){
//        Serial.println("[ IMUSYSTEM - SETUP MPU] Inicializando magnetómetro");  
        initAK8963(magCalibration);
//        Serial.println("[ IMUSYSTEM - SETUP MPU] Magnetómetro inicializado");  
}

  if((TASK == 1)){
      Serial.print("Mueva el dispositivo en forma de 8");
     magCalMPU9250(magBias, magScale); 
     Serial.print(String(magBias[0]) +','+ String(magBias[1]) + ','+ String(magBias[2]) + ',' +String( magScale[0]) + ',' + String(magScale[1])+',' + String(magScale[2]));
  }

}

void loop(){
  if((TASK == 2)){
     refresh_data();
     calc_quaternion();
     view_heading_SM();
    // Serial.println( String(gx) +','+ String(gy) + ','+ String(gz) + ',' +String(ax) + ',' + String(ay)+',' + String(az)+ String(mx)+','+ String(my)+','+ String(mz));
  }
}


// ------------------------
// refresh_data()
// ------------------------
/* Get current MPU-9250 register values */
void refresh_data()
{
  // ----- If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    readAccelData(accelCount);                          // Read the accelerometer registers
                                             // Get accelerometer resolution
    // ----- Accelerometer calculations
    ax = (float)accelCount[0] * aRes;                   // - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes;                   // - accelBias[1];
    az = (float)accelCount[2] * aRes;                   // - accelBias[2];

    // ----- Gyro calculations
    readGyroData(gyroCount);                            // Read the gyro registers
                                             // Get gyro resolution

    // ----- Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    // ----- Magnetometer calculations
    readMagData(magCount);                              // Read the magnetometer x|y| registers
    //getMres();                                          // Get magnetometer resolution

    //    // ----- Kris Winer hard-iron offsets
    //    magBias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    //    magBias[1] = +120.;  // User environmental x-axis correction in milliGauss
    //    magBias[2] = +125.;  // User environmental x-axis correction in milliGauss

    // ----- Copy hard-iron offsets
    magBias[0] = Mag_x_offset;                          // Get hard-iron offsets (from compass_cal)
    magBias[1] = Mag_y_offset;
    magBias[2] = Mag_z_offset;

    // ----- Copy the soft-iron scalefactors
    magScale[0] = Mag_x_scale;
    magScale[1] = Mag_y_scale;
    magScale[2] = Mag_z_scale;

    // ----- Calculate the magnetometer values in milliGauss
    /* The above formula is not using the soft-iron scale factors */
//    mx = ((float)magCount[0] * mRes * magCalibration[0] - magBias[0]) * magScale[0];    // (rawMagX*ASAX*0.6 - magOffsetX)*scalefactor
//    my = ((float)magCount[1] * mRes * magCalibration[1] - magBias[1]) * magScale[1];    // (rawMagY*ASAY*0.6 - magOffsetY)*scalefactor
//    mz = ((float)magCount[2] * mRes * magCalibration[2] - magBias[2]) * magScale[2];    // (rawMagZ*ASAZ*0.6 - magOffsetZ)*scalefactor
//  
      mx = ((float)magCount[0] * mRes * magCalibration[0]);
      my = ((float)magCount[1] * mRes * magCalibration[1]);
      mz = ((float)magCount[2] * mRes * magCalibration[2]);
  }
}

// -------------------
// readAccelData()
// -------------------
/* Read accelerometer registers */
void readAccelData(short * destination)
{
  byte rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((short)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((short)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((short)rawData[4] << 8) | rawData[5] ;
}

// -------------------
// readGyroData()
// -------------------
/* Read gyro registers */
void readGyroData(short * destination)
{
  byte rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((short)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((short)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((short)rawData[4] << 8) | rawData[5] ;
}


// -------------------
// readMagData()
// -------------------
/* Read magnetometer registers */
void readMagData(short * destination)
{
  byte rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    byte c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((short)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((short)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((short)rawData[5] << 8) | rawData[4] ;
    }
  }
}

// -----------------
// magCalMPU9250()
// -----------------
/*
  Function which accumulates magnetometer data after device initialization.
  It calculates the bias and scale in the x, y, and z axes.
*/
void magCalMPU9250(float * bias_dest, float * scale_dest)
{
  unsigned short ii = 0, sample_count = 0;
  short mag_max[3]  = { -32768, -32768, -32768},
                      mag_min[3]  = {32767, 32767, 32767},
                                    mag_temp[3] = {0, 0, 0};
  long mag_bias[3] = {0, 0, 0};
  float mag_chord[3] = {0, 0, 0};
  float avg_chord;


  // ----- Tumble compass for 30 seconds
  /*
    At 8 Hz ODR (output data rate), new mag data is available every 125 ms
    At 100 Hz ODR, new mag data is available every 10 ms
  */

  if (Mmode == M_8HZ) sample_count = 240;         // 240*125mS=30 seconds
  if (Mmode == M_100HZ) sample_count = 3000;      // 3000*10mS=30 seconds

  for (ii = 0; ii < sample_count; ii++)
  {
    readMagData(mag_temp);  // Read the raw mag data

    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }

    if (Mmode == M_8HZ) delay(135);               // At 8 Hz ODR, new mag data is available every 125 ms
    if (Mmode == M_100HZ) delay(12);              // At 100 Hz ODR, new mag data is available every 10 ms
  }

  // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  // ----- Get hard iron correction
  /* long data-type  */
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;                       // data-type: long
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

  // ----- Save mag biases in G for main program
  /* float data-type  */
  bias_dest[0] = (float)mag_bias[0] * magCalibration[0] * mRes;       // rawMagX * ASAX * 0.6
  bias_dest[1] = (float)mag_bias[1] * magCalibration[1] * mRes;       // rawMagY * ASAY * 0.6
  bias_dest[2] = (float)mag_bias[2] * magCalibration[2] * mRes;       // rawMagZ * ASAZ * 0.6

  // ----- Get soft iron correction estimate
  /* float data-type */
  mag_chord[0]  = ((float)(mag_max[0] - mag_min[0])) / 2.0;
  mag_chord[1]  = ((float)(mag_max[1] - mag_min[1])) / 2.0;
  mag_chord[2]  = ((float)(mag_max[2] - mag_min[2])) / 2.0;
  avg_chord = (mag_chord[0] + mag_chord[1] + mag_chord[2]) / 3.0;

  // ----- calculate scale-factors
  /* Destination data-type is float */
  scale_dest[0] = avg_chord / mag_chord[0];
  scale_dest[1] = avg_chord / mag_chord[1];
  scale_dest[2] = avg_chord / mag_chord[2];

}




// -------------------
// initAK8963()
// -------------------
/* Initialize the AK8963 magnetometer */
void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  byte rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

// -------------------
// initMPU9250()
// -------------------
/* Initialize the MPU9250|MPU6050 chipset */
void initMPU9250()
{
  // -----wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // -----get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // ----- Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // -----Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  byte c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x03; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

  // ----- Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

  // ----- Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // ----- Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}



void readBytes(byte address, byte subAddress, byte count, byte * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  byte i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}

byte readByte(byte address, byte subAddress)
{
  byte data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (byte) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

// --------------
// calibrateMPU9250()
// --------------

void calibrateMPU9250(float * dest1, float * dest2)
{
  byte data[12]; // data array to hold accelerometer and gyro x, y, z, data
  unsigned short ii, packet_count, fifo_count;
  long gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // ----- reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // ----- get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // ----- Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // ----- Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  unsigned short  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  unsigned short  accelsensitivity = 16384;  // = 16384 LSB/g

  // ----- Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // ----- At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((unsigned short)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    short accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (short) (((short)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (short) (((short)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (short) (((short)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (short) (((short)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (short) (((short)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (short) (((short)data[10] << 8) | data[11]) ;

    accel_bias[0] += (long) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (long) accel_temp[1];
    accel_bias[2] += (long) accel_temp[2];
    gyro_bias[0]  += (long) gyro_temp[0];
    gyro_bias[1]  += (long) gyro_temp[1];
    gyro_bias[2]  += (long) gyro_temp[2];

  }
  accel_bias[0] /= (long) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (long) packet_count;
  accel_bias[2] /= (long) packet_count;
  gyro_bias[0]  /= (long) packet_count;
  gyro_bias[1]  /= (long) packet_count;
  gyro_bias[2]  /= (long) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (long) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (long) accelsensitivity;
  }

  // ----- Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // ----- Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // ----- Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  long accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (long) (((short)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (long) (((short)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (long) (((short)data[0] << 8) | data[1]);

  unsigned long mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  byte mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // ----- Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8);     // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  //    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  //    data[1] = (accel_bias_reg[0])      & 0xFF;
  //    data[1] = data[1] | mask_bit[0];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  //    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  //    data[3] = (accel_bias_reg[1])      & 0xFF;
  //    data[3] = data[3] | mask_bit[1];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  //    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  //    data[5] = (accel_bias_reg[2])      & 0xFF;
  //    data[5] = data[5] | mask_bit[2];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  //    //   Apparently this is not working for the acceleration biases in the MPU-9250
  //    //   Are we handling the temperature correction bit properly?

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFE;
  data[1] = data[1] | mask_bit[0];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFE;
  data[3] = data[3] | mask_bit[1];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFE;
  data[5] = data[5] | mask_bit[2];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  // see https://github.com/kriswiner/MPU9250/issues/215

  // Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // ----- Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}



// --------------
// writeByte()
// --------------
void writeByte(byte address, byte subAddress, byte data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}


////////////// OBTENCIÓN DE ANGULOS //////////////////////////////////////////////////////////////////

// --------------------------
// MahonyQuaternionUpdate()
// --------------------------
/*
  Similar to Madgwick scheme but uses proportional and integral filtering
  on the error between estimated reference vectors and measured ones.
*/
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // ----- Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // ----- Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // ----- Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // ----- Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // ----- Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  //  ----- Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // ----- Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // ----- Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // ----- Normalise quaternion
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}


// ------------------------
// calc_quaternion()
// ------------------------
/* Send current MPU-9250 register values to Mahony quaternion filter */
void calc_quaternion()
{
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  

  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);

  // ----- Apply NEU (north east up)signs when parsing values
  MahonyQuaternionUpdate(
    ax,                -ay,                  az,
    gx * DEG_TO_RAD,   -gy * DEG_TO_RAD,     gz * DEG_TO_RAD,
    my,                -mx,                 -mz);

Serial.println(deltat,4);    
}




// ------------------------
// view_heading_SM()
// ------------------------
/* View heading on serila monitor */
void view_heading_SM(){

  pitch = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = -atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);

  // ----- convert to degrees
  pitch *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;
  yaw *= RAD_TO_DEG;

  // ----- calculate the heading
  /*
     The yaw and compass heading (after the next two lines) track each other 100%
  */
  float heading = yaw;
  if (heading < 0) heading += 360.0;                        // Yaw goes negative between 180 amd 360 degrees
  if (True_North == true) heading += Declination;           // Calculate True North
  if (heading < 0) heading += 360.0;                        // Allow for under|overflow
  if (heading >= 360) heading -= 360.0;

  // ----- send the results to the Serial Monitor
//  Serial.print("        Pitch ");
//  print_number((short)pitch);
//  Serial.print("        Roll ");
//  print_number((short)roll);
//  Serial.print("        Heading ");
//  print_number((short)heading);
//
//  Serial.println("");

// DATA QUE SE ENVIA A PROCESSING

//  Serial.println(int(pitch));
//  Serial.print("*");
//  Serial.print(int(roll));
//  Serial.print("@");
//  Serial.print(int(heading));
//  Serial.print(".");
  count = millis();

  sumCount = 0;
  sum = 0;
}


// ------------------------
// print_number()
// ------------------------
/* Overloaded routine to stop integer numbers jumping around */
long print_number(short number) {
  String myString = String(number);
  short numberChars = myString.length();
  for (short i = 0; i < 6 - numberChars; i++) {
    Serial.print(" ");
  }
  Serial.print(myString);
}

// ------------------------
// print_number()
// ------------------------
/* Overloaded routine to stop float numbers jumping around */
float print_number(float number) {
  String myString = String(number);
  short numberChars = myString.length();
  for (short i = 0; i < 6 - numberChars; i++) {
    Serial.print(" ");
  }
  Serial.print(myString);
}
