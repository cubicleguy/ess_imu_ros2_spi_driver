//==============================================================================
//
// 	sensor_epsonCommon.h - Epson IMU sensor specific definitions common
//                      for all IMU models
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  NONINFRINGEMENT, SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A
//  PARTICULAR PURPOSE. IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE
//  OR CLAIM, ARISING FROM OR IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE
//  SOFTWARE.
//
//==============================================================================
#ifndef EPSONCOMMON_H_
#define EPSONCOMMON_H_

#include <math.h>
#include <stdio.h>

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#ifdef G354
#include "sensor_epsonG354.h"
#elif G364PDC0
#include "sensor_epsonG364PDC0.h"
#elif G364PDCA
#include "sensor_epsonG364PDCA.h"
#elif G320
#include "sensor_epsonG320.h"
#elif G365PDC0
#include "sensor_epsonG365PDC0.h"
#elif G365PDF0
#include "sensor_epsonG365PDF0.h"
#elif G370PDF0
#include "sensor_epsonG370PDF0.h"
#elif G325PDF0
#include "sensor_epsonG325PDF0.h"
#elif G365PDC1
#include "sensor_epsonG365PDC1.h"
#elif G365PDF1
#include "sensor_epsonG365PDF1.h"
#elif G370PDF1
#include "sensor_epsonG370PDF1.h"
#elif G325PDF1
#include "sensor_epsonG325PDF1.h"
#else /* V340 */
#include "sensor_epsonV340.h"
#endif

#define DELAY_EPSON_RESET 10        // Milliseconds Reset Pulse Width
#define EPSON_POWER_ON_DELAY 800    // Milliseconds
#define EPSON_FLASH_TEST_DELAY 5    // Milliseconds
#define EPSON_SELF_TEST_DELAY 80    // Milliseconds
#define EPSON_FILTER_DELAY 1        // Milliseconds
#define EPSON_ATTI_PROFILE_DELAY 1  // Milliseconds

// Required delay between bus cycles for serial timings
#define EpsonStall() seDelayMicroSecs(EPSON_STALL)

struct EpsonOptions {
  // MSC_CTRL
  int ext_sel;
  int ext_pol;
  int drdy_on;
  int drdy_pol;

  // SMPL_CTRL
  int dout_rate;

  // FILTER_CTRL
  int filter_sel;

  // BURST_CTRL1
  int flag_out;
  int temp_out;
  int gyro_out;
  int accel_out;
  int gyro_delta_out;
  int accel_delta_out;
  int qtn_out;
  int atti_out;

  int gpio_out;
  int count_out;
  int checksum_out;

  // BURST_CTRL2
  int temp_bit;
  int gyro_bit;
  int accel_bit;
  int gyro_delta_bit;
  int accel_delta_bit;
  int qtn_bit;
  int atti_bit;

  // POL_CTRL
  int invert_xgyro;
  int invert_ygyro;
  int invert_zgyro;
  int invert_xaccel;
  int invert_yaccel;
  int invert_zaccel;

  // DLT_CTRL
  int dlt_ovf_en;
  int dlt_range_ctrl;

  // ATTI_CTRL
  int atti_mode;
  int atti_conv;

  // ATTI_MOTION_PROFILE
  int atti_profile;
};

struct EpsonData {
  unsigned short ndflags;
  float temperature;
  float gyro_x, gyro_y, gyro_z;
  float accel_x, accel_y, accel_z;
  float gyro_delta_x, gyro_delta_y, gyro_delta_z;
  float accel_delta_x, accel_delta_y, accel_delta_z;
  float qtn0, qtn1, qtn2, qtn3;
  float roll, pitch, yaw;
  unsigned short gpio;
  int count;
};

int sensorDataReady(void);
int sensorHWReset(void);
int sensorPowerOn(void);
void registerDump(void);
void registerWriteByte(unsigned char, unsigned char, unsigned char,
                       unsigned int);
unsigned short registerRead16(unsigned char, unsigned char, unsigned int);
void registerWriteByteNoId(unsigned char, unsigned char, unsigned int);
unsigned short registerRead16NoId(unsigned char, unsigned int);
void sensorStart(void);
void sensorStop(void);
void sensorReset(void);
int sensorFlashTest(void);
int sensorSelfTest(void);
int sensorInitialBackup(void);
int sensorInitOptions(struct EpsonOptions);
unsigned int sensorDataByteLength(struct EpsonOptions);
void sensorDummyWrite(void);
int sensorDataReadBurstNOptions(struct EpsonOptions, struct EpsonData*);
void populateEpsonData(struct EpsonOptions, struct EpsonData*);

#endif /* EPSONCOMMON_H_ */
