//==============================================================================
//
// 	main_csvlogger.c - Epson IMU sensor test application
//                   - This program initializes the Epson IMU and
//                     sends sensor output to CSV file
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_spi.h"
#include "main_helper.h"
#include "sensor_epsonCommon.h"

// Specify the number of samples to readout before exiting the program
const unsigned int NUM_SAMPLES = 1000;

int main(int argc, char *argv[]) {
  unsigned int sample = 0;
  char prod_id[9];  // Device Product ID
  char ser_id[9];   // Device Serial ID

  // Specify IMU options
  struct EpsonOptions options = {
      .ext_sel = 1,  // 0 = Sample Counter 1=Reset Counter 2=External Trigger
      .ext_pol = 0,
      .drdy_on = 1,
      .drdy_pol = 1,
      .dout_rate = CMD_RATE125,
      .filter_sel = CMD_FLTAP32,
      .flag_out = 1,
      .temp_out = 1,
      .gyro_out = 1,
      .accel_out = 1,
      .gyro_delta_out = 0,
      .accel_delta_out = 0,
      .qtn_out = 0,   // Only valid for G365, G330, G366 otherwise ignored
      .atti_out = 0,  // Only valid for G365, G330, G366 otherwise ignored
      .gpio_out = 0,
      .count_out = 1,
      .checksum_out = 1,

      // Set 0=16bit 1=32bit sensor output
      .temp_bit = 0,
      .gyro_bit = 0,
      .accel_bit = 0,
      .gyro_delta_bit = 0,
      .accel_delta_bit = 0,
      .qtn_bit = 0,
      .atti_bit = 0,

      // Set 0=normal 1=reverse polarity
      .invert_xgyro = 0,
      .invert_ygyro = 0,
      .invert_zgyro = 0,
      .invert_xaccel = 0,
      .invert_yaccel = 0,
      .invert_zaccel = 0,

      .dlt_ovf_en = 0,
      .dlt_range_ctrl = 8,

      // NOTE:  // Only valid for G330, G365, G366 otherwise ignored
      .atti_mode = 1,    // 0=Inclination mode 1=Euler mode
      .atti_conv = 0,    // Attitude Conversion Mode
      .atti_profile = 0  // Attitude Motion Profile 0=modeA 1=modeB 2=modeC
  };

  // Stores the post-processed sensor data
  struct EpsonData epson_data;

  // 1) Initialize the Seiko Epson HCL layer
  printf("\r\nInitializing HCL layer...");
  if (!seInit()) {
    printf(
        "\r\nError: could not initialize the Seiko Epson HCL layer. "
        "Exiting...\r\n");
    return -1;
  }
  printf("...done.\r\n");

  // 2) Initialize the GPIO interfaces, For GPIO control of pins SPI CS, RESET,
  // DRDY
  printf("\r\nInitializing GPIO interface...");
  if (!gpioInit()) {
    printf("\r\nError: could not initialize the GPIO layer. Exiting...\r\n");
    seRelease();
    return -1;
  }
  printf("...done.\r\n");

  // 3) Initialize SPI Interface
  printf("\r\nInitializing SPI interface...");
  // The max SPI clock rate is 1MHz for current model Epson IMUs
  if (!spiInit(SPI_MODE3, 500000)) {
    printf("\r\nError: could not initialize SPI interface. Exiting...\r\n");
    gpioRelease();
    seRelease();
    return -1;
  }
  sensorDummyWrite();
  printf("...done.\r\n");

  // 4) Power on sequence - force sensor to config mode, HW reset sensor
  //      Check for errors
  printf("\r\nChecking sensor NOT_READY status...");
  if (!sensorPowerOn()) {
    printf("\r\nError: failed to power on Sensor. Exiting...\r\n");
    spiRelease();
    gpioRelease();
    seRelease();
    return -1;
  }
  printf("...done.\r\n");

  // Print out which model executable was compiled and identify model
  printf("\r\nCompiled for:\t" BUILD_FOR);
  printf("\r\nReading device info...");
  if (strcmp(BUILD_FOR, getProductId(prod_id)) != 0) {
    printf("\r\n*** Build *mismatch* with detected device ***");
    printf(
        "\r\n*** Ensure you specify a compatible 'MODEL=' variable when "
        "running make when  rebuilding the driver ***\r\n");
  }
  printf("\r\nPRODUCT ID:\t%s", prod_id);
  printf("\r\nSERIAL ID:\t%s", getSerialId(ser_id));

  // Initialize sensor with desired settings
  printf("\r\nInitializing Sensor...");
  if (!sensorInitOptions(options)) {
    printf("\r\nError: could not initialize Epson Sensor. Exiting...\r\n");
    spiRelease();
    gpioRelease();
    seRelease();
    return -1;
  }
  printf("...Epson IMU initialized.\r\n");

  // Initialize text files for data logs
  const time_t date =
      time(NULL);  // Functions for obtaining and printing time and date
  struct tm tm = *localtime(&date);
  char EpsonlogName[128];

  // Create Epson IMU Data Log
  sprintf(EpsonlogName, "EpsonLog_%4d-%02d-%02d_T%02d-%02d-%02d.csv",
          tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
          tm.tm_sec);
  FILE *EpsonLog = fopen(EpsonlogName, "w");
  fprintf(EpsonLog, "#PRODUCT_ID: %s", prod_id);
  fprintf(EpsonLog, "\r\n#SERIAL_ID: %s", ser_id);
  fprintf(EpsonLog, "\r\n#Date: %s", ctime(&date));
  printf("\r\n...Epson IMU Logging.\r\n");
  sensorStart();
  printHeaderRow(EpsonLog, options);
  while (1) {
    // For SPI interface, check if DRDY pin asserted
    // For UART interface, check if UART recv buffer contains a sensor sample
    // packet
    if (sensorDataReady()) {
      if (sensorDataReadBurstNOptions(options, &epson_data) == OK) {
        printSensorRow(EpsonLog, options, &epson_data, sample);
        sample++;
      }
    }
    if (sample > (NUM_SAMPLES - 1)) break;
  }

  const time_t end =
      time(NULL);  // Functions for obtaining and printing time and data
  fprintf(EpsonLog, "\r\n#End: ");
  fprintf(EpsonLog, "%s", ctime(&end));

  sensorStop();
  seDelayMS(1000);
  spiRelease();
  gpioRelease();
  seRelease();
  fclose(EpsonLog);
  printf("\r\n");
  return 0;
}
