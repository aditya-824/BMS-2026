/*
 * LTC681x_HAL.h
 *
 *  Created on: Jul 22, 2025
 *      Author: awnin
 */

#ifndef INC_LTC681X_HAL_H_
#define INC_LTC681X_HAL_H_

#include <stdint.h>
#include <stdbool.h>
#include "LT_HAL_SPI.h"

#define MD_422HZ_1KHZ 0
#define MD_27KHZ_14KHZ 1
#define MD_7KHZ_3KHZ 2
#define MD_26HZ_2KHZ 3

#define ADC_OPT_ENABLED 1
#define ADC_OPT_DISABLED 0

#define CELL_CH_ALL 0
#define CELL_CH_1and7 1
#define CELL_CH_2and8 2
#define CELL_CH_3and9 3
#define CELL_CH_4and10 4
#define CELL_CH_5and11 5
#define CELL_CH_6and12 6

#define SELFTEST_1 1
#define SELFTEST_2 2

#define AUX_CH_ALL 0
#define AUX_CH_GPIO1 1
#define AUX_CH_GPIO2 2
#define AUX_CH_GPIO3 3
#define AUX_CH_GPIO4 4
#define AUX_CH_GPIO5 5
#define AUX_CH_VREF2 6

#define STAT_CH_ALL 0
#define STAT_CH_SOC 1
#define STAT_CH_ITEMP 2
#define STAT_CH_VREGA 3
#define STAT_CH_VREGD 4

#define REG_ALL 0
#define REG_1 1
#define REG_2 2
#define REG_3 3
#define REG_4 4
#define REG_5 5
#define REG_6 6

#define DCP_DISABLED 0
#define DCP_ENABLED 1

#define PULL_UP_CURRENT 1
#define PULL_DOWN_CURRENT 0

#define NUM_RX_BYT 8
#define CELL 1
#define AUX 2
#define STAT 3
#define CFGR 0
#define CFGRB 4
#define CS_PIN 4

/*! Cell Voltage data structure. */
typedef struct
{
  uint16_t c_codes[18]; //!< Cell Voltage Codes
  uint8_t pec_match[6]; //!< If a PEC error was detected during most recent read cmd
} cv;

/*! AUX Reg Voltage Data structure */
typedef struct
{
  uint16_t a_codes[9]; //!< Aux Voltage Codes
  uint8_t pec_match[4]; //!< If a PEC error was detected during most recent read cmd
} ax;

/*! Status Reg data structure. */
typedef struct
{
  uint16_t stat_codes[4]; //!< Status codes.
  uint8_t flags[3]; //!< Byte array that contains the uv/ov flag data
  uint8_t mux_fail[1]; //!< Mux self test status flag
  uint8_t thsd[1]; //!< Thermal shutdown status
  uint8_t pec_match[2]; //!< If a PEC error was detected during most recent read cmd
} st;

/*! IC register structure. */
typedef struct
{
  uint8_t tx_data[6];  //!< Stores data to be transmitted
  uint8_t rx_data[8];  //!< Stores received data
  uint8_t rx_pec_match; //!< If a PEC error was detected during most recent read cmd
} ic_register;

/*! PEC error counter structure. */
typedef struct
{
  uint16_t pec_count; //!< Overall PEC error count
  uint16_t cfgr_pec;  //!< Configuration register data PEC error count
  uint16_t cell_pec[6]; //!< Cell voltage register data PEC error count
  uint16_t aux_pec[4];  //!< Aux register data PEC error count
  uint16_t stat_pec[2]; //!< Status register data PEC error count
} pec_counter;

/*! Register configuration structure */
typedef struct
{
  uint8_t cell_channels; //!< Number of Cell channels
  uint8_t stat_channels; //!< Number of Stat channels
  uint8_t aux_channels;  //!< Number of Aux channels
  uint8_t num_cv_reg;    //!< Number of Cell voltage register
  uint8_t num_gpio_reg;  //!< Number of Aux register
  uint8_t num_stat_reg;  //!< Number of  Status register
} register_cfg;

/*! Cell variable structure */
typedef struct
{
  ic_register config;
  ic_register configb;
  cv  cells;
  ax  aux;
  st  stat;
  ic_register com;
  ic_register pwm;
  ic_register pwmb;
  ic_register sctrl;
  ic_register sctrlb;
  uint8_t sid[6];
  bool isospi_reverse;
  pec_counter crc_count;
  register_cfg ic_reg;
  long system_open_wire;
} cell_asic;

/* Wake isoSPI up from IDlE state and enters the READY state */
void wakeup_idle(uint8_t total_ic); //Number of ICs in the system


/* Generic wakeup command to wake the LTC681x from sleep state */
void wakeup_sleep(uint8_t total_ic); //Number of ICs in the system


/* Generic function to write 68xx commands. Function calculates PEC for tx_cmd data. */
void cmd_68(uint8_t tx_cmd[2]); //The command to be transmitted

/*
 Generic function to write 68xx commands and write payload data.
 Function calculates PEC for tx_cmd data and the data to be transmitted.
 */
void write_68(uint8_t total_ic, //Number of ICs to be written to
		uint8_t tx_cmd[2], //The command to be transmitted
		uint8_t data[] // Payload Data
		);

/* Generic function to write 68xx commands and read data. Function calculated PEC for tx_cmd data */
int8_t read_68(uint8_t total_ic, // Number of ICs in the system
		uint8_t tx_cmd[2], // The command to be transmitted
		uint8_t *rx_data // Data to be read
		);

/* Calculates  and returns the CRC15 */
uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
		uint8_t *data //Array of data that will be used to calculate  a PEC
		);

/* Write the LTC681x CFGRA */
void LTC681x_wrcfg(uint8_t total_ic, //The number of ICs being written to
		cell_asic ic[] // A two dimensional array of the configuration data that will be written
		);

/* Write the LTC681x CFGRB */
void LTC681x_wrcfgb(uint8_t total_ic, //The number of ICs being written to
		cell_asic ic[] // A two dimensional array of the configuration data that will be written
		);

/* Read the LTC681x CFGA */
int8_t LTC681x_rdcfg(uint8_t total_ic, //Number of ICs in the system
		cell_asic ic[] // A two dimensional array that the function stores the read configuration data.
		);

/* Reads the LTC681x CFGB */
int8_t LTC681x_rdcfgb(uint8_t total_ic, //Number of ICs in the system
		cell_asic ic[] // A two dimensional array that the function stores the read configuration data.
		);

/* Starts ADC conversion for cell voltage */
void LTC681x_adcv(uint8_t MD, //ADC Mode
		uint8_t DCP, //Discharge Permit
		uint8_t CH //Cell Channels to be measured
		);

/* Start ADC Conversion for GPIO and Vref2  */
void LTC681x_adax(uint8_t MD, //ADC Mode
		uint8_t CHG //GPIO Channels to be measured
		);

/* Start ADC Conversion for Status  */
void LTC681x_adstat(uint8_t MD, //ADC Mode
		uint8_t CHST //Stat Channels to be measured
		);

/* Starts cell voltage and SOC conversion */
void LTC681x_adcvsc(uint8_t MD, //ADC Mode
		uint8_t DCP //Discharge Permit
		);

/* Starts cell voltage and GPIO 1&2 conversion */
void LTC681x_adcvax(uint8_t MD, //ADC Mode
		uint8_t DCP //Discharge Permit
		);

/*
 Reads and parses the LTC681x cell voltage registers.
 The function is used to read the parsed Cell voltages codes of the LTC681x.
 This function will send the requested read commands parse the data
 and store the cell voltages in c_codes variable.
 */
uint8_t LTC681x_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
		uint8_t total_ic, // The number of ICs in the system
		cell_asic *ic // Array of the parsed cell codes
		);

/*
 The function is used to read the  parsed GPIO codes of the LTC681x.
 This function will send the requested read commands parse the data
 and store the gpio voltages in a_codes variable.
 */
int8_t LTC681x_rdaux(uint8_t reg, //Determines which GPIO voltage register is read back.
		uint8_t total_ic, //The number of ICs in the system
		cell_asic *ic //A two dimensional array of the gpio voltage codes.
		);

/*
 Reads and parses the LTC681x stat registers.
 The function is used to read the  parsed Stat codes of the LTC681x.
 This function will send the requested read commands parse the data
 and store the gpio voltages in stat_codes variable.
 */
int8_t LTC681x_rdstat(uint8_t reg, //Determines which Stat  register is read back.
		uint8_t total_ic, //The number of ICs in the system
		cell_asic *ic //A two dimensional array of the stat codes.
		);

/* Writes the command and reads the raw cell voltage register data */
void LTC681x_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
		uint8_t total_ic, //the number of ICs in the
		uint8_t *data //An array of the unparsed cell codes
		);

/*
 The function reads a single GPIO voltage register and stores the read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC681x_rdaux() command.
 */
void LTC681x_rdaux_reg(uint8_t reg, //Determines which GPIO voltage register is read back
		uint8_t total_ic, //The number of ICs in the system
		uint8_t *data //Array of the unparsed auxiliary codes
		);

/*
 The function reads a single stat  register and stores the read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC681x_rdstat() command.
 */
void LTC681x_rdstat_reg(uint8_t reg, //Determines which stat register is read back
		uint8_t total_ic, //The number of ICs in the system
		uint8_t *data //Array of the unparsed stat codes
		);

/* Helper function that parses voltage measurement registers */
int8_t parse_cells(uint8_t current_ic, // Current IC
		uint8_t cell_reg,  // Type of register
		uint8_t cell_data[], // Unparsed data
		uint16_t *cell_codes, // Parsed data
		uint8_t *ic_pec // PEC error
		);

/* Sends the poll ADC command */
uint8_t LTC681x_pladc();

/* This function will block operation until the ADC has finished it's conversion */
uint32_t LTC681x_pollAdc();

/*
 The command clears the cell voltage registers and initializes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.
 */
void LTC681x_clrcell();

/*
 The command clears the Auxiliary registers and initializes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.
 */
void LTC681x_clraux();

/*
 The command clears the Stat registers and initializes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.

 */
void LTC681x_clrstat();

/* Starts the Mux Decoder diagnostic self test */
void LTC681x_diagn();


/* Starts cell voltage self test conversion */
void LTC681x_cvst(uint8_t MD, //ADC Mode
		uint8_t ST //Self Test
		);

/* Start an Auxiliary Register Self Test Conversion */
void LTC681x_axst(uint8_t MD, //ADC Mode
		uint8_t ST //Self Test
		);

/* Start a Status Register Self Test Conversion */
void LTC681x_statst(uint8_t MD, //ADC Mode
		uint8_t ST //Self Test
		);

/* Starts cell voltage overlap conversion */
void LTC681x_adol(uint8_t MD, //ADC Mode
		uint8_t DCP //Discharge Permit
		);

/* Start an GPIO Redundancy test */
void LTC681x_adaxd(uint8_t MD, //ADC Mode
		uint8_t CHG //GPIO Channels to be measured
		);

/* Start a Status register redundancy test Conversion */
void LTC681x_adstatd(uint8_t MD, //ADC Mode
		uint8_t CHST //Stat Channels to be measured
		);

/* Runs the Digital Filter Self Test */
int16_t LTC681x_run_cell_adc_st(uint8_t adc_reg, // Type of register
		uint8_t total_ic, // Number of ICs in the daisy chain
		cell_asic *ic, // A two dimensional array that will store the data
		uint8_t md, // ADC Mode
		bool adcopt // ADCOPT bit in the configuration register
		);

/* Runs the ADC overlap test for the IC */
uint16_t LTC681x_run_adc_overlap(uint8_t total_ic, // Number of ICs in the daisy chain
		cell_asic *ic  // A two dimensional array that will store the data
		);

/* Runs the redundancy self test */
int16_t LTC681x_run_adc_redundancy_st(uint8_t adc_mode, // ADC Mode
		uint8_t adc_reg, // Type of register
		uint8_t total_ic, // Number of ICs in the daisy chain
		cell_asic *ic // A two dimensional array that will store the data
		);

/* Looks up the result pattern for digital filter self test */
uint16_t LTC681x_st_lookup(uint8_t MD, //ADC Mode
		uint8_t ST, //Self Test
		bool adcopt // ADCOPT bit in the configuration register
		);

/* Start an open wire Conversion */
void LTC681x_adow(uint8_t MD, //ADC Mode
		uint8_t PUP, //Pull up/Pull down current
		uint8_t CH, //Channels
		uint8_t DCP //Discharge Permit
		);

/* Start GPIOs open wire ADC conversion */
void LTC681x_axow(uint8_t MD, //ADC Mode
		uint8_t PUP //Pull up/Pull down current
		);

// REQUIRE SERIAL TO UART PRINT CONVERSION
///* Runs the data sheet algorithm for open wire for single cell detection */
//void LTC681x_run_openwire_single(uint8_t total_ic, // Number of ICs in the daisy chain
//		cell_asic ic[] // A two dimensional array that will store the data
//		);
//
///* Runs the data sheet algorithm for open wire for multiple cell and two consecutive cells detection */
//void LTC681x_run_openwire_multi(uint8_t total_ic, // Number of ICs in the daisy chain
//		cell_asic ic[] // A two dimensional array that will store the data
//		);
//
///* Runs open wire for GPIOs */
//void LTC681x_run_gpio_openwire(uint8_t total_ic, // Number of ICs in the daisy chain
//								cell_asic ic[] // A two dimensional array that will store the data
//								);


/* Clears all of the DCC bits in the configuration registers */
void LTC681x_clear_discharge(uint8_t total_ic, // Number of ICs in the daisy chain
							 cell_asic *ic // A two dimensional array that will store the data
							 );


/* Writes the pwm register */
void LTC681x_wrpwm(uint8_t total_ic, // Number of ICs in the daisy chain
                   uint8_t pwmReg, // The PWM Register to be written A or B
                   cell_asic ic[] // A two dimensional array that stores the data to be written
                  );


/* Reads pwm registers of a LTC681x daisy chain */
int8_t LTC681x_rdpwm(uint8_t total_ic, //Number of ICs in the system
                     uint8_t pwmReg, // The PWM Register to be written A or B
                     cell_asic ic[] // A two dimensional array that will store the data
                    );


/*  Write the LTC681x Sctrl register */
void LTC681x_wrsctrl(uint8_t total_ic, // Number of ICs in the daisy chain
                     uint8_t sctrl_reg, // The Sctrl Register to be written A or B
                     cell_asic *ic  // A two dimensional array that stores the data to be written
                    );


/*  Reads sctrl registers of a LTC681x daisy chain */
int8_t LTC681x_rdsctrl(uint8_t total_ic, // Number of ICs in the daisy chain
                       uint8_t sctrl_reg, // The Sctrl Register to be written A or B
                       cell_asic *ic // A two dimensional array that the function stores the read data
                      );

/*
Start Sctrl data communication
This command will start the sctrl pulse communication over the spins
*/
void LTC681x_stsctrl();


/*
The command clears the Sctrl registers and initializes
all values to 0. The register will read back hexadecimal 0x00
after the command is sent.
*/
void LTC681x_clrsctrl();


/* Writes the comm register */
void LTC681x_wrcomm(uint8_t total_ic, //The number of ICs being written to
                    cell_asic ic[] // A two dimensional array that stores the data to be written
                   );


/* Reads COMM registers of a LTC681x daisy chain */
int8_t LTC681x_rdcomm(uint8_t total_ic, //Number of ICs in the system
                      cell_asic ic[] //A two dimensional array that stores the read data
                     );


/* Shifts data in COMM register out over LTC681x SPI/I2C port */
void LTC681x_stcomm(uint8_t len); //Length of data to be transmitted


/* Helper function that increments PEC counters */
void LTC681x_check_pec(uint8_t total_ic, //Number of ICs in the system
					   uint8_t reg, //Type of Register
					   cell_asic *ic //A two dimensional array that stores the data
					   );


/* Helper Function to reset PEC counters */
void LTC681x_reset_crc_count(uint8_t total_ic, //Number of ICs in the system
							 cell_asic *ic //A two dimensional array that stores the data
							 );


/* Helper function to initialize CFG variables */
void LTC681x_init_cfg(uint8_t total_ic, //Number of ICs in the system
					  cell_asic *ic //A two dimensional array that stores the data
					  );


/* Helper function to set CFGR variable */
void LTC681x_set_cfgr(uint8_t nIC, // Current IC
					 cell_asic *ic, // A two dimensional array that stores the data
					 bool refon, // The REFON bit
					 bool adcopt, // The ADCOPT bit
					 bool gpio[5], // The GPIO bits
					 bool dcc[12], // The DCC bits
					 bool dcto[4], // The Dcto bits
					 uint16_t uv, // The UV value
					 uint16_t  ov // The OV value
					 );


/* Helper function to set the REFON bit */
void LTC681x_set_cfgr_refon(uint8_t nIC, cell_asic *ic, bool refon);


/* Helper function to set the ADCOPT bit */
void LTC681x_set_cfgr_adcopt(uint8_t nIC, cell_asic *ic, bool adcopt);


/* Helper function to set GPIO bits */
void LTC681x_set_cfgr_gpio(uint8_t nIC, cell_asic *ic,bool gpio[5]);


/* Helper function to control discharge */
void LTC681x_set_cfgr_dis(uint8_t nIC, cell_asic *ic,bool dcc[12]);


/* Helper function to control discharge time value */
void LTC681x_set_cfgr_dcto(uint8_t nIC, cell_asic *ic,bool dcto[4]);


/* Helper Function to set UV value in CFG register */
void LTC681x_set_cfgr_uv(uint8_t nIC, cell_asic *ic,uint16_t uv);


/* Helper function to set OV value in CFG register */
void LTC681x_set_cfgr_ov(uint8_t nIC, cell_asic *ic,uint16_t ov);

extern const uint16_t crc15Table[256]; // Variable declaration, defined & re-declared in 681x & 6813 files

#endif /* INC_LTC681X_HAL_H_ */
