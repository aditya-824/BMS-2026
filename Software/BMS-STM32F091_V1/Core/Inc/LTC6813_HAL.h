/*
 * LTC6813_HAL.h
 *
 *  Created on: Jul 24, 2025
 *      Author: awnin
 */

#ifndef INC_LTC6813_HAL_H_
#define INC_LTC6813_HAL_H_

#include "LTC681x_HAL.h"

/* Helper function to initialize register limits. */
void LTC6813_init_reg_limits(uint8_t total_ic, //Number of ICs in the system
							 cell_asic *ic // A two dimensional array that will store the data
							 );

 /*
This command will write the configuration registers of the LTC6813-1s
connected in a daisy chain stack. The configuration is written in descending
order so the last device's configuration is written first.
*/
void LTC6813_wrcfg(uint8_t total_ic, //The number of ICs being written to
                     cell_asic *ic //A two dimensional array of the configuration data that will be written
                    );

/*
This command will write the configuration b registers of the LTC6813-1s
connected in a daisy chain stack. The configuration is written in descending
order so the last device's configuration is written first.
*/
void LTC6813_wrcfgb(uint8_t total_ic, //The number of ICs being written to
                    cell_asic *ic //A two dimensional array of the configuration data that will be written
                   );

/* Reads configuration registers of a LTC6813 daisy chain */
int8_t LTC6813_rdcfg(uint8_t total_ic, //Number of ICs in the system
				   cell_asic *ic //A two dimensional array that the function stores the read configuration data.
				  );

/* Reads configuration b registers of a LTC6813 daisy chain */
int8_t LTC6813_rdcfgb(uint8_t total_ic, //Number of ICs in the system
                   cell_asic *ic //A two dimensional array that the function stores the read configuration data.
                  );

/* Starts cell voltage conversion */
void LTC6813_adcv(uint8_t MD, //ADC Mode
				  uint8_t DCP, //Discharge Permit
				  uint8_t CH //Cell Channels to be measured
				 );

/* Start a GPIO and Vref2 Conversion */
void LTC6813_adax(uint8_t MD, //ADC Mode
				  uint8_t CHG //GPIO Channels to be measured)
                 );

/* Start a Status ADC Conversion */
void LTC6813_adstat(uint8_t MD, //ADC Mode
					uint8_t CHST //Stat Channels to be measured
);

/* Starts cell voltage and GPIO 1&2 conversion */
void LTC6813_adcvax(uint8_t MD, //ADC Mode
					uint8_t DCP //Discharge Permit
					);

/* Starts cell voltage and SOC conversion */
void LTC6813_adcvsc( uint8_t MD, //ADC Mode
                     uint8_t DCP //Discharge Permit
                   );

/*  Reads and parses the LTC6813 cell voltage registers */
uint8_t LTC6813_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
                     uint8_t total_ic, // The number of ICs in the system
                     cell_asic *ic // Array of the parsed cell codes
                    );

/*
The function is used
to read the  parsed GPIO codes of the LTC6813. This function will send the requested
read commands parse the data and store the gpio voltages in aux_codes variable
*/
int8_t LTC6813_rdaux(uint8_t reg, //Determines which GPIO voltage register is read back.
				     uint8_t total_ic,//The number of ICs in the system
				     cell_asic *ic//A two dimensional array of the gpio voltage codes.
				    );

/*
Reads and parses the LTC6813 stat registers.
The function is used
to read the  parsed stat codes of the LTC6813. This function will send the requested
read commands parse the data and store the stat voltages in stat_codes variable
*/
int8_t LTC6813_rdstat(uint8_t reg, //Determines which Stat  register is read back.
                      uint8_t total_ic,//The number of ICs in the system
                      cell_asic *ic //A two dimensional array of the stat codes.
                       );

/* Sends the poll ADC command */
uint8_t LTC6813_pladc();

/* This function will block operation until the ADC has finished it's conversion */
uint32_t LTC6813_pollAdc();

/*
The command clears the cell voltage registers and initializes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.
*/
void LTC6813_clrcell();

/*
The command clears the Auxiliary registers and initializes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.
*/
void LTC6813_clraux();

/*
The command clears the Stat registers and initializes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.
*/
void LTC6813_clrstat();

/* Starts the Mux Decoder diagnostic self test */
void LTC6813_diagn();

/* Starts cell voltage self test conversion */
void LTC6813_cvst(uint8_t MD, //ADC Mode
                  uint8_t ST //Self Test
                 );

/* Start an Auxiliary Register Self Test Conversion */
void LTC6813_axst(uint8_t MD, //ADC Mode
				  uint8_t ST //Self Test
                 );

/* Start a Status Register Self Test Conversion */
void LTC6813_statst(uint8_t MD, //ADC Mode
                    uint8_t ST //Self Test
					);

/* Starts cell voltage overlap conversion */
void LTC6813_adol(uint8_t MD, //ADC Mode
                  uint8_t DCP //Discharge Permit
                 );

/* Start an GPIO Redundancy test */
void LTC6813_adaxd(uint8_t MD, //ADC Mode
				   uint8_t CHG //GPIO Channels to be measured
                   );

/* Start a Status register redundancy test Conversion */
void LTC6813_adstatd(uint8_t MD, //ADC Mode
					 uint8_t CHST //Stat Channels to be measured
                    );

/* Runs the Digital Filter Self Test */
int16_t LTC6813_run_cell_adc_st(uint8_t adc_reg, // Type of register
								uint8_t total_ic, // Number of ICs in the system
								cell_asic *ic, // A two dimensional array that will store the data
								uint8_t md, //ADC Mode
								bool adcopt // The adcopt bit in the configuration register
								);

/*  Runs the ADC overlap test for the IC */
uint16_t LTC6813_run_adc_overlap(uint8_t total_ic, // Number of ICs in the system
								cell_asic *ic // A two dimensional array that will store the data
								);

/* Runs the redundancy self test */
int16_t LTC6813_run_adc_redundancy_st(uint8_t adc_mode, //ADC Mode
									  uint8_t adc_reg, // Type of register
									  uint8_t total_ic, // Number of ICs in the system
									  cell_asic *ic // A two dimensional array that will store the data
									  );

/* Start an open wire Conversion */
void LTC6813_adow(uint8_t MD,   //ADC Mode
                  uint8_t PUP, //Pull up/Pull down current
				  uint8_t CH,  //Sets which Cell channels are converted
				  uint8_t DCP //Discharge Permit
				  );

/* Start GPIOs open wire ADC conversion */
void LTC6813_axow(uint8_t MD, //ADC Mode
				  uint8_t PUP //Pull up/Pull down current
				 );

///* Runs the data sheet algorithm for open wire for single cell detection */
//void LTC6813_run_openwire_single(uint8_t total_ic, // Number of ICs in the system
//								cell_asic *ic // A two dimensional array that will store the data
//								);
//
///* Runs the data sheet algorithm for open wire for multiple cell and two consecutive cells detection */
//void LTC6813_run_openwire_multi(uint8_t total_ic, // Number of ICs in the system
//								cell_asic *ic // A two dimensional array that will store the data
//								);
//
///* Runs open wire for GPIOs */
//void LTC6813_run_gpio_openwire(uint8_t total_ic, // Number of ICs in the system
//								cell_asic *ic // A two dimensional array that will store the data
//								);

/* Helper function to set discharge bit in CFG register */
void LTC6813_set_discharge(int Cell, // Cell to be discharged
						   uint8_t total_ic, // Number of ICs in the system
						   cell_asic *ic // A two dimensional array that will store the data
						   );

/* Clears all of the DCC bits in the configuration registers */
void LTC6813_clear_discharge(uint8_t total_ic, // Number of ICs in the system
                             cell_asic *ic // A two dimensional array that will store the data
							 );

/* Writes the pwm registers of a LTC6813 daisy chain  */
void LTC6813_wrpwm(uint8_t total_ic, // Number of ICs in the system
				   uint8_t pwmReg,  // PWM Register A or B
				   cell_asic *ic //A two dimensional array of the configuration data that will be written
				  );

/* Reads pwm registers of a LTC6813 daisy chain */
int8_t LTC6813_rdpwm(uint8_t total_ic, //Number of ICs in the system
					 uint8_t pwmReg, // PWM Register A or B
				     cell_asic *ic //A two dimensional array that the function stores the read configuration data.
				    );

/* Writes data in S control register the ltc6813-1  connected in a daisy chain stack */
void LTC6813_wrsctrl(uint8_t total_ic, // number of ICs in the daisy chain
                     uint8_t sctrl_reg, // SCTRL Register A or B
                     cell_asic *ic // A two dimensional array that will store the data
                    );

/* Reads sctrl registers of a LTC6813 daisy chain */
int8_t LTC6813_rdsctrl(uint8_t total_ic, // number of ICs in the daisy chain
                       uint8_t sctrl_reg, // SCTRL Register A or B
                       cell_asic *ic //< a two dimensional array that the function stores the read data
                      );

/*
Start Sctrl data communication
This command will start the sctrl pulse communication over the spins
*/
void LTC6813_stsctrl();

/*
The command clears the Sctrl registers and initializes
all values to 0. The register will read back hexadecimal 0x00
after the command is sent.
*/
void LTC6813_clrsctrl();
/* Write the 6813 PWM/Sctrl Register B  */
void LTC6813_wrpsb(uint8_t total_ic, // Number of ICs in the system
					cell_asic *ic // A two dimensional array that will store the data
					);

/* Reading the 6813 PWM/Sctrl Register B */
uint8_t LTC6813_rdpsb(uint8_t total_ic, //< number of ICs in the daisy chain
                      cell_asic *ic //< a two dimensional array that the function stores the read data
                      );

/* Writes the COMM registers of a LTC6813 daisy chain */
void LTC6813_wrcomm(uint8_t total_ic, //The number of ICs being written to
				    cell_asic *ic //A two dimensional array of the comm data that will be written
				   );

/* Reads COMM registers of a LTC6813 daisy chain */
int8_t LTC6813_rdcomm(uint8_t total_ic, //Number of ICs in the system
					  cell_asic *ic //A two dimensional array that the function stores the read data.
				     );

/* Shifts data in COMM register out over LTC6813 SPI/I2C port */
void LTC6813_stcomm(uint8_t len); //Length of data to be transmitted


/* Mutes the LTC6813 discharge transistors */
void LTC6813_mute();

/* Clears the LTC6813 Mute Discharge */
void LTC6813_unmute();

/* Helper function that increments PEC counters */
void LTC6813_check_pec(uint8_t total_ic,//Number of ICs in the system
						uint8_t reg, //  Type of register
						cell_asic *ic // A two dimensional array that will store the data
						);

/* Helper Function to reset PEC counters */
void LTC6813_reset_crc_count(uint8_t total_ic, //Number of ICs in the system
							 cell_asic *ic // A two dimensional array that will store the data
							 );

/* Helper function to initialize CFG variables */
void LTC6813_init_cfg(uint8_t total_ic, cell_asic *ic);

/* Helper function to set CFGR variable */
void LTC6813_set_cfgr(uint8_t nIC, cell_asic *ic, bool refon, bool adcopt, bool gpio[5],bool dcc[12],bool dcto[4], uint16_t uv, uint16_t  ov);

/* Helper function to set the REFON bit */
void LTC6813_set_cfgr_refon(uint8_t nIC, cell_asic *ic, bool refon);

/* Helper function to set the adcopt bit */
void LTC6813_set_cfgr_adcopt(uint8_t nIC, cell_asic *ic, bool adcopt);

/* Helper function to set GPIO bits */
void LTC6813_set_cfgr_gpio(uint8_t nIC, cell_asic *ic,bool gpio[5]);

/* Helper function to control discharge */
void LTC6813_set_cfgr_dis(uint8_t nIC, cell_asic *ic,bool dcc[12]);

/* Helper Function to set uv value in CFG register */
void LTC6813_set_cfgr_uv(uint8_t nIC, cell_asic *ic,uint16_t uv);

/* Helper Function to set dcto value in CFG register */
void LTC6813_set_cfgr_dcto(uint8_t nIC, cell_asic *ic,bool dcto[4]);

/* Helper function to set OV value in CFG register */
void LTC6813_set_cfgr_ov(uint8_t nIC, cell_asic *ic,uint16_t ov);

/* Helper Function to initialize the CFGRB data structures */
void LTC6813_init_cfgb(uint8_t total_ic,cell_asic *ic);

/* Helper Function to set the configuration register B */
void LTC6813_set_cfgrb(uint8_t nIC, cell_asic *ic,bool fdrf,bool dtmen,bool ps[2],bool gpiobits[4],bool dccbits[7]);

/* Helper function to set the FDRF bit */
void LTC6813_set_cfgrb_fdrf(uint8_t nIC, cell_asic *ic, bool fdrf);

/* Helper function to set the DTMEN bit */
void LTC6813_set_cfgrb_dtmen(uint8_t nIC, cell_asic *ic, bool dtmen);

/* Helper function to set the PATH SELECT bit */
void LTC6813_set_cfgrb_ps(uint8_t nIC, cell_asic *ic, bool ps[]);

/*  Helper function to set the gpio bits in configb b register  */
void LTC6813_set_cfgrb_gpio_b(uint8_t nIC, cell_asic *ic, bool gpiobits[]);

/*  Helper function to set the dcc bits in configb b register */
void LTC6813_set_cfgrb_dcc_b(uint8_t nIC, cell_asic *ic, bool dccbits[]);

#endif /* INC_LTC6813_HAL_H_ */
