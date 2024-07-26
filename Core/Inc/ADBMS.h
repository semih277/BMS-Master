#ifndef __ADBMS_H__
#define __ADBMS_H__

#include "stm32f4xx_hal.h"

#include "adBms6830ParseCreate.h"
#include "adBms6830Data.h"
#include "adBms6830CmdList.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define WAKEUP_DELAY 4                          /* BMS ic wakeup delay  */
#define SPI_TIME_OUT HAL_MAX_DELAY              /* SPI Time out delay   */
#define SPI2_CS_Pin GPIO_PIN_0
#define SPI2_CS_GPIO_Port GPIOC

extern SPI_HandleTypeDef hspi2;

extern SPI_HandleTypeDef *hspi;       /* MUC SPI Handler      */

/* ADC Command Configurations */
extern RD      REDUNDANT_MEASUREMENT           ;
extern CH      AUX_CH_TO_CONVERT               ;
extern CONT    CONTINUOUS_MEASUREMENT          ;
extern OW_C_S  CELL_OPEN_WIRE_DETECTION        ;
extern OW_AUX  AUX_OPEN_WIRE_DETECTION         ;
extern PUP     OPEN_WIRE_CURRENT_SOURCE        ;
extern DCP     DISCHARGE_PERMITTED             ;
extern RSTF    RESET_FILTER                    ;
extern ERR     INJECT_ERR_SPI_READ             ;

void adBms6830_init_config(uint8_t tIC, cell_asic *ic);
void adBmsWakeupIc(uint8_t total_ic);
void adBms6830_write_read_config(uint8_t tIC, cell_asic *ic);
void adBms6830_read_config(uint8_t tIC, cell_asic *ic);
void adBms6830_start_adc_cell_voltage_measurment(uint8_t tIC);
void adBms6830_read_cell_voltages(uint8_t tIC, cell_asic *ic);
void adBms6830_start_adc_s_voltage_measurment(uint8_t tIC);
void adBms6830_start_adc_s_voltage_measurment_c(uint8_t tIC);
void adBms6830_start_adc_s_voltage_measurment_t(uint8_t tIC);
void adBms6830_read_s_voltages(uint8_t tIC, cell_asic *ic);
void adBms6830_start_avgcell_voltage_measurment(uint8_t tIC);
void adBms6830_read_avgcell_voltages(uint8_t tIC, cell_asic *ic);
void adBms6830_start_fcell_voltage_measurment(uint8_t tIC);
void adBms6830_read_fcell_voltages(uint8_t tIC, cell_asic *ic);
void adBms6830_start_aux_voltage_measurment(uint8_t tIC, cell_asic *ic);
void adBms6830_read_aux_voltages(uint8_t tIC, cell_asic *ic);
void adBms6830_start_raux_voltage_measurment(uint8_t tIC, cell_asic *ic);
void adBms6830_read_raux_voltages(uint8_t tIC, cell_asic *ic);
void adBms6830_read_status_registers(uint8_t tIC, cell_asic *ic);
void measurement_loop(void);
void adBms6830_read_device_sid(uint8_t tIC, cell_asic *ic);
void adBms6830_set_reset_gpio_pins(uint8_t tIC, cell_asic *ic);
void adBms6830_enable_mute(uint8_t tIC, cell_asic *ic);
void adBms6830_disable_mute(uint8_t tIC, cell_asic *ic);
void adBms6830_soft_reset(uint8_t tIC);
void adBms6830_reset_cmd_count(uint8_t tIC);
void adBms6830_reset_pec_error_flag(uint8_t tIC, cell_asic *ic);
void adBms6830_snap(uint8_t tIC);
void adBms6830_unsnap(uint8_t tIC);
HAL_StatusTypeDef adBms6830_open_wire_detection(uint8_t tIC, cell_asic *ic);
void adBms6830tempMesurementand_open_wire_detection(uint8_t tIC, cell_asic *ic);
float Thermistor(float mesuredVoltage,float refVoltage);
void adBms6830_clear_cell_measurement(uint8_t tIC);
void adBms6830_clear_aux_measurement(uint8_t tIC);
void adBms6830_clear_spin_measurement(uint8_t tIC);
void adBms6830_clear_fcell_measurement(uint8_t tIC);
void adBms6830_clear_ovuv_measurement(uint8_t tIC);
void adBms6830_clear_all_flags(uint8_t tIC, cell_asic *ic);
void adBms6830_set_dcc_discharge(uint8_t tIC, cell_asic *ic);
void adBms6830_clear_dcc_discharge(uint8_t tIC, cell_asic *ic);
void adBms6830_write_read_pwm_duty_cycle(uint8_t tIC, cell_asic *ic);
void adBms6830_gpio_spi_communication(uint8_t tIC, cell_asic *ic);
void adBms6830_gpio_i2c_write_to_slave(uint8_t tIC, cell_asic *ic);
void adBms6830_gpio_i2c_read_from_slave(uint8_t tIC, cell_asic *ic);
void adBms6830_set_dtrng_dcto_value(uint8_t tIC, cell_asic *ic);
void adBms6830_run_osc_mismatch_self_test(uint8_t tIC, cell_asic *ic);
void adBms6830_run_thermal_shutdown_self_test(uint8_t tIC, cell_asic *ic);
void adBms6830_run_supply_error_detection_self_test(uint8_t tIC, cell_asic *ic);
void adBms6830_run_thermal_shutdown_self_test(uint8_t tIC, cell_asic *ic);
void adBms6830_run_fuse_ed_self_test(uint8_t tIC, cell_asic *ic);
void adBms6830_run_fuse_med_self_test(uint8_t tIC, cell_asic *ic);
void adBms6830_run_tmodchk_self_test(uint8_t tIC, cell_asic *ic);
void adBms6830_check_latent_fault_csflt_status_bits(uint8_t tIC, cell_asic *ic);
void adBms6830_check_rdstatc_err_bit_functionality(uint8_t tIC, cell_asic *ic);
void adBms6830_cell_openwire_test(uint8_t tIC, cell_asic *ic);
void adBms6830_redundant_cell_openwire_test(uint8_t tIC, cell_asic *ic);
void adBms6830_cell_ow_volatge_collect(uint8_t tIC, cell_asic *ic, TYPE type, OW_C_S ow_c_s);
void adBms6830_aux_openwire_test(uint8_t tIC, cell_asic *ic);
void adBms6830_gpio_pup_up_down_volatge_collect(uint8_t tIC, cell_asic *ic, PUP pup);
void adBms6830_open_wire_detection_condtion_check(uint8_t tIC, cell_asic *ic, TYPE type);
void adBms6830_read_rdcvall_voltage(uint8_t tIC, cell_asic *ic);
void adBms6830_read_rdacall_voltage(uint8_t tIC, cell_asic *ic);
void adBms6830_read_rdsall_voltage(uint8_t tIC, cell_asic *ic);
void adBms6830_read_rdfcall_voltage(uint8_t tIC, cell_asic *ic);
void adBms6830_read_rdcsall_voltage(uint8_t tIC, cell_asic *ic);
void adBms6830_read_rdacsall_voltage(uint8_t tIC, cell_asic *ic);
void adBms6830_read_rdasall_voltage(uint8_t tIC, cell_asic *ic);
void printWriteConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
void printReadConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
void printVoltages(uint8_t tIC, cell_asic *IC, TYPE type);
void printStatus(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
void printDeviceSID(uint8_t tIC, cell_asic *IC, TYPE type);
void printWritePwmDutyCycle(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
void printReadPwmDutyCycle(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
void printWriteCommData(uint8_t tIC, cell_asic *IC, TYPE type);
void printReadCommData(uint8_t tIC, cell_asic *IC, TYPE type);
void printDiagnosticTestResult(uint8_t tIC, cell_asic *IC, DIAGNOSTIC_TYPE type);
void diagnosticTestResultPrint(uint8_t result);
void printOpenWireTestResult(uint8_t tIC, cell_asic *IC, TYPE type);
void openWireResultPrint(uint8_t result);
float getVoltage(int data);
void printPollAdcConvTime(int count);

/* Calculates and returns the CRC15Table */
uint16_t Pec15_Calc
(
  uint8_t len, /* Number of bytes that will be used to calculate a PEC */
  uint8_t *data /* Array of data that will be used to calculate  a PEC */
);
uint16_t pec10_calc(bool rx_cmd, int len, uint8_t *data);
void spiSendCmd(uint8_t tx_cmd[2]);
void spiReadData
(
uint8_t tIC,
uint8_t tx_cmd[2],
uint8_t *rx_data,
uint8_t *pec_error,
uint8_t *cmd_cntr,
uint8_t regData_size
);
void spiWriteData
(
  uint8_t tIC,
  uint8_t tx_cmd[2],
  uint8_t *data
);
void adBmsReadData(uint8_t tIC, cell_asic *ic, uint8_t cmd_arg[2], TYPE type, GRP group);
void adBmsWriteData(uint8_t tIC, cell_asic *ic, uint8_t cmd_arg[2], TYPE type, GRP group);
uint32_t adBmsPollAdc(uint8_t tx_cmd[2]);
void adBms6830_Adcv
(
  RD rd,
  CONT cont,
  DCP dcp,
  RSTF rstf,
  OW_C_S owcs
);

void adBms6830_Adcv_ALL
(
  RD rd,
  CONT cont,
  DCP dcp,
  RSTF rstf,
  OW_C_S owcs,
  uint8_t totai_ic
);

void adBms6830_Adsv
(
  CONT cont,
  DCP dcp,
  OW_C_S owcs
);

void adBms6830_Adsv_ALL
(
  CONT cont,
  DCP dcp,
  OW_C_S owcs,
  uint8_t total_ic
);

void adBms6830_Adax
(
OW_AUX owaux,
PUP pup,
CH ch
);

void adBms6830_Adax_ALL
(
OW_AUX owaux,
PUP pup,
CH ch,
uint8_t total_ic
);

void adBms6830_Adax2
(
  CH ch
);

#endif
