#include "ADBMS.h"
#include "math.h"
#include <stdint.h>



SPI_HandleTypeDef *hspi = &hspi2;

RD REDUNDANT_MEASUREMENT = RD_OFF;
CH AUX_CH_TO_CONVERT = AUX_ALL;
CONT CONTINUOUS_MEASUREMENT = SINGLE;
OW_C_S CELL_OPEN_WIRE_DETECTION = OW_OFF_ALL_CH;
OW_AUX AUX_OPEN_WIRE_DETECTION = AUX_OW_OFF;
PUP OPEN_WIRE_CURRENT_SOURCE = PUP_DOWN;
DCP DISCHARGE_PERMITTED = DCP_OFF;
RSTF RESET_FILTER = RSTF_OFF;
ERR INJECT_ERR_SPI_READ = WITHOUT_ERR;

const float OV_THRESHOLD = 4.2; /* Volt */
const float UV_THRESHOLD = 3.2; /* Volt */
const int OWC_Threshold = 2000; /* Cell Open wire threshold(mili volt) */
const int OWA_Threshold = 50000; /* Aux Open wire threshold(mili volt) */
const uint32_t LOOP_MEASUREMENT_COUNT = 2; /* Loop measurment count */
const uint16_t MEASUREMENT_LOOP_TIME = 10; /* milliseconds(mS)*/
uint32_t loop_count = 0;
uint32_t pladc_count;

/**
 *******************************************************************************
 * @brief Set configuration register A. Refer to the data sheet
 *        Set configuration register B. Refer to the data sheet
 *******************************************************************************
 */
void adBms6830_init_config(uint8_t tIC, cell_asic *ic) {
	for (uint8_t cic = 0; cic < tIC; cic++) {
		/* Init config A */
		ic[cic].tx_cfga.refon = 0;
		ic[cic].tx_cfga.cth = 1;
		ic[cic].tx_cfga.flag_d = 0;
		ic[cic].tx_cfga.gpo = 1023; /* All GPIO pull down off */
		ic[cic].tx_cfga.soakon = 0;
		ic[cic].tx_cfga.comm_bk = 0;
		ic[cic].tx_cfga.mute_st = 0;
		ic[cic].tx_cfga.comm_bk = 0;
		ic[cic].tx_cfga.owrng = 0;
		ic[cic].tx_cfga.owa = 0;
		ic[cic].tx_cfga.fc = IIR_FPA256;

		/* Init config B */
		ic[cic].tx_cfgb.dcto = 1;
		ic[cic].tx_cfgb.dtrng = 0;
		ic[cic].tx_cfgb.dtmen = DTMEN_ON;
		ic[cic].tx_cfgb.vov = SetOverVoltageThreshold(OV_THRESHOLD);
		ic[cic].tx_cfgb.vuv = SetUnderVoltageThreshold(UV_THRESHOLD);
		ic[cic].tx_cfgb.dcc = 0;

		/* Clearflag */
		ic[cic].clflag.cl_csflt = 0;
		ic[cic].clflag.cl_smed = 0;
		ic[cic].clflag.cl_sed = 0;
		ic[cic].clflag.cl_cmed = 0;
		ic[cic].clflag.cl_ced = 0;
		ic[cic].clflag.cl_vduv = 0;
		ic[cic].clflag.cl_vdov = 0;
		ic[cic].clflag.cl_vauv = 0;
		ic[cic].clflag.cl_vaov = 0;
		ic[cic].clflag.cl_oscchk = 0;
		ic[cic].clflag.cl_tmode = 0;
		ic[cic].clflag.cl_thsd = 0;
		ic[cic].clflag.cl_sleep = 0;
		ic[cic].clflag.cl_spiflt = 0;
		ic[cic].clflag.cl_vdel = 0;
		ic[cic].clflag.cl_vde = 0;

		ic[cic].TempOpenWire = 0;
	}
//	adBmsWakeupIc(tIC);
	adBmsCsLow();
	HAL_Delay(4);
	adBmsCsHigh();
	HAL_Delay(4);
	adBmsWriteData(tIC, &ic[0], CLRFLAG, Clrflag, B);
	adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
	adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
	adBmsWriteData(tIC, &ic[0], WRPWM2, Pwm, B);
	adBmsWriteData(tIC, &ic[0], WRPWM1, Pwm, A);

}

/**
 *******************************************************************************
 * Function: adBmsCsLow
 * @brief Select chip select low
 *
 * @details This function does spi chip select low.
 *
 * @return None
 *
 *******************************************************************************
 */
void adBmsCsLow() {
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
}

/**
 *******************************************************************************
 * Function: adBmsCsHigh
 * @brief Select chip select High
 *
 * @details This function does spi chip select high.
 *
 * @return None
 *
 *******************************************************************************
 */
void adBmsCsHigh() {
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

/**
 *******************************************************************************
 * Function: adBmsWakeupIc
 * @brief Wakeup bms ic using chip select
 *
 * @details This function wakeup thr bms ic using chip select.
 *
 * @param [in]  total_ic    Total_ic
 *
 * @return None
 *
 *******************************************************************************
 */
void adBmsWakeupIc(uint8_t total_ic) {
	for (uint8_t ic = 0; ic < total_ic; ic++) {
		adBmsCsLow();
		vTaskDelay(4);
		adBmsCsHigh();
		vTaskDelay(4);
//		adBmsCsLow();
//		HAL_Delay(4);
//		adBmsCsHigh();
//		HAL_Delay(4);
	}
}

/**
 *******************************************************************************
 * Function: adBmsReadData
 * @brief Adbms Read Data From Bms ic.
 *
 * @details This function send bms command, read payload data parse into function and check pec error.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  *ic      cell_asic stucture pointer
 *
 * @param [in]  cmd_arg   command bytes
 *
 * @param [in]  TYPE   Enum type of resistor
 *
 * @param [in]  GRP   Enum type of resistor group
 *
 * @return None
 *
 *******************************************************************************
 */
void adBmsReadData(uint8_t tIC, cell_asic *ic, uint8_t cmd_arg[2], TYPE type,
		GRP group) {
	uint16_t rBuff_size;
	uint8_t regData_size;
	if (group == ALL_GRP) {
		if (type == Rdcvall) {
			rBuff_size = RDCVALL_SIZE;
			regData_size = RDCVALL_SIZE;
		} else if (type == Rdsall) {
			rBuff_size = RDSALL_SIZE;
			regData_size = RDSALL_SIZE;
		} else if (type == Rdacall) {
			rBuff_size = RDACALL_SIZE;
			regData_size = RDACALL_SIZE;
		} else if (type == Rdfcall) {
			rBuff_size = RDFCALL_SIZE;
			regData_size = RDFCALL_SIZE;
		} else if (type == Rdcsall) {
			rBuff_size = RDCSALL_SIZE;
			regData_size = RDCSALL_SIZE;
		} else if (type == Rdasall) {
			rBuff_size = RDASALL_SIZE;
			regData_size = RDASALL_SIZE;
		} else if (type == Rdacsall) {
			rBuff_size = RDACSALL_SIZE;
			regData_size = RDACSALL_SIZE;
		} else {
			printf("Read All cmd wrong type select \n");
		}
	} else {
		rBuff_size = (tIC * RX_DATA);
		regData_size = RX_DATA;
	}
	uint8_t *read_buffer, *pec_error, *cmd_count;
	read_buffer = (uint8_t*) calloc(rBuff_size, sizeof(uint8_t));
	pec_error = (uint8_t*) calloc(tIC, sizeof(uint8_t));
	cmd_count = (uint8_t*) calloc(tIC, sizeof(uint8_t));
	if ((pec_error == NULL) || (cmd_count == NULL) || (read_buffer == NULL)) {
		printf(" Failed to allocate memory \n");

		exit(0);
	} else {
		spiReadData(tIC, &cmd_arg[0], &read_buffer[0], &pec_error[0],
				&cmd_count[0], regData_size);
		switch (type) {
		case Config:
			adBms6830ParseConfig(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.cfgr_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Cell:
			adBms6830ParseCell(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.cell_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case AvgCell:
			adBms6830ParseAverageCell(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.acell_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case S_volt:
			adBms6830ParseSCell(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.scell_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case F_volt:
			adBms6830ParseFCell(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.fcell_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Aux:
			adBms6830ParseAux(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.aux_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case RAux:
			adBms6830ParseRAux(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.raux_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Status:
			adBms6830ParseStatus(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.stat_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Comm:
			adBms6830ParseComm(tIC, ic, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.comm_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Pwm:
			adBms6830ParsePwm(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.pwm_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Sid:
			adBms6830ParseSID(tIC, ic, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.sid_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Rdcvall:
			/* 32 byte cell data + 2 byte pec */
			adBms6830ParseCell(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.cell_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Rdacall:
			/* 32 byte avg cell data + 2 byte pec */
			adBms6830ParseAverageCell(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.acell_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Rdsall:
			/* 32 byte scell volt data + 2 byte pec */
			adBms6830ParseSCell(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.scell_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Rdfcall:
			/* 32 byte fcell data + 2 byte pec */
			adBms6830ParseFCell(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.fcell_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Rdcsall:
			/* 64 byte + 2 byte pec = 32 byte cell data + 32 byte scell volt data */
			adBms6830ParseCell(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.cell_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			adBms6830ParseSCell(tIC, ic, group, &read_buffer[32]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.scell_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Rdacsall:
			/* 64 byte + 2 byte pec = 32 byte avg cell data + 32 byte scell volt data */
			adBms6830ParseAverageCell(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.acell_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			adBms6830ParseSCell(tIC, ic, group, &read_buffer[32]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.scell_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		case Rdasall:
			/* 68 byte + 2 byte pec:
			 24 byte gpio data + 20 byte Redundant gpio data +
			 24 byte status A(6 byte), B(6 byte), C(4 byte), D(6 byte) & E(2 byte)
			 */
			adBms6830ParseAux(tIC, ic, group, &read_buffer[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.aux_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			adBms6830ParseRAux(tIC, ic, group, &read_buffer[24]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.raux_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			adBms6830ParseStatus(tIC, ic, group, &read_buffer[44]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				ic[cic].cccrc.stat_pec = pec_error[cic];
				ic[cic].cccrc.cmd_cntr = cmd_count[cic];
			}
			break;

		default:
			break;
		}
	}
	free(read_buffer);
	free(pec_error);
	free(cmd_count);
}



/**
 *******************************************************************************
 * Function: adBmsWriteData
 * @brief Adbms Write Data into Bms ic.
 *
 * @details This function write the data into bms ic.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  *ic      cell_asic stucture pointer
 *
 * @param [in]  cmd_arg   command bytes
 *
 * @param [in]  TYPE   Enum type of resistor
 *
 * @param [in]  GRP   Enum type of resistor group
 *
 * @return None
 *
 *******************************************************************************
 */
void adBmsWriteData(uint8_t tIC, cell_asic *ic, uint8_t cmd_arg[2], TYPE type,
		GRP group) {
	uint8_t data_len = TX_DATA, write_size = (TX_DATA * tIC);
	uint8_t *write_buffer = (uint8_t*) calloc(write_size, sizeof(uint8_t));
	if (write_buffer == NULL) {
		printf(" Failed to allocate write_buffer array memory \n");
		exit(0);
	} else {
		switch (type) {
		case Config:
			switch (group) {
			case A:
				adBms6830CreateConfiga(tIC, &ic[0]);
				for (uint8_t cic = 0; cic < tIC; cic++) {
					for (uint8_t data = 0; data < data_len; data++) {
						write_buffer[(cic * data_len) + data] =
								ic[cic].configa.tx_data[data];
					}
				}
				break;
			case B:
				adBms6830CreateConfigb(tIC, &ic[0]);
				for (uint8_t cic = 0; cic < tIC; cic++) {
					for (uint8_t data = 0; data < data_len; data++) {
						write_buffer[(cic * data_len) + data] =
								ic[cic].configb.tx_data[data];
					}
				}
				break;
			}
			break;

		case Comm:
			adBms6830CreateComm(tIC, &ic[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				for (uint8_t data = 0; data < data_len; data++) {
					write_buffer[(cic * data_len) + data] =
							ic[cic].com.tx_data[data];
				}
			}
			break;

		case Pwm:
			switch (group) {
			case A:
				adBms6830CreatePwma(tIC, &ic[0]);
				for (uint8_t cic = 0; cic < tIC; cic++) {
					for (uint8_t data = 0; data < data_len; data++) {
						write_buffer[(cic * data_len) + data] =
								ic[cic].pwma.tx_data[data];
					}
				}
				break;
			case B:
				adBms6830CreatePwmb(tIC, &ic[0]);
				for (uint8_t cic = 0; cic < tIC; cic++) {
					for (uint8_t data = 0; data < data_len; data++) {
						write_buffer[(cic * data_len) + data] =
								ic[cic].pwmb.tx_data[data];
					}
				}
				break;
			}
			break;

		case Clrflag:
			adBms6830CreateClrflagData(tIC, &ic[0]);
			for (uint8_t cic = 0; cic < tIC; cic++) {
				for (uint8_t data = 0; data < data_len; data++) {
					write_buffer[(cic * data_len) + data] =
							ic[cic].clrflag.tx_data[data];
				}
			}
			break;

		default:
			break;
		}
	}
	spiWriteData(tIC, cmd_arg, &write_buffer[0]);

	free(write_buffer);
}

/**
 *******************************************************************************
 * Function: spiReadData
 * @brief Spi Read Bms Data
 *
 * @details This function send bms command in spi line and read command corrospond data byte.
 *
 * Parameters:
 * @param [in]	tIC     Total IC
 *
 * @param [in]  tx_cmd   Tx command bytes
 *
 * @param [in]  *rx_data Rx data pointer
 *
 * @param [in]  *pec_error Pec error pointer
 *
 * @param [in]  *cmd_cntr command counter pointer
 *
 * @return None
 *
 *******************************************************************************
 */
void spiReadData(uint8_t tIC, uint8_t tx_cmd[2], uint8_t *rx_data,
		uint8_t *pec_error, uint8_t *cmd_cntr, uint8_t regData_size) {
	uint8_t *data, *copyArray, src_address = 0;
	uint16_t cmd_pec, received_pec, calculated_pec;
	uint8_t BYTES_IN_REG = regData_size;
	uint8_t RX_BUFFER = (regData_size * tIC);

	data = (uint8_t*) calloc(RX_BUFFER, sizeof(uint8_t));
	copyArray = (uint8_t*) calloc(BYTES_IN_REG, sizeof(uint8_t));
	if ((data == NULL) || (copyArray == NULL)) {
		printf(" Failed to allocate spi read data memory \n");

		exit(0);
	} else {
		uint8_t cmd[4];
		cmd[0] = tx_cmd[0];
		cmd[1] = tx_cmd[1];
		cmd_pec = Pec15_Calc(2, cmd);
		cmd[2] = (uint8_t) (cmd_pec >> 8);
		cmd[3] = (uint8_t) (cmd_pec);
		adBmsCsLow();
		HAL_SPI_Transmit(hspi, &cmd[0], 4, SPI_TIME_OUT);
		HAL_SPI_Receive(hspi, &data[0], RX_BUFFER, SPI_TIME_OUT);
		adBmsCsHigh();
		for (uint8_t current_ic = 0; current_ic < tIC; current_ic++) /* executes for each ic in the daisy chain and packs the data */
		{ /* Into the r_comm array as well as check the received data for any bit errors */
			for (uint8_t current_byte = 0; current_byte < (BYTES_IN_REG - 2);
					current_byte++) {
				rx_data[(current_ic * BYTES_IN_REG) + current_byte] =
						data[current_byte + (current_ic * BYTES_IN_REG)];
			}
			/* Get command counter value */
			cmd_cntr[current_ic] = (data[(current_ic * BYTES_IN_REG)
					+ (BYTES_IN_REG - 2)] >> 2);
			/* Get received pec value from ic*/
			received_pec = (uint16_t) (((data[(current_ic * BYTES_IN_REG)
					+ (BYTES_IN_REG - 2)] & 0x03) << 8)
					| data[(current_ic * BYTES_IN_REG) + (BYTES_IN_REG - 1)]);
			/* Copy each ic correspond data + pec value for calculate data pec */
			memcpy(&copyArray[0], &data[src_address], BYTES_IN_REG);
			src_address = ((current_ic + 1) * (regData_size));
			/* Calculate data pec */
			calculated_pec = (uint16_t) pec10_calc(true, (BYTES_IN_REG - 2),
					&copyArray[0]);
			/* Match received pec with calculated pec */
			if (received_pec == calculated_pec) {
				pec_error[current_ic] = 0;
			}/* If no error is there value set to 0 */
			else {
				pec_error[current_ic] = 1;
			} /* If error is there value set to 1 */
		}
	}
	free(data);
	free(copyArray);
}

/**
 *******************************************************************************
 * Function: spiWriteData
 * @brief Spi Write Bms Data
 *
 * @details This function write the data into bms ic.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  tx_cmd   Tx command bytes
 *
 * @param [in]  *data   Data pointer
 *
 * @return None
 *
 *******************************************************************************
 */
void spiWriteData(uint8_t tIC, uint8_t tx_cmd[2], uint8_t *data) {
	uint8_t BYTES_IN_REG = TX_DATA;
	uint8_t CMD_LEN = 4 + (RX_DATA * tIC);
	uint16_t data_pec, cmd_pec;
	uint8_t *cmd, copyArray[TX_DATA], src_address = 0;
	uint8_t cmd_index;
	cmd = (uint8_t*) calloc(CMD_LEN, sizeof(uint8_t));
	if (cmd == NULL) {
		printf(" Failed to allocate cmd array memory \n");
		exit(0);
	} else {
		cmd[0] = tx_cmd[0];
		cmd[1] = tx_cmd[1];
		cmd_pec = Pec15_Calc(2, cmd);
		cmd[2] = (uint8_t) (cmd_pec >> 8);
		cmd[3] = (uint8_t) (cmd_pec);
		cmd_index = 4;
		/* executes for each LTC68xx, this loops starts with the last IC on the stack */
		for (uint8_t current_ic = tIC; current_ic > 0; current_ic--) {
			src_address = ((current_ic - 1) * TX_DATA);
			/* The first configuration written is received by the last IC in the daisy chain */
			for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG;
					current_byte++) {
				cmd[cmd_index] = data[((current_ic - 1) * 6) + current_byte];
				cmd_index = cmd_index + 1;
			}
			/* Copy each ic correspond data + pec value for calculate data pec */
			memcpy(&copyArray[0], &data[src_address], TX_DATA); /* dst, src, size */
			/* calculating the PEC for each Ics configuration register data */
			data_pec = (uint16_t) pec10_calc(true, BYTES_IN_REG, &copyArray[0]);
			cmd[cmd_index] = (uint8_t) (data_pec >> 8);
			cmd_index = cmd_index + 1;
			cmd[cmd_index] = (uint8_t) data_pec;
			cmd_index = cmd_index + 1;
		}
		adBmsCsLow();
		HAL_SPI_Transmit(hspi, &cmd[0], CMD_LEN, SPI_TIME_OUT); /* SPI1 , data, size, timeout */
		adBmsCsHigh();
	}
	free(cmd);
}

/* Precomputed CRC15 Table */
const uint16_t Crc15Table[256] = { 0x0000, 0xc599, 0xceab, 0xb32, 0xd8cf,
		0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac, 0xff35, 0x2cc8, 0xe951,
		0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1, 0xbbf3,
		0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d,
		0x5b2e, 0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29,
		0x6ab0, 0x6182, 0xa41b, 0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320,
		0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd, 0x2544, 0x2be, 0xc727, 0xcc15,
		0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c, 0x3d6e, 0xf8f7,
		0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
		0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304,
		0x4836, 0x8daf, 0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31,
		0x79a8, 0xa8eb, 0x6d72, 0x6640, 0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16,
		0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba, 0x4a88, 0x8f11, 0x57c,
		0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b, 0x34e2,
		0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8,
		0xa921, 0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26,
		0x8edb, 0x4b42, 0x4070, 0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b,
		0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528, 0xf0b1, 0x234c, 0xe6d5,
		0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
		0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7,
		0x54aa, 0x9133, 0x9a01, 0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad,
		0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9, 0x7350, 0x51d6, 0x944f,
		0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
		0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773,
		0x248e, 0xe117, 0xea25, 0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089,
		0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453, 0x1ca, 0xd237, 0x17ae,
		0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
		0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492,
		0x5368, 0x96f1, 0x9dc3, 0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095 };

/**
 *******************************************************************************
 * Function: Pec15_Calc
 * @brief CRC15 Pec Calculation Function
 *
 * @details This function calculates and return the CRC15 value
 *
 * Parameters:
 * @param [in]	Len	Data length
 *
 * @param [in] *data    Data pointer
 *
 * @return CRC15_Value
 *
 *******************************************************************************
 */
uint16_t Pec15_Calc(uint8_t len, /* Number of bytes that will be used to calculate a PEC */
uint8_t *data /* Array of data that will be used to calculate  a PEC */
) {
	uint16_t remainder, addr;
	remainder = 16; /* initialize the PEC */
	for (uint8_t i = 0; i < len; i++) /* loops for each byte in data array */
	{
		addr = (((remainder >> 7) ^ data[i]) & 0xff);/* calculate PEC table address */
		remainder = ((remainder << 8) ^ Crc15Table[addr]);
	}
	return (remainder * 2);/* The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2 */
}

uint16_t pec10_calc(bool rx_cmd, int len, uint8_t *data) {
	uint16_t remainder = 16; /* PEC_SEED;   0000010000 */
	uint16_t polynom = 0x8F; /* x10 + x7 + x3 + x2 + x + 1 <- the CRC15 polynomial         100 1000 1111   48F */

	/* Perform modulo-2 division, a byte at a time. */
	for (uint8_t pbyte = 0; pbyte < len; ++pbyte) {
		/* Bring the next byte into the remainder. */
		remainder ^= (uint16_t) (data[pbyte] << 2);
		/* Perform modulo-2 division, a bit at a time.*/
		for (uint8_t bit_ = 8; bit_ > 0; --bit_) {
			/* Try to divide the current data bit. */
			if ((remainder & 0x200) > 0) //equivalent to remainder & 2^14 simply check for MSB
					{
				remainder = (uint16_t) ((remainder << 1));
				remainder = (uint16_t) (remainder ^ polynom);
			} else {
				remainder = (uint16_t) (remainder << 1);
			}
		}
	}
	if (rx_cmd == true) {
		remainder ^= (uint16_t) ((data[len] & 0xFC) << 2);
		/* Perform modulo-2 division, a bit at a time */
		for (uint8_t bit_ = 6; bit_ > 0; --bit_) {
			/* Try to divide the current data bit */
			if ((remainder & 0x200) > 0) //equivalent to remainder & 2^14 simply check for MSB
					{
				remainder = (uint16_t) ((remainder << 1));
				remainder = (uint16_t) (remainder ^ polynom);
			} else {
				remainder = (uint16_t) ((remainder << 1));
			}
		}
	}
	return ((uint16_t) (remainder & 0x3FF));
}

/**
 *******************************************************************************
 * Function: adBmsPollAdc
 * @brief PLADC Command.
 *
 * @details Send poll adc command and retun adc conversion count.
 *
 * Parameters:
 *
 * @param [in]  tIC      Total IC
 *
 * @param [in]  tx_cmd   Tx command byte
 *
 * @return None
 *
 *******************************************************************************
 */
uint32_t adBmsPollAdc(uint8_t tx_cmd[2]) {
	uint32_t conv_count = 0; // sure olcmek icindi kullanilmiyor
	uint8_t cmd[4];
	uint16_t cmd_pec;
	uint8_t read_data = 0x00;
	uint8_t SDO_Line = 0xFF;
	cmd[0] = tx_cmd[0];
	cmd[1] = tx_cmd[1];
	cmd_pec = Pec15_Calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);
	adBmsCsLow();
	HAL_SPI_Transmit(hspi, &cmd[0], 4, SPI_TIME_OUT);
	do {
		HAL_SPI_Receive(hspi, &read_data, 1, SPI_TIME_OUT);
	} while (!(read_data == SDO_Line));
	adBmsCsHigh();
	return (conv_count);
}

/**
 *******************************************************************************
 * Function: adBms6830_Adcv
 * @brief ADCV Command.
 *
 * @details Send adcv command to start cell voltage conversion.
 *
 * Parameters:
 *
 * @param [in]  RD      Enum type Read bit
 *
 * @param [in]  CONT   Enum type continuous measurement bit
 *
 * @param [in]  DCP   Enum type discharge bit
 *
 * @param [in]  RSTF   Enum type Reset filter
 *
 * @param [in]  OW_C_S   Enum type open wire c/s
 *
 * @return None
 *
 *******************************************************************************
 */
void adBms6830_Adcv(RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs) {
	uint8_t cmd[2];
	cmd[0] = 0x02 + rd;
	cmd[1] = (cont << 7) + (dcp << 4) + (rstf << 2) + (owcs & 0x03) + 0x60;
	spiSendCmd(cmd);
}

void adBms6830_Adcv_ALL(RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S owcs, uint8_t total_ic) {
	for(uint8_t icCounter = 0; icCounter < total_ic; icCounter++){
		uint8_t cmd[2];
		cmd[0] = 0x02 + rd;
		cmd[1] = (cont << 7) + (dcp << 4) + (rstf << 2) + (owcs & 0x03) + 0x60;
		spiSendCmd(cmd);
	}

}

/**
 *******************************************************************************
 * Function: adBms6830_Adsv
 * @brief ADSV Command.
 *
 * @details Send s_adcv command to start cell voltage conversion.
 *
 * Parameters:
 *
 * @param [in]  cont    Enum type continuous measurement bit
 *
 * @param [in]  dcp    Enum type discharge bit
 *
 * @param [in]  owcs   Enum type open wire c/s
 *
 * @return None
 *file:///C:/Users/An%C4%B1l/Desktop/ADBMS6830B_ADI%20-%20converted.pdfsend
 *******************************************************************************
 */
void adBms6830_Adsv(CONT cont, DCP dcp, OW_C_S owcs) {
	uint8_t cmd[2];
	cmd[0] = 0x01;
	cmd[1] = (cont << 7) + (dcp << 4) + (owcs & 0x03) + 0x68;
	spiSendCmd(cmd);
}

void adBms6830_Adsv_ALL(CONT cont, DCP dcp, OW_C_S owcs, uint8_t total_ic) {
	for(uint8_t icCounter = 0; icCounter < total_ic; icCounter++){
	uint8_t cmd[2];
	cmd[0] = 0x01;
	cmd[1] = (cont << 7) + (dcp << 4) + (owcs & 0x03) + 0x68;
	spiSendCmd(cmd);
	}
}

/**
 *******************************************************************************
 * Function: adBms6830_Adax
 * @brief ADAX Command.
 *
 * @details Send Aux command to starts auxiliary conversion.
 *
 * Parameters:
 *
 * @param [in]  owcs    Enum type open wire c/s
 *
 * @param [in]  pup    Enum type Pull Down current during aux conversion
 *
 * @param [in]  ch    Enum type gpio Channel selection
 *
 * @return None
 *
 *******************************************************************************
 */
void adBms6830_Adax(OW_AUX owaux, PUP pup, CH ch) {
	uint8_t cmd[2];
	cmd[0] = 0x04 + owaux;
	cmd[1] = (pup << 7) + (((ch >> 4) & 0x01) << 6) + (ch & 0x0F) + 0x10;
	spiSendCmd(cmd);
}

void adBms6830_Adax_ALL(OW_AUX owaux, PUP pup, CH ch, uint8_t total_ic) {
	for(uint8_t icCounter = 0; icCounter < total_ic; icCounter++){
	uint8_t cmd[2];
	cmd[0] = 0x04 + owaux;
	cmd[1] = (pup << 7) + (((ch >> 4) & 0x01) << 6) + (ch & 0x0F) + 0x10;
	spiSendCmd(cmd);
	}
}
/**
 *******************************************************************************
 * Function: adBms6830_Adax2
 * @brief ADAX2 Command.
 *
 * @details Send Aux2 command to starts auxiliary conversion.
 *
 * Parameters:
 *
 * @param [in]  ch    Enum type gpio Channel selection
 *
 * @return None
 *
 *******************************************************************************
 */
void adBms6830_Adax2(CH ch) {
	uint8_t cmd[2];
	cmd[0] = 0x04;
	cmd[1] = (ch & 0x0F);
	spiSendCmd(cmd);
}

/**
 *******************************************************************************
 * @brief Write and Read Configuration Register A/B
 *******************************************************************************
 */
void adBms6830_write_read_config(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
	adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
	adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
	adBmsReadData(tIC, &ic[0], RDCFGB, Config, B);
	printWriteConfig(tIC, &ic[0], Config, ALL_GRP);
	printReadConfig(tIC, &ic[0], Config, ALL_GRP);
}

/**
 *******************************************************************************
 * @brief Read Configuration Register A/B
 *******************************************************************************
 */
void adBms6830_read_config(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
	adBmsReadData(tIC, &ic[0], RDCFGB, Config, B);
	printReadConfig(tIC, &ic[0], Config, ALL_GRP);
}

/**
 *******************************************************************************
 * @brief Start ADC Cell Voltage Measurement
 *******************************************************************************
 */
void adBms6830_start_adc_cell_voltage_measurment(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	adBms6830_Adcv(REDUNDANT_MEASUREMENT, CONTINUOUS_MEASUREMENT, DISCHARGE_PERMITTED, RESET_FILTER, CELL_OPEN_WIRE_DETECTION);
	pladc_count = adBmsPollAdc(PLADC);
	printf("Cell conversion completed\n");

	printPollAdcConvTime(pladc_count);
}

/**
 *******************************************************************************
 * @brief Read Cell Voltages
 *******************************************************************************
 */
void adBms6830_read_cell_voltages(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBmsReadData(tIC, &ic[0], RDCVA, Cell, A);
	adBmsReadData(tIC, &ic[0], RDCVB, Cell, B);
	adBmsReadData(tIC, &ic[0], RDCVC, Cell, C);
	adBmsReadData(tIC, &ic[0], RDCVD, Cell, D);
	adBmsReadData(tIC, &ic[0], RDCVE, Cell, E);
	adBmsReadData(tIC, &ic[0], RDCVF, Cell, F);
	printVoltages(tIC, &ic[0], Cell);
}

/**
 *******************************************************************************
 * @brief Start ADC S-Voltage Measurement
 *******************************************************************************
 */
void adBms6830_start_adc_s_voltage_measurment(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	adBms6830_Adsv(CONTINUOUS_MEASUREMENT, DISCHARGE_PERMITTED,
			CELL_OPEN_WIRE_DETECTION);
	pladc_count = adBmsPollAdc(PLADC);
	printf("S-Voltage conversion completed\n");

	printPollAdcConvTime(pladc_count);
}
void adBms6830_start_adc_s_voltage_measurment_c(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	adBms6830_Adsv(CONTINUOUS_MEASUREMENT, DISCHARGE_PERMITTED, OW_ON_EVEN_CH);
	pladc_count = adBmsPollAdc(PLADC);
	printf("S-Voltage conversion completed\n");

	printPollAdcConvTime(pladc_count);
}
void adBms6830_start_adc_s_voltage_measurment_t(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	adBms6830_Adsv(CONTINUOUS_MEASUREMENT, DISCHARGE_PERMITTED, OW_ON_ODD_CH);
	pladc_count = adBmsPollAdc(PLADC);
	printf("S-Voltage conversion completed\n");

	printPollAdcConvTime(pladc_count);
}

/**
 *******************************************************************************
 * @brief Read S-Voltages
 *******************************************************************************
 */
void adBms6830_read_s_voltages(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBmsReadData(tIC, &ic[0], RDSVA, S_volt, A);
	adBmsReadData(tIC, &ic[0], RDSVB, S_volt, B);
	adBmsReadData(tIC, &ic[0], RDSVC, S_volt, C);
	adBmsReadData(tIC, &ic[0], RDSVD, S_volt, D);
	adBmsReadData(tIC, &ic[0], RDSVE, S_volt, E);
	adBmsReadData(tIC, &ic[0], RDSVF, S_volt, F);
	printVoltages(tIC, &ic[0], S_volt);
}

float Thermistor(float mesuredVoltage, float refVoltage) {

	double temperature;
	temperature = log((mesuredVoltage * 10000) / (refVoltage - mesuredVoltage));
	temperature = 1
			/ (0.001153145702
					+ (0.0002307020098
							+ (0.00000009730588083 * temperature * temperature))
							* temperature);
	temperature = temperature - 273.15;
	return temperature;
}

void adBms6830tempMesurementand_open_wire_detection(uint8_t tIC,
		cell_asic *ic) {

//	uint8_t ntcSellectionList[16] { 15, 1, 14, 2, 13, 3, 12, 4, 11, 5, 10, 6, 9,
//			7, 8, 0 };

	uint8_t ntcSellectionList[16] = { 15, 1, 14, 2, 13, 3, 12, 4, 11, 5, 10, 6, 9, 7, 8, 0 };

	for (uint8_t icCounter = 0; icCounter < tIC; icCounter++) {
		ic[icCounter].TempOpenWire = 0;
		for (uint8_t ntcSellection = 0; ntcSellection < 16; ntcSellection++) {
			ic[icCounter].tx_cfga.gpo = ntcSellectionList[ntcSellection] << 6
					| 0x3F;

			adBmsWakeupIc(tIC);
			adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
			adBms6830_Adax(AUX_OW_OFF, PUP_DOWN, AUX_ALL);
			adBmsReadData(tIC, &ic[0], RDAUXB, Aux, B);
			adBmsReadData(tIC, &ic[0], RDSTATA, Status, A);
			if (ic[icCounter].cccrc.aux_pec == 0) {

			if (abs(
					getVoltage(ic[icCounter].aux.a_codes[5])
							- getVoltage(ic[icCounter].stata.vref2)) < 0.001) {
				ic[icCounter].TempOpenWire |= 1 << ntcSellection;
			}
			ic[icCounter].mesured_cell_temps[ntcSellection] = Thermistor(
					getVoltage(ic[icCounter].aux.a_codes[5]),
					getVoltage(ic[icCounter].stata.vref2));
			printf("ntc%d:%f %f \n",ntcSellection+1,ic[icCounter].mesured_cell_temps[ntcSellection],getVoltage(ic[icCounter].aux.a_codes[5]));
		}
			else ic[icCounter].TempOpenWire=0xFFFF;


		}

	}

}


HAL_StatusTypeDef adBms6830_open_wire_detection(uint8_t tIC, cell_asic *ic) {

	float voltage[tIC][2][16];

	adBmsWakeupIc(tIC);
	adBms6830_Adsv(CONTINUOUS, DISCHARGE_PERMITTED, OW_ON_EVEN_CH);
	HAL_Delay(2);
	adBmsReadData(tIC, &ic[0], RDSVA, S_volt, A);
	adBmsReadData(tIC, &ic[0], RDSVB, S_volt, B);
	adBmsReadData(tIC, &ic[0], RDSVC, S_volt, C);
	adBmsReadData(tIC, &ic[0], RDSVD, S_volt, D);
	adBmsReadData(tIC, &ic[0], RDSVE, S_volt, E);
	adBmsReadData(tIC, &ic[0], RDSVF, S_volt, F);

	for (uint8_t icCounter = 0; icCounter < tIC; icCounter++) {
		for (uint8_t cell = 0; cell < 16; cell++) {
			if (ic[icCounter].cccrc.scell_pec == 0) {
				voltage[icCounter][0][cell] = getVoltage(
						ic[icCounter].scell.sc_codes[cell]);
//				printf("ciftpack:%d,", (icCounter));
//				printf("C%d=%fV,", (cell + 1), voltage[icCounter][0][cell]);
//				printf("\n");

			} else
				return HAL_ERROR;
		}
	}
	HAL_Delay(8);
	adBmsWakeupIc(tIC);
	adBms6830_Adsv(CONTINUOUS, DISCHARGE_PERMITTED, OW_ON_ODD_CH);
	HAL_Delay(2);
	adBmsReadData(tIC, &ic[0], RDSVB, S_volt, B);
	adBmsReadData(tIC, &ic[0], RDSVA, S_volt, A);
	adBmsReadData(tIC, &ic[0], RDSVC, S_volt, C);
	adBmsReadData(tIC, &ic[0], RDSVD, S_volt, D);
	adBmsReadData(tIC, &ic[0], RDSVE, S_volt, E);
	adBmsReadData(tIC, &ic[0], RDSVF, S_volt, F);

	for (uint8_t icCounter = 0; icCounter < tIC; icCounter++) {
		for (uint8_t cell = 0; cell < 16; cell++) {
			if (ic[icCounter].cccrc.scell_pec == 0) {
				voltage[icCounter][1][cell] = getVoltage(
						ic[icCounter].scell.sc_codes[cell]);
//				printf("tekpack:%d,", (icCounter));
//				printf("C%d=%fV,", (cell + 1), voltage[icCounter][1][cell]);
//				printf("\n");

			} else
				return HAL_ERROR;
		}
	}
	for (uint8_t icCounter = 0; icCounter < tIC; icCounter++) {
		for (uint8_t cell = 0; cell < 16; cell++) {
			for (uint8_t cifttek = 0; cifttek < 2; cifttek++) {
//				printf("ic:%d cell %d %d: %f\n", icCounter, cell + 1, cifttek,
//						voltage[icCounter][cifttek][cell]);
				if (voltage[icCounter][cifttek][cell] < 2 || voltage[icCounter][cifttek][cell] > 4.5) {
					//printf("HATA:%d\n",cell+1);
					return HAL_ERROR;
				}

			}

		}

	}

	return HAL_OK;

}

/**
 *******************************************************************************
 * @brief Start Avarage Cell Voltage Measurement
 *******************************************************************************
 */
void adBms6830_start_avgcell_voltage_measurment(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	adBms6830_Adcv(RD_ON, CONTINUOUS_MEASUREMENT, DISCHARGE_PERMITTED,
			RESET_FILTER, CELL_OPEN_WIRE_DETECTION);
	pladc_count = adBmsPollAdc(PLADC);
	printf("Avg Cell voltage conversion completed\n");

	printPollAdcConvTime(pladc_count);
}

/**
 *******************************************************************************
 * @brief Read Avarage Cell Voltages
 *******************************************************************************
 */
void adBms6830_read_avgcell_voltages(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBmsReadData(tIC, &ic[0], RDACA, AvgCell, A);
	adBmsReadData(tIC, &ic[0], RDACB, AvgCell, B);
	adBmsReadData(tIC, &ic[0], RDACC, AvgCell, C);
	adBmsReadData(tIC, &ic[0], RDACD, AvgCell, D);
	adBmsReadData(tIC, &ic[0], RDACE, AvgCell, E);
	adBmsReadData(tIC, &ic[0], RDACF, AvgCell, F);
	printVoltages(tIC, &ic[0], AvgCell);
}

/**
 *******************************************************************************
 * @brief Start Filtered Cell Voltages Measurement
 *******************************************************************************
 */
void adBms6830_start_fcell_voltage_measurment(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	adBms6830_Adcv(REDUNDANT_MEASUREMENT, CONTINUOUS_MEASUREMENT,
			DISCHARGE_PERMITTED, RESET_FILTER, CELL_OPEN_WIRE_DETECTION);
	pladc_count = adBmsPollAdc(PLADC);
	printf("F Cell voltage conversion completed\n");

	printPollAdcConvTime(pladc_count);
}

/**
 *******************************************************************************
 * @brief Read Filtered Cell Voltages
 *******************************************************************************
 */
void adBms6830_read_fcell_voltages(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBmsReadData(tIC, &ic[0], RDFCA, F_volt, A);
	adBmsReadData(tIC, &ic[0], RDFCB, F_volt, B);
	adBmsReadData(tIC, &ic[0], RDFCC, F_volt, C);
	adBmsReadData(tIC, &ic[0], RDFCD, F_volt, D);
	adBmsReadData(tIC, &ic[0], RDFCE, F_volt, E);
	adBmsReadData(tIC, &ic[0], RDFCF, F_volt, F);
	printVoltages(tIC, &ic[0], F_volt);
}

/**
 *******************************************************************************
 * @brief Start AUX, VMV, V+ Voltages Measurement
 *******************************************************************************
 */
void adBms6830_start_aux_voltage_measurment(uint8_t tIC, cell_asic *ic) {
	for (uint8_t cic = 0; cic < tIC; cic++) {
		/* Init config A */
		ic[cic].tx_cfga.refon = PWR_UP;
		ic[cic].tx_cfga.gpo = 0X3FF; /* All GPIO pull down off */
	}
	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
	adBms6830_Adax(AUX_OPEN_WIRE_DETECTION, OPEN_WIRE_CURRENT_SOURCE,
			AUX_CH_TO_CONVERT);
	pladc_count = adBmsPollAdc(PLADC);
	printf("Aux voltage conversion completed\n");

	printPollAdcConvTime(pladc_count);
}

/**
 *******************************************************************************
 * @brief Read AUX, VMV, V+ Voltages
 *******************************************************************************
 */
void adBms6830_read_aux_voltages(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBmsReadData(tIC, &ic[0], RDAUXA, Aux, A);
	adBmsReadData(tIC, &ic[0], RDAUXB, Aux, B);
	adBmsReadData(tIC, &ic[0], RDAUXC, Aux, C);
	adBmsReadData(tIC, &ic[0], RDAUXD, Aux, D);
	printVoltages(tIC, &ic[0], Aux);
}

/**
 *******************************************************************************
 * @brief Start Redundant GPIO Voltages Measurement
 *******************************************************************************
 */
void adBms6830_start_raux_voltage_measurment(uint8_t tIC, cell_asic *ic) {
	for (uint8_t cic = 0; cic < tIC; cic++) {
		/* Init config A */
		ic[cic].tx_cfga.refon = PWR_UP;
		ic[cic].tx_cfga.gpo = 0X3FF; /* All GPIO pull down off */
	}
	adBmsWakeupIc(tIC);
	adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
	adBms6830_Adax2(AUX_CH_TO_CONVERT);
	pladc_count = adBmsPollAdc(PLADC);
	printf("RAux voltage conversion completed\n");

	printPollAdcConvTime(pladc_count);
	HAL_Delay(10);
}

/**
 *******************************************************************************
 * @brief Read Redundant GPIO Voltages
 *******************************************************************************
 */
void adBms6830_read_raux_voltages(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);
	adBmsReadData(tIC, &ic[0], RDRAXA, RAux, A);
	adBmsReadData(tIC, &ic[0], RDRAXB, RAux, B);
	adBmsReadData(tIC, &ic[0], RDRAXC, RAux, C);
	adBmsReadData(tIC, &ic[0], RDRAXD, RAux, D);
	printVoltages(tIC, &ic[0], RAux);
}

/**
 *******************************************************************************
 * @brief Read Status Reg. A, B, C, D and E.
 *******************************************************************************
 */
void adBms6830_read_status_registers(uint8_t tIC, cell_asic *ic) {
	adBmsWakeupIc(tIC);

	adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);

	adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);

	adBms6830_Adax(AUX_OPEN_WIRE_DETECTION, OPEN_WIRE_CURRENT_SOURCE,
			AUX_CH_TO_CONVERT);

	pladc_count = adBmsPollAdc(PLADC);
	adBms6830_Adcv(REDUNDANT_MEASUREMENT, CONTINUOUS_MEASUREMENT,
			DISCHARGE_PERMITTED, RESET_FILTER, CELL_OPEN_WIRE_DETECTION);
	pladc_count = pladc_count + adBmsPollAdc(PLADC);

	adBmsReadData(tIC, &ic[0], RDSTATA, Status, A);

	adBmsReadData(tIC, &ic[0], RDSTATB, Status, B);

	adBmsWakeupIc(tIC);

	adBmsReadData(tIC, &ic[0], RDSTATC, Status, C);

	adBmsReadData(tIC, &ic[0], RDSTATD, Status, D);

	adBmsReadData(tIC, &ic[0], RDSTATE, Status, E);
//printPollAdcConvTime(pladc_count);
//printStatus(tIC, &ic[0], Status, ALL_GRP);
}

/**
 *******************************************************************************
 * @brief Clear Cell measurement reg.
 *******************************************************************************
 */
void adBms6830_clear_cell_measurement(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	spiSendCmd(CLRCELL);

	printf("Cell Registers Cleared\n\n");
}

/**
 *******************************************************************************
 * @brief Clear Aux measurement reg.
 *******************************************************************************
 */
void adBms6830_clear_aux_measurement(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	spiSendCmd(CLRAUX);

	printf("Aux Registers Cleared\n\n");
}

/**
 *******************************************************************************
 * @brief Clear spin measurement reg.
 *******************************************************************************
 */
void adBms6830_clear_spin_measurement(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	spiSendCmd(CLRSPIN);

	printf("Spin Registers Cleared\n\n");
}

/**
 *******************************************************************************
 * @brief Clear fcell measurement reg.
 *******************************************************************************
 */
void adBms6830_clear_fcell_measurement(uint8_t tIC) {
	adBmsWakeupIc(tIC);
	spiSendCmd(CLRFC);

	printf("Fcell Registers Cleared\n\n");
}

/**
 *******************************************************************************
 * Function: spiSendCmd
 * @brief Send command in spi line
 *
 * @details This function send bms command in spi line
 *
 * Parameters:
 * @param [in]	tx_cmd	Tx command bytes
 *
 * @return None
 *
 *******************************************************************************
 */
void spiSendCmd(uint8_t tx_cmd[2]) {
	uint8_t cmd[4];
	uint16_t cmd_pec;
	cmd[0] = tx_cmd[0];
	cmd[1] = tx_cmd[1];
	cmd_pec = Pec15_Calc(2, cmd);
	cmd[2] = (uint8_t) (cmd_pec >> 8);
	cmd[3] = (uint8_t) (cmd_pec);
	adBmsCsLow();
	HAL_SPI_Transmit(hspi, &cmd[0], 4, SPI_TIME_OUT);
	adBmsCsHigh();
}
/**
 *******************************************************************************
 * Function: printWriteConfig
 * @brief Print write config A/B result.
 *
 * @details This function Print write config result into terminal.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  *IC      cell_asic stucture pointer
 *
 * @param [in]  type     Enum type of resistor
 *
 * @param [in]  grp      Enum type of resistor group
 *
 * @return None
 *
 *******************************************************************************
 */

void printWriteConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp) {
	for (uint8_t ic = 0; ic < tIC; ic++) {
		printf("IC%d:\n", (ic + 1));
		if (type == Config) {
			if (grp == A) {
				printf("Write Config A:\n");
				printf("0x%X, ", IC[ic].configa.tx_data[0]);
				printf("0x%X, ", IC[ic].configa.tx_data[1]);
				printf("0x%X, ", IC[ic].configa.tx_data[2]);
				printf("0x%X, ", IC[ic].configa.tx_data[3]);
				printf("0x%X, ", IC[ic].configa.tx_data[4]);
				printf("0x%X\n\n", IC[ic].configa.tx_data[5]);
			} else if (grp == B) {
				printf("Write Config B:\n");
				printf("0x%X, ", IC[ic].configb.tx_data[0]);
				printf("0x%X, ", IC[ic].configb.tx_data[1]);
				printf("0x%X, ", IC[ic].configb.tx_data[2]);
				printf("0x%X, ", IC[ic].configb.tx_data[3]);
				printf("0x%X, ", IC[ic].configb.tx_data[4]);
				printf("0x%X\n\n", IC[ic].configb.tx_data[5]);
			} else if (grp == ALL_GRP) {
				printf("Write Config A:\n");
				printf("0x%X, ", IC[ic].configa.tx_data[0]);
				printf("0x%X, ", IC[ic].configa.tx_data[1]);
				printf("0x%X, ", IC[ic].configa.tx_data[2]);
				printf("0x%X, ", IC[ic].configa.tx_data[3]);
				printf("0x%X, ", IC[ic].configa.tx_data[4]);
				printf("0x%X\n\n", IC[ic].configa.tx_data[5]);

				printf("Write Config B:\n");
				printf("0x%X, ", IC[ic].configb.tx_data[0]);
				printf("0x%X, ", IC[ic].configb.tx_data[1]);
				printf("0x%X, ", IC[ic].configb.tx_data[2]);
				printf("0x%X, ", IC[ic].configb.tx_data[3]);
				printf("0x%X, ", IC[ic].configb.tx_data[4]);
				printf("0x%X\n\n", IC[ic].configb.tx_data[5]);
			} else {
				printf("Wrong Register Group Select\n");
			}
		}
	}
}

/**
 *******************************************************************************
 * Function: printReadConfig
 * @brief Print read config result.
 *
 * @details This function Print read config result into terminal.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  *IC      cell_asic stucture pointer
 *
 * @param [in]  type     Enum type of resistor
 *
 * @param [in]  grp      Enum type of resistor group
 *
 * @return None
 *
 *******************************************************************************
 */
void printReadConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp) {
	for (uint8_t ic = 0; ic < tIC; ic++) {
		printf("IC%d:\n", (ic + 1));
		if (type == Config) {
			if (grp == A) {
				printf("Read Config A:\n");
				printf("REFON:0x%X, ", IC[ic].rx_cfga.refon);
				printf("CTH:0x%X\n", IC[ic].rx_cfga.cth & 0x07);
				printf("FLAG_D[0]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x01));
				printf("FLAG_D[1]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x02) >> 1);
				printf("FLAG_D[2]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x04) >> 2);
				printf("FLAG_D[3]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x08) >> 3);
				printf("FLAG_D[4]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x10) >> 4);
				printf("FLAG_D[5]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x20) >> 5);
				printf("FLAG_D[6]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x40) >> 6);
				printf("FLAG_D[7]:0x%X\n", (IC[ic].rx_cfga.flag_d & 0x80) >> 7);
				printf("OWA[2:0]:0x%X, ", (IC[ic].rx_cfga.owa));
				printf("OWRNG:0x%X, ", (IC[ic].rx_cfga.owrng));
				printf("SOAKON:0x%X, ", (IC[ic].rx_cfga.soakon));
				printf("GPO:0x%X, ", (IC[ic].rx_cfga.gpo));
				printf("FC:0x%X, ", (IC[ic].rx_cfga.fc));
				printf("COMM_BK:0x%X, ", (IC[ic].rx_cfga.comm_bk));
				printf("MUTE_ST:0x%X, ", (IC[ic].rx_cfga.mute_st));
				printf("SNAP:0x%X\n\n", (IC[ic].rx_cfga.snap));
				printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
				printf("PECError:%d\n\n", IC[ic].cccrc.cfgr_pec);
			} else if (grp == B) {
				printf("Read Config B:\n");
				printf("VUV:0x%X, ", IC[ic].rx_cfgb.vuv);
				printf("VOV:0x%X, ", IC[ic].rx_cfgb.vov);
				printf("DCTO:0x%X, ", IC[ic].rx_cfgb.dcto);
				printf("DTRNG:0x%X, ", IC[ic].rx_cfgb.dtrng);
				printf("DTMEN:0x%X, ", IC[ic].rx_cfgb.dtmen);
				printf("DCC:0x%X\n\n", IC[ic].rx_cfgb.dcc);
				printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
				printf("PECError:%d\n\n", IC[ic].cccrc.cfgr_pec);
			} else if (grp == ALL_GRP) {
				printf("Read Config A:\n");
				printf("REFON:0x%X, ", IC[ic].rx_cfga.refon);
				printf("CTH:0x%X\n", IC[ic].rx_cfga.cth & 0x07);
				printf("FLAG_D[0]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x01));
				printf("FLAG_D[1]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x02) >> 1);
				printf("FLAG_D[2]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x04) >> 2);
				printf("FLAG_D[3]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x08) >> 3);
				printf("FLAG_D[4]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x10) >> 4);
				printf("FLAG_D[5]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x20) >> 5);
				printf("FLAG_D[6]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x40) >> 6);
				printf("FLAG_D[7]:0x%X\n", (IC[ic].rx_cfga.flag_d & 0x80) >> 7);
				printf("OWA[2:0]:0x%X, ", (IC[ic].rx_cfga.owa));
				printf("OWRNG:0x%X, ", (IC[ic].rx_cfga.owrng));
				printf("SOAKON:0x%X, ", (IC[ic].rx_cfga.soakon));
				printf("GPO:0x%X, ", (IC[ic].rx_cfga.gpo));
				printf("FC:0x%X, ", (IC[ic].rx_cfga.fc));
				printf("COMM_BK:0x%X, ", (IC[ic].rx_cfga.comm_bk));
				printf("MUTE_ST:0x%X, ", (IC[ic].rx_cfga.mute_st));
				printf("SNAP:0x%X\n\n", (IC[ic].rx_cfga.snap));

				printf("Read Config B:\n");
				printf("VUV:0x%X, ", IC[ic].rx_cfgb.vuv);
				printf("VOV:0x%X, ", IC[ic].rx_cfgb.vov);
				printf("DCTO:0x%X, ", IC[ic].rx_cfgb.dcto);
				printf("DTRNG:0x%X, ", IC[ic].rx_cfgb.dtrng);
				printf("DTMEN:0x%X, ", IC[ic].rx_cfgb.dtmen);
				printf("DCC:0x%X\n\n", IC[ic].rx_cfgb.dcc);
				printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
				printf("PECError:%d\n\n", IC[ic].cccrc.cfgr_pec);
			} else {
				printf("Wrong Register Group Select\n");
			}
		}
	}
}

/**
 *******************************************************************************
 * Function: printVoltages
 * @brief Print Voltages.
 *
 * @details This function Print Voltages into IAR I/O terminal.
 *
 * Parameters:
 * @param [in]	tIC    Total IC
 *
 * @param [in]  *IC    cell_asic stucture pointer
 *
 * @param [in]  type    Enum type of resistor group
 *
 * @return None
 *
 *******************************************************************************
 */
void printVoltages(uint8_t tIC, cell_asic *IC, TYPE type) {
	float voltage;
	int16_t temp;
	uint8_t channel;
	if ((type == Cell) || (type == AvgCell) || (type == F_volt)
			|| (type == S_volt)) {
		channel = CELL;
	} else if (type == Aux) {
		channel = AUX;
	} else if (type == RAux) {
		channel = RAUX;
	}
	for (uint8_t ic = 0; ic < tIC; ic++) {
		printf("IC%d:", (ic + 1));
		for (uint8_t index = 0; index < channel; index++) {
			if (type == Cell) {
				temp = IC[ic].cell.c_codes[index];
			} else if (type == AvgCell) {
				temp = IC[ic].acell.ac_codes[index];
			} else if (type == F_volt) {
				temp = IC[ic].fcell.fc_codes[index];
			} else if (type == S_volt) {
				temp = IC[ic].scell.sc_codes[index];
			} else if (type == Aux) {
				temp = IC[ic].aux.a_codes[index];
			} else if (type == RAux) {
				temp = IC[ic].raux.ra_codes[index];
			}
			voltage = getVoltage(temp);
			if (type == Cell) {
				printf("C%d=%fV,", (index + 1), voltage);
				if (index == (channel - 1)) {
					printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
					printf("PECError:%d", IC[ic].cccrc.cell_pec);
				}
			} else if (type == AvgCell) {
				printf("AC%d=%fV,", (index + 1), voltage);
				if (index == (channel - 1)) {
					printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
					printf("PECError:%d", IC[ic].cccrc.acell_pec);
				}
			} else if (type == F_volt) {
				printf("FC%d=%fV,", (index + 1), voltage);
				if (index == (channel - 1)) {
					printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
					printf("PECError:%d", IC[ic].cccrc.fcell_pec);
				}
			} else if (type == S_volt) {
				printf("S%d=%fV,", (index + 1), voltage);
				if (index == (channel - 1)) {
					printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
					printf("PECError:%d", IC[ic].cccrc.scell_pec);
				}
			} else if (type == Aux) {
				if (index <= 9) {
					printf("AUX%d=%fV,", (index + 1), voltage);
				} else if (index == 10) {
					printf("VMV:%fV,", (20 * voltage));
				} else if (index == 11) {
					printf("V+:%fV,", (20 * voltage));
					printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
					printf("PECError:%d", IC[ic].cccrc.aux_pec);
				}
			} else if (type == RAux) {
				printf("RAUX%d=%fV,", (index + 1), voltage);
				if (index == (channel - 1)) {
					printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
					printf("PECError:%d", IC[ic].cccrc.raux_pec);
				}
			} else {
				printf("Wrong Register Group Select\n");
			}
		}
		printf("\n\n");
	}
}

/**
 *******************************************************************************
 * Function: PrintStatus
 * @brief Print status reg. result.
 *
 * @details This function Print status result into IAR I/O terminal.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  *IC      cell_asic stucture pointer
 *
 * @param [in]  type     Enum type of resistor
 *
 * @param [in]  grp      Enum type of resistor group
 *
 * @return None
 *
 *******************************************************************************
 */
void printStatus(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp) {
	float voltage;
	for (uint8_t ic = 0; ic < tIC; ic++) {
		printf("IC%d:\n", (ic + 1));
		if (type == Status) {
			if (grp == A) {
				printf("Status A:\n");
				voltage = getVoltage(IC[ic].stata.vref2);
				printf("VREF2:%fV, ", voltage);
				voltage = getVoltage(IC[ic].stata.vref3);
				printf("VREF3:%fV, ", voltage);
				voltage = getVoltage(IC[ic].stata.itmp);
				printf("ITMP:%f�C\n", (voltage / 0.0075) - 273);

				printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
				printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
			} else if (grp == B) {
				printf("Status B:\n");
				voltage = getVoltage(IC[ic].statb.va);
				printf("VA:%fV, ", voltage);
				voltage = getVoltage(IC[ic].statb.vd);
				printf("VD:%fV, ", voltage);
				voltage = getVoltage(IC[ic].statb.vr4k);
				printf("VR4K:%fV\n", voltage);

				printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
				printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
			} else if (grp == C) {
				printf("Status C:\n");
				printf("CSFLT:0x%X, ", IC[ic].statc.cs_flt);

				printf("OTP2_MED:0x%X, ", IC[ic].statc.otp2_med);
				printf("OTP2_ED:0x%X, ", IC[ic].statc.otp2_ed);
				printf("OTP1_MED:0x%X ", IC[ic].statc.otp1_med);
				printf("OTP1_ED:0x%X, ", IC[ic].statc.otp1_ed);
				printf("VD_UV:0x%X, ", IC[ic].statc.vd_uv);
				printf("VD_OV:0x%X, ", IC[ic].statc.vd_ov);
				printf("VA_UV:0x%X, ", IC[ic].statc.va_uv);
				printf("VA_OV:0x%X\n", IC[ic].statc.va_ov);

				printf("OSCCHK:0x%X, ", IC[ic].statc.oscchk);
				printf("TMODCHK:0x%X, ", IC[ic].statc.tmodchk);
				printf("THSD:0x%X, ", IC[ic].statc.thsd);
				printf("SLEEP:0x%X, ", IC[ic].statc.sleep);
				printf("SPIFLT:0x%X, ", IC[ic].statc.spiflt);
				printf("COMP:0x%X, ", IC[ic].statc.comp);
				printf("VDEL:0x%X, ", IC[ic].statc.vdel);
				printf("VDE:0x%X\n", IC[ic].statc.vde);

				printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
				printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
			} else if (grp == D) {
				printf("Status D:\n");
				printf("C1UV:0x%X, ", IC[ic].statd.c_uv[0]);
				printf("C2UV:0x%X, ", IC[ic].statd.c_uv[1]);
				printf("C3UV:0x%X, ", IC[ic].statd.c_uv[2]);
				printf("C4UV:0x%X, ", IC[ic].statd.c_uv[3]);
				printf("C5UV:0x%X, ", IC[ic].statd.c_uv[4]);
				printf("C6UV:0x%X, ", IC[ic].statd.c_uv[5]);
				printf("C7UV:0x%X, ", IC[ic].statd.c_uv[6]);
				printf("C8UV:0x%X, ", IC[ic].statd.c_uv[7]);
				printf("C9UV:0x%X, ", IC[ic].statd.c_uv[8]);
				printf("C10UV:0x%X, ", IC[ic].statd.c_uv[9]);
				printf("C11UV:0x%X, ", IC[ic].statd.c_uv[10]);
				printf("C12UV:0x%X, ", IC[ic].statd.c_uv[11]);
				printf("C13UV:0x%X, ", IC[ic].statd.c_uv[12]);
				printf("C14UV:0x%X, ", IC[ic].statd.c_uv[13]);
				printf("C15UV:0x%X, ", IC[ic].statd.c_uv[14]);
				printf("C16UV:0x%X\n", IC[ic].statd.c_uv[15]);

				printf("C1OV:0x%X, ", IC[ic].statd.c_ov[0]);
				printf("C2OV:0x%X, ", IC[ic].statd.c_ov[1]);
				printf("C3OV:0x%X, ", IC[ic].statd.c_ov[2]);
				printf("C4OV:0x%X, ", IC[ic].statd.c_ov[3]);
				printf("C5OV:0x%X, ", IC[ic].statd.c_ov[4]);
				printf("C6OV:0x%X, ", IC[ic].statd.c_ov[5]);
				printf("C7OV:0x%X, ", IC[ic].statd.c_ov[6]);
				printf("C8OV:0x%X, ", IC[ic].statd.c_ov[7]);
				printf("C9OV:0x%X, ", IC[ic].statd.c_ov[8]);
				printf("C10OV:0x%X, ", IC[ic].statd.c_ov[9]);
				printf("C11OV:0x%X, ", IC[ic].statd.c_ov[10]);
				printf("C12OV:0x%X, ", IC[ic].statd.c_ov[11]);
				printf("C13OV:0x%X, ", IC[ic].statd.c_ov[12]);
				printf("C14OV:0x%X, ", IC[ic].statd.c_ov[13]);
				printf("C15OV:0x%X, ", IC[ic].statd.c_ov[14]);
				printf("C16OV:0x%X\n", IC[ic].statd.c_ov[15]);

				printf("CTS:0x%X, ", IC[ic].statd.cts);
				printf("CT:0x%X, ", IC[ic].statd.ct);
				printf("OC_CNTR:0x%X\n", IC[ic].statd.oc_cntr);

				printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
				printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
			} else if (grp == E) {
				printf("Status E:\n");
				printf("GPI:0x%X, ", IC[ic].state.gpi);
				printf("REV_ID:0x%X\n", IC[ic].state.rev);

				printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
				printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
			} else if (grp == ALL_GRP) {
				printf("Status A:\n");
				voltage = getVoltage(IC[ic].stata.vref2);
				printf("VREF2:%fV, ", voltage);
				voltage = getVoltage(IC[ic].stata.vref3);
				printf("VREF3:%fV, ", voltage);
				voltage = getVoltage(IC[ic].stata.itmp);
				printf("ITMP:%f�C\n\n", (voltage / 0.0075) - 273);

				printf("Status B:\n");
				voltage = getVoltage(IC[ic].statb.va);
				printf("VA:%fV, ", voltage);
				voltage = getVoltage(IC[ic].statb.vd);
				printf("VD:%fV, ", voltage);
				voltage = getVoltage(IC[ic].statb.vr4k);
				printf("VR4K:%fV\n\n", voltage);

				printf("Status C:\n");
				printf("CSFLT:0x%X, ", IC[ic].statc.cs_flt);

				printf("OTP2_MED:0x%X, ", IC[ic].statc.otp2_med);
				printf("OTP2_ED:0x%X, ", IC[ic].statc.otp2_ed);
				printf("OTP1_MED:0x%X, ", IC[ic].statc.otp1_med);
				printf("OTP1_ED:0x%X, ", IC[ic].statc.otp1_ed);
				printf("VD_UV:0x%X, ", IC[ic].statc.vd_uv);
				printf("VD_OV:0x%X, ", IC[ic].statc.vd_ov);
				printf("VA_UV:0x%X, ", IC[ic].statc.va_uv);
				printf("VA_OV:0x%X\n", IC[ic].statc.va_ov);

				printf("OSCCHK:0x%X, ", IC[ic].statc.oscchk);
				printf("TMODCHK:0x%X, ", IC[ic].statc.tmodchk);
				printf("THSD:0x%X, ", IC[ic].statc.thsd);
				printf("SLEEP:0x%X, ", IC[ic].statc.sleep);
				printf("SPIFLT:0x%X, ", IC[ic].statc.spiflt);
				printf("COMP:0x%X, ", IC[ic].statc.comp);
				printf("VDEL:0x%X, ", IC[ic].statc.vdel);
				printf("VDE:0x%X\n\n", IC[ic].statc.vde);

				printf("Status D:\n");
				printf("C1UV:0x%X, ", IC[ic].statd.c_uv[0]);
				printf("C2UV:0x%X, ", IC[ic].statd.c_uv[1]);
				printf("C3UV:0x%X, ", IC[ic].statd.c_uv[2]);
				printf("C4UV:0x%X, ", IC[ic].statd.c_uv[3]);
				printf("C5UV:0x%X, ", IC[ic].statd.c_uv[4]);
				printf("C6UV:0x%X, ", IC[ic].statd.c_uv[5]);
				printf("C7UV:0x%X, ", IC[ic].statd.c_uv[6]);
				printf("C8UV:0x%X, ", IC[ic].statd.c_uv[7]);
				printf("C9UV:0x%X, ", IC[ic].statd.c_uv[8]);
				printf("C10UV:0x%X, ", IC[ic].statd.c_uv[9]);
				printf("C11UV:0x%X, ", IC[ic].statd.c_uv[10]);
				printf("C12UV:0x%X, ", IC[ic].statd.c_uv[11]);
				printf("C13UV:0x%X, ", IC[ic].statd.c_uv[12]);
				printf("C14UV:0x%X, ", IC[ic].statd.c_uv[13]);
				printf("C15UV:0x%X, ", IC[ic].statd.c_uv[14]);
				printf("C16UV:0x%X\n", IC[ic].statd.c_uv[15]);

				printf("C1OV:0x%X, ", IC[ic].statd.c_ov[0]);
				printf("C2OV:0x%X, ", IC[ic].statd.c_ov[1]);
				printf("C3OV:0x%X, ", IC[ic].statd.c_ov[2]);
				printf("C4OV:0x%X, ", IC[ic].statd.c_ov[3]);
				printf("C5OV:0x%X, ", IC[ic].statd.c_ov[4]);
				printf("C6OV:0x%X, ", IC[ic].statd.c_ov[5]);
				printf("C7OV:0x%X, ", IC[ic].statd.c_ov[6]);
				printf("C8OV:0x%X, ", IC[ic].statd.c_ov[7]);
				printf("C9OV:0x%X, ", IC[ic].statd.c_ov[8]);
				printf("C10OV:0x%X, ", IC[ic].statd.c_ov[9]);
				printf("C11OV:0x%X, ", IC[ic].statd.c_ov[10]);
				printf("C12OV:0x%X, ", IC[ic].statd.c_ov[11]);
				printf("C13OV:0x%X, ", IC[ic].statd.c_ov[12]);
				printf("C14OV:0x%X, ", IC[ic].statd.c_ov[13]);
				printf("C15OV:0x%X, ", IC[ic].statd.c_ov[14]);
				printf("C16OV:0x%X\n", IC[ic].statd.c_ov[15]);

				printf("CTS:0x%X, ", IC[ic].statd.cts);
				printf("CT:0x%X\n\n", IC[ic].statd.ct);

				printf("Status E:\n");
				printf("GPI:0x%X, ", IC[ic].state.gpi);
				printf("REV_ID:0x%X\n\n", IC[ic].state.rev);

				printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
				printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
			} else {
				printf("Wrong Register Group Select\n");
			}
		}
	}
}

/**
 *******************************************************************************
 * Function: PrintDeviceSID
 * @brief Print Device SID.
 *
 * @details This function Print Device SID into IAR I/O terminal.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  *IC      cell_asic stucture pointer
 *
 * @param [in]  type     Enum type of resistor
 *
 * @return None
 *
 *******************************************************************************
 */
void printDeviceSID(uint8_t tIC, cell_asic *IC, TYPE type) {
	for (uint8_t ic = 0; ic < tIC; ic++) {
		printf("IC%d:\n", (ic + 1));
		if (type == Sid) {
			printf("Read Device SID:\n");
			printf("0x%X, ", IC[ic].sid.sid[0]);
			printf("0x%X, ", IC[ic].sid.sid[1]);
			printf("0x%X, ", IC[ic].sid.sid[2]);
			printf("0x%X, ", IC[ic].sid.sid[3]);
			printf("0x%X, ", IC[ic].sid.sid[4]);
			printf("0x%X, ", IC[ic].sid.sid[5]);
			printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
			printf("PECError:%d\n\n", IC[ic].cccrc.sid_pec);
		} else {
			printf("Wrong Register Type Select\n");
		}
	}
}

/**
 *******************************************************************************
 * Function: printWritePwmDutyCycle
 * @brief Print Write Pwm Duty Cycle.
 *
 * @details This function Print write pwm duty cycle value.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  *IC      cell_asic stucture pointer
 *
 * @param [in]  type     Enum type of resistor
 *
 * @param [in]  grp      Enum group of resistor
 *
 * @return None
 *
 *******************************************************************************
 */
void printWritePwmDutyCycle(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp) {
	for (uint8_t ic = 0; ic < tIC; ic++) {
		printf("IC%d:\n", (ic + 1));
		if (grp == A) {
			printf("Write Pwma Duty Cycle:\n");
			printf("0x%X, ", IC[ic].pwma.tx_data[0]);
			printf("0x%X, ", IC[ic].pwma.tx_data[1]);
			printf("0x%X, ", IC[ic].pwma.tx_data[2]);
			printf("0x%X, ", IC[ic].pwma.tx_data[3]);
			printf("0x%X, ", IC[ic].pwma.tx_data[4]);
			printf("0x%X\n\n", IC[ic].pwma.tx_data[5]);
		} else if (grp == B) {
			printf("Write Pwmb Duty Cycle:\n");
			printf("0x%X, ", IC[ic].pwmb.tx_data[0]);
			printf("0x%X\n\n", IC[ic].pwmb.tx_data[1]);
		} else if (grp == ALL_GRP) {
			printf("Write Pwma Duty Cycle:\n");
			printf("0x%X, ", IC[ic].pwma.tx_data[0]);
			printf("0x%X, ", IC[ic].pwma.tx_data[1]);
			printf("0x%X, ", IC[ic].pwma.tx_data[2]);
			printf("0x%X, ", IC[ic].pwma.tx_data[3]);
			printf("0x%X, ", IC[ic].pwma.tx_data[4]);
			printf("0x%X\n", IC[ic].pwma.tx_data[5]);

			printf("Write Pwmb Duty Cycle:\n");
			printf("0x%X, ", IC[ic].pwmb.tx_data[0]);
			printf("0x%X\n\n", IC[ic].pwmb.tx_data[1]);
		} else {
			printf("Wrong Register Group Select\n");
		}
	}
}

/**
 *******************************************************************************
 * Function: printReadPwmDutyCycle
 * @brief Print Read Pwm Duty Cycle.
 *
 * @details This function print read pwm duty cycle value.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  *IC      cell_asic stucture pointer
 *
 * @param [in]  type     Enum type of resistor
 *
 * @param [in]  grp      Enum group of resistor
 *
 * @return None
 *
 *******************************************************************************
 */
void printReadPwmDutyCycle(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp) {
	for (uint8_t ic = 0; ic < tIC; ic++) {
		printf("IC%d:\n", (ic + 1));
		if (grp == A) {
			printf("Read PWMA Duty Cycle:\n");
			printf("PWM1:0x%X, ", IC[ic].PwmA.pwma[0]);
			printf("PWM2:0x%X, ", IC[ic].PwmA.pwma[1]);
			printf("PWM3:0x%X, ", IC[ic].PwmA.pwma[2]);
			printf("PWM4:0x%X, ", IC[ic].PwmA.pwma[3]);
			printf("PWM5:0x%X, ", IC[ic].PwmA.pwma[4]);
			printf("PWM6:0x%X, ", IC[ic].PwmA.pwma[5]);
			printf("PWM7:0x%X, ", IC[ic].PwmA.pwma[6]);
			printf("PWM8:0x%X, ", IC[ic].PwmA.pwma[7]);
			printf("PWM9:0x%X, ", IC[ic].PwmA.pwma[8]);
			printf("PWM10:0x%X, ", IC[ic].PwmA.pwma[9]);
			printf("PWM11:0x%X, ", IC[ic].PwmA.pwma[10]);
			printf("PWM12:0x%X, ", IC[ic].PwmA.pwma[11]);
			printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
			printf("PECError:%d\n\n", IC[ic].cccrc.pwm_pec);
		} else if (grp == B) {
			printf("Read PWMB Duty Cycle:\n");
			printf("PWM13:0x%X, ", IC[ic].PwmB.pwmb[0]);
			printf("PWM14:0x%X, ", IC[ic].PwmB.pwmb[1]);
			printf("PWM15:0x%X, ", IC[ic].PwmB.pwmb[2]);
			printf("PWM16:0x%X, ", IC[ic].PwmB.pwmb[3]);
			printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
			printf("PECError:%d\n\n", IC[ic].cccrc.pwm_pec);
		} else if (grp == ALL_GRP) {
			printf("Read PWMA Duty Cycle:\n");
			printf("PWM1:0x%X, ", IC[ic].PwmA.pwma[0]);
			printf("PWM2:0x%X, ", IC[ic].PwmA.pwma[1]);
			printf("PWM3:0x%X, ", IC[ic].PwmA.pwma[2]);
			printf("PWM4:0x%X, ", IC[ic].PwmA.pwma[3]);
			printf("PWM5:0x%X, ", IC[ic].PwmA.pwma[4]);
			printf("PWM6:0x%X, ", IC[ic].PwmA.pwma[5]);
			printf("PWM7:0x%X, ", IC[ic].PwmA.pwma[6]);
			printf("PWM8:0x%X, ", IC[ic].PwmA.pwma[7]);
			printf("PWM9:0x%X, ", IC[ic].PwmA.pwma[8]);
			printf("PWM10:0x%X, ", IC[ic].PwmA.pwma[9]);
			printf("PWM11:0x%X, ", IC[ic].PwmA.pwma[10]);
			printf("PWM12:0x%X\n", IC[ic].PwmA.pwma[11]);

			printf("Read PWMB Duty Cycle:\n");
			printf("PWM13:0x%X, ", IC[ic].PwmB.pwmb[0]);
			printf("PWM14:0x%X, ", IC[ic].PwmB.pwmb[1]);
			printf("PWM15:0x%X, ", IC[ic].PwmB.pwmb[2]);
			printf("PWM16:0x%X, ", IC[ic].PwmB.pwmb[3]);
			printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
			printf("PECError:%d\n\n", IC[ic].cccrc.pwm_pec);
		} else {
			printf("Wrong Register Type Select\n");
		}
	}
}

/**
 *******************************************************************************
 * Function: printWriteCommData
 * @brief Print Write Comm data.
 *
 * @details This function Print write comm data.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  *IC      cell_asic stucture pointer
 *
 * @param [in]  type     Enum type of resistor
 *
 * @return None
 *
 *******************************************************************************
 */
void printWriteCommData(uint8_t tIC, cell_asic *IC, TYPE type) {
	for (uint8_t ic = 0; ic < tIC; ic++) {
		printf("IC%d:\n", (ic + 1));
		if (type == Comm) {
			printf("Write Comm Data:\n");
			printf("0x%X, ", IC[ic].com.tx_data[0]);
			printf("0x%X, ", IC[ic].com.tx_data[1]);
			printf("0x%X, ", IC[ic].com.tx_data[2]);
			printf("0x%X, ", IC[ic].com.tx_data[3]);
			printf("0x%X, ", IC[ic].com.tx_data[4]);
			printf("0x%X\n\n", IC[ic].com.tx_data[5]);
		} else {
			printf("Wrong Register Group Select\n");
		}
	}
}

/**
 *******************************************************************************
 * Function: printReadCommData
 * @brief Print Read Comm Data.
 *
 * @details This function print read comm data.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  *IC      cell_asic stucture pointer
 *
 * @param [in]  type     Enum type of resistor
 *
 * @return None
 *
 *******************************************************************************
 */
void printReadCommData(uint8_t tIC, cell_asic *IC, TYPE type) {
	for (uint8_t ic = 0; ic < tIC; ic++) {
		printf("IC%d:\n", (ic + 1));
		if (type == Comm) {
			printf("Read Comm Data:\n");
			printf("ICOM0:0x%X, ", IC[ic].comm.icomm[0]);
			printf("ICOM1:0x%X, ", IC[ic].comm.icomm[1]);
			printf("ICOM2:0x%X\n", IC[ic].comm.icomm[2]);
			printf("FCOM0:0x%X, ", IC[ic].comm.fcomm[0]);
			printf("FCOM1:0x%X, ", IC[ic].comm.fcomm[1]);
			printf("FCOM2:0x%X\n", IC[ic].comm.fcomm[2]);
			printf("DATA0:0x%X, ", IC[ic].comm.data[0]);
			printf("DATA1:0x%X, ", IC[ic].comm.data[1]);
			printf("DATA2:0x%X\n", IC[ic].comm.data[2]);
			printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
			printf("PECError:%d\n\n", IC[ic].cccrc.comm_pec);
		} else {
			printf("Wrong Register Type Select\n");
		}
	}
}

/**
 *******************************************************************************
 * Function: printDiagnosticTestResult
 * @brief Print diagnostic test result.
 *
 * @details This function Print diagnostic test result (PASS,FAIL) into console terminal.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  *IC      cell_asic stucture pointer
 *
 * @param [in]  TEST     Enum type diagnostic test
 *
 * @return None
 *
 *******************************************************************************
 */
void printDiagnosticTestResult(uint8_t tIC, cell_asic *IC,
		DIAGNOSTIC_TYPE type) {
	if (type == OSC_MISMATCH) {
		printf("OSC Diagnostic Test:\n");
		for (uint8_t ic = 0; ic < tIC; ic++) {
			printf("IC%d:", (ic + 1));
			diagnosticTestResultPrint(IC[ic].diag_result.osc_mismatch);
		}
		printf("\n\n");
	}

	else if (type == SUPPLY_ERROR) {
		printf("Force Supply Error Detection Test:\n");
		for (uint8_t ic = 0; ic < tIC; ic++) {
			printf("IC%d:", (ic + 1));
			diagnosticTestResultPrint(IC[ic].diag_result.supply_error);
		}
		printf("\n\n");
	}

	else if (type == THSD) {
		printf("Thsd Diagnostic Test:\n");
		for (uint8_t ic = 0; ic < tIC; ic++) {
			printf("IC%d:", (ic + 1));
			diagnosticTestResultPrint(IC[ic].diag_result.thsd);
		}
		printf("\n\n");
	}

	else if (type == FUSE_ED) {
		printf("Fuse_ed Diagnostic Test:\n");
		for (uint8_t ic = 0; ic < tIC; ic++) {
			printf("IC%d:", (ic + 1));
			diagnosticTestResultPrint(IC[ic].diag_result.fuse_ed);
		}
		printf("\n\n");
	}

	else if (type == FUSE_MED) {
		printf("Fuse_med Diagnostic Test:\n");
		for (uint8_t ic = 0; ic < tIC; ic++) {
			printf("IC%d:", (ic + 1));
			diagnosticTestResultPrint(IC[ic].diag_result.fuse_med);
		}
		printf("\n\n");
	}

	else if (type == TMODCHK) {
		printf("TMODCHK Diagnostic Test:\n");
		for (uint8_t ic = 0; ic < tIC; ic++) {
			printf("IC%d:", (ic + 1));
			diagnosticTestResultPrint(IC[ic].diag_result.tmodchk);
		}
		printf("\n\n");
	} else {
		printf("Wrong Diagnostic Selected\n");
	}
}

/**
 *******************************************************************************
 * Function: diagnosticResultPrint
 * @brief Print diagnostic (PASS/FAIL) result.
 *
 * @details This function print diagnostic (PASS/FAIL) result into console.
 *
 * Parameters:
 * @param [in]	result   Result byte
 *
 * @return None
 *
 *******************************************************************************
 */
void diagnosticTestResultPrint(uint8_t result) {
	if (result == 1) {
		printf("PASS\n");
	} else {
		printf("FAIL\n");
	}
}

/**
 *******************************************************************************
 * Function: printOpenWireTestResult
 * @brief Print open wire test result.
 *
 * @details This function print open wire test result.
 *
 * Parameters:
 * @param [in]	tIC      Total IC
 *
 * @param [in]  *IC      cell_asic stucture pointer
 *
 * @param [in]  type     Enum type of resistor
 *
 * @return None
 *
 *******************************************************************************
 */
void printOpenWireTestResult(uint8_t tIC, cell_asic *IC, TYPE type) {
	if (type == Cell) {
		printf("Cell Open Wire Test\n");
		for (uint8_t ic = 0; ic < tIC; ic++) {
			printf("IC%d:\n", (ic + 1));
			for (uint8_t cell = 0; cell < CELL; cell++) {
				printf("CELL%d:", (cell + 1));
				openWireResultPrint(IC[ic].diag_result.cell_ow[cell]);
			}
			printf("\n\n");
		}
	} else if (type == S_volt) {
		printf("Cell redundant Open Wire Test\n");
		for (uint8_t ic = 0; ic < tIC; ic++) {
			printf("IC%d:\n", (ic + 1));
			for (uint8_t cell = 0; cell < CELL; cell++) {
				printf("CELL%d:", (cell + 1));
				openWireResultPrint(IC[ic].diag_result.cellred_ow[cell]);
			}
			printf("\n\n");
		}
	} else if (type == Aux) {
		printf("Aux Open Wire Test\n");
		for (uint8_t ic = 0; ic < tIC; ic++) {
			printf("IC%d:\n", (ic + 1));
			for (uint8_t gpio = 0; gpio < (AUX - 2); gpio++) {
				printf("GPIO%d:", (gpio + 1));
				openWireResultPrint(IC[ic].diag_result.aux_ow[gpio]);
			}
			printf("\n\n");
		}
	} else {
		printf("Wrong Resistor Type Selected\n");
	}
}

/**
 *******************************************************************************
 * Function: openWireResultPrint
 * @brief Print open wire (OPEN/CLOSE) result.
 *
 * @details This function print open wire result into console.
 *
 * Parameters:
 * @param [in]	result   Result byte
 *
 * @return None
 *
 *******************************************************************************
 */
void openWireResultPrint(uint8_t result) {
	if (result == 1) {
		printf(" OPEN\n");
	} else {
		printf(" CLOSE\n");
	}
}

/**
 *******************************************************************************
 * Function: printPollAdcConvTime
 * @brief Print Poll adc conversion Time.
 *
 * @details This function print poll adc conversion Time.
 *
 * @return None
 *
 *******************************************************************************
 */
void printPollAdcConvTime(int count) {
	printf("Adc Conversion Time = %fms\n", (float) (count / 64000.0));
}

/**
 *******************************************************************************
 * Function: printMenu
 * @brief Print Command Menu.
 *
 * @details This function print all command menu.
 *
 * @return None
 *
 *******************************************************************************
 */
void printMenu() {
	printf("List of ADBMS6830 Command:\n");
	printf("Write and Read Configuration: 1 \n");
	printf("Read Configuration: 2 \n");
	printf("Start Cell Voltage Conversion: 3 \n");
	printf("Read Cell Voltages: 4 \n");
	printf("Start S-Voltage Conversion: 5 \n");
	printf("Read S-Voltages: 6 \n");
	printf("Start Avg Cell Voltage Conversion: 7 \n");
	printf("Read Avg Cell Voltages: 8 \n");
	printf("Start F-Cell Voltage Conversion: 9 \n");
	printf("Read F-Cell Voltages: 10 \n");
	printf("Start Aux Voltage Conversion: 11 \n");
	printf("Read Aux Voltages: 12 \n");
	printf("Start RAux Voltage Conversion: 13 \n");
	printf("Read RAux Voltages: 14 \n");
	printf("Read Status Registers: 15 \n");
	printf("Loop Measurements: 16 \n");
	printf("Clear Cell registers: 17 \n");
	printf("Clear Aux registers: 18 \n");
	printf("Clear Spin registers: 19 \n");
	printf("Clear Fcell registers: 20 \n");

	printf("\n");
	printf("Print '0' for menu\n");
	printf("Please enter command: \n");
	printf("\n\n");

	HAL_Delay(1000);
}

/**
 *******************************************************************************
 * Function: getVoltage
 * @brief Get voltages with multiplication factor.
 *
 * @details This function calculate the voltage.
 *
 * Parameters:
 * @param [in]	data    voltages(uint16_t)
 *
 * @return voltage(float)
 *
 *******************************************************************************
 */
float getVoltage(int data) {
	float voltage_float; //voltage in Volts
	voltage_float = ((data + 10000) * 0.000150);
	return voltage_float;
}
