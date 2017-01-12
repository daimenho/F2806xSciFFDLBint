/*
 * Example_2806xSci_FFDLB_int.h
 *
 *  Created on: Jan 12, 2017
 *      Author: barrymok
 */

#ifndef CMD_PARSER_H_
#define CMD_PARSER_H_

#define bool Uint8



#define COMMAND_LINE_LENGTH		16

typedef struct{
	volatile bool					start;
	volatile bool					stop;
	volatile bool					ready;
	volatile bool					reset;
	volatile unsigned int			idx;
	volatile unsigned int			type;
	volatile unsigned char *		line[COMMAND_LINE_LENGTH];
	volatile bool					time_timeout_flag;
	volatile unsigned long			time_timeout_count;
	volatile unsigned long			time_count;
}S_CommandParser;

typedef struct{
	volatile float					watts;
	volatile float					va;
	volatile float					var;
	volatile float					vrms;
	volatile float					arms;
}S_PM6000_FNC;


enum TrueFasle {
	FALSE = 0,
	TRUE =1
};



enum PM6000CmdType {
	PM6000_CMD_FNC_WAT = 0,
	PM6000_CMD_FNC_VAS,
	PM6000_CMD_FNC_VAR,
	PM6000_CMD_FNC_VLT,
	PM6000_CMD_FNC_AMP,
	PM6000_CMD_FNC_PWF
};

#endif /* CMD_PARSER_H_ */
