#ifndef FLASH_UTIL_HPP_
#define FLASH_UTIL_HPP_

#include "MazeDef.hpp"
#include "stdint.h"
#include "stdbool.h"

const uint8_t param_data_num=8;

void FlashInit();
uint32_t FlashEraseData();
void FlashGetData(int maze_data[MAZESIZE_X][MAZESIZE_Y],int param_data[param_data_num]);
void FlashSetData(int maze_data[MAZESIZE_X][MAZESIZE_Y],int param_data[param_data_num]);
void FlashPrintMazeData(int data[MAZESIZE_X][MAZESIZE_Y]);
void FlashSetGoalFlag(bool flag);
uint8_t FlashGetGoalFlag();
uint8_t FlashGetGoalX();
uint8_t FlashGetGoalY();

#endif /* FLASH_UTIL_HPP_ */
