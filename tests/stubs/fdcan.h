#ifndef TEST_FDCAN_H
#define TEST_FDCAN_H
#include "main.h"
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h, uint32_t flags);
#endif
