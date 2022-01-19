#pragma once

#define HAL_BOARD_NAME "EMPTY"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_MEM_CLASS HAL_MEM_CLASS_192
#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE            16384
#endif
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_NONE
#define HAL_BARO_DEFAULT HAL_BARO_NONE
#define HAL_COMPASS_DEFAULT HAL_COMPASS_NONE
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE

#define HAL_HAVE_BOARD_VOLTAGE 1
#define HAL_HAVE_SERVO_VOLTAGE 0
#define HAL_HAVE_SAFETY_SWITCH 1

#define HAL_Semaphore Empty::Semaphore
