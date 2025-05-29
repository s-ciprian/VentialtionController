#ifndef BSW_CNTMAN_H                               /* To avoid double inclusion */
#define BSW_CNTMAN_H

/* ============================= INCLUDES =================================== */
#include "stdint.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* ============================== DEFINES =================================== */
#define B_TIMER_OFF  (0xFFu)         /* 8 bit counter is expired */
#define W_TIMER_OFF  (0xFFFFu)       /* 16 bit counter is expired */
#define L_TIMER_OFF  (0xFFFFFFFFu)   /* 32 bit counter is expired */

#define B_TIMER_NULL (0x00u)         /* 8 bit counter is 0 */
#define W_TIMER_NULL (0x0000u)       /* 16 bit counter is 0 */
#define L_TIMER_NULL (0x00000000u)   /* 32 bit counter is 0 */

/* STOP_COUNTx definition */
#define STOP_COUNT_W(counter)       ((counter) = 0xFFFFu)
#define STOP_COUNT_B(counter)       ((counter) = 0xFFu)
#define STOP_COUNT_L(counter)       ((counter) = 0xFFFFFFFFu)


/* ======================== EXPORTED DATATYPES ============================== */
/*! @brief Counters type definition
 */
typedef  uint8_t     T_COUNT_B;
typedef  uint16_t    T_COUNT_W;
typedef  uint32_t    T_COUNT_L;

/* ======================== EXPORTED FUNCTIONS ============================== */
/*! @brief Decrement counter functions
 */
T_COUNT_B bsw_DecCnt_Byte(T_COUNT_B* const rpu8_address);
T_COUNT_W bsw_DecCnt_Word(T_COUNT_W* const rpu16_address);
T_COUNT_L bsw_DecCnt_Long(T_COUNT_L* const rpu32_address);

/*! @brief Load counter functions
 */
void bsw_LoadCnt_Byte(T_COUNT_B* const rpu8_address, uint8_t value);
void bsw_LoadCnt_Word(T_COUNT_W* const rpu16_address, uint16_t value);
void bsw_LoadCnt_Long(T_COUNT_L* const rpu32_address, uint32_t value);

/*! @brief Get counter functions
 */
T_COUNT_B bsw_GetCnt_Byte(T_COUNT_B const * const rpu8_address);
T_COUNT_W bsw_GetCnt_Word(T_COUNT_W const * const rpu16_address);
T_COUNT_L bsw_GetCnt_Long(T_COUNT_L const * const rpu32_address);


 /* END bsw_cntman */
 /*!
 ** @}
 */
#endif /* BSW_CNTMAN_H */

