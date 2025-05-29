/* ============================= INCLUDES =================================== */
#include "bsw_cntman.h"


/* ================= EXPORTED FUNCTIONS Implementation ====================== */

/*******************************************************************************
 *  DESCRIPTION  : bsw_DecCnt_Byte
 *  Description  : decrement byte counter
 *  Parameters   : rpu8_addres [Input/Output]
 *  Return       : counter value
 *******************************************************************************/
T_COUNT_B bsw_DecCnt_Byte(T_COUNT_B* const rpu8_address)
{
  if(*rpu8_address != 0xFFu)
  {
    (*rpu8_address)--;
  }

  return(*rpu8_address);
}

/*******************************************************************************
 *  Name                 : bsw_DecCnt_Word
 *  Description          : decrement word (16 bits) counters
 *  Parameters           : rpu16_addres [Input/Output]
 *  Return               : counter value
 *******************************************************************************/
T_COUNT_W bsw_DecCnt_Word(T_COUNT_W* const rpu16_address)
{
  if(*rpu16_address != 0xFFFFu)
  {
    (*rpu16_address)--;
  }

  return(*rpu16_address);
}

/*******************************************************************************
 *  Name                 : bsw_DecCnt_Long
 *  Description          : decrement long (32 bits) counters
 *  Parameters           : rpu32_addres [Input/Output]
 *  Return               : counter value
 *******************************************************************************/
T_COUNT_L bsw_DecCnt_Long(T_COUNT_L* const rpu32_address)
{
  if(*rpu32_address != 0xFFFFFFFFu)
  {
    (*rpu32_address)--;
  }

  return(*rpu32_address);
}

/*******************************************************************************
 *  Name                 : bsw_LoadCnt_Byte
 *  Description          : load byte (8 bits) counters
 *  Parameters           : rpu8_addres [Input/Output], counter value [Input]
 *  Return               : nothing
 *******************************************************************************/
void bsw_LoadCnt_Byte(T_COUNT_B* const rpu8_address, uint8_t value)
{
    (*rpu8_address) = value;
}

/*******************************************************************************
 *  Name                 : bsw_LoadCnt_Word
 *  Description          : load word (16 bits) counters
 *  Parameters           : rpu16_addres [Input/Output], counter value [Input]
 *  Return               : nothing
 *******************************************************************************/
void bsw_LoadCnt_Word(T_COUNT_W* const rpu16_address, uint16_t value)
{
    (*rpu16_address) = value;
}

/*******************************************************************************
 *  Name                 : bsw_LoadCnt_Long
 *  Description          : load long (32 bits) counters
 *  Parameters           : rpu32_addres [Input/Output], counter value [Input]
 *  Return               : nothing
 *******************************************************************************/
void bsw_LoadCnt_Long(T_COUNT_L* const rpu32_address, uint32_t value)
{
    (*rpu32_address) = value;
}

/*******************************************************************************
 *  Name                 : bsw_GetCnt_Byte
 *  Description          : return 8 bit counter value
 *  Parameters           : rpu8_addres [Input]
 *  Return               : counter value
 *******************************************************************************/
T_COUNT_B bsw_GetCnt_Byte(T_COUNT_B const * const rpu8_address)
{
    return (*rpu8_address);
}

/*******************************************************************************
 *  Name                 : bsw_GetCnt_Word
 *  Description          : return 16 bit counter value
 *  Parameters           : rpu16_addres [Input]
 *  Return               : counter value
 *******************************************************************************/
T_COUNT_W bsw_GetCnt_Word(T_COUNT_W const * const rpu16_address)
{
    return (*rpu16_address);
}

/*******************************************************************************
 *  Name                 : bsw_GetCnt_Long
 *  Description          : return 32 bit counter value
 *  Parameters           : rpu32_addres [Input]
 *  Return               : counter value
 *******************************************************************************/
T_COUNT_L bsw_GetCnt_Long(T_COUNT_L const * const rpu32_address)
{
    return (*rpu32_address);
}



 /* END bsw_cntman */
 /*!
 ** @}
 */
