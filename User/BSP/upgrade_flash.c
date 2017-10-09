/* 
*  Author: Adam Huang
*  Date:2016/8/1
*/

#include "common.h"
#include "upgrade_flash.h"
#include "nbosDriverFlash.h"
#include "LibMd5.h"
#include "StringUtils.h"

#define upgrade_log(M, ...) custom_log("Upgrade", M, ##__VA_ARGS__)
#define upgrade_log_trace() custom_log_trace("Upgrade")

static uint32_t flash_addr = UPDATE_START_ADDRESS;
upgrate_t *p_upgrade;
/*OTA options*/
boot_table_t             bootTable;

OSStatus MICOUpdateConfiguration(boot_table_t *bootTable);

OSStatus upgradePrepareFlash( uint8_t *md5, uint32_t Size )
{
  OSStatus err = kNoErr;
  
  flash_addr = UPDATE_START_ADDRESS;
  require_action( Size <= UPDATE_FLASH_SIZE , exit, err = kGeneralErr );
  
  if( p_upgrade )
  {
    free( p_upgrade );
  }

  p_upgrade = (upgrate_t *)malloc(sizeof( upgrate_t ));
  require_action( p_upgrade, exit, err = kNoMemoryErr );

  if( p_upgrade )
  {
    memset( p_upgrade, 0x0, sizeof(upgrate_t) );
    memcpy( p_upgrade->md5, md5, 16 );
    p_upgrade->len = Size;
  }
    
  MicoFlashInitialize( MICO_FLASH_FOR_UPDATE );
      
  if( MicoFlashErase(MICO_FLASH_FOR_UPDATE, UPDATE_START_ADDRESS, UPDATE_END_ADDRESS) == 0 )
  {
    upgrade_log("erase OK");
  }
  else
    upgrade_log("erase error");
exit:
  return err;
}

OSStatus upgradeWriteFlashData( uint32_t* Data, uint32_t DataLength )
{
  OSStatus err = kNoErr;
   
  if( MicoFlashWrite(MICO_FLASH_FOR_UPDATE, &flash_addr, (uint8_t *)Data, DataLength) == 0 )
  {
    upgrade_log("have write data @%08x %d bytes", flash_addr - DataLength, DataLength);
  }
  else
  {
    upgrade_log("write error");
    err = kGeneralErr;
    goto exit;
  }
exit:
  return err;
}

OSStatus upgradeCheckFlash( void )
{
  OSStatus err = kNoErr;
  MD5_HASH md5_ret;
  Md5Context ctx;
  
  MicoFlashFinalize( MICO_FLASH_FOR_UPDATE );
  
  Md5Initialise( &ctx );
  Md5Update( &ctx, (uint8_t *)UPDATE_START_ADDRESS, flash_addr - UPDATE_START_ADDRESS);
  Md5Finalise( &ctx, &md5_ret );
  
  if( memcmp(md5_ret.bytes, p_upgrade->md5, 16) != 0 )
  {
#if 1
    char *debugString;
    debugString = DataToHexString(p_upgrade->md5,16);
    upgrade_log("correct Md5 is:%s",debugString);
    free(debugString);
    debugString = DataToHexString(md5_ret.bytes,16);
    upgrade_log("Md5:%s,is error",debugString);
    free(debugString);
    
#endif
    err = kGeneralErr;
    goto exit;
  }
  else
  {
    upgrade_log("Md5:%s,is correct",DataToHexString(md5_ret.bytes,16));
    memset(&bootTable, 0, sizeof(boot_table_t));
    bootTable.length = flash_addr - UPDATE_START_ADDRESS;
    bootTable.start_address = UPDATE_START_ADDRESS;
    bootTable.type = 'A';
    bootTable.upgrade_type = 'U';
    MICOUpdateConfiguration(&bootTable);
  }
exit:  
  if(p_upgrade)
    free( p_upgrade );
  return err;
}

OSStatus MICOUpdateConfiguration(boot_table_t *bootTable)
{
  OSStatus err = kNoErr;
  uint32_t paraStartAddress, paraEndAddress;
 
  paraStartAddress = PARA_START_ADDRESS;
  paraEndAddress = PARA_END_ADDRESS;

  err = MicoFlashInitialize(MICO_FLASH_FOR_PARA);
  require_noerr(err, exit);
  err = MicoFlashErase(MICO_FLASH_FOR_PARA, paraStartAddress, paraEndAddress);
  require_noerr(err, exit);
  err = MicoFlashWrite(MICO_FLASH_FOR_PARA, &paraStartAddress, (uint8_t *)bootTable, sizeof(boot_table_t));
  require_noerr(err, exit);
  err = MicoFlashFinalize(MICO_FLASH_FOR_PARA);
  require_noerr(err, exit);

exit:
  return err;
}