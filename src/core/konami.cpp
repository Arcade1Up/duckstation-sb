#include "bus.h"
#include "common/file_system.h"
#include "common/log.h"
#include "cpu_core.h"
#include "interrupt_controller.h"
#include "konami.h"
#include "timers.h"
#include "timing_event.h"
#include <stdio.h>

Log_SetChannel(Konami);

#define EEPROM_FILE "/sdcard/arcade1up/simpsons/eeprom"

#define FLASH_FILE0 "/sdcard/arcade1up/simpsons/flash0"
#define FLASH_FILE1 "/sdcard/arcade1up/simpsons/flash1"
#define FLASH_FILE2 "/sdcard/arcade1up/simpsons/flash2"
#define FLASH_FILE3 "/sdcard/arcade1up/simpsons/flash3"

#define SCSI_CD_IMAGE "/sdcard/arcade1up/simpsons/arcade.iso"

// SCSI
enum {
  REG_XFERCNTLOW = 0, // read = current xfer count lo byte, write = set xfer count lo byte
  REG_XFERCNTMID,     // read = current xfer count mid byte, write = set xfer count mid byte
  REG_FIFO,           // read/write = FIFO
  REG_COMMAND,        // read/write = command
  REG_STATUS,         // read = status, write = destination SCSI ID (4)
  REG_IRQSTATE,       // read = IRQ status, write = timeout         (5)
  REG_INTSTATE,       // read = internal state, write = sync xfer period (6)
  REG_FIFOSTATE,      // read = FIFO status, write = sync offset
  REG_CTRL1,          // read/write = control 1
  REG_CLOCKFCTR,      // clock factor (write only)
  REG_TESTMODE,       // test mode (write only)
  REG_CTRL2,          // read/write = control 2
  REG_CTRL3,          // read/write = control 3
  REG_CTRL4,          // read/write = control 4
  REG_XFERCNTHI,      // read = current xfer count hi byte, write = set xfer count hi byte
  REG_DATAALIGN       // data alignment (write only)
};

static std::FILE* ScsiCd = NULL;

static u8 ScsiRegs[16];
static u8 ScsiFifo[16];
static u8 ScsiFifoPtr;
static u8 ScsiCommand[12];
static bool ScsiIsRead;
static u32 ScsiSectorLba;

// Buttons
static u32 CurrentButtons;

// FLASH
static u8 Flash[4][0x200000];
static u32 FlashAddress;

// EEPROM
static std::FILE* EepromFp;
static uint16_t Eeprom[64];

// Trackball
static u16 TrackballX;
static u16 TrackballY;

// Score table
static struct {
        u8 Name[4];
        u16 Character;
        u16 Score;
} ScoreTable[2][10];

// Private API

static bool LoadEepromFile(const char* Path)
{
  EepromFp = FileSystem::OpenCFile(Path, "r+b");
  if (!EepromFp)
  {
    return false;
  }
  if (fread(Eeprom, 1, sizeof(Eeprom), EepromFp) != sizeof(Eeprom))
  {
    return false;
  }

  return true;
}

static bool LoadFlashFile(const char *Path, void *Buffer)
{
  std::FILE* fp;

  fp = FileSystem::OpenCFile(Path, "rb");
  if (!fp)
  {
    return false;
  }
  if (fread(Buffer, 1, 0x200000, fp) != 0x200000)
  {
    return false;
  }
  fclose(fp);

  return true;
}

static void AssertScsiInterrupt(void)
{
  g_interrupt_controller.InterruptRequest(InterruptController::IRQ::IRQ10);
}

// Public API

void KonamiInit(void)
{
  LoadEepromFile(EEPROM_FILE);
  LoadFlashFile(FLASH_FILE0, Flash[0]);
  LoadFlashFile(FLASH_FILE1, Flash[1]);
  LoadFlashFile(FLASH_FILE2, Flash[2]);
  LoadFlashFile(FLASH_FILE3, Flash[3]);

  ScsiCd = FileSystem::OpenCFile(SCSI_CD_IMAGE, "rb");
}

void KonamiTerm(void)
{
}

// DMA

void KonamiDmaControlWrite(u32& ControlBits, u32& Address, u32 Value)
{
  size_t ReadSize = (ControlBits >> 16) * 4;
  u8* Ram = Bus::g_ram;
  static u8 Sector[2048];
  int ret;

  if ((Value & 1) == 0)
  {
    switch (ScsiCommand[0])
    {
      case 0x03:
        memset(Ram + Address, 0, 12);
        break;
      case 0x12:
        memset(Ram + Address, 0, 32);
        break;
      case 0x1A:
        memset(Ram + Address, 0, 28);
        break;
      case 0x28:
        while (ReadSize >= 2048)
        {
          std::fseek(ScsiCd, ScsiSectorLba * 2048, SEEK_SET);
          ScsiSectorLba++;
          ret = fread(Sector, 1, 2048, ScsiCd);
          if (ret != 2048)
          {
            // Log_WarningPrintf("Error reading sector! ret=%08X", ret);
          }
          memcpy(Ram + Address, Sector, ret);
          Address += 2048;
          ReadSize -= 2048;
        }
        ControlBits &= 0xFFFF;
        break;
      case 0x43:
      {
        const uint32_t v0 = 0x01010008;
        const uint32_t v1 = 0x00010400;
        const uint32_t v2 = 0x00000000;

        memcpy(Ram + Address + 0, &v0, sizeof(v0));
        memcpy(Ram + Address + 4, &v1, sizeof(v1));
        memcpy(Ram + Address + 8, &v2, sizeof(v2));
        break;
      }
      default:
        //Log_WarningPrintf("Unimplemented SCSI command %02X", ScsiCmd[0]);
        break;
    }
  }

  ScsiRegs[4] &= ~0x7U;
}

// SCSI

void KonamiScsiRead(u32 Size, u32 Offset, u32& Value)
{
  const u8 Register = (Offset & 0x1F) >> 1;

  Value = ScsiRegs[Register];

  switch (Register)
  {
    case REG_FIFO:
      Value = 0;
      break;
    case REG_IRQSTATE:
      ScsiRegs[REG_STATUS] &= ~0x80U;
      break;
  }
}

void KonamiScsiWrite(u32 Size, u32 Offset, u32 Value)
{
  const u8 Register = (Offset & 0x1F) >> 1;

  switch (Register)
  {
    case REG_XFERCNTLOW:
    case REG_XFERCNTMID:
    case REG_XFERCNTHI:
      ScsiRegs[REG_STATUS] &= ~0x10;
      break;
    case REG_FIFO:
      ScsiFifo[ScsiFifoPtr++] = (u8)Value;
      if (ScsiFifoPtr > 15)
      {
        ScsiFifoPtr = 15;
      }
      break;
    case REG_COMMAND:
      ScsiFifoPtr = 0;

      switch (Value & 0x7F)
      {
        case 0x00:
          ScsiRegs[REG_IRQSTATE] = 8;
          break;
        case 0x02:
          ScsiRegs[REG_IRQSTATE] = 8;
          ScsiRegs[REG_STATUS] |= 0x80;
          AssertScsiInterrupt();
          break;
        case 0x03:
          ScsiRegs[REG_INTSTATE] = 0x04;
          AssertScsiInterrupt();
          break;
        case 0x42:
          if (ScsiFifo[1] == 0 || ScsiFifo[1] == 0x48 || ScsiFifo[1] == 0x4B)
          {
            ScsiRegs[REG_INTSTATE] = 0x06;
          }
          else
          {
            ScsiRegs[REG_INTSTATE] = 0x04;
          }

          for (int i = 0; i < 12; i++)
          {
            ScsiCommand[i] = ScsiFifo[1 + i];
          }

          ScsiIsRead = false;
          switch (ScsiCommand[0])
          {
            case 0x03:
            case 0x12:
            case 0x1A:
            case 0x43:
              ScsiRegs[REG_STATUS] = (ScsiRegs[REG_STATUS] & ~0x07) | 1;
              break;
            case 0x15:
              ScsiRegs[REG_STATUS] = (ScsiRegs[REG_STATUS] & ~0x07) | 0;
              break;
            case 0x28:
              ScsiSectorLba = (ScsiFifo[3] << 24) | (ScsiFifo[4] << 16) | (ScsiFifo[5] << 8) | ScsiFifo[6];
              ScsiIsRead = true;
              ScsiRegs[REG_STATUS] = (ScsiRegs[REG_STATUS] & ~0x07) | 1;
              break;
          }
          AssertScsiInterrupt();
          break;
        case 0x44:
          break;
        case 0x10:
          if (Value & 0x80U)
          {
            ScsiRegs[REG_STATUS] = (ScsiRegs[REG_STATUS] & ~0x07) | 3;
            ScsiRegs[REG_INTSTATE] = 0x00;
            AssertScsiInterrupt();
            break;
          }
          [[fallthrough]];

        case 0x11:
          AssertScsiInterrupt();
          ScsiRegs[REG_STATUS] &= ~0x87;
          ScsiRegs[REG_INTSTATE] = 0x00;
          [[fallthrough]]; // ?? Why ??

        case 0x12:
          ScsiRegs[REG_STATUS] |= 0x80U;
          ScsiRegs[REG_INTSTATE] = 0x06U;
          break;

        default:
          // Log_WarningPrintf("Unknown command %02X!", value);
          break;
      }
      break;
  }
  if (Register != REG_STATUS && Register != REG_INTSTATE && Register != REG_IRQSTATE && Register != REG_FIFOSTATE)
  {
    ScsiRegs[Register] = (uint8_t)Value;
  }
}

// Player 1 controls

void KonamiP1Read(u32 Size, u32 Offset, u32& Value)
{
  Value = 0xFFFFFFFF;
  if (CurrentButtons & 0x0080) Value &= ~(1 << 0); // LEFT
  if (CurrentButtons & 0x0020) Value &= ~(1 << 1); // RIGHT
  if (CurrentButtons & 0x0010) Value &= ~(1 << 2); // UP
  if (CurrentButtons & 0x0040) Value &= ~(1 << 3); // DOWN
  if (CurrentButtons & 0x4000) Value &= ~(1 << 4); // BUTTON 1 PLAYER 1
  if (CurrentButtons & 0x2000) Value &= ~(1 << 4); // BUTTON 1 PLAYER 1
  if (CurrentButtons & 0x0D08) Value &= ~(1 << 9);  // START
  //if (CurrentButtons & 0x0001) Value &= ~(1 << 10); // COIN
  //if (CurrentButtons & 0x0001) Value &= ~(3 << 11);
}

void KonamiP1Write(u32 Size, u32 Offset, u32 Value)
{
  // Ignored
}

// Player 2 controls

void KonamiP2Read(u32 Size, u32 Offset, u32& Value)
{
  Value = 0xFFFFFFFF;
}

void KonamiP2Write(u32 Size, u32 Offset, u32 Value)
{
  // Ignored
}

// FLASH

void KonamiFlashRead(u32 Size, u32 Offset, u32& Value)
{
  u8 Chip;

  Offset &= 0xF;

  switch (Offset)
  {
    case 0:
      Chip = (FlashAddress >= 0x200000) ? 2 : 0;
      Value = Flash[Chip][FlashAddress & 0x1FFFFF] | (Flash[Chip + 1][FlashAddress & 0x1FFFFF] << 8);
      FlashAddress++;
      break;
    case 8:
      FlashAddress |= 1;
      [[fallthrough]];
    default:
      Value = 0;
      break;
  }
}

void KonamiFlashWrite(u32 Size, u32 Offset, u32 Value)
{
  Offset &= 0xF;

  switch (Offset)
  {
    case 0:
      // Ignored
      break;
    case 2:
      FlashAddress = 0;
      FlashAddress |= Value << 1;
      break;
    case 4:
      FlashAddress &= 0xFF00FF;
      FlashAddress |= Value << 8;
      break;
    case 6:
      FlashAddress &= 0x00FFFF;
      FlashAddress |= Value << 15;
      break;
  }
}

// EEPROM

void KonamiEepromRead(u32 Size, u32 Offset, u32& Value)
{
  if (Offset >= 0x00180080 && Offset < 0x00180100)
  {
    Value = Eeprom[((Offset - 0x80) & 0x7F) >> 1];
  }
  else
  {
    Log_WarningPrintf("%s: %08X", __FUNCTION__, Offset);
    Value = 0;
  }
}

void KonamiEepromWrite(u32 Size, u32 Offset, u32 Value)
{
  if (Offset >= 0x00180080 && Offset < 0x00180100)
  {
    u8 RelativeOffset = (Offset - 0x80) & 0x7F;
    Eeprom[RelativeOffset >> 1] = (u16)Value;
    std::fseek(EepromFp, 0, SEEK_SET);
    fwrite(Eeprom, 1, sizeof(Eeprom), EepromFp);
  }
  else
  {
    Log_WarningPrintf("%s: %08X %08X", __FUNCTION__, Offset, Value);
  }
}

// Trackball
void KonamiTrackballRead(u32 Size, u32 Offset, u32& Value)
{
  switch (Offset)
  {
    case 0x006800C0:
      Value = (TrackballX & 0x0FF) << 8;
      break;
    case 0x006800C2:
      Value = (TrackballX & 0xF00);
      break;
    case 0x006800C4:
      Value = (TrackballY & 0x0FF) << 8;
      break;
    case 0x006800C6:
      Value = (TrackballY & 0xF00);
      break;
    default:
      TrackballX = 0;
      TrackballY = 0;
      break;
  }
}

void KonamiTrackballWrite(u32 Size, u32 Offset, u32 Value)
{
  // Ignored
}

void KonamiButtonsSet(u32 Buttons)
{
  CurrentButtons = Buttons ^ 0xFFFFFFFF;
}

void KonamiTrackballSetXY(u16 X, u16 Y)
{
  TrackballX = X;
  TrackballY = Y;
}

// Misc

void KonamiScsiIrqDeassert(void)
{
  CPU::ClearExternalInterrupt((u8)InterruptController::IRQ::IRQ10);
}

void KonamiEepromFixup(void)
{
        size_t i;

        Eeprom[0] = 0;
        for (i = 1; i < 64; i++) {
                Eeprom[0] += Eeprom[i];
        }
        std::fseek(EepromFp, 0, SEEK_SET);
        fwrite(Eeprom, 1, sizeof(Eeprom), EepromFp);
}

void KonamiScoreInit(void)
{
  u8* Ram = Bus::g_ram;

  // Update the template score table
  memcpy(Ram + 0x132A8, &ScoreTable, sizeof(ScoreTable));
}

void KonamiScoreUpdate(void)
{
  u8* Ram = Bus::g_ram;

  // Copy the live score table
  memcpy(&ScoreTable, Ram + 0xE31E0, sizeof(ScoreTable));
}
