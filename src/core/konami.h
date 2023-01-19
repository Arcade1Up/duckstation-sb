#pragma once

#include "common/types.h"

void KonamiInit(void);
void KonamiTerm(void);

void KonamiDmaControlWrite(u32& ControlBits, u32& Address, u32 Value);

void KonamiScsiRead(u32 Size, u32 Offset, u32& Value);
void KonamiScsiWrite(u32 Size, u32 Offset, u32 Value);

void KonamiP1Read(u32 Size, u32 Offset, u32& Value);
void KonamiP1Write(u32 Size, u32 Offset, u32 Value);

void KonamiP2Read(u32 Size, u32 Offset, u32& Value);
void KonamiP2Write(u32 Size, u32 Offset, u32 Value);

void KonamiFlashRead(u32 Size, u32 Offset, u32& Value);
void KonamiFlashWrite(u32 Size, u32 Offset, u32 Value);

void KonamiEepromRead(u32 Size, u32 Offset, u32& Value);
void KonamiEepromWrite(u32 Size, u32 Offset, u32 Value);

void KonamiTrackballRead(u32 Size, u32 Offset, u32& Value);
void KonamiTrackballWrite(u32 Size, u32 Offset, u32 Value);

void KonamiButtonsSet(u32 Buttons);
void KonamiTrackballSetXY(u16 X, u16 Y);

void KonamiScsiIrqDeassert(void);
void KonamiEepromFixup(void);

void KonamiScoreInit(void);
void KonamiScoreUpdate(void);
