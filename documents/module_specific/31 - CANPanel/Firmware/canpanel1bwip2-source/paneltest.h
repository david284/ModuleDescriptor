#ifndef __PANELTEST_H
#define __PANELTEST_H

/*

 Copyright (C) Pete Brownlow 2014-2017   software@upsys.co.uk

  CBUS CANPanel - test routine definitions

 This code is for a CANPanel CBUS module, to control up to 64 LEDs (or 8 x 7 segment displays)
 and up to 64 push buttons or on/off switches

 This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                   and indicate if changes were made. You may do so in any reasonable manner,
                   but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                  your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                  legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

**************************************************************************************************************
  Note:   This source code has been written using a tab stop and indentation setting
          of 4 characters. To see everything lined up correctly, please set your
          IDE or text editor to the same settings.
******************************************************************************************************
	
 For version number and revision history see CANPanel.h

*/

#include "canpanel.h"


// Time delays for test mode states

#define TEST_MODE_TIME      FIVE_SECOND
#define HELLO_TIME          FIVE_SECOND
#define TEST_NUM_TIME       FIVE_SECOND
#define TEST_X_TIME         FIVE_SECOND
#define TEST_LED_TIME       HALF_SECOND

#define TEST_LED_PASSES     2

// Parameters for sending test events

#define TEST_EVENT_COUNT    250
#define TEST_EVENT_INTERVAL 47  // 800uS      ONE_MILI_SECOND



void panelTestInit(void);
void panelTest(void);


                

#endif	// __PANELTEST_H
