#ifndef __LJACK_DAC_PLUGIN__
#define __LJACK_DAC_PLUGIN__


/* GLOS (Graphic Library in Open Source), an ANSI Common Lisp OpenGL subset.
   Copyright (C) 2000 the GLOS development team (http://glos.sourceforge.net) */


#include "u3.h"
#include "U3_manage.h"


typedef struct dummy{

  bool error_on_last_output;
  unsigned int communication_errors;

  int err;
  uint8 options, speedAdjust, sdaPinNum, sclPinNum, address, numBytesToSend, numBytesToReceive, errorcode;
  uint16 binaryVoltage;
  uint8 bytesCommand[5];
  uint8 bytesResponse[64];
  uint8 ackArray[4];

  // info about the labjack hosting this DAC plugin
  HANDLE hDevice;
  u3TdacCalibrationInfo* caliInfo;
  

}Ljack_DAC_plugin; 



/* Initialize communication with the already-calibrated DAC plugin,
   connected to the LabJack at plugin_port.
   Possible values: 7 (ports FIO7/6), 5 (FIO5/4), 3 (AIN2/3), 1 (AIN0/1)
*/
void 
Ljack_DAC_com_init(Ljack_DAC_plugin* DAC, HANDLE hDevice, u3TdacCalibrationInfo* caliInfo, unsigned int plugin_port);


/* next values output to DACA */
void
Ljack_DAC_setoutput_A(Ljack_DAC_plugin* DAC);


/* next values output to DACB */
void
Ljack_DAC_setoutput_B(Ljack_DAC_plugin* DAC);


/* output voltage to selected output */ 
void
Ljack_DAC_output(Ljack_DAC_plugin* DAC, float value);


#endif /* __LJACK_DAC_PLUGIN__ */
