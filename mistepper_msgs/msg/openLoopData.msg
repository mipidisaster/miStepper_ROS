std_msgs/Header header

# Packet count
uint16   PktCount

# Angular Position
float32  AngPos

# Power Parameters
float32  VoltRef
float32  FANVolt
float32  FANCurt
float32  STPVolt
float32  STPCurt

# Temperatures
float32  ExtTmp
float32  IntTmp
#float32  MtrTmp

#float32  Humidity
#float32  Pressure

# Control surfaces
float32  FANAct
uint16   STPFrequency
uint16   STPState
uint32   STPcalcPosition

# Task times
float32  SPIDur
float32  SPIPer
float32  I2CDur
float32  I2CPer
float32  ADCDur
float32  ADCPer
float32  USTDur
float32  USTPer
float32  FANDur
float32  FANPer
float32  STPDur
float32  STPPer

# Fault flags
uint8    SPICmmFlt
uint8    AS5048_ComFlt
uint8    AS5048_DevFlt
uint8    AS5048_FltCount

uint8    I2CCmmFlt
uint8    AD7415_ComFlt
uint8    AD7415_DevFlt
uint8    AD7415_FltCount

uint8    ADCFlt