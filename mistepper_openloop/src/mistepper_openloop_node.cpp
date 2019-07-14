#include <ros/ros.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <unistd.h>         // Needed for the I2C port
#include <fcntl.h>          // Needed for the I2C port
#include <sys/ioctl.h>      // Needed for the I2C port
#include <linux/i2c-dev.h>  // Needed for the I2C port

//#include <std_msgs/UInt16.h>
#include "mistepper_msgs/openLoopData.h"
#include "mistepper_msgs/openLoopReqt.h"

#include "milibrary/drv/UARTPeriph/UARTPeriph.h"
#include "milibrary/com/DataManip/DataManip.h"

float GetActTime(uint8_t *data) {
    uint16_t test = DataManip::_2x8bit_2_16bit(data);

    return ( (float) test / 20L );
}

typedef struct {
    uint8_t     InterfaceReg;

    float       FanDmd;

    uint8_t     STPEnable;
    uint8_t     STPGear;
    uint8_t     STPDir;
    uint16_t    STPFreq;
}   _miStepperDmd;

_miStepperDmd       Demand_Handle = {  0  };

class ExtDevice {
    private:
        struct _calib_data {
            uint16_t    dig_T1;
            int16_t     dig_T2;
            int16_t     dig_T3;

            uint16_t    dig_P1;
            int16_t     dig_P2;
            int16_t     dig_P3;
            int16_t     dig_P4;
            int16_t     dig_P5;
            int16_t     dig_P6;
            int16_t     dig_P7;
            int16_t     dig_P8;
            int16_t     dig_P9;

            uint8_t     dig_H1;
            int16_t     dig_H2;
            uint8_t     dig_H3;
            int16_t     dig_H4;
            int16_t     dig_H5;
            int8_t      dig_H6;
        }   calibrationData;

        int I2CHandle;
        int32_t t_fine;

    public:
        float Temp;
        float Pres;
        float Humd;

        uint8_t ManualBulkWrite(uint8_t *data, uint16_t size) {
            if (write(this->I2CHandle, data, size) != size)
                return(1);
            else
                return(0);
        }

        uint8_t ManualBulkRead(uint8_t *data, uint16_t size) {
            if (read(this->I2CHandle, data, size) != size)
                return(1);
            else
                return(0);
        }

        uint32_t parse_Temp_PressSensor(uint8_t *data) {
            uint32_t data_xlsb;
            uint32_t data_lsb;
            uint32_t data_msb;

            data_msb    = (uint32_t) data[0] << 12;
            data_lsb    = (uint32_t) data[1] << 4;
            data_xlsb   = (uint32_t) data[2] >> 4;

            return(data_msb | data_lsb | data_xlsb);
        }

         uint32_t parse_HumSensor(uint8_t *data) {
            uint32_t data_lsb;
            uint32_t data_msb;

            data_msb    = (uint32_t) data[0] << 8;
            data_lsb    = (uint32_t) data[1];

            return(data_msb | data_lsb);
        }

        void getSensorRegister(uint32_t *data) {
            uint8_t enviroData[8] = { 0 };
            uint8_t fromAddress = 0xF7;
            this->ManualBulkWrite(&fromAddress, 1);
            this->ManualBulkRead(&enviroData[0], 8);

            data[0] = this->parse_Temp_PressSensor(&enviroData[0]);
            data[1] = this->parse_Temp_PressSensor(&enviroData[3]);
            data[2] = this->parse_HumSensor(&enviroData[6]);
        }

        float compensate_Temp(uint32_t regData) {
            float var1;
            float var2;
            float temperature;
            float temperature_min = -40;
            float temperature_max = 85;

            var1 = ((float) regData) / 16384.0 - ((float)this->calibrationData.dig_T1) / 1024.0;
            var1 = var1 * ((float)this->calibrationData.dig_T2);
            var2 = (((float) regData) / 131072.0 - ((float)this->calibrationData.dig_T1) / 8192.0);
            var2 = (var2 * var2) * ((float)this->calibrationData.dig_T3);
            this->t_fine = (int32_t)(var1 + var2);
            temperature = (var1 + var2) / 5120.0;
            if (temperature < temperature_min)
            {
                temperature = temperature_min;
            }
            else if (temperature > temperature_max)
            {
                temperature = temperature_max;
            }

            return temperature;
        }

        float compensate_Press(uint32_t regData) {
            float var1;
            float var2;
            float var3;
            float pressure;
            float pressure_min = 30000.0;
            float pressure_max = 110000.0;

            var1 = ((float)this->t_fine / 2.0) - 64000.0;
            var2 = var1 * var1 * ((float)this->calibrationData.dig_P6) / 32768.0;
            var2 = var2 + var1 * ((float)this->calibrationData.dig_P5) * 2.0;
            var2 = (var2 / 4.0) + (((float)this->calibrationData.dig_P4) * 65536.0);
            var3 = ((float)this->calibrationData.dig_P3) * var1 * var1 / 524288.0;
            var1 = (var3 + ((float)this->calibrationData.dig_P2) * var1) / 524288.0;
            var1 = (1.0 + var1 / 32768.0) * ((float)this->calibrationData.dig_P1);

            /* avoid exception caused by division by zero */
            if (var1)
            {
                pressure = 1048576.0 - (float) regData;
                pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
                var1 = ((float)this->calibrationData.dig_P9) * pressure * pressure / 2147483648.0;
                var2 = pressure * ((float)this->calibrationData.dig_P8) / 32768.0;
                pressure = pressure + (var1 + var2 + ((float)this->calibrationData.dig_P7)) / 16.0;
                if (pressure < pressure_min)
                {
                    pressure = pressure_min;
                }
                else if (pressure > pressure_max)
                {
                    pressure = pressure_max;
                }
            }
            else /* Invalid case */
            {
                pressure = pressure_min;
            }

            return pressure;
        }

        float compensate_Hum(uint32_t regData) {
            float humidity;
            float humidity_min = 0.0;
            float humidity_max = 100.0;
            float var1;
            float var2;
            float var3;
            float var4;
            float var5;
            float var6;

            var1 = ((float)this->t_fine) - 76800.0;
            var2 = (((float)this->calibrationData.dig_H4) * 64.0 + (((float)this->calibrationData.dig_H5) / 16384.0) * var1);
            var3 = regData - var2;
            var4 = ((float)this->calibrationData.dig_H2) / 65536.0;
            var5 = (1.0 + (((float)this->calibrationData.dig_H3) / 67108864.0) * var1);
            var6 = 1.0 + (((float)this->calibrationData.dig_H6) / 67108864.0) * var1 * var5;
            var6 = var3 * var4 * (var5 * var6);
            humidity = var6 * (1.0 - ((float)this->calibrationData.dig_H1) * var6 / 524288.0);
            if (humidity > humidity_max)
            {
                humidity = humidity_max;
            }
            else if (humidity < humidity_min)
            {
                humidity = humidity_min;
            }

            return humidity;
        }

        void getSensor(void) {
            uint32_t regDATA[3] = { 0 };
            this->getSensorRegister(&regDATA[0]);

            this->Temp      = this->compensate_Temp(regDATA[1]);
            this->Pres      = this->compensate_Press(regDATA[0]);
            this->Humd      = this->compensate_Hum(regDATA[2]);
        }

        ExtDevice(void) {
            this->I2CHandle = wiringPiI2CSetup(0x76);
            this->Temp      = -999;
            this->Pres      = -999;
            this->Humd      = -999;

            this->getCalibrationData();

            // Setup device with the following configuration:
            //      Oversampling of humidity    = x1
            //      Oversampling of temperature = x16 (gives 4 bits extra resolution)
            //      Oversampling of pressure    = x16 (gives 4 bits extra resolution)
            //      Device filters disabled
            //      Device to be idle for 10ms

            uint8_t     config_reg[6]   = {         // Array to store Register Addresses and data to configure device
            // Data needs to be provided as Address, then Data, etc.
                        0xF2,                       // First Address is "ctrl_hum", as needs to be updated prior to "ctrl_meas"
                        0x01,                       // Set value, to have humidity oversample by x1
                        // ------------------------ //
                        0xF5,                       // Now select the "config" register
                        0xE0,                       // Set value, to have device idle for 10ms, and no filters enabled
                        // ------------------------ //
                        0xF4,                       // Finally select the "ctrl_meas" register
                        0xB7                        // Set value, to have Pressure and Temperature oversample by x16
            };
            this->ManualBulkWrite(&config_reg[0], 6);       // Bulk write to all registers

            this->getSensor();
        }

        void getCalibrationData(void) {
            this->calibrationData.dig_T1 = wiringPiI2CReadReg16(this->I2CHandle, 0x88);
            this->calibrationData.dig_T2 = wiringPiI2CReadReg16(this->I2CHandle, 0x8A);
            this->calibrationData.dig_T3 = wiringPiI2CReadReg16(this->I2CHandle, 0x8C);

            this->calibrationData.dig_P1 = wiringPiI2CReadReg16(this->I2CHandle, 0x8E);
            this->calibrationData.dig_P2 = wiringPiI2CReadReg16(this->I2CHandle, 0x90);
            this->calibrationData.dig_P3 = wiringPiI2CReadReg16(this->I2CHandle, 0x92);
            this->calibrationData.dig_P4 = wiringPiI2CReadReg16(this->I2CHandle, 0x94);
            this->calibrationData.dig_P5 = wiringPiI2CReadReg16(this->I2CHandle, 0x96);
            this->calibrationData.dig_P6 = wiringPiI2CReadReg16(this->I2CHandle, 0x98);
            this->calibrationData.dig_P7 = wiringPiI2CReadReg16(this->I2CHandle, 0x9A);
            this->calibrationData.dig_P8 = wiringPiI2CReadReg16(this->I2CHandle, 0x9C);
            this->calibrationData.dig_P9 = wiringPiI2CReadReg16(this->I2CHandle, 0x9E);

            this->calibrationData.dig_H1 = wiringPiI2CReadReg8(this->I2CHandle, 0xA1);
            this->calibrationData.dig_H2 = wiringPiI2CReadReg16(this->I2CHandle, 0xE1);

            this->calibrationData.dig_H3 = wiringPiI2CReadReg8(this->I2CHandle, 0xE3);

            uint8_t temp[3] = { 0 };
            temp[0] = wiringPiI2CReadReg8(this->I2CHandle, 0xE4);
            temp[1] = wiringPiI2CReadReg8(this->I2CHandle, 0xE5);
            temp[2] = wiringPiI2CReadReg8(this->I2CHandle, 0xE6);

            this->calibrationData.dig_H4 = (int16_t) (temp[0] << 4) | (int16_t) (temp[1] & 0x0F);
            this->calibrationData.dig_H5 = (int16_t) (temp[2] << 4) | (int16_t) (temp[1] >> 4);

            this->calibrationData.dig_H6 = wiringPiI2CReadReg8(this->I2CHandle, 0xE7);
        }

        uint8_t readDeviceID(void) {  return(wiringPiI2CReadReg8(this->I2CHandle, 0xD0));  }

        ~ExtDevice()  {};
};

void UserRequestCallback(const mistepper_msgs::openLoopReqt::ConstPtr& msg) {
    Demand_Handle.InterfaceReg  = msg->InterfaceReg;
    
    Demand_Handle.FanDmd        = msg->FanDmd;

    Demand_Handle.STPEnable     = msg->STPEnable;
    Demand_Handle.STPGear       = msg->STPGear;
    Demand_Handle.STPDir        = msg->STPDir;
    Demand_Handle.STPFreq       = msg->STPFreq;
}

#define MISTEPPER_WRIT_PACKETSIZE       16
#define MISTEPPER_READ_PACKETSIZE       84

#define MISTEPPER_TRANSMIT_DATA_BIT     0
#define MISTEPPER_RESET_DEVICE_BIT      1

void miStepperUserRequest(UARTPeriph *Device, 
                          uint8_t InterfaceRe, float FanDemd, uint8_t STPEnb, uint8_t STPGer, uint8_t STPDir, uint16_t STPFreq) {

    uint8_t reqData[MISTEPPER_WRIT_PACKETSIZE]  =  {  0  };
    reqData[0x00]   = 0xFF;
    reqData[0x01]   = 0xA5;

    reqData[0x03]   = InterfaceRe;
    DataManip::_float_2_4x8bit(  FanDemd,  &reqData[0x04]  );

    reqData[0x08]   = STPEnb;
    reqData[0x09]   = STPGer;
    reqData[0x0A]   = STPDir;

    DataManip::_16bit_2_2x8bit(  STPFreq,  &reqData[0x0C]  );

    reqData[0x0F]   = 0xFB;

    Device->PoleTransmit(&reqData[0], MISTEPPER_WRIT_PACKETSIZE);
}


int main(int argc, char** argv) {
    wiringPiSetupGpio();

    UARTPeriph miStp("/dev/serial0", 115200, 1024);

    uint8_t arrayData[10] = { 0 };

    ros::init(argc, argv, "miStepper_HAL");
    ros::NodeHandle nh;

    ros::Publisher  pub1 = nh.advertise<mistepper_msgs::openLoopData>("miStepper", 10);
    ros::Subscriber sub1 = nh.subscribe("miStepperReqt", 100, UserRequestCallback);

    ros::Rate loop_rate(100);

    uint8_t     i = 0;
    uint32_t    seq = 0;

    uint8_t     temparray[MISTEPPER_READ_PACKETSIZE] = { 0 };

    // Test area for external temperature sensor
    uint8_t return_ID;
    ExtDevice BME280;

    //miStp.PoleSingleTransmit('D');      // Transmit code to get miStepper to provide data
    miStepperUserRequest(&miStp, (uint8_t)(1 << MISTEPPER_RESET_DEVICE_BIT) ,  0.00, 0, 0, 0, 0);

    ros::Duration(0.5).sleep(); // sleep for half a second

    while (ros::ok()) {
        miStepperUserRequest(&miStp,    (Demand_Handle.InterfaceReg | (1 << MISTEPPER_TRANSMIT_DATA_BIT) ),
                                        Demand_Handle.FanDmd,
                                        Demand_Handle.STPEnable,
                                        Demand_Handle.STPGear,
                                        Demand_Handle.STPDir,
                                        Demand_Handle.STPFreq
        );

        //miStp.PoleSingleTransmit('D');      // Transmit code to get miStepper to provide data

        for (i = 0; i != MISTEPPER_READ_PACKETSIZE; i++) {// Expect "MISTEPPER_PACKETSIZE bytes of data
            //temparray[i] = miStp.PoleSingleRead();      // Read and put data into array
        }

        // Read the sensor values
        BME280.getSensor();

        mistepper_msgs::openLoopData    parameters;

        parameters.header.seq = seq;
        seq++;
        parameters.header.stamp = ros::Time::now();

        parameters.MtrTmp           = BME280.Temp;
        parameters.Humidity         = BME280.Humd;
        parameters.Pressure         = BME280.Pres;

        // Layout of data in alignment with "PacketTransmission.xlsx" Sheet "USARTPacket"
        //                                                                    Version 1.1
        
        parameters.PktCount         = DataManip::_2x8bit_2_16bit(&temparray[2]);
        parameters.AngPos           = DataManip::_4x8bit_2_float(&temparray[0x04]);
        parameters.SPICmmFlt        = temparray[0x08];
        parameters.AS5048_ComFlt    = temparray[0x09];
        parameters.AS5048_DevFlt    = temparray[0x0A];
        parameters.AS5048_FltCount  = temparray[0x0B];
        parameters.SPIDur           = GetActTime(&temparray[0x0C]);
        parameters.SPIPer           = GetActTime(&temparray[0x0E]);

        parameters.ExtTmp           = DataManip::_4x8bit_2_float(&temparray[0x10]);
        parameters.I2CCmmFlt        = temparray[0x14];
        parameters.AD7415_ComFlt    = temparray[0x15];
        parameters.AD7415_DevFlt    = temparray[0x16];
        parameters.AD7415_FltCount  = temparray[0x17];
        parameters.I2CDur           = GetActTime(&temparray[0x18]);
        parameters.I2CPer           = GetActTime(&temparray[0x1A]);

        parameters.VoltRef          = DataManip::_4x8bit_2_float(&temparray[0x1C]);
        parameters.IntTmp           = DataManip::_4x8bit_2_float(&temparray[0x20]);
        parameters.FANVolt          = DataManip::_4x8bit_2_float(&temparray[0x24]);
        parameters.FANCurt          = DataManip::_4x8bit_2_float(&temparray[0x28]);
        parameters.STPVolt          = DataManip::_4x8bit_2_float(&temparray[0x2C]);
        parameters.STPCurt          = DataManip::_4x8bit_2_float(&temparray[0x30]);
        parameters.ADCFlt           = temparray[0x37];

        parameters.ADCDur           = GetActTime(&temparray[0x38]);
        parameters.ADCPer           = GetActTime(&temparray[0x3A]);

        parameters.FANAct           = DataManip::_4x8bit_2_float(&temparray[0x3C]);
        
        parameters.FANDur           = GetActTime(&temparray[0x40]);
        parameters.FANPer           = GetActTime(&temparray[0x42]);

        parameters.STPFrequency     = DataManip::_2x8bit_2_16bit(&temparray[0x44]);
        parameters.STPState         = DataManip::_2x8bit_2_16bit(&temparray[0x46]);
        parameters.STPcalcPosition  = DataManip::_4x8bit_2_32bit(&temparray[0x48]);

        parameters.STPDur           = GetActTime(&temparray[0x4C]);
        parameters.STPPer           = GetActTime(&temparray[0x4E]);

        parameters.USTDur           = GetActTime(&temparray[0x50]);
        parameters.USTPer           = GetActTime(&temparray[0x52]);

        pub1.publish(parameters);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Open Loop finished, due to ROS exit");
}