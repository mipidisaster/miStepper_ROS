#include <ros/ros.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <unistd.h>         // Needed for the I2C port
#include <fcntl.h>          // Needed for the I2C port
#include <sys/ioctl.h>      // Needed for the I2C port
#include <linux/i2c-dev.h>  // Needed for the I2C port

#include <boost/shared_ptr.hpp>
#include <thread>

//#include <std_msgs/UInt16.h>
#include "mistepper_msgs/openLoopData.h"
#include "mistepper_msgs/openLoopReqt.h"

#include "milibrary/drv/UARTPeriph/UARTPeriph.h"
#include "milibrary/com/DataManip/DataManip.h"

///////////////////////////////////////////////////////////////////////////
#define MISTEPPER_CTRLLOOP_UART_HZ      100L

#define MISTEPPER_CTRLLOOP_I2C_HZ       100L

#define MISTEPPER_PUB_HZ                100L

//////
#define MISTEPPER_WRIT_PACKETSIZE       18L
#define MISTEPPER_READ_PACKETSIZE       86L

#define MISTEPPER_TRANSMIT_DATA_BIT     0
#define MISTEPPER_RESET_DEVICE_BIT      1
///////////////////////////////////////////////////////////////////////////

uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size) {

    uint16_t i, j;

    uint16_t crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

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

        ~ExtDevice()  {}
};

class miStepper {
    private:
    ExtDevice   BME280;
    UARTPeriph  *comm;

    boost::shared_ptr<std::thread> uart_communication_loop_thread;      // Loop to build/read UART request packages
    boost::shared_ptr<std::thread> i2c_communication_loop_thread;       // Loop to build/read I2C request packages

    boost::shared_ptr<std::thread> publish_hardware_data_thread;
    ros::Publisher  hardware_data_publisher;

    ros::NodeHandle nh_;
    ros::Subscriber userWriteReq_subscriber;

    std::string packetDataFrame;

    uint32_t seq = 0;

    _miStepperDmd       Demand_Handle = {  0  };
    struct UARTRecord {
        uint8_t     packetArray[MISTEPPER_READ_PACKETSIZE];
    };

    UARTRecord  ThreadIntercfBuff[10];
    GenBuffer<UARTRecord> ThdIntfc_Handle;

    public:
    void writeRequest(uint8_t InterfaceRe, float FanDemd, uint8_t STPEnb, uint8_t STPGer, uint8_t STPDir, uint16_t STPFreq) {
        uint8_t reqData[MISTEPPER_WRIT_PACKETSIZE]  =  {  0  };
        reqData[0x00]   = 0xFF;
        reqData[0x01]   = 0xA5;

        reqData[0x03]   = InterfaceRe;
        DataManip::_float_2_4x8bit(  FanDemd,  &reqData[0x04]  );

        reqData[0x08]   = STPEnb;
        reqData[0x09]   = STPGer;
        reqData[0x0A]   = STPDir;

        DataManip::_16bit_2_2x8bit(  STPFreq,  &reqData[0x0C]  );

        uint16_t temp = update_crc(0, &reqData[0], MISTEPPER_WRIT_PACKETSIZE - 2);
        DataManip::_16bit_2_2x8bit(     temp,  &reqData[0x10]  );


        comm->PoleTransmit(&reqData[0], MISTEPPER_WRIT_PACKETSIZE);
    }

    void manageUartConnectionLoop() {
        UARTRecord  tempRecord = { 0 };
        UARTPeriph lcComm("/dev/serial0", 500000, 1024);
        /* So as to support this higher UART BAUD RATE, the Raspi needs the following changes:
         *   - /boot/config.txt has to be updated with 'init_uart_clock=14745600' (0xE10000)
         */

        uint32_t datReadPnt = 2;

        comm = &lcComm;

        packetDataFrame = "Data - INIT";

        ros::Rate uart_comm_rate(MISTEPPER_CTRLLOOP_UART_HZ);

        // Transmit initial requests to device, to get initialised
        this->writeRequest((uint8_t)(1 << MISTEPPER_RESET_DEVICE_BIT) ,  0.00, 0, 0, 0, 0);
        ros::Duration(0.5).sleep(); // sleep for half a second

        int dataCount = 0;
        uint8_t RdStatus = 0;
        uint8_t readBack = 0;

        while (ros::ok()) {
            // FIRST READ ANY DATA THAT IS IN THE USART HARDWARE BUFFER!!
            //###########################################################
            dataCount = comm->AnySerDataAvil();     // Get count of entries within buffer (if none will
                                                    // return -1)

            packetDataFrame = "Data - Search";

            if (dataCount <= 0) { packetDataFrame = "Data - None"; }

            while ( (dataCount != 0) && (dataCount != -1) ) {
                // Only loop whilst there is data still to read in buffer, and there is valid data
                // (i.e. not "-1")
                readBack  =  comm->PoleSingleRead();
                dataCount--;    // Decrement number of bytes left in buffer to read

                if      ( (RdStatus == 0x00) && (readBack == 0xFF) ) {
                    RdStatus = 0x01;
                    tempRecord.packetArray[datReadPnt++] = readBack;
                    packetDataFrame = "Data - Stp1";
                }
                else if ( (RdStatus == 0x01) && (readBack == 0x5A) ) {
                    RdStatus = 0x02;
                    tempRecord.packetArray[datReadPnt++] = readBack;
                    packetDataFrame = "Data - Stp2";
                }
                else if (RdStatus == 0x02) {
                    tempRecord.packetArray[datReadPnt++] = readBack;

                    packetDataFrame = "Data - Stp3";

                    if (datReadPnt >= MISTEPPER_READ_PACKETSIZE) {   // If all data has been read back then
                        // Check to see if data is valid (passes CRC)
                        if (update_crc(0, &tempRecord.packetArray[0], MISTEPPER_READ_PACKETSIZE) == 0) {
                            packetDataFrame = "Data - OK";
                            ThdIntfc_Handle.InputWrite(tempRecord);
                        } else {
                            packetDataFrame = "Data - BAD";
                        }

                        for (datReadPnt = 0; datReadPnt != MISTEPPER_READ_PACKETSIZE; datReadPnt++) {    // Cycle through array
                            tempRecord.packetArray[datReadPnt] = 0;                             // and clear contents
                        }
                    RdStatus   = 0x00;      // Return ReadStatus back to start
                    datReadPnt = 0;         // Reset pointer to start valid point
                    }
                }
                else { RdStatus = 0x00; datReadPnt = 0; }
            }
            // SECOND WRITE DATA TO THE USART HARDWARE BUFFER FOR TRANSMISSION
            //################################################################
            // Using pole transmission, build request to hardware
            // Now kick off a write transmit to the miStepper device, with the user requests, and 
            // the flag indicating data is to be transmitted back (to local computer)
            this->writeRequest((Demand_Handle.InterfaceReg | (uint8_t)(1 << MISTEPPER_TRANSMIT_DATA_BIT) ),
                                Demand_Handle.FanDmd,
                                Demand_Handle.STPEnable,
                                Demand_Handle.STPGear,
                                Demand_Handle.STPDir,
                                Demand_Handle.STPFreq);

            uart_comm_rate.sleep();
        }
    }

    void manageI2CConnectionLoop() {
        ros::Rate i2c_comm_rate(MISTEPPER_CTRLLOOP_I2C_HZ);

        while (ros::ok()) {
            BME280.getSensor();
            i2c_comm_rate.sleep();
        }
    }

    void publishHardwareData() {
        mistepper_msgs::openLoopData    parameters;
        UARTRecord  tempRecord = { 0 };

        ros::Rate publish_hardware_data_rate(MISTEPPER_PUB_HZ);

        // Wait for a complete packet
        while (ros::ok()) {
            // Only update the miStepper specific parameters if there is valid data to be decoded
            // otherwise skip this step and use pre-existing values (last good values)
            while (ThdIntfc_Handle.OutputRead(&tempRecord)  != GenBuffer_Empty) {};
            
            parameters.PktCount         = DataManip::_2x8bit_2_16bit(&tempRecord.packetArray[0x02]);
            parameters.AngPos           = DataManip::_4x8bit_2_float(&tempRecord.packetArray[0x04]);
            parameters.SPICmmFlt        = tempRecord.packetArray[0x08];
            parameters.AS5048_ComFlt    = tempRecord.packetArray[0x09];
            parameters.AS5048_DevFlt    = tempRecord.packetArray[0x0A];
            parameters.AS5048_FltCount  = tempRecord.packetArray[0x0B];
            parameters.SPIDur           = GetActTime(&tempRecord.packetArray[0x0C]);
            parameters.SPIPer           = GetActTime(&tempRecord.packetArray[0x0E]);

            parameters.ExtTmp           = DataManip::_4x8bit_2_float(&tempRecord.packetArray[0x10]);
            parameters.I2CCmmFlt        = tempRecord.packetArray[0x14];
            parameters.AD7415_ComFlt    = tempRecord.packetArray[0x15];
            parameters.AD7415_DevFlt    = tempRecord.packetArray[0x16];
            parameters.AD7415_FltCount  = tempRecord.packetArray[0x17];
            parameters.I2CDur           = GetActTime(&tempRecord.packetArray[0x18]);
            parameters.I2CPer           = GetActTime(&tempRecord.packetArray[0x1A]);

            parameters.VoltRef          = DataManip::_4x8bit_2_float(&tempRecord.packetArray[0x1C]);
            parameters.IntTmp           = DataManip::_4x8bit_2_float(&tempRecord.packetArray[0x20]);
            parameters.FANVolt          = DataManip::_4x8bit_2_float(&tempRecord.packetArray[0x24]);
            parameters.FANCurt          = DataManip::_4x8bit_2_float(&tempRecord.packetArray[0x28]);
            parameters.STPVolt          = DataManip::_4x8bit_2_float(&tempRecord.packetArray[0x2C]);
            parameters.STPCurt          = DataManip::_4x8bit_2_float(&tempRecord.packetArray[0x30]);
            parameters.ADCFlt           = tempRecord.packetArray[0x37];

            parameters.ADCDur           = GetActTime(&tempRecord.packetArray[0x38]);
            parameters.ADCPer           = GetActTime(&tempRecord.packetArray[0x3A]);

            parameters.FANAct           = DataManip::_4x8bit_2_float(&tempRecord.packetArray[0x3C]);
            
            parameters.FANDur           = GetActTime(&tempRecord.packetArray[0x40]);
            parameters.FANPer           = GetActTime(&tempRecord.packetArray[0x42]);

            parameters.STPFrequency     = DataManip::_2x8bit_2_16bit(&tempRecord.packetArray[0x44]);
            parameters.STPState         = DataManip::_2x8bit_2_16bit(&tempRecord.packetArray[0x46]);
            parameters.STPcalcPosition  = DataManip::_4x8bit_2_32bit(&tempRecord.packetArray[0x48]);

            parameters.STPDur           = GetActTime(&tempRecord.packetArray[0x4C]);
            parameters.STPPer           = GetActTime(&tempRecord.packetArray[0x4E]);

            parameters.USTDur           = GetActTime(&tempRecord.packetArray[0x50]);
            parameters.USTPer           = GetActTime(&tempRecord.packetArray[0x52]);

            // At this point the "parameter" variable will contain valid data to be published, or
            // hold last good values
            // Following parameters will always be as per latest data
            parameters.header.seq = seq;
            seq++;
            parameters.header.stamp = ros::Time::now();

            //packetDataFrame = "TestMe";
            parameters.header.frame_id = packetDataFrame;

            parameters.MtrTmp           = BME280.Temp;
            parameters.Humidity         = BME280.Humd;
            parameters.Pressure         = BME280.Pres;

            hardware_data_publisher.publish(parameters);
            publish_hardware_data_rate.sleep();
        }
    }

    void UserRequestCallback(const mistepper_msgs::openLoopReqt::ConstPtr& msg) {
        Demand_Handle.InterfaceReg  = msg->InterfaceReg;
        
        Demand_Handle.FanDmd        = msg->FanDmd;

        Demand_Handle.STPEnable     = msg->STPEnable;
        Demand_Handle.STPGear       = msg->STPGear;
        Demand_Handle.STPDir        = msg->STPDir;
        Demand_Handle.STPFreq       = msg->STPFreq;
    }

    miStepper(void) {
        wiringPiSetupGpio();

        ThdIntfc_Handle.create(&ThreadIntercfBuff[0], 10);

        userWriteReq_subscriber     = nh_.subscribe("miStepperReqt", 100, &miStepper::UserRequestCallback, this);

#ifndef DISCONNECT_HARDWARE 
        uart_communication_loop_thread.reset(new std::thread(boost::bind(&miStepper::manageUartConnectionLoop, this)));
#endif
        i2c_communication_loop_thread.reset(new std::thread(boost::bind(&miStepper::manageI2CConnectionLoop, this)));

        hardware_data_publisher = nh_.advertise<mistepper_msgs::openLoopData>("miStepper", 10);
        publish_hardware_data_thread.reset(new std::thread(boost::bind(&miStepper::publishHardwareData, this)));
    }

    ~miStepper() {}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "miStepper_HAL");

    ros::AsyncSpinner spinner(4);   // Utilise all 4 cores
    spinner.start();

    ros::NodeHandle nh;

    miStepper nd;

    ros::waitForShutdown();

    ROS_INFO("Open Loop finished, due to ROS exit");
}