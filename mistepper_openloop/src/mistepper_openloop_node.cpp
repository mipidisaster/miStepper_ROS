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
#define MISTEPPER_WRIT_PACKETSIZE       16L
#define MISTEPPER_READ_PACKETSIZE       84L

#define MISTEPPER_TRANSMIT_DATA_BIT     0
#define MISTEPPER_RESET_DEVICE_BIT      1
///////////////////////////////////////////////////////////////////////////

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

        comm->PoleTransmit(&reqData[0], MISTEPPER_WRIT_PACKETSIZE);
    }

    void manageUartConnectionLoop() {
        UARTRecord  tempRecord = { 0 };
        UARTPeriph lcComm("/dev/serial0", 115200, 1024);
        uint32_t i = 0;

        comm = &lcComm;

        comm->ReceiveIT(UART_Enable);   // Enable recieve "interrupt"

        //ros::Rate uart_comm_rate(MISTEPPER_CTRLLOOP_UART_HZ);

        // Transmit initial requests to device, to get initialised
        this->writeRequest((uint8_t)(1 << MISTEPPER_RESET_DEVICE_BIT) ,  0.00, 0, 0, 0, 0);
        ros::Duration(0.5).sleep(); // sleep for half a second

        double time_hw_last_update;
        time_hw_last_update = ros::Time::now().toSec();

        while (ros::ok()) {
            if (ros::Time::now().toSec() - time_hw_last_update > 1.0/MISTEPPER_CTRLLOOP_UART_HZ) {
                time_hw_last_update += 1.0/MISTEPPER_CTRLLOOP_UART_HZ;

                // Using pole transmission, build request to hardware
                // Now kick off a write transmit to the miStepper device, with the user requests, and 
                // the flag indicating data is to be transmitted back (to local computer)
                this->writeRequest((Demand_Handle.InterfaceReg | (uint8_t)(1 << MISTEPPER_TRANSMIT_DATA_BIT) ),
                                    Demand_Handle.FanDmd,
                                    Demand_Handle.STPEnable,
                                    Demand_Handle.STPGear,
                                    Demand_Handle.STPDir,
                                    Demand_Handle.STPFreq);

                while (!(ros::Time::now().toSec() - time_hw_last_update > 1.0/MISTEPPER_CTRLLOOP_UART_HZ)) {
                    if (comm->AnySerDataAvil() >= MISTEPPER_READ_PACKETSIZE) {  // see how many packets of data are
                        // available within buffer
                        // As the input has a certain sequence, need to ensure that the data is read correctly if
                        // either device looses sync:
                        // As will enter this loop only when the required number of bytes is available, just need to
                        // check for the first 2 bytes to be correct, otherwise break out of if
                        if (comm->PoleSingleRead() == 0xFF) {
                            if (comm->PoleSingleRead() == 0x5A) {
                                for (i = 2; i != MISTEPPER_READ_PACKETSIZE; i++) {  // Cycle through array
                                    tempRecord.packetArray[i] = comm->PoleSingleRead();
                                }

                                ThdIntfc_Handle.InputWrite(tempRecord);
                                for (i = 0; i != MISTEPPER_READ_PACKETSIZE; i++) {  // Cycle through array
                                    tempRecord.packetArray[i] = 0;                  // and clear contents
                                }
                            }
                        }
                    }
                }
            }
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
        uint32_t seq = 0;

        ros::Rate publish_hardware_data_rate(MISTEPPER_PUB_HZ);

        // Wait for a complete packet
        while (ros::ok()) {
            // Only update the miStepper specific parameters if there is valid data to be decoded
            // otherwise skip this step and use pre-existing values (last good values)
            while (ThdIntfc_Handle.OutputRead(&tempRecord)  != GenBuffer_Empty) {
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
            };
            // At this point the "parameter" variable will contain valid data to be published, or
            // hold last good values
            // Following parameters will always be as per latest data
            parameters.header.seq = seq;
            seq++;
            parameters.header.stamp = ros::Time::now();

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
        ros::Duration(2).sleep(); // sleep for two seconds, to allow hardware to go through initial cycle(s)

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