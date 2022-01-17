/**************************************************************************************************
 * @file        miStepperUSART.cpp
 * @author      Thomas
 * @brief       << Manually Entered >>
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include "mistepper_driver/miStepperUSART.h"    // Header for miStepper UART interface

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
// None

#elif (defined(zz__MiEmbedType__zz)) && (zz__MiEmbedType__zz ==  0)
//     If using the Linux (No Hardware) version then
//=================================================================================================
// None

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Project Libraries
// -----------------
#include FilInd_GENBUF_TP               // Provide the template for the circular buffer class
#include FilInd_DATMngrHD               // Provide the function set for Data Manipulation

//=================================================================================================
static uint16_t mistepper_crc_table[256] = {
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

miStepperUSART::miStepperUSART(miStepperUSART::DeviceSource configuration,
                               uint8_t *out_array, uint16_t out_array_size,
                               uint8_t *in_array,  uint16_t in_array_size) {
/**************************************************************************************************
 * Construct the miStepperUSART, and populate with the externally defined arrays for message out,
 * and in.
 * Initialise the state machine(s) to "IDLE"/"FREE"/, and clear the message pointer
 *************************************************************************************************/
    device_configuration  =  configuration;
    if (device_configuration == DeviceSource::kmiStepper) {
        read_key        = kSource_PC;
        write_key       = kSource_miStepper;

        _expected_packet_size_  =  18;  // Total bytes expected to be READ by this device
                                        // (including calling bytes, and crc bytes)
    }
    else {
        read_key        = kSource_miStepper;
        write_key       = kSource_PC;

        _expected_packet_size_  =  86;
    }


    message_out.create(out_array, out_array_size);
    message_in.create(in_array,   in_array_size);

    _state_             =   RdState::kIdle;
    _message_in_start_  = 0;

    // PC source signals
    // ==================
    reqt_mode                         = 0;
    reqt_fan_demand                   = 0;


    reqt_stepper_enable               = 0;
    reqt_stepper_gear                 = 0;
    reqt_stepper_direction            = 0;
    reqt_stepper_frequency            = 0;


    // Device source signals
    // ======================
    packet_count                    = 0;

    // SPI parameters
    angular_position                = 0;
    spi1_fault                      = 0;
    angle_sensor_spi_fault          = 0;
    angle_sensor_fault              = 0;
    angle_sensor_idle_count         = 0;
    spi1_task_time                  = 0;

    // I2C parameters (top only)
    internal_temperature_top        = 0;
    i2c1_fault                      = 0;
    top_temp_sensor_i2c_fault       = 0;
    top_temp_sensor_fault           = 0;
    top_temp_sensor_idle_count      = 0;
    i2c1_task_time                  = 0;

    // ADC parameters
    internal_voltage_reference      = 0;
    cpu_temperature                 = 0;
    fan_voltage                     = 0;
    fan_current                     = 0;
    stepper_voltage                 = 0;
    stepper_current                 = 0;
    conversion_fault                = 0;
    adc1_task_time                  = 0;

    // Fan Parameters
    fan_demand                      = 0;
    fan_task_time                   = 0;

    // Stepper Parameters
    stepper_frequency               = 0;
    stepper_state                   = 0;
    stepper_calc_position           = 0;
    stepper_task_time               = 0;

    // USART Parameters
    usart1_task_time                = 0;
}

void miStepperUSART::updateReadStateMachine(uint8_t data_value, uint16_t position) {
/**************************************************************************************************
 * Manage the state machine:
 *      1-> IDLE        : Will listen on the USART till the initial parameter is captured
 *                        ('kdial_tone'). Once this is confirmed, move to CONFIRMING
 *      2-> CONFIRMING  : Will now check for the 'ksource_key', then transition to LISTENING.
 *                        If not 'ksource_key' then return to IDLE
 *      3-> LISTENING   : Valid data to be read to be downloaded. Once expected size of data
 *                        has been read - then check CRC
 *************************************************************************************************/
    if      ( (_state_ == RdState::kIdle)       && (data_value == kdial_tone) ) {
        _message_in_start_ = position;
        _state_ = RdState::kConfirming;
    }
    else if ( (_state_ == RdState::kConfirming) && (data_value == read_key) ) {

        _state_ = RdState::kListening;
    }
    else {
        _state_ = RdState::kIdle;
    }
}

uint16_t miStepperUSART::update_crc(uint16_t crc_accum,
                                    uint8_t *data_blk_ptr, uint16_t data_blk_size) {
/**************************************************************************************************
 * Calculate the crc of the miStepperUSART
 *************************************************************************************************/
    uint16_t i, j;

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ mistepper_crc_table[i];
    }

    return crc_accum;
}

uint16_t miStepperUSART::update_crc(uint16_t crc_accum, GenBuffer<uint8_t> *data_blk_ptr,
                                    uint16_t message_start, uint16_t data_blk_size) {
/**************************************************************************************************
 * Calculate the crc of the miStepperUSART (overloaded function)
 * This version makes use of the GenBuffer, starting the crc calculation from position
 * '_message_in_start_', upto 'data_blk_size'.
 *************************************************************************************************/
    uint16_t i, j;

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr->pa[( (_message_in_start_ + j) %
                                                              message_in.length )])
            & 0xFF;

        crc_accum = (crc_accum << 8) ^ mistepper_crc_table[i];
    }

    return crc_accum;
}

void miStepperUSART::encodedecode_ByteMessage(uint8_t position, uint8_t&  data,
                                              miStepperUSART::DeviceSource source) {
/**************************************************************************************************
 * Will add the contents of 'data' to buffer.
 *************************************************************************************************/
    if (source == device_configuration) {
        message_out.pa[position]  =  data;
    }
    else {
        data = message_in.pa[ ( (_message_in_start_ + position) % message_in.length ) ];
    }
}

void miStepperUSART::encodedecode_WordMessage(uint8_t position, uint16_t&  data,
                                              miStepperUSART::DeviceSource source) {
/**************************************************************************************************
 * Will add the contents of 'data' to buffer.
 *************************************************************************************************/
    if (source == device_configuration) {
        DataManip::_16bit_2_2x8bit(data,  &message_out.pa[position]);
    }
    else {
        uint8_t temp_data[2] = { 0 };
        for (uint8_t i = 0; i != 2; i++) {
            temp_data[i] = message_in.pa[
                           ( (_message_in_start_ + position + i) % message_in.length )
                           ];
        }

        data = DataManip::_2x8bit_2_16bit(temp_data);
    }
}

void miStepperUSART::encodedecodeDWordMessage(uint8_t position, uint32_t&  data,
                                              miStepperUSART::DeviceSource source) {
/**************************************************************************************************
 * Will add the contents of 'data' to buffer.
 *************************************************************************************************/
    if (source == device_configuration) {
        DataManip::_32bit_2_4x8bit(data,  &message_out.pa[position]);
    }
    else {
        uint8_t temp_data[4] = { 0 };
        for (uint8_t i = 0; i != 4; i++) {
            temp_data[i] = message_in.pa[
                           ( (_message_in_start_ + position + i) % message_in.length )
                           ];
        }

        data = DataManip::_4x8bit_2_32bit(temp_data);
    }
}

void miStepperUSART::encodedecodeFloatMessage(uint8_t position, float&  data,
                                              miStepperUSART::DeviceSource source) {
/**************************************************************************************************
 * Will add the contents of 'data' to buffer.
 *************************************************************************************************/
    if (source == device_configuration) {
        DataManip::_float_2_4x8bit(data,  &message_out.pa[position]);
    }
    else {
        uint8_t temp_data[4] = { 0 };
        for (uint8_t i = 0; i != 4; i++) {
            temp_data[i] = message_in.pa[
                           ( (_message_in_start_ + position + i) % message_in.length )
                           ];
        }

        data = DataManip::_4x8bit_2_float(temp_data);
    }
}

void miStepperUSART::decodeMessage(void) {
/**************************************************************************************************
 * Will check for any new input messages, then feed it through the state machine till the correct
 * message configuration is provided.
 *************************************************************************************************/
    uint8_t     read_back = 0x00;   // Variable for retrieving the data from array
    uint16_t    temp_value = message_in.output_pointer;
        // Store the current position of GenBuffer (so as to save on calculating the back step if
        // the 'message_in' is part of a valid message

    while(message_in.outputRead(&read_back) != kGenBuffer_Empty) {
        if (_state_ != RdState::kListening) {
            updateReadStateMachine(read_back, temp_value);
        }
        else {
            temp_value = (uint16_t) ( ( message_in.length + message_in.output_pointer
                                          - _message_in_start_ )
                                      % message_in.length);
            /* Calculate the size of data read in; by taking the delta of the message start -
             * '_message_in_start_', and the current read data from buffer.
             * Rest of calculation takes into account of buffer size
             */

            if (temp_value >= _expected_packet_size_) {
                _state_ = RdState::kIdle;

                if (update_crc(0, &message_in, _message_in_start_, _expected_packet_size_) == 0)
                {
                    if (device_configuration == DeviceSource::kmiStepper) {
                        miStepperIn();
                    }
                    else {
                        miStepperOut();
                    }
                    //temp_value = (uint16_t) ( (_message_in_start_ + 3) % _message_in_.length );

                    //mode = _message_in_.pa[ ( (_message_in_start_ + 3) % _message_in_.length ) ];

            }}
        }

        temp_value = message_in.output_pointer;
    }
}

void miStepperUSART::miStepperIn(void) {
/**************************************************************************************************
 * Function will generate the required packet and structure for transmission out of PC
 *  OR
 * Decode the data read FROM PC
 *************************************************************************************************/
    if (device_configuration == DeviceSource::kControlPC) {
    message_out.qFlush();   // Flush contents of buffer, to ensure that entry [0] is starting
                            // point.

    uint8_t dial_tone = kdial_tone;
    encodedecode_ByteMessage(0x00,  dial_tone                       , device_configuration);
    encodedecode_ByteMessage(0x01,  write_key                       , device_configuration);
    }

    encodedecode_ByteMessage(0x03,  reqt_mode                       , DeviceSource::kControlPC);

    encodedecodeFloatMessage(0x04,  reqt_fan_demand                 , DeviceSource::kControlPC);

    encodedecode_ByteMessage(0x08,  reqt_stepper_enable             , DeviceSource::kControlPC);
    encodedecode_ByteMessage(0x09,  reqt_stepper_gear               , DeviceSource::kControlPC);
    encodedecode_ByteMessage(0x0A,  reqt_stepper_direction          , DeviceSource::kControlPC);
    encodedecode_WordMessage(0x0C,  reqt_stepper_frequency          , DeviceSource::kControlPC);

    if (device_configuration == DeviceSource::kControlPC) {
        uint16_t crc_value = update_crc(0, &message_out.pa[0],
                                        16);

        encodedecode_WordMessage(0x10,  crc_value, device_configuration);

        message_out.input_pointer = 0x12;
    }
}

void miStepperUSART::miStepperOut(void) {
/**************************************************************************************************
 * Function will generate the required packet and structure for transmission out of miStepper
 *  OR
 * Decode the data read FROM miStepper
 *************************************************************************************************/
    if (device_configuration == DeviceSource::kmiStepper) {
    message_out.qFlush();   // Flush contents of buffer, to ensure that entry [0] is starting
                            // point.

    uint8_t dial_tone = kdial_tone;
    encodedecode_ByteMessage(0x00,  dial_tone                       , device_configuration    );
    encodedecode_ByteMessage(0x01,  write_key                       , device_configuration    );
    }

    encodedecode_WordMessage(0x02,  packet_count                    , DeviceSource::kmiStepper);

    // SPI parameters
    encodedecodeFloatMessage(0x04,  angular_position                , DeviceSource::kmiStepper);
    encodedecode_ByteMessage(0x08,  spi1_fault                      , DeviceSource::kmiStepper);
    encodedecode_ByteMessage(0x09,  angle_sensor_spi_fault          , DeviceSource::kmiStepper);
    encodedecode_ByteMessage(0x0A,  angle_sensor_fault              , DeviceSource::kmiStepper);
    encodedecode_ByteMessage(0x0B,  angle_sensor_idle_count         , DeviceSource::kmiStepper);
    encodedecodeDWordMessage(0x0C,  spi1_task_time                  , DeviceSource::kmiStepper);

    // I2C parameters (top only)
    encodedecodeFloatMessage(0x10,  internal_temperature_top        , DeviceSource::kmiStepper);
    encodedecode_ByteMessage(0x14,  i2c1_fault                      , DeviceSource::kmiStepper);
    encodedecode_ByteMessage(0x15,  top_temp_sensor_i2c_fault       , DeviceSource::kmiStepper);
    encodedecode_ByteMessage(0x16,  top_temp_sensor_fault           , DeviceSource::kmiStepper);
    encodedecode_ByteMessage(0x17,  top_temp_sensor_idle_count      , DeviceSource::kmiStepper);
    encodedecodeDWordMessage(0x18,  i2c1_task_time                  , DeviceSource::kmiStepper);

    // ADC parameters
    encodedecodeFloatMessage(0x1C,  internal_voltage_reference      , DeviceSource::kmiStepper);
    encodedecodeFloatMessage(0x20,  cpu_temperature                 , DeviceSource::kmiStepper);
    encodedecodeFloatMessage(0x24,  fan_voltage                     , DeviceSource::kmiStepper);
    encodedecodeFloatMessage(0x28,  fan_current                     , DeviceSource::kmiStepper);
    encodedecodeFloatMessage(0x2C,  stepper_voltage                 , DeviceSource::kmiStepper);
    encodedecodeFloatMessage(0x30,  stepper_current                 , DeviceSource::kmiStepper);

    encodedecode_ByteMessage(0x37,  conversion_fault                , DeviceSource::kmiStepper);
    encodedecodeDWordMessage(0x38,  adc1_task_time                  , DeviceSource::kmiStepper);

    // Fan Parameters
    encodedecodeFloatMessage(0x3C,  fan_demand                      , DeviceSource::kmiStepper);
    encodedecodeDWordMessage(0x40,  fan_task_time                   , DeviceSource::kmiStepper);

    // Stepper Parameters
    encodedecode_WordMessage(0x44,  stepper_frequency               , DeviceSource::kmiStepper);
    encodedecode_WordMessage(0x46,  stepper_state                   , DeviceSource::kmiStepper);
    encodedecodeDWordMessage(0x48,  stepper_calc_position           , DeviceSource::kmiStepper);
    encodedecodeDWordMessage(0x4C,  stepper_task_time               , DeviceSource::kmiStepper);

    // USART Parameters
    encodedecodeDWordMessage(0x50,  usart1_task_time                , DeviceSource::kmiStepper);

    if (device_configuration == DeviceSource::kmiStepper) {
        uint16_t crc_value = update_crc(0, &message_out.pa[0],
                                        84);

        encodedecode_WordMessage(0x54,  crc_value, device_configuration);

        message_out.input_pointer = 0x56;
    }
}

float miStepperUSART::getTaskDuration(uint32_t data) {
/**************************************************************************************************
 * Calculate the Task duration based upon the 32bit compressed data from the miStepper device.
 *************************************************************************************************/
// Duration data is located in the upper 16bits of this data
    uint16_t temp = (uint16_t) ((data & 0xFFFF0000) >> 16);

    return ((float) temp / ktask_count_rate );
}

float miStepperUSART::getTaskPeriod(uint32_t data) {
/**************************************************************************************************
 * Calculate the Task period based upon the 32bit compressed data from the miStepper device.
 *************************************************************************************************/
    // Period data is located in the lower 16bits of this data
    uint16_t temp = (uint16_t) (data & 0x0000FFFF);

    return ((float) temp / ktask_count_rate );
}

miStepperUSART::~miStepperUSART()
{
    // TODO Auto-generated destructor stub
}

