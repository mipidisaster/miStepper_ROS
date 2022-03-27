/**************************************************************************************************
 * @file        mistepper_hst_node.cpp
 * @author      Thomas
 * @brief       ROS node for USART/direct interface to miStepper
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This node is called "mistepper_hst"
 *   Configuration parameters:
 *      ~/config        /loop_rate
 *                      [ Floating data value, specifying the rate that the miStepper device is
 *                      [ interrogated.
 *                      [ - Note, if none is provided will defaul to '100Hz'
 *
 *   Publishers:
 *      data_stream
 *                      [ Provides a constant stream of data read from the miStepper device;
 *                      [ rate of update is based upon '~/config/loop_rate' as above.
 *                      [ Uses mistepper_msgs/msg/openLoopData.msg
 *
 *   Subscribers:
 *      request
 *                     [ Stream of data from 'user' specify what the device is met to do.
 *                     [ Uses mistepper_msgs/msg/openLoopData.msg
 *
 *   Services:
 *      None
 *
 *************************************************************************************************/
// C System Header(s)
// ------------------
#include <stdint.h>
#include <vector>       // for std::vector
#include <string>       // for strings

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// ---------------
#include <ros/ros.h>
#include <ros/callback_queue.h>

// Project Libraries
// -----------------
#include "milibrary/BUSctrl.h"

#include "mistepper_msgs/openLoopData.h"
#include "mistepper_msgs/openLoopReqt.h"

#include "mistepper_driver/miStepperUSART.h"    // Header for miStepper UART interface

//=================================================================================================
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
#define         MISTEPPER_BUFFER_SIZE       1024

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
class rosmiStepperHST {
private:
    std::string         kconfig_sub_area            = "config/";

    std::string         knode_loop_rate             = "loop_rate";

    std::string         kMStp_publish_data          = "data_stream";
    std::string         kMStp_subscribe_request     = "request";

    // Inputs
    std::string         kUART_read_service          = "read_uart";
    std::string         kUART_write_service         = "write_uart";

private:
    miStepperUSART          *_hardware_handle_;

    uint8_t                 _comm_buffer_[2][MISTEPPER_BUFFER_SIZE] = { 0 };
    uint32_t                _packet_sequence_                       =  0;

/**************************************************************************************************
 * == ROS STUFF == >>>   ROBOT OPERATING SYSTEM (ROS) OBJECTS    <<<
 *   -----------
 *  The follow are objects used for interfacing with the Robot Operating System.
 *************************************************************************************************/
    ros::NodeHandle         _nh_;
    ros::NodeHandle         _private_nh_;

    ros::NodeHandle         _nh_hardware_;
    ros::CallbackQueue      _hardware_callback_queue_;

    // PARAMETERS
    ////////////////////////
    double                          _loop_rate_parameter_;

    // MESSAGES
    ////////////////////////

    // PUBLISHERS
    ////////////////////////
    ros::Publisher                  _mistepper_data_publisher_;

    // SUBSCRIBERS
    ////////////////////////
    ros::Subscriber                 _mistepper_request_subscriber_;

    // TIMERS
    ////////////////////////

    // SERVICES
    ////////////////////////
    ros::ServiceClient              _usart_read_client_;
    ros::ServiceClient              _usart_write_client_;

    // ACTIONS
    ////////////////////////

public:
/*
 *  @brief:  Create the mistepper openloop node class, basic construction
 *
 *  @param:  Pointer to the 'NodeHandle'
 *  @param:  Pointer to the 'NodeHandle', setup for private - ("~")
 *  @retval: rosUART class
 */
    rosmiStepperHST(ros::NodeHandle* normal, ros::NodeHandle* private_namespace):
    _nh_(*normal), _private_nh_(*private_namespace)
    {
        //_nh_hardware_.setCallbackQueue(&_hardware_callback_queue_);

        if (configNode() < 0) {
            ROS_ERROR("Error detected during miStepper open loop construction, exiting node...");
            return;
        }

        nodeLoop();
    }

    /*
     *  @brief:  Function to call up the usart write client to send the packet(s) of data to
     *           miStepper
     *
     *  @param:  void
     *  @retval: void
     */
    void callUSARTWriteClient(void) {
        milibrary::BUSctrl          msg;

        // Clear the data initially
        msg.request.address   = 0;
        msg.request.read_size = 0;
        msg.request.write_data.clear();

        _hardware_handle_->miStepperIn();       // Construct the miStepper packet
        // Push data into the ROS Bus message
        while(_hardware_handle_->message_out.state() != kGenBuffer_Empty) {
            uint8_t temp_character = 0x00;
            _hardware_handle_->message_out.outputRead(&temp_character);
            msg.request.write_data.push_back(temp_character);
        }
        _usart_write_client_.call(msg);
    }

    /*
     *  @brief:  Function to call up the usart read client to receive the packet(s) of data from
     *           miStepper
     *
     *  @param:  void
     *  @retval: void
     */
    void callUSARTReadClient(void) {
        milibrary::BUSctrl          msg;

        msg.request.address   = 0;
        msg.request.read_size = 0;
        msg.request.write_data.clear();

        _usart_read_client_.call(msg);
        // Pull data out og ROS Bus message
        for (uint8_t j = 0; j != msg.response.read_data.size(); j++) {
            _hardware_handle_->message_in.inputWrite(
                               msg.response.read_data.at(j)
                               );
        }

        _hardware_handle_->decodeMessage();
    }

    /*
     *  @brief:  Separate function, to handle the hardware service callback queue.
     *           Intended to be used within a dedicated thread.
     *
     *  @param:  void
     *  @retval: void
     */
    void hardwareCallbackThread(void) {
        ros::SingleThreadedSpinner spinner;
        spinner.spin(&_hardware_callback_queue_);
    }

    /*
     *  @brief:  Function to encapsulate the looping of this node.
     *
     *  @param:  void
     *  @retval: void
     */
    void nodeLoop(void) {
        ROS_INFO("miStepper open loop node ready for use");

        ros::Rate loop_rate(_loop_rate_parameter_);

        _hardware_handle_->reqt_mode        = miStepperUSART::kreset_packetcount;
        callUSARTWriteClient();
        ros::Duration(0.5).sleep(); // sleep for half a second.

        _hardware_handle_->reqt_mode        = 0;

        while (ros::ok())
        {
            callUSARTReadClient();
            callbackDatapublish();

            _hardware_handle_->reqt_mode    |= miStepperUSART::kenable_transmit;
            callUSARTWriteClient();

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    /*
     *  @brief:
     *
     *  @param:  void
     *  @retval: Integer value - 0 = no issues, -1 = issues detected
     */
    int configNode(void)
    {
        //=========================================================================================
        // Input parameters
        _private_nh_.param<double>(kconfig_sub_area + knode_loop_rate,
                                   _loop_rate_parameter_,
                                   100.0);

        //=========================================================================================
        // Duplication check


        //=========================================================================================

        _hardware_handle_ = new miStepperUSART(miStepperUSART::DeviceSource::kControlPC,
                                               &_comm_buffer_[0][0], MISTEPPER_BUFFER_SIZE,
                                               &_comm_buffer_[1][0], MISTEPPER_BUFFER_SIZE);

        ROS_INFO("miStepper open loop has been setup");
        //=========================================================================================
        // Publishers
        _mistepper_data_publisher_ = _nh_.advertise<mistepper_msgs::openLoopData>(
                                                kMStp_publish_data,
                                                20);

        _mistepper_request_subscriber_ = _nh_.subscribe(
                                                kMStp_subscribe_request,
                                                20,
                                                &rosmiStepperHST::callbackReqestSubscriber,
                                                this);

        //=========================================================================================
        // Timers

        //=========================================================================================
        // Clients/Servers
        for (uint8_t i = 0; i != 10; i++) {
            if ( !(ros::service::exists(kUART_read_service, false))  ||
                 !(ros::service::exists(kUART_write_service, false)) )  {
                ROS_WARN("Required services are not currently available...pause count %d", i);
                ros::Duration(1).sleep(); // sleep for a second.
            }
            else {
                break;
            }

            // On last iteration
            if (i == 9) {
                ROS_ERROR("Timed out waiting for the services to be setup, shutdowning node...");
                return -1;
            }
        }

        _usart_read_client_  = _nh_.serviceClient<milibrary::BUSctrl>(
                                                kUART_read_service,
                                                true);
        _usart_write_client_ = _nh_.serviceClient<milibrary::BUSctrl>(
                                                kUART_write_service,
                                                true);

        //=========================================================================================

        ROS_INFO("miStepper open loop node constructed");
        return 0;   // If got to this point, no errors were detected
    }

    /*
     *  @brief:  Function (could be callback) to be used to publish the read data from miStepper
     *           into ROS workspace.
     *
     *  @param:  None
     *  @retval: None
     */
    void callbackDatapublish(void) {
        mistepper_msgs::openLoopData    msg;
        msg.header.seq      = _packet_sequence_++;
        msg.header.stamp    = ros::Time::now();

        // Packet Count
        msg.PktCount        = _hardware_handle_->packet_count;

        // Angular Position
        msg.AngPos          = _hardware_handle_->angular_position;

        // Power Parameters
        msg.VoltRef         = _hardware_handle_->internal_voltage_reference;
        msg.FANVolt         = _hardware_handle_->fan_voltage;
        msg.FANCurt         = _hardware_handle_->fan_current;
        msg.STPVolt         = _hardware_handle_->stepper_voltage;
        msg.STPCurt         = _hardware_handle_->stepper_current;

        // Temperatures
        msg.ExtTmp          = _hardware_handle_->internal_temperature_top;
        msg.IntTmp          = _hardware_handle_->cpu_temperature;

        // Control surfaces
        msg.FANAct          = _hardware_handle_->fan_demand;

        msg.STPFrequency    = _hardware_handle_->stepper_frequency;
        msg.STPState        = _hardware_handle_->stepper_state;
        msg.STPcalcPosition = _hardware_handle_->stepper_calc_position;

        // Task timers
        msg.SPIDur          = _hardware_handle_->getTaskDuration(
                                                 _hardware_handle_->spi1_task_time);
        msg.SPIPer          = _hardware_handle_->getTaskPeriod(
                                                 _hardware_handle_->spi1_task_time);

        msg.I2CDur          = _hardware_handle_->getTaskDuration(
                                                 _hardware_handle_->i2c1_task_time);
        msg.I2CPer          = _hardware_handle_->getTaskPeriod(
                                                 _hardware_handle_->i2c1_task_time);

        msg.ADCDur          = _hardware_handle_->getTaskDuration(
                                                 _hardware_handle_->adc1_task_time);
        msg.ADCPer          = _hardware_handle_->getTaskPeriod(
                                                 _hardware_handle_->adc1_task_time);

        msg.USTDur          = _hardware_handle_->getTaskDuration(
                                                 _hardware_handle_->usart1_task_time);
        msg.USTPer          = _hardware_handle_->getTaskPeriod(
                                                 _hardware_handle_->usart1_task_time);

        msg.FANDur          = _hardware_handle_->getTaskDuration(
                                                 _hardware_handle_->fan_task_time);
        msg.FANPer          = _hardware_handle_->getTaskPeriod(
                                                 _hardware_handle_->fan_task_time);

        msg.STPDur          = _hardware_handle_->getTaskDuration(
                                                 _hardware_handle_->stepper_task_time);
        msg.STPPer          = _hardware_handle_->getTaskPeriod(
                                                 _hardware_handle_->stepper_task_time);

        // Fault flags
        msg.SPICmmFlt       = _hardware_handle_->spi1_fault;
        msg.AS5048_ComFlt   = _hardware_handle_->angle_sensor_fault;
        msg.AS5048_DevFlt   = _hardware_handle_->angle_sensor_spi_fault;
        msg.AS5048_FltCount = _hardware_handle_->angle_sensor_idle_count;

        msg.I2CCmmFlt       = _hardware_handle_->i2c1_fault;
        msg.AD7415_ComFlt   = _hardware_handle_->top_temp_sensor_fault;
        msg.AD7415_DevFlt   = _hardware_handle_->top_temp_sensor_i2c_fault;
        msg.AD7415_FltCount = _hardware_handle_->top_temp_sensor_idle_count;

        msg.ADCFlt          = _hardware_handle_->angle_sensor_fault;

        _mistepper_data_publisher_.publish(msg);
    }

    /*
     *  @brief:  Callback function for the subscription to read any requests on the miStepper
     *           device.
     *
     *  @param:  mistepper_msg OpenLoopReqt
     *  @retval: None
     */
    void callbackReqestSubscriber(const mistepper_msgs::openLoopReqt::ConstPtr& msg) {
        _hardware_handle_->reqt_mode        = msg->InterfaceReg |
                                              miStepperUSART::kenable_transmit;

        _hardware_handle_->reqt_fan_demand          = msg->FanDmd;

        _hardware_handle_->reqt_stepper_enable      = msg->STPEnable;
        _hardware_handle_->reqt_stepper_gear        = msg->STPGear;
        _hardware_handle_->reqt_stepper_direction   = msg->STPDir;
        _hardware_handle_->reqt_stepper_frequency   = msg->STPFreq;
    }

    ~rosmiStepperHST() {
        ROS_INFO("Shutting down the node, and killing functions");
        delete _hardware_handle_;
    }

};

//=================================================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mistepper_hst");
    ros::NodeHandle n;
    ros::NodeHandle private_params("~");

    rosmiStepperHST  node_miStepper_openloop(&n, &private_params);

    ros::waitForShutdown();

    // On node shutdown, don't think it reaches this part of main()
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
// None

