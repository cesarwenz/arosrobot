#include <ros/node_handle.h>

#include "ros_phoenix/TalonNode.h"
#include <ros/console.h>
#include "std_msgs/Float32.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

namespace ros_phoenix {
TalonNode::TalonNode(const ros::NodeHandle& parent, const std::string& name, int id, const TalonConfig& config)
    : nh(parent)
    , _name(name)
    , server(mutex, nh)
    , _config(config)
    , talon(id)
    , statusPub(nh.advertise<MotorStatus>("status", 1))
    , anglePub(nh.advertise<std_msgs::Float32>("angle", 1))
    , setSub(nh.subscribe("set", 1, &TalonNode::set, this))
    , lastUpdate(ros::Time::now()) // watchdog - turn off talon if we haven't gotten an update in a while
    , _controlMode(ControlMode::PercentOutput)
    , disabled(false)
    , configured(false)
    , not_configured_warned(false)
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    server.updateConfig(_config);
    server.setCallback(boost::bind(&TalonNode::reconfigure, this, _1, _2));
    talon.NeutralOutput();
}

void TalonNode::set(MotorControl output)
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    this->_controlMode = (ControlMode) output.mode;
    this->_output.data = output.value;
    this->lastUpdate = ros::Time::now();
}

void TalonNode::reconfigure(const TalonConfig& config, uint32_t level)
{
    if (level == 0) {
        boost::recursive_mutex::scoped_lock scoped_lock(mutex);
        ROS_INFO("Reconfigure called on %s", _name.c_str());
        this->_config = config;
        this->configured = false;
    }
}

void TalonNode::configure()
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    if (talon.GetFirmwareVersion() == -1) {
        if (!not_configured_warned) {
            ROS_WARN("Talon hasn't been seen: %d", talon.GetDeviceID());
            not_configured_warned = true;
        }
        return;
    }

    TalonFXConfiguration c;
    SlotConfiguration slot;
    slot.kP = _config.P;
    slot.kI = _config.I;
    slot.kD = _config.D;
    slot.kF = _config.F;
    c.slot0 = slot;
    c.voltageCompSaturation = _config.peak_voltage;
    SupplyCurrentLimitConfiguration(
	_config.supply_current_limit_enable, 
	_config.supply_current_limit,
    	_config.supply_current_trigger_threshold_current,
    	_config.supply_current_trigger_threshold_time);
    c.pulseWidthPeriod_EdgesPerRot = 2048;
    ErrorCode error = talon.ConfigAllSettings(c, 10);

    if (error != ErrorCode::OK) {
        if (!not_configured_warned) {
            ROS_WARN("Reconfiguring Talon %s %d failed!", _name.c_str(), talon.GetDeviceID());
            not_configured_warned = true;
        }
        this->configured = false;
    } else {
        configureStatusPeriod();
        talon.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
        talon.SetSensorPhase(_config.invert_sensor);
        talon.SelectProfileSlot(0, 0);
        talon.SetInverted(_config.inverted);
        talon.EnableVoltageCompensation(true);

        if (_config.brake_mode) {
            talon.SetNeutralMode(NeutralMode::Brake);
        } else {
            talon.SetNeutralMode(NeutralMode::Coast);
        }

        ROS_INFO("Reconfigured Talon: %s with %d %f %f %f", _name.c_str(), talon.GetDeviceID(), _config.P, _config.I,
            _config.D);
        this->configured = true;
        this->not_configured_warned = false;
    }
}

void TalonNode::update()
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);

    // Configure Talon if necessary
    if (!this->configured) {
        this->configure();
    }

    // Disable the Talon if we aren't getting commands
    if (ros::Time::now() - lastUpdate > ros::Duration(0.2)) {
        if (!this->disabled)
            ROS_WARN("Talon disabled for not receiving updates: %s", _name.c_str());
        this->disabled = true;
        this->_controlMode = ControlMode::PercentOutput;
        this->_output.data = 0.0;
    } else {
        if (this->disabled)
            ROS_INFO("Talon re-enabled for receiving updates: %s", _name.c_str());
        this->disabled = false;
    }

    // Set Talon outpuudo 
    if (this->_output.data == 0.0 or !this->configured) {
        talon.NeutralOutput();
    } else {
        talon.Set(this->_controlMode, this->_output.data);
    }

    MotorStatus status;
    status.temperature = talon.GetTemperature();
    status.bus_voltage = talon.GetBusVoltage();

    status.output_percent = talon.GetMotorOutputPercent();
    status.output_voltage = talon.GetMotorOutputVoltage();
    status.output_current = talon.GetOutputCurrent();

    status.position = talon.GetSelectedSensorPosition();
    status.velocity = talon.GetSelectedSensorVelocity() * 10;
    
    status.fwd_limit = talon.GetSensorCollection().IsFwdLimitSwitchClosed();
    status.rev_limit = talon.GetSensorCollection().IsRevLimitSwitchClosed();

    std_msgs::Float32 angle;
    angle.data = (status.position * (2*M_PI / 43008)); 

    statusPub.publish(status);
    anglePub.publish(angle);
}

void TalonNode::configureStatusPeriod()
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);

    // Sets status frame polling rate for methods like getTemperature()
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 20);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 20);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 20);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 100);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 20);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 20);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 20);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 100);
}

} // namespace ros_phoenix
