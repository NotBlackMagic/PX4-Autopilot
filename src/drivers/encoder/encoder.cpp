#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

#include <drivers/device/i2c.h>

#include <lib/perf/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/topics/wheel_encoders.h>
#include <uORB/PublicationMulti.hpp>

#define DRV_DIST_DEVTYPE_ENCODER	0xE0

#define ENCODER_BASEADDR		0x02

class Encoder : public device::I2C, public I2CSPIDriver<Encoder>
{
public:
	Encoder(const I2CSPIDriverConfig &config);

	~Encoder() override;

	static void print_usage();

	int init() override;

	void print_status() override;

	void RunImpl();

private:
	enum class Register : uint8_t {
		EncoderID = 0x00,
		EncoderData = 0x01,
		EncoderPPR = 0x02,
	};

	uORB::PublicationMulti<wheel_encoders_s> _wheelEncodersAdv[2] { ORB_ID(wheel_encoders), ORB_ID(wheel_encoders)};
	wheel_encoders_s _wheelEncoderMsg[2];

	int probe() override;

	void start();

	int readRegister(Register reg, uint8_t *data, int len);

	int collect();

	int _conversion_interval{-1};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com err")};
};

Encoder::Encoder(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{

}

Encoder::~Encoder()
{
	//Free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int Encoder::init()
{
	int ret = PX4_ERROR;

	//Set conversion interval (sampling time?), in us
	_conversion_interval = 50000;

	//Init Wheel Encoder Messages
	//memset(&_wheelEncoderMsg, 0, sizeof(_wheelEncoderMsg));
	//_wheelEncodersAdv = orb_advertise(ORB_ID(wheel_encoders), &_wheelEncoderMsg);
	_wheelEncoderMsg[0].pulses_per_rev = 2;
	_wheelEncoderMsg[1].pulses_per_rev = 2;

	//Do I2C init (and probe) first
	ret = I2C::init();

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

int Encoder::readRegister(Register reg, uint8_t *data, int len)
{
	const uint8_t cmd = (uint8_t)reg;
	//CMD, Send Length, RX Buffer, RX Length
	return transfer(&cmd, 1, data, len);
}

int Encoder::probe()
{
	int ret = PX4_OK;

	//Read ID
	uint8_t id[2];
	ret = readRegister(Register::EncoderID, id, 1);

	PX4_DEBUG("Enocder ID: %d", id);

	//Validate ID
	if(ret == 0 && id[0] == 'N') {
		return 0;
	}

	return -1;
}

int Encoder::collect()
{
	//Read data from the encoder
	perf_begin(_sample_perf);

	uint8_t data[20];
	if(readRegister(Register::EncoderData, data, 12) < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	perf_end(_sample_perf);

	int dataIndex = 0;
	int _encoderStepsLeft = data[dataIndex++] << 24;
	_encoderStepsLeft += data[dataIndex++] << 16;
	_encoderStepsLeft += data[dataIndex++] << 8;
	_encoderStepsLeft += data[dataIndex++];

	int _motorSpeedsLeft = data[dataIndex++] << 8;
	_motorSpeedsLeft = data[dataIndex++];

	int _encoderStepsRight = data[dataIndex++] << 24;
	_encoderStepsRight += data[dataIndex++] << 16;
	_encoderStepsRight += data[dataIndex++] << 8;
	_encoderStepsRight += data[dataIndex++];

	int _motorSpeedsRight = data[dataIndex++] << 8;
	_motorSpeedsRight = data[dataIndex++];

	//Create a wheel_encoders uORB message from collected data
	_wheelEncoderMsg[0].timestamp = hrt_absolute_time();
	_wheelEncoderMsg[0].encoder_position = _encoderStepsLeft;
	_wheelEncoderMsg[0].speed = _motorSpeedsLeft;

	_wheelEncoderMsg[1].timestamp = hrt_absolute_time();
	_wheelEncoderMsg[1].encoder_position = _encoderStepsRight;
	_wheelEncoderMsg[1].speed = _motorSpeedsRight;

	//Published creaded uORB message
	_wheelEncodersAdv[0].publish(_wheelEncoderMsg[0]);
	_wheelEncodersAdv[1].publish(_wheelEncoderMsg[1]);
	// orb_publish(ORB_ID(wheel_encoders), _wheelEncodersAdv, &_wheelEncoderMsg);

	return PX4_OK;
}

void Encoder::start()
{
	//Schedule a cycle to start things
	ScheduleDelayed(_conversion_interval);
}

void Encoder::RunImpl()
{
	//This is the actuall running code part
	//Collect data
	if (PX4_OK != collect()) {
		PX4_DEBUG("collection error");
	}
	//Delay for _conversion_interval us
	ScheduleDelayed(_conversion_interval);
}

void Encoder::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void Encoder::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
	R"DESCR_STR(
		### Description
	)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("encoder", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("encoder");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(ENCODER_BASEADDR);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int encoder_main(int argc, char *argv[]);

int encoder_main(int argc, char *argv[])
{
	using ThisDriver = Encoder;
	BusCLIArguments cli{true, false};
	// cli.rotation = (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = ENCODER_BASEADDR;

	// while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
	// 	switch (ch) {
	// 	case 'R':
	// 		cli.rotation = (Rotation)atoi(cli.optArg());
	// 		break;
	// 	}
	// }
	//
	// const char *verb = cli.optArg();

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_ENCODER);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
