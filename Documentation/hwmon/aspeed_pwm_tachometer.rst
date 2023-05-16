Kernel driver aspeed_pwm_tachometer
===================================

Supported chips:
	ASPEED AST2600

Authors:
	Ryan Chen <ryan_chen@aspeedtech.com>

Description:
------------
This driver implements support for ASPEED AST2600 PWM and Fan Tacho
controller. The PWM controller supports upto 16 PWM outputs. The Fan tacho
controller supports up to 16 tachometer inputs.

The driver provides the following sensor accesses in sysfs:

=============== ======= =====================================================
fanX_input	ro	provide current fan rotation value in RPM as reported
			by the fan to the device.

pwmX		rw	get or set PWM fan control value. This is an integer
			value between 0(off) and 255(full speed).
=============== ======= =====================================================