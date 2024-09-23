/**
 * Test Battery
 *
 *
 * @unit V
 * @min 0
 * @max 53.4
 * @group DB
 */
PARAM_DEFINE_FLOAT(BAT_CELL_V, 22.5f);

/**
 * DB Control Interval
 *
 * @unit us
 * @min 0
 * @max 1000000
 * @group DB
 * @value 10000 100Hz
 * @value 5000 200Hz
 * @value 2500 400Hz
 * @reboot_required true
 */
PARAM_DEFINE_INT32(DB_INTERVAL, 10000);

/**
 * DB Servo Bias
 *
 *
 *
 * @min -500
 * @max 500
 * @group DB
 */
PARAM_DEFINE_INT32(DB_SERVO_BIAS, 0);

/**
 * CAN RATE
 *
 * @min 250000
 * @max 1000000
 * @group DB
 * @value 250000 250K
 * @value 500000 500K
 * @value 800000 800K
 * @value 1000000 1M
 * @reboot_required true
 */
PARAM_DEFINE_INT32(DB_CAN_RATE, 1000000);

/**
 * DB BMS EN
 *
 * @min 0
 * @max 1
 * @group DB
 * @value 0 Disable
 * @value 1 Enable
 */
PARAM_DEFINE_INT32(DB_BMS_EN, 1);

/**
 * DB RC Select
 *
 * @min 0
 * @max 1
 * @group DB
 * @value 0 RC_MAIN
 * @value 1 RC_AUX
 */
PARAM_DEFINE_INT32(DB_RC_SEL, 0);

