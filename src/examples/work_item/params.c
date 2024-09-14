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

