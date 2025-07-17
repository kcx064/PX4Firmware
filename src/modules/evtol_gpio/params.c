/**
 * DB BMS EN
 *
 * @min 0
 * @max 1
 * @group DB
 * @value 0 Disable
 * @value 1 Enable
 */
PARAM_DEFINE_INT32(BMS_EN, 0);

/**
 * DB RC Select
 *
 * @min 0
 * @max 1
 * @group DB
 * @value 0 RC_MAIN
 * @value 1 RC_AUX
 */
PARAM_DEFINE_INT32(RC_SEL, 0);

/**
 * MIX Precharge
 *
 * @min 0
 * @max 1
 * @group DB
 * @value 0 Disable
 * @value 1 Enable
 */
PARAM_DEFINE_INT32(PRECHARGE, 0);

/**
 * MIX Shutdown
 *
 * @min 0
 * @max 1
 * @group DB
 * @value 0 Disable
 * @value 1 Enable
 */
PARAM_DEFINE_INT32(SHUTDOWN, 0);

/**
 * MIX Shutdown rc channel map
 *
 * @min 4
 * @max 17
 * @group DB
 * @value 4 Channel 5
 * @value 5 Channel 6
 * @value 6 Channel 7
 * @value 7 Channel 8
 * @value 8 Channel 9
 * @value 9 Channel 10
 * @value 10 Channel 11
 * @value 11 Channel 12
 * @value 12 Channel 13
 * @value 13 Channel 14
 * @value 14 Channel 15
 * @value 15 Channel 16
 * @value 16 Channel 17
 * @value 17 Channel 18
 */
PARAM_DEFINE_INT32(STD_CH, 8);
