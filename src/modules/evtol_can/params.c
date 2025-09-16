/**20250331
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
 * Evtol Can Interval
 *
 * @unit us
 * @min 0
 * @max 1000000
 * @group DB
 * @value 10000 100Hz
 * @value 5000 200Hz
 * @value 2500 400Hz
 * @value 1250 800Hz
 * @reboot_required true
 */
PARAM_DEFINE_INT32(CANRVE_INTERVAL, 10000);

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
 * @value 100000 100K
 * @value 125000 125K
 * @value 250000 250K
 * @value 500000 500K
 * @value 800000 800K
 * @value 1000000 1M
 * @reboot_required true
 */
PARAM_DEFINE_INT32(CAN_BITRATE, 1000000);

/**
 * DB Servo Check
 *
 * @min 0
 * @max 1
 * @group DB
 * @value 0 Disbale
 * @value 1 Enable
 */
PARAM_DEFINE_INT32(DB_SRV_CHK, 1);

/**
 * Whether send ESC cmd when disarmed
 *
 * @min 0
 * @max 7
 * @group DB
 *
 * @bit 0 enbale left
 * @bit 1 enable right
 * @bit 2 enable rear
 */
PARAM_DEFINE_INT32(DB_ESC_SEND, 7);

/**
 * DB ESC Vendor
 *
 * @min 0
 * @max 1
 * @group DB
 * @value 0 IntelligenceGull
 * @value 1 Sinemotion
 * @reboot_required true
 */
PARAM_DEFINE_INT32(DB_ESC_VDR, 0);

/**
 * DB Aerial Wearable Enable
 *
 * @min 0
 * @max 1
 * @group DB
 * @value 0 Disable
 * @value 1 Enable
 */
PARAM_DEFINE_INT32(DB_AW_EN, 0);

/**
 * HD DCDC power control
 *
 * This param will be reset to 0 (WAITE_CMD) after power cmd sent.
 *
 * @min 0
 * @max 1
 * @group DB
 * @value 0 WAIE_CMD
 * @value 1 POWER_OFF
 * @value 2 POWER_ON
 * @value 4 RESET
 */
PARAM_DEFINE_INT32(DCDC_POW, 0);

/**
 * HD DCDC address
 *
 * for changing dcdc id
 *
 * @min 1
 * @max 254
 * @group DB
 */
PARAM_DEFINE_INT32(DCDC_ADDR, 1);

/**
 * DC Converter Subscribe
 *
 * @min 0
 * @max 1
 * @value 0 Disable
 * @value 1 Enable
 * @group DB
 */
PARAM_DEFINE_INT32(SUB_DCDC, 0);

/**
 * Sinemotion ESC Subscribe
 *
 * @min 0
 * @max 1
 * @value 0 Disable
 * @value 1 Enable
 * @group DB
 */
PARAM_DEFINE_INT32(SUB_SM_ESC, 0);

/**
 * IntelligenceGull ESC Subscribe
 *
 * @min 0
 * @max 1
 * @value 0 Disable
 * @value 1 Enable
 * @group DB
 */
PARAM_DEFINE_INT32(SUB_IG_ESC, 0);

/**
 * Himark Servo Subscribe
 *
 * @min 0
 * @max 1
 * @value 0 Disable
 * @value 1 Enable
 * @group DB
 */
PARAM_DEFINE_INT32(SUB_HMARK_SERVO, 0);

/**
 * Fullymax Battery Subscribe
 *
 * @min 0
 * @max 1
 * @value 0 Disable
 * @value 1 Enable
 * @group DB
 */
PARAM_DEFINE_INT32(SUB_FM_BATT, 0);

/**
 * Redundancy Detector Subscribe
 *
 * @min 0
 * @max 1
 * @value 0 Disable
 * @value 1 Enable
 * @group DB
 */
PARAM_DEFINE_INT32(SUB_REDU_DETEC, 0);

/**
 * Local node id for sinemotion esc
 *
 * @min 1
 * @max 255
 * @group DB
 */
PARAM_DEFINE_INT32(EVTOL_NODE_ID, 1);
