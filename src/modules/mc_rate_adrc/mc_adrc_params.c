
/**
 * @file mc_adrc_params.c
 *
 * Parameters for Adrc behavior
 */

/**
 * ADRC Step
 *
 * ADRC Step.
 * 0 means that use dynamic dt
 * otherwises, indicate by this param.
 *
 * @unit s
 * @min 0.0
 * @max 0.02
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(ADRC_STEP, 0.f);

/**
 * ROLL ADRC ENABLE
 *
 *
 * @value 0 Disbale
 * @value 1 Enable
 * @group Multicopter ADRC
 */
PARAM_DEFINE_INT32(ADRC_ROLL, 0);

/**
 * PITCH ADRC ENABLE
 *
 *
 * @value 0 Disbale
 * @value 1 Enable
 * @group Multicopter ADRC
 */
PARAM_DEFINE_INT32(ADRC_PITCH, 0);

/**
 * YAW ADRC ENABLE
 *
 *
 * @value 0 Disbale
 * @value 1 Enable
 * @group Multicopter ADRC
 */
PARAM_DEFINE_INT32(ADRC_YAW, 0);

/**
 * Observer bandwidth
 *
 * Observer bandwidth.
 *
 * @unit rad/s
 * @min 0.0
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(ROLL_BW_O, 10.f);

/**
 * Control bandwidth
 *
 * Control bandwidth.
 *
 * @unit rad/s
 * @min 0.0
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(ROLL_BW_C, 1.f);

/**
 * System Gain
 *
 * System Gain.
 *
 * @min 0.0001
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(ROLL_B, 1.f);

/**
 * Roll Anti-Saturation Gain
 *
 * Roll Anti-Saturation Gain. 0 means disable anti-saturation. 1~3 is suggested.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(ROLL_SAT_K, 1.f);

/**
 * Roll torgue Saturation MAX
 *
 * Roll torgue Saturation MAX.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(ROLL_SAT_TAU, 1.f);

/**
 * Observer bandwidth
 *
 * Observer bandwidth.
 *
 * @unit rad/s
 * @min 0.0
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(PITCH_BW_O, 10.f);

/**
 * Control bandwidth
 *
 * Control bandwidth.
 *
 * @unit rad/s
 * @min 0.0
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(PITCH_BW_C, 1.f);

/**
 * System Gain
 *
 * System Gain.
 *
 * @min 0.0001
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(PITCH_B, 1.f);

/**
 * Roll Anti-Saturation Gain
 *
 * Roll Anti-Saturation Gain. 0 means disable anti-saturation. 1~3 is suggested.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(PITCH_SAT_K, 1.f);

/**
 * Roll torgue Saturation MAX
 *
 * Roll torgue Saturation MAX.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(PITCH_SAT_TAU, 1.f);



/**
 * Observer bandwidth YAW
 *
 * Observer bandwidth.
 *
 * @unit rad/s
 * @min 0.0
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(YAW_BW_O, 10.f);

/**
 * Control bandwidth YAW
 *
 * Control bandwidth.
 *
 * @unit rad/s
 * @min 0.0
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(YAW_BW_C, 1.f);

/**
 * System Gain YAW
 *
 * System Gain.
 *
 * @min 0.0001
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(YAW_B, 1.f);

/**
 * Yaw Anti-Saturation Gain
 *
 * Yaw Anti-Saturation Gain. 0 means disable anti-saturation. 1~3 is suggested.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(YAW_SAT_K, 1.f);


/**
 * Yaw torgue Saturation MAX
 *
 * Yaw torgue Saturation MAX.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(YAW_SAT_TAU, 1.0f);

/**
 * Observer bandwidth YAW
 *
 * Observer bandwidth.
 *
 * @unit rad/s
 * @min 0.0
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(YAW2_BW_O, 10.f);

/**
 * Control bandwidth YAW
 *
 * Control bandwidth.
 *
 * @unit rad/s
 * @min 0.0
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(YAW2_BW_C, 1.f);

/**
 * System Gain YAW
 *
 * System Gain.
 *
 * @min 0.0001
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(YAW2_B, 1.f);

/**
 * Yaw Anti-Saturation Gain
 *
 * Yaw Anti-Saturation Gain. 0 means disable anti-saturation. 1~3 is suggested.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(YAW2_SAT_K, 1.f);


/**
 * Yaw torgue Saturation MAX
 *
 * Yaw torgue Saturation MAX.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(YAW2_SAT_TAU, 1.f);
