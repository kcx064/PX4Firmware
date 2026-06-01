
/**
 * @file mc_adrc_params.c
 *
 * Parameters for Adrc behavior
 */

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
PARAM_DEFINE_FLOAT(ADRC_BW_O, 10.f);

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
PARAM_DEFINE_FLOAT(ADRC_BW_C, 1.f);

/**
 * System Gain
 *
 * System Gain.
 *
 * @unit rad/s
 * @min 0.0
 * @max 2000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter ADRC
 */
PARAM_DEFINE_FLOAT(ADRC_B, 1.f);
