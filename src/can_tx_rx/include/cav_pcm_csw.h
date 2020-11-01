/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018-2019 Erik Moqvist
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * This file was generated by cantools version 35.1.0 Wed Oct 28 14:01:36 2020.
 */

#ifndef EMC_PCM_CAV_INTERFACE_H
#define EMC_PCM_CAV_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef EINVAL
#    define EINVAL 22
#endif

/* Frame ids. */
#define EMC_PCM_CAV_INTERFACE_CAV_LONG_CTRL_FRAME_ID (0x420u)
#define EMC_PCM_CAV_INTERFACE_CAV_LATERAL_CTRL_FRAME_ID (0x410u)
#define EMC_PCM_CAV_INTERFACE_PCM_TO_CAV_3_FRAME_ID (0x432u)
#define EMC_PCM_CAV_INTERFACE_PCM_TO_CAV_2_FRAME_ID (0x431u)
#define EMC_PCM_CAV_INTERFACE_PCM_TO_CAV_1_FRAME_ID (0x430u)

/* Frame lengths in bytes. */
#define EMC_PCM_CAV_INTERFACE_CAV_LONG_CTRL_LENGTH (8u)
#define EMC_PCM_CAV_INTERFACE_CAV_LATERAL_CTRL_LENGTH (8u)
#define EMC_PCM_CAV_INTERFACE_PCM_TO_CAV_3_LENGTH (8u)
#define EMC_PCM_CAV_INTERFACE_PCM_TO_CAV_2_LENGTH (8u)
#define EMC_PCM_CAV_INTERFACE_PCM_TO_CAV_1_LENGTH (8u)

/* Extended or standard frame types. */
#define EMC_PCM_CAV_INTERFACE_CAV_LONG_CTRL_IS_EXTENDED (0)
#define EMC_PCM_CAV_INTERFACE_CAV_LATERAL_CTRL_IS_EXTENDED (0)
#define EMC_PCM_CAV_INTERFACE_PCM_TO_CAV_3_IS_EXTENDED (0)
#define EMC_PCM_CAV_INTERFACE_PCM_TO_CAV_2_IS_EXTENDED (0)
#define EMC_PCM_CAV_INTERFACE_PCM_TO_CAV_1_IS_EXTENDED (0)

/* Frame cycle times in milliseconds. */


/* Signal choices. */


/**
 * Signals in message CavLongCtrl.
 *
 * All signal values are as on the CAN bus.
 */
struct emc_pcm_cav_interface_cav_long_ctrl_t {
    /**
     * Range: 0..3 (0..3 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t cav_to_pcm_long_rc;

    /**
     * CAV Controller sending to Hybrid Supervisory Controller that AEB is Engaged
     *
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t cav_aeb_valid;

    /**
     * CAV Controller sending to Hybrid Supervisory Controller that ACC is Engaged
     *
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t cav_acc_valid;

    /**
     * CAV Controller sending to Hybrid Supervisory Controller value for ACC Accel Request
     *
     * Range: -128..127 (-20.48..20.32 m/s^2)
     * Scale: 0.16
     * Offset: 0
     */
    int8_t cav_long_accel;

    /**
     * Range: 0..2147483647 (0..2147483647 -)
     * Scale: 1
     * Offset: 0
     */
    uint32_t cav_to_pcm_long_pv;
};

/**
 * Signals in message CavLateralCtrl.
 *
 * All signal values are as on the CAN bus.
 */
struct emc_pcm_cav_interface_cav_lateral_ctrl_t {
    /**
     * Range: 0..2147483647 (0..2147483647 -)
     * Scale: 1
     * Offset: 0
     */
    uint32_t cav_to_pcm_lat_pv;

    /**
     * CAV Controller sending to Hybrid Supervisory Controller that LCC is Engaged
     *
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t cav_lcc_valid;

    /**
     * CAV Controller sending to Hybrid Supervisory Controller the LCC Steering Angle Request
     *
     * Range: -2048..2047 (-256..255.875 deg)
     * Scale: 0.125
     * Offset: 0
     */
    int16_t cav_lcc_str_ang;

    /**
     * CAV Controller sending to Hybrid Supervisory Controller that CAV systems are alive
     *
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t cav_cav_sys_alive;

    /**
     * Range: 0..3 (0..3 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t cav_to_pcm_lat_rc;
};

/**
 * Signals in message PcmToCav_3.
 *
 * All signal values are as on the CAN bus.
 */
struct emc_pcm_cav_interface_pcm_to_cav_3_t {
    /**
     * Range: 0..3 (0..3 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t pcm_to_cav3_rc;
};

/**
 * Signals in message PcmToCav_2.
 *
 * All signal values are as on the CAN bus.
 */
struct emc_pcm_cav_interface_pcm_to_cav_2_t {
    /**
     * Range: 0..3 (0..3 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t pcm_to_cav2_rc;
};

/**
 * Signals in message PcmToCav_1.
 *
 * All signal values are as on the CAN bus.
 */
struct emc_pcm_cav_interface_pcm_to_cav_1_t {
    /**
     * Range: 0..3 (0..3 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t pcm_to_cav1_rc;

    /**
     * Hybrid Supervisory Controller to Tank LCC is allowed
     *
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t pcm_lcc_allowed;

    /**
     * Hybrid Supervisory Controller allowing ACC to be calculated from CAVs Controller
     *
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t pcm_acc_allowed;

    /**
     * Hybrid Supervisory Controller allowing AEB to be calculated from CAVs Controller
     *
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t pcm_aeb_allowed;

    /**
     * Range: -128..127 (-64..63.5 m/s)
     * Scale: 0.5
     * Offset: 0
     */
    int8_t pcm_veh_spd;

    /**
     * Hybrid Supervisory Controller to Tank Steering Angle deg
     *
     * Range: -2048..2047 (-256..255.875 deg)
     * Scale: 0.125
     * Offset: 0
     */
    int16_t pcm_str_ang;

    /**
     * Hybrid Supervisory Controller to Tank Hybrid Supervisory Controller Alive Status
     *
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t pcm_hsc_alive;
};

/**
 * Pack message CavLongCtrl.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int emc_pcm_cav_interface_cav_long_ctrl_pack(
    uint8_t *dst_p,
    const struct emc_pcm_cav_interface_cav_long_ctrl_t *src_p,
    size_t size);

/**
 * Unpack message CavLongCtrl.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int emc_pcm_cav_interface_cav_long_ctrl_unpack(
    struct emc_pcm_cav_interface_cav_long_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_cav_long_ctrl_cav_to_pcm_long_rc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_cav_long_ctrl_cav_to_pcm_long_rc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_cav_long_ctrl_cav_to_pcm_long_rc_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_cav_long_ctrl_cav_aeb_valid_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_cav_long_ctrl_cav_aeb_valid_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_cav_long_ctrl_cav_aeb_valid_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_cav_long_ctrl_cav_acc_valid_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_cav_long_ctrl_cav_acc_valid_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_cav_long_ctrl_cav_acc_valid_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int8_t emc_pcm_cav_interface_cav_long_ctrl_cav_long_accel_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_cav_long_ctrl_cav_long_accel_decode(int8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_cav_long_ctrl_cav_long_accel_is_in_range(int8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t emc_pcm_cav_interface_cav_long_ctrl_cav_to_pcm_long_pv_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_cav_long_ctrl_cav_to_pcm_long_pv_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_cav_long_ctrl_cav_to_pcm_long_pv_is_in_range(uint32_t value);

/**
 * Pack message CavLateralCtrl.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int emc_pcm_cav_interface_cav_lateral_ctrl_pack(
    uint8_t *dst_p,
    const struct emc_pcm_cav_interface_cav_lateral_ctrl_t *src_p,
    size_t size);

/**
 * Unpack message CavLateralCtrl.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int emc_pcm_cav_interface_cav_lateral_ctrl_unpack(
    struct emc_pcm_cav_interface_cav_lateral_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t emc_pcm_cav_interface_cav_lateral_ctrl_cav_to_pcm_lat_pv_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_cav_lateral_ctrl_cav_to_pcm_lat_pv_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_cav_lateral_ctrl_cav_to_pcm_lat_pv_is_in_range(uint32_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_cav_lateral_ctrl_cav_lcc_valid_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_cav_lateral_ctrl_cav_lcc_valid_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_cav_lateral_ctrl_cav_lcc_valid_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t emc_pcm_cav_interface_cav_lateral_ctrl_cav_lcc_str_ang_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_cav_lateral_ctrl_cav_lcc_str_ang_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_cav_lateral_ctrl_cav_lcc_str_ang_is_in_range(int16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_cav_lateral_ctrl_cav_cav_sys_alive_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_cav_lateral_ctrl_cav_cav_sys_alive_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_cav_lateral_ctrl_cav_cav_sys_alive_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_cav_lateral_ctrl_cav_to_pcm_lat_rc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_cav_lateral_ctrl_cav_to_pcm_lat_rc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_cav_lateral_ctrl_cav_to_pcm_lat_rc_is_in_range(uint8_t value);

/**
 * Pack message PcmToCav_3.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int emc_pcm_cav_interface_pcm_to_cav_3_pack(
    uint8_t *dst_p,
    const struct emc_pcm_cav_interface_pcm_to_cav_3_t *src_p,
    size_t size);

/**
 * Unpack message PcmToCav_3.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int emc_pcm_cav_interface_pcm_to_cav_3_unpack(
    struct emc_pcm_cav_interface_pcm_to_cav_3_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_pcm_to_cav_3_pcm_to_cav3_rc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_pcm_to_cav_3_pcm_to_cav3_rc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_pcm_to_cav_3_pcm_to_cav3_rc_is_in_range(uint8_t value);

/**
 * Pack message PcmToCav_2.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int emc_pcm_cav_interface_pcm_to_cav_2_pack(
    uint8_t *dst_p,
    const struct emc_pcm_cav_interface_pcm_to_cav_2_t *src_p,
    size_t size);

/**
 * Unpack message PcmToCav_2.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int emc_pcm_cav_interface_pcm_to_cav_2_unpack(
    struct emc_pcm_cav_interface_pcm_to_cav_2_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_pcm_to_cav_2_pcm_to_cav2_rc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_pcm_to_cav_2_pcm_to_cav2_rc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_pcm_to_cav_2_pcm_to_cav2_rc_is_in_range(uint8_t value);

/**
 * Pack message PcmToCav_1.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int emc_pcm_cav_interface_pcm_to_cav_1_pack(
    uint8_t *dst_p,
    const struct emc_pcm_cav_interface_pcm_to_cav_1_t *src_p,
    size_t size);

/**
 * Unpack message PcmToCav_1.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int emc_pcm_cav_interface_pcm_to_cav_1_unpack(
    struct emc_pcm_cav_interface_pcm_to_cav_1_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_pcm_to_cav_1_pcm_to_cav1_rc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_pcm_to_cav_1_pcm_to_cav1_rc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_pcm_to_cav_1_pcm_to_cav1_rc_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_pcm_to_cav_1_pcm_lcc_allowed_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_pcm_to_cav_1_pcm_lcc_allowed_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_pcm_to_cav_1_pcm_lcc_allowed_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_pcm_to_cav_1_pcm_acc_allowed_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_pcm_to_cav_1_pcm_acc_allowed_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_pcm_to_cav_1_pcm_acc_allowed_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_pcm_to_cav_1_pcm_aeb_allowed_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_pcm_to_cav_1_pcm_aeb_allowed_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_pcm_to_cav_1_pcm_aeb_allowed_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int8_t emc_pcm_cav_interface_pcm_to_cav_1_pcm_veh_spd_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_pcm_to_cav_1_pcm_veh_spd_decode(int8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_pcm_to_cav_1_pcm_veh_spd_is_in_range(int8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t emc_pcm_cav_interface_pcm_to_cav_1_pcm_str_ang_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_pcm_to_cav_1_pcm_str_ang_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_pcm_to_cav_1_pcm_str_ang_is_in_range(int16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t emc_pcm_cav_interface_pcm_to_cav_1_pcm_hsc_alive_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double emc_pcm_cav_interface_pcm_to_cav_1_pcm_hsc_alive_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool emc_pcm_cav_interface_pcm_to_cav_1_pcm_hsc_alive_is_in_range(uint8_t value);


#ifdef __cplusplus
}
#endif

#endif
