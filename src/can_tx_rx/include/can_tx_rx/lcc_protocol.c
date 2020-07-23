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
 * This file was generated by cantools version 35.1.0 Tue Jul 21 17:05:57 2020.
 */

#include <string.h>
#include "lcc_protocol.h"

static inline uint8_t pack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_right_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint16_t unpack_left_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint8_t unpack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) >> shift);
}

static inline uint16_t unpack_right_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

int lcc_protocol_num_of_next_lane_marks_reported_pack(
    uint8_t *dst_p,
    const struct lcc_protocol_num_of_next_lane_marks_reported_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 1);

    dst_p[0] |= pack_left_shift_u8(src_p->num_of_next_lane_mark_reported, 0u, 0xffu);

    return (1);
}

int lcc_protocol_num_of_next_lane_marks_reported_unpack(
    struct lcc_protocol_num_of_next_lane_marks_reported_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    dst_p->num_of_next_lane_mark_reported = unpack_right_shift_u8(src_p[0], 0u, 0xffu);

    return (0);
}

uint8_t lcc_protocol_num_of_next_lane_marks_reported_num_of_next_lane_mark_reported_encode(double value)
{
    return (uint8_t)(value);
}

double lcc_protocol_num_of_next_lane_marks_reported_num_of_next_lane_mark_reported_decode(uint8_t value)
{
    return ((double)value);
}

bool lcc_protocol_num_of_next_lane_marks_reported_num_of_next_lane_mark_reported_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int lcc_protocol_reference_points_pack(
    uint8_t *dst_p,
    const struct lcc_protocol_reference_points_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u16(src_p->ref_point_1_position, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(src_p->ref_point_1_position, 8u, 0xffu);
    dst_p[2] |= pack_left_shift_u16(src_p->ref_point_1_distance, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->ref_point_1_distance, 8u, 0x7fu);
    dst_p[3] |= pack_left_shift_u8(src_p->ref_point_1_validity, 7u, 0x80u);
    dst_p[4] |= pack_left_shift_u16(src_p->ref_point_2_position, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u16(src_p->ref_point_2_position, 8u, 0xffu);
    dst_p[6] |= pack_left_shift_u16(src_p->ref_point_2_distance, 0u, 0xffu);
    dst_p[7] |= pack_right_shift_u16(src_p->ref_point_2_distance, 8u, 0x7fu);
    dst_p[7] |= pack_left_shift_u8(src_p->ref_point_2_validity, 7u, 0x80u);

    return (8);
}

int lcc_protocol_reference_points_unpack(
    struct lcc_protocol_reference_points_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->ref_point_1_position = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    dst_p->ref_point_1_position |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->ref_point_1_distance = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->ref_point_1_distance |= unpack_left_shift_u16(src_p[3], 8u, 0x7fu);
    dst_p->ref_point_1_validity = unpack_right_shift_u8(src_p[3], 7u, 0x80u);
    dst_p->ref_point_2_position = unpack_right_shift_u16(src_p[4], 0u, 0xffu);
    dst_p->ref_point_2_position |= unpack_left_shift_u16(src_p[5], 8u, 0xffu);
    dst_p->ref_point_2_distance = unpack_right_shift_u16(src_p[6], 0u, 0xffu);
    dst_p->ref_point_2_distance |= unpack_left_shift_u16(src_p[7], 8u, 0x7fu);
    dst_p->ref_point_2_validity = unpack_right_shift_u8(src_p[7], 7u, 0x80u);

    return (0);
}

uint16_t lcc_protocol_reference_points_ref_point_1_position_encode(double value)
{
    return (uint16_t)((value - -127.99609375) / 0.00390625);
}

double lcc_protocol_reference_points_ref_point_1_position_decode(uint16_t value)
{
    return (((double)value * 0.00390625) + -127.99609375);
}

bool lcc_protocol_reference_points_ref_point_1_position_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

uint16_t lcc_protocol_reference_points_ref_point_1_distance_encode(double value)
{
    return (uint16_t)(value / 0.00390625);
}

double lcc_protocol_reference_points_ref_point_1_distance_decode(uint16_t value)
{
    return ((double)value * 0.00390625);
}

bool lcc_protocol_reference_points_ref_point_1_distance_is_in_range(uint16_t value)
{
    return (value <= 32767u);
}

uint8_t lcc_protocol_reference_points_ref_point_1_validity_encode(double value)
{
    return (uint8_t)(value);
}

double lcc_protocol_reference_points_ref_point_1_validity_decode(uint8_t value)
{
    return ((double)value);
}

bool lcc_protocol_reference_points_ref_point_1_validity_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint16_t lcc_protocol_reference_points_ref_point_2_position_encode(double value)
{
    return (uint16_t)((value - -127.99609375) / 0.00390625);
}

double lcc_protocol_reference_points_ref_point_2_position_decode(uint16_t value)
{
    return (((double)value * 0.00390625) + -127.99609375);
}

bool lcc_protocol_reference_points_ref_point_2_position_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

uint16_t lcc_protocol_reference_points_ref_point_2_distance_encode(double value)
{
    return (uint16_t)(value / 0.00390625);
}

double lcc_protocol_reference_points_ref_point_2_distance_decode(uint16_t value)
{
    return ((double)value * 0.00390625);
}

bool lcc_protocol_reference_points_ref_point_2_distance_is_in_range(uint16_t value)
{
    return (value <= 32767u);
}

uint8_t lcc_protocol_reference_points_ref_point_2_validity_encode(double value)
{
    return (uint8_t)(value);
}

double lcc_protocol_reference_points_ref_point_2_validity_decode(uint8_t value)
{
    return ((double)value);
}

bool lcc_protocol_reference_points_ref_point_2_validity_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

int lcc_protocol_lka_right_lane_b_pack(
    uint8_t *dst_p,
    const struct lcc_protocol_lka_right_lane_b_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    dst_p[0] |= pack_left_shift_u16(src_p->heading_angle, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(src_p->heading_angle, 8u, 0xffu);
    dst_p[2] |= pack_left_shift_u16(src_p->view_range, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->view_range, 8u, 0x7fu);
    dst_p[3] |= pack_left_shift_u8(src_p->view_range_availability, 7u, 0x80u);

    return (4);
}

int lcc_protocol_lka_right_lane_b_unpack(
    struct lcc_protocol_lka_right_lane_b_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    dst_p->heading_angle = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    dst_p->heading_angle |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->view_range = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->view_range |= unpack_left_shift_u16(src_p[3], 8u, 0x7fu);
    dst_p->view_range_availability = unpack_right_shift_u8(src_p[3], 7u, 0x80u);

    return (0);
}

uint16_t lcc_protocol_lka_right_lane_b_heading_angle_encode(double value)
{
    return (uint16_t)((value - -31.9990234375) / 0.0009765625);
}

double lcc_protocol_lka_right_lane_b_heading_angle_decode(uint16_t value)
{
    return (((double)value * 0.0009765625) + -31.9990234375);
}

bool lcc_protocol_lka_right_lane_b_heading_angle_is_in_range(uint16_t value)
{
    return ((value >= 32401u) && (value <= 33132u));
}

uint16_t lcc_protocol_lka_right_lane_b_view_range_encode(double value)
{
    return (uint16_t)(value / 0.00390625);
}

double lcc_protocol_lka_right_lane_b_view_range_decode(uint16_t value)
{
    return ((double)value * 0.00390625);
}

bool lcc_protocol_lka_right_lane_b_view_range_is_in_range(uint16_t value)
{
    return (value <= 32767u);
}

uint8_t lcc_protocol_lka_right_lane_b_view_range_availability_encode(double value)
{
    return (uint8_t)(value);
}

double lcc_protocol_lka_right_lane_b_view_range_availability_decode(uint8_t value)
{
    return ((double)value);
}

bool lcc_protocol_lka_right_lane_b_view_range_availability_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

int lcc_protocol_lka_right_lane_a_pack(
    uint8_t *dst_p,
    const struct lcc_protocol_lka_right_lane_a_t *src_p,
    size_t size)
{
    uint16_t position;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->lane_type, 0u, 0x0fu);
    dst_p[0] |= pack_left_shift_u8(src_p->quality, 4u, 0x30u);
    dst_p[0] |= pack_left_shift_u8(src_p->model_degree, 6u, 0xc0u);
    position = (uint16_t)src_p->position;
    dst_p[1] |= pack_left_shift_u16(position, 0u, 0xffu);
    dst_p[2] |= pack_right_shift_u16(position, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(src_p->curvature, 0u, 0xffu);
    dst_p[4] |= pack_right_shift_u16(src_p->curvature, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(src_p->curvature_derivative, 0u, 0xffu);
    dst_p[6] |= pack_right_shift_u16(src_p->curvature_derivative, 8u, 0xffu);
    dst_p[7] |= pack_left_shift_u8(src_p->width_right_marking, 0u, 0xffu);

    return (8);
}

int lcc_protocol_lka_right_lane_a_unpack(
    struct lcc_protocol_lka_right_lane_a_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t position;

    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->lane_type = unpack_right_shift_u8(src_p[0], 0u, 0x0fu);
    dst_p->quality = unpack_right_shift_u8(src_p[0], 4u, 0x30u);
    dst_p->model_degree = unpack_right_shift_u8(src_p[0], 6u, 0xc0u);
    position = unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    position |= unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    dst_p->position = (int16_t)position;
    dst_p->curvature = unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->curvature |= unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    dst_p->curvature_derivative = unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->curvature_derivative |= unpack_left_shift_u16(src_p[6], 8u, 0xffu);
    dst_p->width_right_marking = unpack_right_shift_u8(src_p[7], 0u, 0xffu);

    return (0);
}

uint8_t lcc_protocol_lka_right_lane_a_lane_type_encode(double value)
{
    return (uint8_t)(value);
}

double lcc_protocol_lka_right_lane_a_lane_type_decode(uint8_t value)
{
    return ((double)value);
}

bool lcc_protocol_lka_right_lane_a_lane_type_is_in_range(uint8_t value)
{
    return (value <= 6u);
}

uint8_t lcc_protocol_lka_right_lane_a_quality_encode(double value)
{
    return (uint8_t)(value);
}

double lcc_protocol_lka_right_lane_a_quality_decode(uint8_t value)
{
    return ((double)value);
}

bool lcc_protocol_lka_right_lane_a_quality_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t lcc_protocol_lka_right_lane_a_model_degree_encode(double value)
{
    return (uint8_t)(value);
}

double lcc_protocol_lka_right_lane_a_model_degree_decode(uint8_t value)
{
    return ((double)value);
}

bool lcc_protocol_lka_right_lane_a_model_degree_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

int16_t lcc_protocol_lka_right_lane_a_position_encode(double value)
{
    return (int16_t)(value / 0.00390625);
}

double lcc_protocol_lka_right_lane_a_position_decode(int16_t value)
{
    return ((double)value * 0.00390625);
}

bool lcc_protocol_lka_right_lane_a_position_is_in_range(int16_t value)
{
    return (value <= 32512);
}

uint16_t lcc_protocol_lka_right_lane_a_curvature_encode(double value)
{
    return (uint16_t)((value - -0.031999023438) / 9.765625E-7);
}

double lcc_protocol_lka_right_lane_a_curvature_decode(uint16_t value)
{
    return (((double)value * 9.765625E-7) + -0.031999023438);
}

bool lcc_protocol_lka_right_lane_a_curvature_is_in_range(uint16_t value)
{
    return ((value >= 12287u) && (value <= 53247u));
}

uint16_t lcc_protocol_lka_right_lane_a_curvature_derivative_encode(double value)
{
    return (uint16_t)((value - -0.00012206658721) / 3.7252902985E-9);
}

double lcc_protocol_lka_right_lane_a_curvature_derivative_decode(uint16_t value)
{
    return (((double)value * 3.7252902985E-9) + -0.00012206658721);
}

bool lcc_protocol_lka_right_lane_a_curvature_derivative_is_in_range(uint16_t value)
{
    return ((value >= 554u) && (value <= 64979u));
}

uint8_t lcc_protocol_lka_right_lane_a_width_right_marking_encode(double value)
{
    return (uint8_t)(value / 0.01);
}

double lcc_protocol_lka_right_lane_a_width_right_marking_decode(uint8_t value)
{
    return ((double)value * 0.01);
}

bool lcc_protocol_lka_right_lane_a_width_right_marking_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int lcc_protocol_lka_left_lane_b_pack(
    uint8_t *dst_p,
    const struct lcc_protocol_lka_left_lane_b_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    dst_p[0] |= pack_left_shift_u16(src_p->heading_angle, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(src_p->heading_angle, 8u, 0xffu);
    dst_p[2] |= pack_left_shift_u16(src_p->view_range, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->view_range, 8u, 0x7fu);
    dst_p[3] |= pack_left_shift_u8(src_p->view_range_availability, 7u, 0x80u);

    return (4);
}

int lcc_protocol_lka_left_lane_b_unpack(
    struct lcc_protocol_lka_left_lane_b_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    dst_p->heading_angle = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    dst_p->heading_angle |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->view_range = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->view_range |= unpack_left_shift_u16(src_p[3], 8u, 0x7fu);
    dst_p->view_range_availability = unpack_right_shift_u8(src_p[3], 7u, 0x80u);

    return (0);
}

uint16_t lcc_protocol_lka_left_lane_b_heading_angle_encode(double value)
{
    return (uint16_t)((value - -31.9990234375) / 0.0009765625);
}

double lcc_protocol_lka_left_lane_b_heading_angle_decode(uint16_t value)
{
    return (((double)value * 0.0009765625) + -31.9990234375);
}

bool lcc_protocol_lka_left_lane_b_heading_angle_is_in_range(uint16_t value)
{
    return ((value >= 32401u) && (value <= 33132u));
}

uint16_t lcc_protocol_lka_left_lane_b_view_range_encode(double value)
{
    return (uint16_t)(value / 0.00390625);
}

double lcc_protocol_lka_left_lane_b_view_range_decode(uint16_t value)
{
    return ((double)value * 0.00390625);
}

bool lcc_protocol_lka_left_lane_b_view_range_is_in_range(uint16_t value)
{
    return (value <= 32767u);
}

uint8_t lcc_protocol_lka_left_lane_b_view_range_availability_encode(double value)
{
    return (uint8_t)(value);
}

double lcc_protocol_lka_left_lane_b_view_range_availability_decode(uint8_t value)
{
    return ((double)value);
}

bool lcc_protocol_lka_left_lane_b_view_range_availability_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

int lcc_protocol_lka_left_lane_a_pack(
    uint8_t *dst_p,
    const struct lcc_protocol_lka_left_lane_a_t *src_p,
    size_t size)
{
    uint16_t position;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->lane_type, 0u, 0x0fu);
    dst_p[0] |= pack_left_shift_u8(src_p->quality, 4u, 0x30u);
    dst_p[0] |= pack_left_shift_u8(src_p->model_degree, 6u, 0xc0u);
    position = (uint16_t)src_p->position;
    dst_p[1] |= pack_left_shift_u16(position, 0u, 0xffu);
    dst_p[2] |= pack_right_shift_u16(position, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(src_p->curvature, 0u, 0xffu);
    dst_p[4] |= pack_right_shift_u16(src_p->curvature, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(src_p->curvature_derivative, 0u, 0xffu);
    dst_p[6] |= pack_right_shift_u16(src_p->curvature_derivative, 8u, 0xffu);
    dst_p[7] |= pack_left_shift_u8(src_p->width_left_marking, 0u, 0xffu);

    return (8);
}

int lcc_protocol_lka_left_lane_a_unpack(
    struct lcc_protocol_lka_left_lane_a_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t position;

    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->lane_type = unpack_right_shift_u8(src_p[0], 0u, 0x0fu);
    dst_p->quality = unpack_right_shift_u8(src_p[0], 4u, 0x30u);
    dst_p->model_degree = unpack_right_shift_u8(src_p[0], 6u, 0xc0u);
    position = unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    position |= unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    dst_p->position = (int16_t)position;
    dst_p->curvature = unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->curvature |= unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    dst_p->curvature_derivative = unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->curvature_derivative |= unpack_left_shift_u16(src_p[6], 8u, 0xffu);
    dst_p->width_left_marking = unpack_right_shift_u8(src_p[7], 0u, 0xffu);

    return (0);
}

uint8_t lcc_protocol_lka_left_lane_a_lane_type_encode(double value)
{
    return (uint8_t)(value);
}

double lcc_protocol_lka_left_lane_a_lane_type_decode(uint8_t value)
{
    return ((double)value);
}

bool lcc_protocol_lka_left_lane_a_lane_type_is_in_range(uint8_t value)
{
    return (value <= 6u);
}

uint8_t lcc_protocol_lka_left_lane_a_quality_encode(double value)
{
    return (uint8_t)(value);
}

double lcc_protocol_lka_left_lane_a_quality_decode(uint8_t value)
{
    return ((double)value);
}

bool lcc_protocol_lka_left_lane_a_quality_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t lcc_protocol_lka_left_lane_a_model_degree_encode(double value)
{
    return (uint8_t)(value);
}

double lcc_protocol_lka_left_lane_a_model_degree_decode(uint8_t value)
{
    return ((double)value);
}

bool lcc_protocol_lka_left_lane_a_model_degree_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

int16_t lcc_protocol_lka_left_lane_a_position_encode(double value)
{
    return (int16_t)(value / 0.00390625);
}

double lcc_protocol_lka_left_lane_a_position_decode(int16_t value)
{
    return ((double)value * 0.00390625);
}

bool lcc_protocol_lka_left_lane_a_position_is_in_range(int16_t value)
{
    return (value <= 32512);
}

uint16_t lcc_protocol_lka_left_lane_a_curvature_encode(double value)
{
    return (uint16_t)((value - -0.031999023438) / 9.765625E-7);
}

double lcc_protocol_lka_left_lane_a_curvature_decode(uint16_t value)
{
    return (((double)value * 9.765625E-7) + -0.031999023438);
}

bool lcc_protocol_lka_left_lane_a_curvature_is_in_range(uint16_t value)
{
    return ((value >= 12287u) && (value <= 53247u));
}

uint16_t lcc_protocol_lka_left_lane_a_curvature_derivative_encode(double value)
{
    return (uint16_t)((value - -0.00012206658721) / 3.7252902985E-9);
}

double lcc_protocol_lka_left_lane_a_curvature_derivative_decode(uint16_t value)
{
    return (((double)value * 3.7252902985E-9) + -0.00012206658721);
}

bool lcc_protocol_lka_left_lane_a_curvature_derivative_is_in_range(uint16_t value)
{
    return ((value >= 554u) && (value <= 64979u));
}

uint8_t lcc_protocol_lka_left_lane_a_width_left_marking_encode(double value)
{
    return (uint8_t)(value / 0.01);
}

double lcc_protocol_lka_left_lane_a_width_left_marking_decode(uint8_t value)
{
    return ((double)value * 0.01);
}

bool lcc_protocol_lka_left_lane_a_width_left_marking_is_in_range(uint8_t value)
{
    return (value <= 250u);
}
