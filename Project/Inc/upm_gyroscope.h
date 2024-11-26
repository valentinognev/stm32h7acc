/*
 * Author: Jon Trulson <jtrulson@ics.com>
 * Copyright (c) 2017 Intel Corporation.
 *
 * This program and the accompanying materials are made available under the
 * terms of the The MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef UPM_GYROSCOPE_H_
#define UPM_GYROSCOPE_H_

#ifdef __cplusplus
extern "C" {
#endif

// Gyroscope function table
typedef struct _upm_gyroscope_ft {
    upm_result_t (*upm_gyroscope_set_scale) (void* dev, float* scale);
    upm_result_t (*upm_gyroscope_set_offset) (void* dev, float* offset);
    upm_result_t (*upm_gyroscope_get_value) (void* dev, float* value);
} upm_gyroscope_ft;

#ifdef __cplusplus
}
#endif

#endif /* UPM_GYROSCOPE_H_ */
