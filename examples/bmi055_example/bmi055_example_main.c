/****************************************************************************
 * examples/bmi055_example/bmi055_example_main.c
 *
 *   Copyright (C) 2018 Giorgio Gross. All rights reserved.
 *   Author: Giorgio Gross <giorgio.gross@robodev.eu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_SENSORS_BMI055

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <stdio.h>
#include <fixedmath.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <nuttx/sensors/bmi055.h>
#include <nuttx/sensors/bmg160_lib.h>
#include <nuttx/sensors/bma2x2_lib.h>

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, char *argv[])
#else
int bmi055_example_main(int argc, char *argv[])
#endif
{
  if (argc < 2) {
    printf("usage: bmi055_example /dev/bmiN [samples]\n");
    return 1;
  }

  char* accel_dev = argv[1];

  int samples = 1;
  if (argc > 2) {
    samples = atoi(argv[2]);
  }

  int accel_fd = open(accel_dev, O_RDONLY);
  if (!accel_fd) {
    printf("Failed to open %s: %d\n", accel_dev, errno);
    return 1;
  }

  struct bma2x2_accel_data accel_data;
  struct bmg160_data_t gyro_data;
  int sample;
  for (sample = 0; sample < samples; ++sample) {

    /* read some accel data via ordinary read */

    if (read(accel_fd, &accel_data, sizeof(accel_data)) < 0) {
      printf("Failed to read from %s: %d\n", accel_dev, errno);
      return 1;
    }

    printf("acceleration: x = %d ; y = %d ; z = %d\n", accel_data.x, accel_data.y, accel_data.z);

    /* read some gyro data via ioctl */

    if(ioctl(accel_fd, BMI055_GYRO_GET_DATA_XYZ, (unsigned long)&gyro_data) != 0)
    {
      printf("Error getting gyro data\n");
    } else {
      printf("gyro: a = %d ; b = %d ; c = %d\n", gyro_data.datax, gyro_data.datay, gyro_data.dataz);
    }

  }

  /* test writing to device */

  uint8_t hbw_buf = 0;

  /* read hbw */
  if(ioctl(accel_fd, BMI055_ACCEL_GET_HIGH_BW, (unsigned long)&hbw_buf) != 0)
  {
    printf("Error getting hbw\n");
  } else {
    printf("Hbw is %d\n", hbw_buf);
  }

  /* set device to hbw */

  if(ioctl(accel_fd, BMI055_ACCEL_SET_HIGH_BW, 1 != 0))
  {
    printf("Error setting low bandwidth\n");
  }

  /* read hbw */
  if(ioctl(accel_fd, BMI055_ACCEL_GET_HIGH_BW, (unsigned long)&hbw_buf) != 0)
  {
    printf("Error getting hbw\n");
  } else {
    printf("Hbw is now %d\n", hbw_buf);
  }

  /* reset device to lbw */

  if(ioctl(accel_fd, BMI055_ACCEL_SET_HIGH_BW, 0) != 0)
  {
    printf("Error setting hbw\n");
  }

  /* set fifo data select (returns an error due to internal device state) */

  if(ioctl(accel_fd, BMI055_ACCEL_SET_FIFO_DATA_SELECT, 0x02) != 0)
  {
    printf("Error setting fifo data select\n");
  } else {
    printf("Set fifo data select successful\n");
  }

  /* read fifo data select */

  if(ioctl(accel_fd, BMI055_ACCEL_GET_FIFO_DATA_SELECT, (unsigned long)&hbw_buf) != 0)
  {
    printf("Error getting fifo data select\n");
  } else {
    printf("Get fifo data select: %d\n", hbw_buf);
  }


  return 0;
}
#endif // CONFIG_SENSORS_BMI055
