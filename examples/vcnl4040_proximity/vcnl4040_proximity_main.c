/****************************************************************************
 * examples/vcnl4040_proximity/vcnl4040_proximity.c
 *
 *   Copyright (C) 2017 Giorgio Groß. All rights reserved.
 *   Author: Giorgio Groß <giorgio.gross@robodev.eu>
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <stdio.h>
#include <fixedmath.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>

#include <nuttx/sensors/vcnl4040.h>

//#include <nuttx/board.h>

#define DECIMAL_PLACES3(x) abs(((int)(((x)-((int)x))*1000)))

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, char *argv[])
#else
int vcnl4040_proximity_main(int argc, char *argv[])
#endif
{
  char* dev0 = "/dev/vcnl0";
  char* dev1 = "/dev/vcnl1";

  int samples = 1;
  if (argc > 1)
    {
      samples = atoi(argv[1]);
    }

  printf("VCNL 0 ------\n");
  int fd = open(dev0, O_RDONLY);
  if (!fd)
    {
      printf("Failed to open %s: %d\n", dev0, errno);
      return 1;
    }
  vcnl4040_data_t buffer;
  int sample;
  for (sample = 0; sample < samples; ++sample)
    {
      if (read(fd, &buffer, sizeof(buffer)) < 0)
        {
          printf("Failed to read from %s: %d\n", dev0, errno);
          return 1;
        }
      printf("Read ALS: %d   -   PS: %d\n", (int)b16tof(buffer.als_data), (int)b16tof(buffer.ps_data));
    }
  close(fd);

  printf("VCNL 1 ------\n");
  fd = open(dev1, O_RDONLY);
  if (!fd)
    {
      printf("Failed to open %s: %d\n", dev1, errno);
      return 1;
    }
  for (sample = 0; sample < samples; ++sample)
    {
      if (read(fd, &buffer, sizeof(buffer)) < 0)
        {
          printf("Failed to read from %s: %d\n", dev1, errno);
          return 1;
        }
      printf("Read ALS: %d   -   PS: %d\n", (int)b16tof(buffer.als_data), (int)b16tof(buffer.ps_data));
    }
  close(fd);

  return 0;
}
