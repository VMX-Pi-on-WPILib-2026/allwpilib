// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
package com.studica.frc;

import java.util.Arrays;

/** Bounded decoder for the original Studica/YDLIDAR non-intensity packets. */
final class LidarProtocol {
  private byte[] pending = new byte[0];
  private final float[] angles = new float[360];
  private final float[] distances = new float[360];
  private boolean started;

  void feed(byte[] data) {
    if (pending.length + data.length > 4096) {
      pending = new byte[0];
      started = false;
    }
    int old = pending.length;
    pending = Arrays.copyOf(pending, old + data.length);
    System.arraycopy(data, 0, pending, old, data.length);
  }

  private int word(int offset) {
    return (pending[offset] & 255) | ((pending[offset + 1] & 255) << 8);
  }

  private void drop(int count) {
    pending = Arrays.copyOfRange(pending, count, pending.length);
  }

  boolean next(float[] outAngles, float[] outDistances) {
    while (pending.length >= 10) {
      if ((pending[0] & 255) != 0xaa || (pending[1] & 255) != 0x55) {
        drop(1);
        continue;
      }
      int count = pending[3] & 255;
      int size = 10 + 2 * count;
      if (count == 0 || (pending[4] & 1) == 0 || (pending[6] & 1) == 0) {
        drop(1);
        continue;
      }
      if (pending.length < size) return false;
      int sum = word(0) ^ word(2) ^ word(4) ^ word(6);
      for (int i = 0; i < count; i++) sum ^= word(10 + 2 * i);
      if (sum != word(8)) {
        drop(1);
        continue;
      }
      double first = (word(4) >> 1) / 64.0, last = (word(6) >> 1) / 64.0;
      if (first >= 360 || last >= 360) {
        drop(size);
        continue;
      }
      boolean complete = false;
      if ((pending[2] & 1) != 0) {
        if (started) {
          System.arraycopy(angles, 0, outAngles, 0, 360);
          System.arraycopy(distances, 0, outDistances, 0, 360);
          complete = true;
        }
        started = true;
        for (int i = 0; i < 360; i++) {
          angles[i] = i;
          distances[i] = Float.NaN;
        }
      }
      if (started) {
        double diff = last - first;
        if (diff < 0) diff += 360;
        for (int i = 0; i < count; i++) {
          double distance = word(10 + 2 * i) / 4.0;
          if (distance <= 0) continue;
          double angle = first + (count > 1 ? diff * i / (count - 1) : 0);
          angle += Math.toDegrees(Math.atan(21.8 * (155.3 - distance) / (155.3 * distance)));
          angle = (angle + 360) % 360;
          int bin = (int) angle;
          angles[bin] = (float) angle;
          distances[bin] = (float) distance;
        }
      }
      drop(size);
      if (complete) return true;
    }
    return false;
  }
}
