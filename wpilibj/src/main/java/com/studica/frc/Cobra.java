// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
package com.studica.frc;

import edu.wpi.first.wpilibj.I2C;

/** Cobra ADS1015 at address 0x48 on MXP. Channels are 0..3. */
public class Cobra implements AutoCloseable {
  private static final Object lock = new Object();
  private final I2C i2c;
  private final float vRef;
  private boolean closed;

  public Cobra() {
    this(5F);
  }

  public Cobra(float vRef) {
    if (!Float.isFinite(vRef) || vRef <= 0)
      throw new IllegalArgumentException("Cobra reference voltage must be finite and positive");
    this.vRef = vRef;
    i2c = new I2C(I2C.Port.kMXP, 0x48);
  }

  public int getRawValue(int channel) {
    if (channel < 0 || channel > 3)
      throw new IllegalArgumentException("Cobra channel must be 0..3");
    synchronized (lock) {
      if (closed) throw new IllegalStateException("Cobra is closed");
      int config = 0x8583 | ((4 + channel) << 12);
      if (i2c.transaction(new byte[] {1, (byte) (config >> 8), (byte) config}, 3, null, 0))
        throw new IllegalStateException("Cobra configuration write failed");
      long deadline = System.nanoTime() + 20_000_000;
      byte[] rx = new byte[2];
      do {
        try {
          Thread.sleep(1);
        } catch (InterruptedException ex) {
          Thread.currentThread().interrupt();
          throw new IllegalStateException("Cobra conversion interrupted", ex);
        }
        if (i2c.transaction(new byte[] {1}, 1, rx, 2))
          throw new IllegalStateException("Cobra status read failed");
        if ((rx[0] & 128) != 0) {
          if (i2c.transaction(new byte[] {0}, 1, rx, 2))
            throw new IllegalStateException("Cobra conversion read failed");
          return ((rx[0] & 255) << 8) | (rx[1] & 255);
        }
      } while (System.nanoTime() < deadline);
      throw new IllegalStateException("Cobra conversion timed out");
    }
  }

  public float getVoltage(int channel) {
    return getRawValue(channel) * (vRef / 32768F);
  }

  @Override
  public void close() {
    synchronized (lock) {
      if (!closed) {
        closed = true;
        i2c.close();
      }
    }
  }
}
