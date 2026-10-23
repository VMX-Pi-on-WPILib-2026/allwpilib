// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
package com.studica.frc;

import edu.wpi.first.wpilibj.CounterBase;

/** Legacy encoder API. getSpeed() is signed shaft RPM; distance uses distPerTick. */
public class TitanQuadEncoder implements CounterBase {
  private final TitanQuad titan;
  private final int encoder;
  private final double distancePerTick;
  private boolean reverse;
  private boolean invertSpeed;
  private double maxPeriod = 0.5;
  private boolean sampled;
  private int lastCount;
  private long lastEdge;
  private double period = Double.POSITIVE_INFINITY;

  public TitanQuadEncoder(TitanQuad titan, int encoder, double distPerTick) {
    this.titan = java.util.Objects.requireNonNull(titan);
    TitanBus.channel(encoder);
    if (!Double.isFinite(distPerTick))
      throw new IllegalArgumentException("Distance per tick must be finite");
    this.encoder = encoder;
    distancePerTick = distPerTick;
  }

  @Override
  public int get() {
    int count = titan.device().count(encoder);
    return reverse ? Math.negateExact(count) : count;
  }

  @Override
  public synchronized void reset() {
    titan.device().write(13, new byte[] {(byte) encoder, 0, 0, 0, 0, 0, 0, 0});
    sampled = false;
    period = Double.POSITIVE_INFINITY;
  }

  @Override
  public synchronized double getPeriod() {
    int count = titan.device().count(encoder);
    long now = System.nanoTime();
    if (!sampled) {
      sampled = true;
      lastCount = count;
      lastEdge = now;
      return Double.POSITIVE_INFINITY;
    }
    double elapsed = (now - lastEdge) * 1e-9;
    long delta = count - lastCount; // subtract as int to handle a 32-bit counter wrap
    if (delta != 0) {
      period = elapsed / Math.abs(delta);
      lastEdge = now;
      lastCount = count;
      return period;
    }
    return Math.max(period, elapsed);
  }

  @Override
  public void setMaxPeriod(double value) {
    if (!Double.isFinite(value) || value <= 0)
      throw new IllegalArgumentException("Max period must be finite and positive");
    maxPeriod = value;
  }

  @Override
  public boolean getStopped() {
    return getPeriod() > maxPeriod;
  }

  public double getEncoderDistance() {
    return get() * distancePerTick;
  }

  public double getRaw() {
    return titan.device().count(encoder);
  }

  public void invertSpeed() {
    invertSpeed = true;
  }

  public double getSpeed() {
    return titan.device().rpm(encoder) * (invertSpeed ? -1 : 1);
  }

  @Override
  public boolean getDirection() {
    return titan.device().rpm(encoder) * (reverse ? -1 : 1) >= 0;
  }

  public void setReverseDirection() {
    reverse = true;
  }
}
