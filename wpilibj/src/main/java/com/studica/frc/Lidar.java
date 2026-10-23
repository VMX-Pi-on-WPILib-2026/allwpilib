// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
package com.studica.frc;

import edu.wpi.first.wpilibj.SerialPort;

/**
 * Original Studica 360-degree LiDAR. Distance is in mm; missing rays are NaN. Uses the original
 * 128000-baud non-intensity triangle protocol, not T-Mini.
 */
public class Lidar implements AutoCloseable {
  public enum Port {
    kUSB1(2),
    kUSB2(3);
    public final int value;

    Port(int value) {
      this.value = value;
    }
  }

  private final SerialPort serial;
  private final LidarProtocol parser = new LidarProtocol();
  private boolean closed;

  public Lidar(Port port) {
    this(port, 128000);
  }

  public Lidar(Port port, int baudRate) {
    java.util.Objects.requireNonNull(port);
    serial =
        new SerialPort(
            baudRate, port == Port.kUSB1 ? SerialPort.Port.kUSB1 : SerialPort.Port.kUSB2);
    try {
      serial.setTimeout(0.05);
      serial.disableTermination();
      serial.setReadBufferSize(1024);
      serial.setDTR(false);
      if (serial.write(new byte[] {(byte) 0xa5, 0x60}, 2) != 2)
        throw new IllegalStateException("Lidar scan command failed");
    } catch (RuntimeException ex) {
      serial.close();
      throw ex;
    }
  }

  public synchronized void getData(float[] angle, float[] distance) {
    java.util.Objects.requireNonNull(angle);
    java.util.Objects.requireNonNull(distance);
    if (angle.length < 360 || distance.length < 360 || angle == distance)
      throw new IllegalArgumentException(
          "Lidar requires two separate arrays of at least 360 floats");
    if (closed) throw new IllegalStateException("Lidar is closed");
    long deadline = System.nanoTime() + 1_000_000_000L;
    do {
      if (parser.next(angle, distance)) return;
      parser.feed(serial.read(512));
    } while (System.nanoTime() < deadline);
    throw new IllegalStateException(
        "Lidar scan timed out (check port, baud rate, model and motor power)");
  }

  @Override
  public synchronized void close() {
    if (closed) return;
    closed = true;
    try {
      serial.write(new byte[] {(byte) 0xa5, 0x65}, 2);
      serial.setDTR(true);
    } finally {
      serial.close();
    }
  }
}
