// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
package com.studica.frc;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

/** Shared Titan transport. Package-private injection supports hardware-free tests. */
final class TitanBus {
  interface Link extends AutoCloseable {
    void write(int api, byte[] data);

    byte[] read(int api);

    @Override
    default void close() {}
  }

  private static final Map<Integer, TitanBus> devices = new HashMap<>();
  private static final ScheduledExecutorService worker =
      Executors.newSingleThreadScheduledExecutor(
          r -> {
            Thread t = new Thread(r, "Studica-Titan");
            t.setDaemon(true);
            return t;
          });

  static synchronized TitanBus acquire(int id) {
    if (id < 1 || id > 63) throw new IllegalArgumentException("Titan CAN ID must be 1..63");
    TitanBus bus = devices.get(id);
    if (bus == null) {
      CAN can = new CAN(id, 12, 2);
      Link link =
          new Link() {
            public void write(int api, byte[] data) {
              can.writePacket(data, api);
            }

            public byte[] read(int api) {
              CANData data = new CANData();
              if (!can.readPacketTimeout(api, 250, data))
                throw new IllegalStateException("Titan telemetry unavailable: " + api);
              return java.util.Arrays.copyOf(data.data, data.length);
            }

            public void close() {
              can.close();
            }
          };
      try {
        bus =
            new TitanBus(
                id,
                link,
                () ->
                    DriverStation.isDSAttached()
                        && DriverStation.isEnabled()
                        && !DriverStation.isEStopped(),
                System::nanoTime);
      } catch (RuntimeException ex) {
        link.close();
        throw ex;
      }
      TitanBus target = bus;
      bus.task = worker.scheduleAtFixedRate(target::service, 20, 20, TimeUnit.MILLISECONDS);
      devices.put(id, bus);
    }
    bus.references++;
    return bus;
  }

  static synchronized void release(TitanBus bus) {
    if (--bus.references != 0) return;
    devices.remove(bus.id);
    bus.shutdown();
  }

  final int id;
  private final Link link;
  private final BooleanSupplier enabled;
  private final LongSupplier now;
  private final boolean[] allocated = new boolean[4];
  private final double[] speeds = new double[4];
  private final long[] updates = new long[4];
  private final boolean[] commanded = new boolean[4];
  private int references;
  private boolean closed;
  private RuntimeException fault;
  private ScheduledFuture<?> task;

  TitanBus(int id, Link link, BooleanSupplier enabled, LongSupplier now) {
    this.id = id;
    this.link = link;
    this.enabled = enabled;
    this.now = now;
    write(0, new byte[8]);
  }

  static void channel(int motor) {
    if (motor < 0 || motor > 3)
      throw new IllegalArgumentException("Titan motor/encoder must be 0..3");
  }

  synchronized void register(int motor, int frequency) {
    channel(motor);
    if (frequency < 0 || frequency > 20000)
      throw new IllegalArgumentException("Titan frequency must be 0..20000 Hz");
    if (allocated[motor]) throw new IllegalStateException("Titan motor already allocated");
    write(10, new byte[] {(byte) motor, (byte) frequency, (byte) (frequency >> 8), 0, 0, 0, 0, 0});
    allocated[motor] = true;
  }

  synchronized void remove(int motor) {
    speeds[motor] = 0;
    commanded[motor] = false;
    allocated[motor] = false;
    write(3, new byte[] {(byte) motor, 0, 0, 0, 0, 0, 0, 0});
  }

  synchronized void set(int motor, double speed) {
    if (!Double.isFinite(speed)) {
      stop(motor);
      throw new IllegalArgumentException("Titan speed must be finite");
    }
    if (fault != null)
      throw new IllegalStateException("Titan CAN fault; recreate the controller", fault);
    boolean safe = enabled.getAsBoolean();
    speeds[motor] = safe ? Math.max(-1, Math.min(1, speed)) : 0;
    updates[motor] = now.getAsLong();
    commanded[motor] = safe;
    try {
      tick();
    } catch (RuntimeException ex) {
      fail(ex);
      throw ex;
    }
  }

  synchronized double get(int motor) {
    return enabled.getAsBoolean()
            && commanded[motor]
            && now.getAsLong() - updates[motor] <= 100_000_000L
        ? speeds[motor]
        : 0;
  }

  synchronized void stop(int motor) {
    speeds[motor] = 0;
    commanded[motor] = false;
    write(3, new byte[] {(byte) motor, 0, 0, 0, 0, 0, 0, 0});
  }

  synchronized void tick() {
    boolean safe = enabled.getAsBoolean() && fault == null && !closed;
    boolean active = false;
    for (int m = 0; m < 4; m++) {
      if (!safe || now.getAsLong() - updates[m] > 100_000_000L) {
        speeds[m] = 0;
        commanded[m] = false;
      }
      active |= allocated[m] && commanded[m];
    }
    if (!active) {
      write(0, new byte[8]);
      return;
    }
    write(1, new byte[8]);
    for (int m = 0; m < 4; m++) {
      if (!allocated[m]) continue;
      if (!commanded[m]) {
        write(3, new byte[] {(byte) m, 0, 0, 0, 0, 0, 0, 0});
        continue;
      }
      double v = speeds[m];
      write(
          2,
          new byte[] {
            (byte) m,
            (byte) (Math.abs(v) * 100),
            (byte) (v >= 0 ? 1 : 0),
            (byte) (v <= 0 ? 1 : 0),
            0,
            0,
            0,
            0
          });
    }
  }

  private synchronized void service() {
    if (closed) return;
    try {
      tick();
    } catch (RuntimeException ex) {
      fail(ex);
    }
  }

  private void fail(RuntimeException ex) {
    fault = ex;
    java.util.Arrays.fill(speeds, 0);
    java.util.Arrays.fill(commanded, false);
    try {
      link.write(0, new byte[8]);
    } catch (RuntimeException ignored) {
      /* Retry disabled on next service. */
    }
  }

  synchronized void write(int api, byte[] data) {
    if (closed) throw new IllegalStateException("Titan is closed");
    try {
      link.write(api, data);
    } catch (RuntimeException ex) {
      fail(ex);
      throw ex;
    }
  }

  synchronized byte[] read(int api, int length) {
    if (closed) throw new IllegalStateException("Titan is closed");
    byte[] data = link.read(api);
    if (data.length < length) throw new IllegalStateException("Truncated Titan frame: " + api);
    return data;
  }

  byte[] request(int request, int response, int length, byte[] payload) {
    write(request, payload);
    for (int i = 0; i < 10; i++) {
      try {
        Thread.sleep(10);
      } catch (InterruptedException ex) {
        Thread.currentThread().interrupt();
        throw new IllegalStateException("Titan request interrupted", ex);
      }
      try {
        return read(response, length);
      } catch (IllegalStateException ignored) {
        /* bounded receive wait */
      }
    }
    throw new IllegalStateException("Titan telemetry request timed out");
  }

  static int int32(byte[] d) {
    return (d[0] & 255) | ((d[1] & 255) << 8) | ((d[2] & 255) << 16) | ((d[3] & 255) << 24);
  }

  int count(int encoder) {
    channel(encoder);
    return int32(read(37 + encoder, 4));
  }

  double rpm(int motor) {
    channel(motor);
    return int32(read(41 + motor, 4)) / 100.0;
  }

  synchronized void shutdown() {
    if (closed) return;
    if (task != null) task.cancel(false);
    try {
      link.write(0, new byte[8]);
    } finally {
      closed = true;
      link.close();
    }
  }
}
