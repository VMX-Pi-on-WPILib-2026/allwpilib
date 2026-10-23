// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md.
package com.studica.frc;

import java.util.*;
import java.util.concurrent.atomic.*;

public final class StudicaRegression {
  private static void check(boolean result) {
    if (!result) throw new AssertionError();
  }

  private static void throwsError(Runnable action) {
    try {
      action.run();
    } catch (RuntimeException ex) {
      return;
    }
    throw new AssertionError("Expected exception");
  }

  private static byte[] packet(boolean ring, int angle, int distance) {
    int a = angle * 128 + 1, d = distance * 4;
    byte[] p = {
      (byte) 0xaa,
      0x55,
      (byte) (ring ? 1 : 0),
      1,
      (byte) a,
      (byte) (a >> 8),
      (byte) a,
      (byte) (a >> 8),
      0,
      0,
      (byte) d,
      (byte) (d >> 8)
    };
    int sum = 0;
    for (int i : new int[] {0, 2, 4, 6, 10}) sum ^= (p[i] & 255) | ((p[i + 1] & 255) << 8);
    p[8] = (byte) sum;
    p[9] = (byte) (sum >> 8);
    return p;
  }

  public static void main(String[] args) {
    AtomicBoolean enabled = new AtomicBoolean();
    AtomicLong clock = new AtomicLong();
    Map<Integer, byte[]> sent = new HashMap<>(), replies = new HashMap<>();
    AtomicBoolean fail = new AtomicBoolean();
    TitanBus.Link link =
        new TitanBus.Link() {
          public void write(int api, byte[] data) {
            if (fail.get()) throw new IllegalStateException("fake CAN failure");
            sent.put(api, data.clone());
          }

          public byte[] read(int api) {
            byte[] d = replies.get(api);
            if (d == null) throw new IllegalStateException("missing");
            return d;
          }
        };
    TitanBus bus = new TitanBus(42, link, enabled::get, clock::get);
    bus.register(0, 15600);
    bus.register(1, 15600);
    throwsError(() -> bus.register(0, 15600));
    bus.set(0, 0.7);
    check(bus.get(0) == 0 && !sent.containsKey(1));
    enabled.set(true);
    bus.set(0, -2);
    check((sent.get(2)[1] & 255) == 100 && sent.get(2)[2] == 0 && sent.get(2)[3] == 1);
    bus.set(1, 0.5);
    bus.stop(0);
    check(bus.get(1) == 0.5 && sent.get(3)[0] == 0);
    clock.set(101_000_000L);
    sent.clear();
    bus.tick();
    check(sent.containsKey(0) && !sent.containsKey(1));
    bus.set(1, 0.5);
    enabled.set(false);
    bus.tick();
    enabled.set(true);
    sent.clear();
    bus.tick();
    check(!sent.containsKey(1));
    replies.put(37, new byte[] {-2, -1, -1, -1});
    check(bus.count(0) == -2);
    replies.put(41, new byte[] {(byte) 0xe4, (byte) 0xf9, -1, -1});
    check(Math.abs(bus.rpm(0) + 15.64) < 1e-9);
    replies.put(37, new byte[] {1});
    throwsError(() -> bus.count(0));
    throwsError(() -> bus.set(0, Double.NaN));
    fail.set(true);
    throwsError(() -> bus.set(0, 0.5));
    fail.set(false);
    throwsError(() -> bus.set(0, 0.5));
    bus.shutdown();
    throwsError(() -> bus.read(37, 1));
    LidarProtocol parser = new LidarProtocol();
    float[] angles = new float[360], distances = new float[360];
    byte[] start = packet(true, 0, 1000);
    parser.feed(Arrays.copyOf(start, 3));
    check(!parser.next(angles, distances));
    parser.feed(Arrays.copyOfRange(start, 3, start.length));
    parser.feed(packet(false, 90, 2000));
    byte[] bad = packet(false, 180, 4000);
    bad[8] ^= 1;
    parser.feed(bad);
    parser.feed(packet(true, 0, 3000));
    check(parser.next(angles, distances));
    boolean a = false, b = false;
    for (float d : distances) {
      a |= d == 1000;
      b |= d == 2000;
      check(d != 4000);
    }
    check(a && b);
    for (int i = 0; i < 100; i++) {
      byte[] junk = new byte[1024];
      Arrays.fill(junk, (byte) 0xaa);
      parser.feed(junk);
      parser.next(angles, distances);
    }
    System.out.println("Studica Java CAN protocol, safety, encoders and LiDAR tests passed");
  }
}
