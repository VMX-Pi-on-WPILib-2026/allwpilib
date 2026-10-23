import edu.wpi.first.hal.JNIWrapper;
import edu.wpi.first.hal.VMXIMUJNI;
import edu.wpi.first.wpilibj.VMXIMU;

public class IMURegression {
  private static native void setInitialized(boolean initialized);
  private static void check(boolean value) {
    if (!value) throw new AssertionError();
  }
  private static void expectFailure(Runnable action) {
    try { action.run(); } catch (RuntimeException expected) { return; }
    throw new AssertionError("expected failure");
  }
  public static void main(String[] args) {
    JNIWrapper.Helper.setExtractOnStaticLoad(false);
    System.load(args[0]);
    expectFailure(() -> new VMXIMU());
    setInitialized(true);
    VMXIMU first = new VMXIMU();
    check(first.isConnected());
    check(first.getYaw() == -170 && first.getAngle() == 450 && first.getRate() == -15);
    check(Math.abs(first.getRotation2d().getDegrees() + 450) < 1e-9);
    check(first.getFirmwareVersion().equals("3.0.436"));
    VMXIMU second = new VMXIMU();
    second.zeroYaw();
    check(first.getYaw() == 0);
    second.reset();
    check(first.getAngle() == 0);
    second.close();
    second.close();
    expectFailure(second::getYaw);
    expectFailure(second::zeroYaw);
    check(first.isConnected());
    expectFailure(() -> first.updateDisplacement(0, 0, 0, true));
    expectFailure(() -> first.updateDisplacement(Double.NaN, 0, 50, true));
    expectFailure(() -> VMXIMUJNI.getValue(999));
    setInitialized(false);
    expectFailure(first::getYaw);
    first.close();
    System.out.println("PASS VMXIMU Java/JNI: errors, resets, shared lifetime, rotation");
  }
}
