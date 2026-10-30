package org.firstinspires.ftc.teamcode.pedro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Dual-motor flywheel tuner.
 *
 * Motor roles:
 *   leftFlywheel  → RUN_USING_ENCODER  — PIDF master, velocity is commanded here
 *   rightFlywheel → RUN_WITHOUT_ENCODER — power follower, mirrors left's output
 *
 * Running two independent velocity PID loops on motors sharing the same
 * mechanical load causes them to fight each other. Having one motor own the
 * PIDF loop and the other simply copy its power output eliminates that conflict
 * while still spinning both motors at the same speed.
 *
 * Controls (Gamepad 1):
 *   Y          → Toggle between highVelocity and lowVelocity targets
 *   B          → Cycle step size for P/F adjustments
 *   DPad Up    → Increase P by current step
 *   DPad Down  → Decrease P by current step
 *   DPad Right → Increase F by current step
 *   DPad Left  → Decrease F by current step
 */
@TeleOp
public class FlywheelTuner extends OpMode {

    public DcMotorEx flywheel;

    public double highVelocity = 1400;
    public double lowVelocity  = 1200;

    double curTargetVelocity = highVelocity;

    double F = 13.9984;
    double P = 310;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init() {

        // ── Left motor: PIDF master ───────────────────────────────────────────
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );

        // ── Right motor: power follower ───────────────────────────────────────
        // RUN_WITHOUT_ENCODER means it accepts setPower() directly.
        // We copy leftFlywheelMotor.getPower() every loop so both motors
        // spin together without a second PID loop interfering.\
        telemetry.addLine("Init complete");
        telemetry.addLine("flywheel: PIDF master (RUN_USING_ENCODER)");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ── Button handling ───────────────────────────────────────────────────

        if (gamepad1.yWasPressed()) {
            curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed())  F -= stepSizes[stepIndex];
        if (gamepad1.dpadRightWasPressed()) F += stepSizes[stepIndex];
        if (gamepad1.dpadUpWasPressed())    P += stepSizes[stepIndex];
        if (gamepad1.dpadDownWasPressed())  P -= stepSizes[stepIndex];

        // ── Motor commands ────────────────────────────────────────────────────

        // Update PIDF on the master and command target velocity
        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );
        flywheel.setVelocity(curTargetVelocity);

        // ── Measurements ─────────────────────────────────────────────────────
        double velocity  = flywheel.getVelocity();
        double error         = curTargetVelocity - velocity;

        // ── Driver Station telemetry ──────────────────────────────────────────
        telemetry.addData("Target Velocity",          curTargetVelocity);
        telemetry.addData("velocity (master)",   "%.2f", velocity);
        telemetry.addData("Error",                    "%.2f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (DPad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (DPad L/R)", F);
        telemetry.addData("Step Size","%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();
    }
}