package org.firstinspires.ftc.teamcode.pedro.Autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;

@Autonomous
public class BlueGardenShoot extends OpMode {
    private DcMotor intake;
    private DcMotorEx flywheel;
    private CRServo crServo;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo pollenServo;

    private ElapsedTime autoTimer;

    private ElapsedTime servoTimer;
    double F = 13.9984;
    double P = 310;
    int targetVelocity = 1200;
    private Follower follower;
    public void init() {
        autoTimer = new ElapsedTime();
        servoTimer= new ElapsedTime();
        flywheel = hardwareMap.get(DcMotorEx .class, "flywheel");
        intake= hardwareMap.get(DcMotor.class, "intake");
        crServo = hardwareMap.get(CRServo .class, "crServo");
        pollenServo = hardwareMap.get(Servo .class, "pollenServo");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );
    }
    public void start() {
        follower = Constants.create(hardwareMap);
        follower.setPose(PoseStorage.goalGardenBlue);
        schedule(follow(follower, PoseStorage.toShootBlueNoFlower()));
        Scheduler.execute();
        follower.update();
        flywheel.setVelocity(targetVelocity);
    }
    public void loop() {
        while(autoTimer.seconds()<15) {
            follower.update();
            if(!follower.isBusy()) {
                follower.update();
                telemetry();
                while(flywheel.getVelocity()>targetVelocity+20 || flywheel.getVelocity()<targetVelocity-20) {
                    flywheel.setPIDFCoefficients(
                            DcMotor.RunMode.RUN_USING_ENCODER,
                            new PIDFCoefficients(P, 0, 0, F)
                    );
                    flywheel.setVelocity(targetVelocity);
                    follower.update();
                    telemetry();
                }
                crServo.setPower(1);
                intake.setPower(0);
                servoTimer.reset();
                while(servoTimer.seconds()<1.5) {
                    follower.update();
                    telemetry();
                }
                crServo.setPower(0);
                intake.setPower(1);
                servoTimer.reset();
                while(servoTimer.milliseconds()<400) {
                    follower.update();
                    telemetry();
                }
                intake.setPower(0);
                crServo.setPower(0);
                servoTimer.reset();
                while(servoTimer.milliseconds()<200) {
                    follower.update();
                    telemetry();
                }
            }
        }
        follower.update();
        if(!follower.isBusy() && autoTimer.seconds()>15) {
            flywheel.setPower(0);
            follower.update();
            schedule(follow(follower, PoseStorage.toParkBlueYesGarden()));
            Scheduler.execute();
        }
    }


    public void telemetry() {
        telemetry.addData("Robot X", follower.pose().x());
        telemetry.addData("Robot Y", follower.pose().y());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Target Velocity:     " , targetVelocity);
        telemetry.addData("Flywheel Velocity:     " , flywheel.getVelocity());
    }

    public void stop() {
        PoseStorage.autonomousEndPose = follower.pose();
        flywheel.setVelocity(0);
    }
}
