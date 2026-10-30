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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;

@Autonomous
public class RedGoParkGarden extends OpMode {
    private DcMotor intake;
    private DcMotorEx flywheel;
    private CRServo crServo;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo pollenServo;

    private ElapsedTime autoTimer;

    private Follower follower;


    public void init() {
        autoTimer = new ElapsedTime();
        flywheel = hardwareMap.get(DcMotorEx .class, "flywheel");
        crServo = hardwareMap.get(CRServo .class, "crServo");
        pollenServo = hardwareMap.get(Servo .class, "pollenServo");
    }
    public void start() {
        follower = Constants.create(hardwareMap);
        follower.setPose(PoseStorage.goalGardenRed);
        schedule(follow(follower, PoseStorage.toParkRedYesGarden()));
        Scheduler.execute();
    }
    public void loop() {
        telemetry.addData("Robot X", follower.pose().x());
        telemetry.addData("Robot Y", follower.pose().y());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.pose().heading()));
        follower.update();
        schedule(follow(follower, PoseStorage.toParkRedYesGarden()));
        Scheduler.execute();
    }

    public void stop() {
        PoseStorage.autonomousEndPose= follower.pose();
    }
}
