package org.firstinspires.ftc.teamcode.pedro.TeleOps;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.pedro.PoseStorage.goalNoGardenBlue;
import static org.firstinspires.ftc.teamcode.pedro.PoseStorage.p;
import static org.firstinspires.ftc.teamcode.pedro.PoseStorage.shootBlueFlower;
import static org.firstinspires.ftc.teamcode.pedro.PoseStorage.shootBlueNoFlower;
import static org.firstinspires.ftc.teamcode.pedro.PoseStorage.shootRedFlower;
import static org.firstinspires.ftc.teamcode.pedro.PoseStorage.shootRedNoFlower;

import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
@TeleOp
public class PedroTeleBlue extends LinearOpMode {

    private DcMotor intake;
    private DcMotorEx flywheel;
    private CRServo crServo;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo pollenServo;

    private ElapsedTime autoTimer;

    int targetVelocity = 1160;

    double F = 13.9984;
    double P = 310;
    int gate = 0; // 1 means flywheel on, 0 means its off

    /**
     * The section below contains the iterative OpMode for the robot to operate so
     * long as "Stop" has not been pressed OR communication to the Driver Station
     * has not been lost.
     */
    private Follower follower;

    boolean flowerGoal = true;
    double a = 0.0;
    double b= 0.0;

    double c = 0.0;
    double d = 0.0;

    double deltaX = 0.0;

    double deltaY = 0.0;

    double targetHeading= 0.0;

    double headingError= 0.0;

    double distance = 0.0;

    private Path toShootFlow() {
        return  line(follower.pose(), shootBlueFlower).linear(follower.pose(), shootBlueFlower);
    }

    private Path toShootNoFlow() {
        return line(follower.pose(), shootBlueNoFlower).linear(follower.pose(), shootBlueNoFlower);
    }

    private Pose emergencyZeroed= p.of(134, 135, 90);
    @Override
    public void runOpMode() {
        autoTimer = new ElapsedTime();
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        crServo = hardwareMap.get(CRServo.class, "crServo");
        pollenServo = hardwareMap.get(Servo.class, "pollenServo");
//hjgjtdjgfhgfz
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        // Setting up motors and variables
        // flywheel.setDirection();


        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );
        waitForStart();
        if (opModeIsActive()) {
            // Ensures the servo is up. Runs once after "Play" is pressed.
            follower = Constants.create(hardwareMap);
            follower.setPose(PoseStorage.autonomousEndPose);
            pollenServo.setPosition(1);
            // Loop for the OpMode containing the functions
            while (opModeIsActive()) {
                follower.update();
            intakeControls();
                pollenPoker();
                pedroDrive();
                flywheelControls();
                shootControls();
                //velocityControls();
                //alignWithGoal();
                goToShootPosition();
                emergencyZero();
                distanceCalcs();



                // Allows for velocity to be seen on the Driver Station
                /*telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
                telemetry.addData("Target Velocity", targetVelocity);
                //pedro coords telemetry
                follower.update();
                Pose robotPose = follower.pose(); // returns a Pose object
                telemetry.addData("Robot X", robotPose.x());
                telemetry.addData("Robot Y", robotPose.y());
                telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
                telemetry.addData("Distance: ", distanceCalcs());
                telemetry.update(); */





            }
        }
    }

//Below are the individual functions, also called methods, for each system.

    /**
     * Intake Controls
     * This function contains the controls for the intake system using the Core Hex.
     */
    private void emergencyZero() {
        if(gamepad1.dpadRightWasPressed()) {
            follower.setPose(emergencyZeroed);
            follower.update();
        }
    }

    private void telemetry() {
        telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
        telemetry.addData("Target Velocity", targetVelocity);
        //pedro coords telemetry
        telemetry.addData("Robot X", follower.pose().x());
        telemetry.addData("Robot Y", follower.pose().y());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Distance: ", distance);
        //telemetry.addData("Target Heading: ", targetHeading);
        //telemetry.addData("Robot Heading: ", robotPose.heading());
        //telemetry.addData("Heading Error: ", headingError);
        telemetry.addData("Flower goal: ", flowerGoal);
        if(follower.pose().y()<72) {
            telemetry.addData("Should be: ", true);
        }else{
            telemetry.addData("Should be: ", false);
        }
        telemetry.update();
    }
    private void goToShootPosition() {
        if(gamepad1.triangleWasPressed()) {
            if(flowerGoal) {
                follower.hold(p.of(follower.pose().x(), follower.pose().y(),90));
                follower.update();
            }else{
                follower.hold(p.of(follower.pose().x(), follower.pose().y(), 270));
                follower.update();
            }

        }
        if(gamepad1.circleWasPressed()) {
            if(flowerGoal) {
                schedule(follow(follower, toShootFlow()));
            }else {
                schedule(follow(follower, toShootNoFlow()));
            }
            Scheduler.execute();
        }
    }
    private void alignWithGoal() {
        Pose robotPose = follower.pose();
        targetHeading = Math.atan2(deltaY, deltaX);
        Pose currentPose = p.of(robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading()));
        Pose alignedHeading = p.of(robotPose.x(), robotPose.y(), Math.toDegrees(targetHeading));
        if(gamepad1.circleWasPressed()) {
            headingError = targetHeading - robotPose.heading();
            robotPose = follower.pose();
            telemetry();
            follower.hold(alignedHeading);
            follower.update();
        }
    }
    private void distanceCalcs() {
        if(gamepad1.xWasPressed() && flowerGoal) {
            flowerGoal=false;
        }
        if(gamepad1.xWasPressed() && !flowerGoal) {
            flowerGoal=true;
        }
        crServo.setPower(0);
        Pose robotPose = follower.pose();
        a= robotPose.x();
        b= robotPose.y();

        c= 84.0;
        if(flowerGoal) {
            d= 57.0;
        }else{
            d= 84.0;
        }
        deltaX = a-c;
        deltaY = b-d;
        distance= Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }
    private void velocityControls() {
        if(gamepad1.dpadRightWasPressed()) {
            targetVelocity += 20;
        }else if(gamepad1.dpadLeftWasPressed()) {
            targetVelocity -= 20;
        }
    }
    private void intakeControls() {
        // Intake in and out
        intake = hardwareMap.get(DcMotor.class, "intake");
        if (gamepad1.left_trigger!=0) {
            intake.setPower(gamepad1.left_trigger);
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0);
        }
    }

    /**
     * Controls for the Pollen poking servo
     * This function contains the code for the servo powered pollen poker on the front of the robot.
     */

    private void pollenPoker() {
        // Servo control for poking out pollen
        if (gamepad1.dpadDownWasPressed()) {
            pollenServo.setPosition(0.6);
        } else if (gamepad1.dpadUpWasPressed()) {
            pollenServo.setPosition(1);
        }
    }

    /**
     * Controls for Arcade Drive
     * This function contains the code for the arcade drive controls.
     */

    private void pedroDrive() {
        ManualDrive.driveOrHold(
                follower,
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        telemetry();
        follower.update();
        telemetry();
    }

    /**
     * Controls for the flywheel - manual and auto sequence
     * This function contains the controls for the flywheel system.
     */
    private void flywheelControls() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        if (gamepad1.rightBumperWasPressed()) {
            if(gate==0) {
                // flywheel.setPower(flyPower);
                flywheel.setPIDFCoefficients(
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(P, 0, 0, F)
                );
                flywheel.setVelocity(targetVelocity);
                gate=1;
            }else{
                if(gate==1) {
                    flywheel.setPower(0);
                    gate=0;
                }
            }
        }else if(gamepad1.dpad_left) {
            // Manual control for feeder servo
            while(gamepad1.dpad_left) {
                crServo.setPower(-1.0); //reverse
                pedroDrive();
                intakeControls();
                flywheelControls();
            }
        }
        crServo.setPower(0);
    }

    private void shootControls() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        //any use of arcadeDrive(); is so the driven retains control of the bot's movement, but cant mess up the intake side
        while(gamepad1.right_trigger!=0) {
            if(isStopRequested()) return;
            if(gate==0) {
                flywheel.setPIDFCoefficients(
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(P, 0, 0, F)
                );
                flywheel.setVelocity(targetVelocity);
                pedroDrive();
                //gives time to rev up, about 2.5 sec
                gate=1;
                autoTimer.reset();
                while ((flywheel.getVelocity()<(targetVelocity-20) && flywheel.getVelocity()>(targetVelocity+20))) {
                    if(isStopRequested()) return;
                    if(gamepad1.right_trigger==0) {
                        crServo.setPower(0);
                        gate=0;
                        break;
                    }
                    pedroDrive();
                    //flywheel.setPower(flyPower);
                    gate=1;
                }
            }
            //now that flywheel is revved up, goes to shoot.
            //intake.setPower(1);
            autoTimer.reset();
            while (autoTimer.seconds() < 1.5) {
                if(isStopRequested()) return;
                intake.setPower(0);
                crServo.setPower(1);
                pedroDrive();
                if(gamepad1.right_trigger==0) {
                    crServo.setPower(0);
                    gate=0;
                    break;
                }
            }
            crServo.setPower(0);
            //now that the crservo has done its job, it intakes
            autoTimer.reset();
            while(autoTimer.milliseconds()<500) {
                if(isStopRequested()) return;
                intake.setPower(1);
                pedroDrive();
            }
            intake.setPower(0);
            autoTimer.reset();
            while(autoTimer.milliseconds()<200) {
                if(isStopRequested()) return;
                pedroDrive();
            }
            if(gamepad1.right_trigger==0) {
                crServo.setPower(0);
                gate=0;
                break;
            }
            pedroDrive();

        }
        crServo.setPower(0);
        intake.setPower(0);
    }
}

