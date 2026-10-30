package org.firstinspires.ftc.teamcode.pedro;

//pedro imports
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import static com.pedropathing.ivy.Scheduler.schedule;
//by statically importing, we can use line() and curve() instead of Paths.line() and Paths.curve()
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
public class PoseStorage {
    //easy reference to poses
    private Follower follower;
    //Pose factory is used to make the poses. declared here as public, since its children are
    //you must  use this if you are using degrees
    public static PoseFactory p = PoseFactory.degrees();

    //poses here should be specified as public static. this is because they must be
    //accessed and possibly changed by other classes

    //pose defining skeleton is: p.of(x, y, heading);


    //below are the starting positions of the robot against the wall facing the specified goal
    public static Pose goalNoGardenRed = p.of(64, 133, 270);

    public static Pose goalGardenRed= p.of(64, 8.3, 90);

    public static Pose goalGardenBlue = p.of(77, 133, 270);

    public static  Pose goalNoGardenBlue=  p.of(77, 8.3, 90);



    //below are the 4 shooting positions.
    public static Pose shootRedFlower = p.of(64, 130, 270);

    public static Pose shootRedNoFlower = p.of(58, 10, 90);

    public static Pose shootBlueFlower = p.of(77, 10, 90);

    public static Pose shootBlueNoFlower = p.of(85, 130, 270);

    //below are Poses for parking.
    //a

    public static Pose parkGardenBlue = p.of(132.6, 22.4, 180);

    public static Pose parkNoGardenBlue= p.of(132.6, 48.8, 180);

    public static Pose parkGardenRed = p.of(8.8, 120, 0);

    public static Pose parkNoGardenRed = p.of(8.8, 92.3, 0);


    //testing pose to transfer pose directly from auto to teleop in case of error.
    public static Pose autonomousEndPose = p.of(0, 0, 0);
    // default pose to avoid null errors

    /* distance form from point (a, b) to goal (c, d)
    distance= Math.sqrt(((a-c)*(a-c))+((b-d)*(b-d)))
    blue flower goal=  (84, 57)
    blue noflower goal= (84, 84)
    red flower goal=  (58, 84)
    red noflower goal= ( 58, 57)
    //these are also the control points for the park paths
     */









    //below are paths from the starting position to its opposite goal.
    public static Path noToYesGardenRed() {

        return line(goalNoGardenRed, goalGardenRed).linear(goalNoGardenRed, goalGardenRed);
    }
    public static Path yesToNoGardenRed() {
        return line(goalGardenRed, goalNoGardenRed).linear(goalGardenRed, goalNoGardenRed);
    }


    public static Path noToYesGardenBlue() {
        return line(goalNoGardenBlue, goalGardenBlue).linear(goalNoGardenBlue, goalGardenBlue);
    }

    public static Path noToYesFlowerBlue() {
        return line(goalGardenBlue, goalNoGardenBlue).linear(goalGardenBlue, goalNoGardenBlue);
    }

    //below are pose areas for the robots 'harm list' in progress as of 10/13


    //below are the paths for starting position to shooting position.

    public static Path toShootRedFlower() {
        return line(goalNoGardenRed, shootRedFlower).linear(goalNoGardenRed, shootRedFlower);
    }
    public static Path toShootRedNoFlower() {
        return line(goalGardenRed, shootRedNoFlower).linear(goalGardenRed, shootRedNoFlower);
    }

    public static Path toShootBlueFlower() {
        return line(goalNoGardenBlue, shootBlueFlower).linear(goalNoGardenBlue, shootBlueFlower);
    }
    public static Path toShootBlueNoFlower() {
        return line(goalGardenBlue, shootBlueNoFlower).linear(goalGardenBlue, shootBlueNoFlower);
    }
    public static Pose noGardenBluePark = p.of(128, 25, 180);

    public static Pose gardenBluePark = p.of(128, 43, 180);

    public static Pose noGardenRedPark = p.of(8, 116, 0);

    public static Pose gardenRedPark = p.of(8.5, 95, 0);
    public static Pose blueFlowerGoalControl = p.of(84, 57, 0);

    public static Pose blueNoFlowerGoalControl = p.of(84, 84, 0);

    public static Pose redFlowerGoalControl = p.of(58, 99, 0);

    public static Pose redNoFlowerGoalControl= p.of(58, 57, 0);

    public static Pose control1 = p.of( 117, 108, 0); //for far blue park

    public static Pose control2 = p.of(20, 14, 0); //for far red park
    public static Path toParkBlueNoGarden() {
        return Paths.curve(goalNoGardenBlue, blueFlowerGoalControl, noGardenBluePark).linear(goalNoGardenBlue, blueFlowerGoalControl);
    }
    public static Path toParkBlueYesGarden() {
        return Paths.curve(goalGardenBlue, control1, gardenBluePark).linear(goalGardenBlue, control1);
    }
    public static Path toParkRedNoGarden() {
        return Paths.curve(goalNoGardenRed, redFlowerGoalControl, noGardenRedPark).linear(goalNoGardenRed, redFlowerGoalControl);
    }
    public static Path toParkRedYesGarden() {
        return Paths.curve(goalGardenRed, control2, gardenRedPark).linear(goalGardenRed, control2);
    }

    //below are paths
}
