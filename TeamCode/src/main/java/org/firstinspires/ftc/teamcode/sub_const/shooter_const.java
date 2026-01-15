package org.firstinspires.ftc.teamcode.sub_const;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

public class shooter_const {
    public static Pose BLUE_GOAL = new Pose(138,138);
    public static Pose RED_GOAL = BLUE_GOAL.mirror();
    public static double SCORE_HEIGHT = 26; //inch
    public static double SCORE_ANGLE = Math.toRadians(-30); //rad
    //public static double PASS_THROUGH_POINT_RADIUS = 5; //inch

    public static double HOOD_MIN_ANGLE = Math.toRadians(2);    //NEED ADJ
    public static double HOOD_MAX_ANGLE = Math.toRadians(6);    //NEED ADJ
    public static double HOOD_SERVO_MIN = 0.1;    //NEED ADJ
    public static double HOOD_SERVO_MAX = 0.2;    //NEED ADJ

    public static double TICKS_PER_REV_SHOOTER = 103.8;
    public static double WHEEL_RADIUS = 2.0;    //NEED ADJ


}
