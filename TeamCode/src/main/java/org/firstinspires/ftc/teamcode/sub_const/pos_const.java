package org.firstinspires.ftc.teamcode.sub_const;

import com.pedropathing.geometry.Pose;

public class pos_const {
    public static Pose RED_GOAL = new Pose(138,134);
    public static Pose BLUE_GOAL = RED_GOAL.mirror();

    public static Pose RED_CLOSE_START = new Pose(111, 135, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT1 = new Pose(130, 84, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT2 = new Pose(131,60, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT1 = new Pose(96, 100, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT2 = new Pose(136, 64, Math.toRadians(0));


}
