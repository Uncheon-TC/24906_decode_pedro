package org.firstinspires.ftc.teamcode.sub_const;

import com.pedropathing.geometry.Pose;

public class pos_const {
    public static Pose RED_GOAL = new Pose(138,134);
    public static Pose BLUE_GOAL = RED_GOAL.mirror();

    public static Pose RED_CLOSE_START = new Pose(121.25, 121.24, 0.73);
    public static Pose RED_CLOSE_ST_SHOOT = new Pose(96, 95, 0.73);
    public static Pose RED_CLOSE_EAT1 = new Pose(124, 83.5, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT2 = new Pose(124,59.5, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT_SLIDE = new Pose (135, 64, Math.toRadians(37));
    public static Pose RED_CLOSE_SHOOT1 = new Pose(84, 83.5, Math.toRadians(0));


}
