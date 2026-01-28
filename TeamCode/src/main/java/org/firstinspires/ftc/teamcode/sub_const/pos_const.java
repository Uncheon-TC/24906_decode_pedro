package org.firstinspires.ftc.teamcode.sub_const;

import com.pedropathing.geometry.Pose;

public class pos_const {
    public static Pose RED_GOAL = new Pose(144,144);
    public static Pose BLUE_GOAL = RED_GOAL.mirror();

    public static Pose RED_CLOSE_START = new Pose(121.25, 121.24, 0.73);
    public static Pose RED_CLOSE_ST_SHOOT = new Pose(96, 95, 0.73);
    public static Pose RED_CLOSE_EAT1 = new Pose(122, 83.5, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT2 = new Pose(124,59.5, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT3 = new Pose(124, 35.5, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT_SLIDE = new Pose (130.25, 60.8, 0.654);
    public static Pose RED_CLOSE_SLIDE_OPEN = new Pose(129, 70, Math.toRadians(270));
    public static Pose RED_CLOSE_SHOOT1 = new Pose(84, 83.5, Math.toRadians(0));


}
