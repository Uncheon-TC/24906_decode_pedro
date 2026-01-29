package org.firstinspires.ftc.teamcode.sub_const;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;

public class pos_const {
    public static Pose RED_GOAL = new Pose(144,144);
    public static Pose BLUE_GOAL = RED_GOAL.mirror();

    public static Pose RED_CLOSE_START = new Pose(122, 121, 0.7);
    public static Pose RED_CLOSE_ST_SHOOT = new Pose(96, 95, 0.7);
    public static Pose RED_CLOSE_EAT1 = new Pose(120, 83.5, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT2 = new Pose(124,59.5, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT3 = new Pose(124, 35.5, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT4 = new Pose(139.5, 16.5, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT_SLIDE = new Pose (130, 60.8, 0.654);
    public static Pose RED_CLOSE_SLIDE_OPEN = new Pose(131.5, 68, Math.toRadians(270));
    public static Pose RED_CLOSE_SHOOT1 = new Pose(84, 83.5, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT1_CP = new Pose(68, 82, 0);
    public static Pose RED_CLOSE_EAT2_CP = new Pose(72, 57, 0);
    public static Pose RED_CLOSE_SLIDE_OPEN_CP = new Pose(113, 61, 0);
    public static Pose RED_CLOSE_EAT_SLIDE_CP = new Pose(116, 36, 0);
    public static Pose RED_CLOSE_EAT3_CP = new Pose(68, 29, 0);
    public static Pose RED_CLOSE_EAT4_CP = new Pose(134, 78.2, 0);
    public static Pose RED_CLOSE_END = new Pose (103, 83.5, Math.toRadians(90));



    public static Pose BLUE_CLOSE_START = RED_CLOSE_START.mirror();
    public static Pose BLUE_CLOSE_ST_SHOOT = RED_CLOSE_ST_SHOOT.mirror();
    public static Pose BLUE_CLOSE_EAT1 = RED_CLOSE_EAT1.mirror();
    public static Pose BLUE_CLOSE_EAT2 = RED_CLOSE_EAT2.mirror();
    public static Pose BLUE_CLOSE_EAT3 = RED_CLOSE_EAT3.mirror();
    public static Pose BLUE_CLOSE_EAT4 = RED_CLOSE_EAT4.mirror();
    public static Pose BLUE_CLOSE_EAT_SLIDE = RED_CLOSE_EAT_SLIDE.mirror();
    public static Pose BLUE_CLOSE_SLIDE_OPEN = RED_CLOSE_SLIDE_OPEN.mirror();
    public static Pose BLUE_CLOSE_SHOOT1 = RED_CLOSE_SHOOT1.mirror();


    public static Pose BLUE_CLOSE_EAT1_CP = RED_CLOSE_EAT1_CP.mirror();
    public static Pose BLUE_CLOSE_EAT2_CP = RED_CLOSE_EAT2_CP.mirror();
    public static Pose BLUE_CLOSE_SLIDE_OPEN_CP = RED_CLOSE_SLIDE_OPEN_CP.mirror();
    public static Pose BLUE_CLOSE_EAT_SLIDE_CP = RED_CLOSE_EAT_SLIDE_CP.mirror();
    public static Pose BLUE_CLOSE_EAT3_CP = RED_CLOSE_EAT3_CP.mirror();
    public static Pose BLUE_CLOSE_EAT4_CP = RED_CLOSE_EAT4_CP.mirror();
    public static Pose BLUE_CLOSE_END = RED_CLOSE_END.mirror();


    public static Pose savedAutoPose = new Pose(0,0,0);


}
