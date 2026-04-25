package org.firstinspires.ftc.teamcode.sub_const;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;

public class pos_const {
  // 시작위치
    public static Pose RED_GOAL = new Pose(144,144);
    public static Pose BLUE_GOAL = new Pose(0, 144);
// RED CLOSE 18 Artifact
    public static Pose RED_CLOSE_START = new Pose(120, 122, Math.toRadians(41));
    public static Pose RED_CLOSE_ST_SHOOT = new Pose(108, 100, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT2_CP = new Pose(85, 60, 0);
    public static Pose RED_CLOSE_EAT2 = new Pose(125,59.5, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT1 = new Pose(90, 80, Math.toRadians(0));
    public static Pose RED_CLOSE_GATE1 = new Pose(128,57, Math.toRadians(40)); // 게이트열러대기
    public static Pose RED_CLOSE_GATE2 = new Pose(134,63, Math.toRadians(40)); // 게이트여는위치
    public static Pose RED_CLOSE_END = new Pose (108, 83.5, Math.toRadians(90));


//===================================================================================
// RED 예전 코드에서 쓰는 좌표
    public static Pose RED_CLOSE_EAT1 = new Pose(120, 83.5, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT3 = new Pose(124, 35.5, Math.toRadians(0));
    public static Pose RED_CLOSE_EAT4 = new Pose(136, 16.5, Math.toRadians(0));
    public static Pose RED_CLOSE_GATE3 = new Pose(130,57, Math.toRadians(40));
    public static Pose RED_CLOSE_EAT_SLIDE = new Pose (130, 60.8, 0.654);
    public static Pose RED_CLOSE_SLIDE_OPEN = new Pose(131.5, 68, Math.toRadians(270));
    public static Pose RED_CLOSE_EAT1_CP = new Pose(68, 82, 0);
    public static Pose RED_CLOSE_SLIDE_OPEN_CP = new Pose(113, 61, 0);
    public static Pose RED_CLOSE_EAT_SLIDE_CP = new Pose(116, 36, 0);
    public static Pose RED_CLOSE_EAT3_CP = new Pose(68, 29, 0);
    public static Pose RED_CLOSE_EAT4_CP = new Pose(134, 78.2, 0);

// ==============================================================================


    // BLUE_CLOSE
    public static Pose BLUE_CLOSE_START = new Pose(24,122,Math.toRadians(140));
    public static Pose BLUE_CLOSE_ST_SHOOT = RED_CLOSE_ST_SHOOT.mirror();
    public static Pose BLUE_CLOSE_EAT1 = RED_CLOSE_EAT1.mirror();
    public static Pose BLUE_CLOSE_EAT2 = RED_CLOSE_EAT2.mirror();
    public static Pose BLUE_CLOSE_EAT3 = RED_CLOSE_EAT3.mirror();
    public static Pose BLUE_CLOSE_EAT4 = RED_CLOSE_EAT4.mirror();
    public static Pose BLUE_CLOSE_EAT_SLIDE = RED_CLOSE_EAT_SLIDE.mirror();
    public static Pose BLUE_CLOSE_SLIDE_OPEN = RED_CLOSE_SLIDE_OPEN.mirror();
    public static Pose BLUE_CLOSE_SHOOT1 = RED_CLOSE_SHOOT1.mirror();
    public static Pose BLUE_CLOSE_GATE1 = RED_CLOSE_GATE1.mirror();
    public static Pose BLUE_CLOSE_GATE2 = RED_CLOSE_GATE2.mirror();
    public static Pose BLUE_CLOSE_GATE3 = RED_CLOSE_GATE3.mirror();


    public static Pose BLUE_CLOSE_EAT1_CP = RED_CLOSE_EAT1_CP.mirror();
    public static Pose BLUE_CLOSE_EAT2_CP = RED_CLOSE_EAT2_CP.mirror();
    public static Pose BLUE_CLOSE_SLIDE_OPEN_CP = RED_CLOSE_SLIDE_OPEN_CP.mirror();
    public static Pose BLUE_CLOSE_EAT_SLIDE_CP = RED_CLOSE_EAT_SLIDE_CP.mirror();
    public static Pose BLUE_CLOSE_EAT3_CP = RED_CLOSE_EAT3_CP.mirror();
    public static Pose BLUE_CLOSE_EAT4_CP = RED_CLOSE_EAT4_CP.mirror();
    public static Pose BLUE_CLOSE_END = RED_CLOSE_END.mirror();


// RED_FAR
    public static Pose RED_FAR_15_START = new Pose(95, 8, Math.toRadians(0));
    public static Pose RED_FAR_15_EAT1 = new Pose(128,35, Math.toRadians(0));
    public static Pose RED_FAR_15_EAT1_CP = new Pose(93,39, Math.toRadians(0));
    public static Pose RED_FAR_15_SHOOT = new Pose(94, 12, Math.toRadians(0));
    public static Pose RED_FAR_15_EAT2 = new Pose(130,8,Math.toRadians(0));
    public static Pose RED_FAR_15_EAT2_CP = new Pose(123,10,Math.toRadians(0));
    public static Pose RED_FAR_15_EAT2_Again = new Pose(130,12, Math.toRadians(0));
    public static Pose RED_FAR_15_EAT3 = new Pose(130, 8, Math.toRadians(40));
    public static Pose RED_FAR_15_EAT3_Again = new Pose(130,40,Math.toRadians(40));
    public static Pose RED_FAR_END = new Pose(110,8, Math.toRadians(0));

    // BLUE_FAR
    public static Pose BLUE_FAR_15_START =  RED_FAR_15_START.mirror();
    public static Pose BLUE_FAR_15_EAT1 = RED_FAR_15_EAT1.mirror();
    public static Pose BLUE_FAR_15_EAT1_CP = RED_FAR_15_EAT1_CP.mirror();
    public static Pose BLUE_FAR_15_SHOOT = RED_FAR_15_SHOOT.mirror();
    public static Pose BLUE_FAR_15_EAT2 = RED_FAR_15_EAT2.mirror();
    public static Pose BLUE_FAR_15_EAT2_CP = RED_FAR_15_EAT2_CP.mirror();
    public static Pose BLUE_FAR_15_EAT2_Again = RED_FAR_15_EAT2_Again.mirror();
    public static Pose BLUE_FAR_15_EAT3 = RED_FAR_15_EAT3.mirror();
    public static Pose BLUE_FAR_15_EAT3_Again = RED_FAR_15_EAT3_Again.mirror();
    public static Pose BLUE_FAR_END = RED_FAR_END.mirror();



    public static Pose savedAutoPose = new Pose(0,0,0);


}
