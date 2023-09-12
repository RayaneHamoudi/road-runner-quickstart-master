package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52, 52, Math.toRadians(177), Math.toRadians(187), 15.22)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.7,-65, Math.toRadians(0))).setTangent(Math.toRadians(0))
                                .strafeLeft(12)
                                .addTemporalMarker(() -> {
                                    /*state = State.DETECT;*/
                                })
                                .strafeLeft(54)
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                                    /*state = State.LIFT;*/
                                })
                                .forward(4.5)
                                .addTemporalMarker(() -> {
                                    /*state = State.DROP;*/
                                })
                                .back(6)
                                .strafeRight(12)
                                .turn(Math.toRadians(180))


                                //reaches cone tower
                                //.setReversed(true)
                                .forward(28)
                                .addTemporalMarker(() -> {
                                    //close claw and go up at cone
                                  /*  clawLeft.setPosition(0);
                                    clawRight.setPosition(0.5);
                                    state = State.LIFT;*/
                                })

                                //cycle 1
                                //wait a bit so the cone tower doesn't fall and head to tall junction
                                .waitSeconds(0.3)
                                //.setReversed(true)
                                //.lineToLinearHeading(new Pose2d(-23.5, -12, Math.toRadians(90)))
                                .back(28)
                                .turn(Math.toRadians(-135))
                                .forward(4)
                                .addDisplacementMarker(() -> {
                                    //open claw and go down
                                    /*clawLeft.setPosition(0.5);
                                    clawRight.setPosition(0);
                                    state = State.DROP;*/
                                })
                                .back(6)
                                .setReversed(true)


                                //.lineToLinearHeading(new Pose2d(-67, -9, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-35, -5, Math.toRadians(180)))
                                .forward(27)
                                //cycle 2
                                .addTemporalMarker(() -> {
                                    //close claw and go up at cone
                                   /* clawLeft.setPosition(0);
                                    clawRight.setPosition(0.5);
                                    state = State.LIFT;*/
                                })
                                .waitSeconds(0.5)

                                //.setReversed(true)
                                //.lineToLinearHeading(new Pose2d(-25, -9, Math.toRadians(90)))
                                .back(28)
                                .turn(Math.toRadians(-135))
                                .forward(4)
                                .addTemporalMarker(() -> {
                                    //open claw and go down
                                   /* clawLeft.setPosition(0.5);
                                    clawRight.setPosition(0);
                                    state = State.DROP;*/
                                })
                                .back(6)
                                .setReversed(true)


                                //.lineToLinearHeading(new Pose2d(-67, -9, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-35, -5, Math.toRadians(180)))
                                .forward(27)
                                //cycle 2
                                .addTemporalMarker(() -> {
                                    //close claw and go up at cone
                                   /* clawLeft.setPosition(0);
                                    clawRight.setPosition(0.5);
                                    state = State.LIFT;*/
                                })
                                .waitSeconds(0.5)

                                //.setReversed(true)
                                //.lineToLinearHeading(new Pose2d(-25, -9, Math.toRadians(90)))
                                .back(28)
                                .turn(Math.toRadians(-135))
                                .forward(4)
                                .addTemporalMarker(() -> {
                                    //open claw and go down
                                    /*clawLeft.setPosition(0.5);
                                    clawRight.setPosition(0);
                                    state = State.DROP;*/
                                })
                                .back(6)
                                .build());
                ;


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}