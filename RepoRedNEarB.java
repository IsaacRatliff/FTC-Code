package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Reposition, Red, Near, B start" )

public class RepoRedNEarB extends AutoOp {
    // Start all the way to the build side on the first build side tile from
    // the middle.
    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();
        fingerUp();
        forward(0.2);
        pause(0.5);
        strafeRight(2);
        pause(0.5);
        armUp();
        pause(0.5);
        forward(0.45);
        pause(0.5);
        fingerDown();
        pause(2);
        backward(2);
        pause(2);
        fingerUp();
        pause(1);
        armDown();
        strafeLeft(1.4);
        forward(0.5);
        left(1);
        forward(1);
    }
}