package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Reposition, Blue, Near, B start" )

public class RepoBlueNearB extends AutoOp {

    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();
        fingerUp();
        forward(0.76);
        pause(0.5);
        strafeLeft(2);
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
        strafeRight(1.4);
        right(1);
        forward(1);
    // todo: write your code here
    }
}