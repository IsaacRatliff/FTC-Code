package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Reposition, Blue, Far, S start" )

public class RepoBlueFarS extends AutoOp {
    // Done, 2020-1-17
    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();
        fingerUp();
        openClaw();
        forward(0.7); //0.45
        pause(0.5);
        //strafeRight(3.8);  //3.75
        left(1);
        forward(4.2);
        right(1);
        pause(0.5);
        //armUp();
       // pause(0.5);
        forward(0.475);
        pause(0.5);
        fingerDown();
        pause(2);
        backward(2.25);  //2
        //left(2);
        pause(2);
      //  left(1);
      //  pause(1);
       // armDown();
        left(.3);
        pause(1);
        fingerUp();
        pause(0.5);
        right(0.2);
        strafeRight(1.3);
        forward(2);
        strafeLeft(2);
        backward(2);
        forward(.45);
        right(1.2);
        forward(1.7);
        //strafeRight(2);
    }
}