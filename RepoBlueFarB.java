package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Reposition, Blue, Far, B start" )

public class RepoBlueFarB extends AutoOp{
    // Start all the way toward the build side on the second tile from
    // the middle on the stone side.
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
        forward(1.8);
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
        strafeRight(1.45);
        forward(2.1);
        strafeLeft(1.5);
        backward(2.1);
        forward(.45);
        right(1.2);
        forward(2.3);
        closeClaw();
        /*fingerUp();
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
        forward(1.5);
        right(1);
        forward(1);*/
    }
}