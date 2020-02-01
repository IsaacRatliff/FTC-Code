package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Reposition (Red, Far)", group="Linear Opmode")

public class RepositionRedFar extends AutoOp {

    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();
        fingerUp();
        //forward(0.75);
        //right(2);  // turn around
        backward(0.5);
        right(0.5);
        backward(0.5);
        left(0.5);
        backward(0.5);
        pause(0.5);
        fingerDown();
        pause(1.0);
        
        left(6);
        backward(1.0);
        left(8);
        backward(6);
        //pause(10.0);
        
        
        /* Drag
        //left(0.5);
        forward(2.0);
        //right(0.75);
        forward(0.775);
        pause(0.5);
        fingerUp();
        pause(1.0);*/
        
        // Park
        right(2);
        forward(1.5);
        pause(10.0);
        
        
        // Rotate Wall
        /*backward(0.25);  // unnecessary?
        right(2);
        fingerUp();
        forward(3);
        armUp(0.5);
        right(1);
        armDown(0.5);
        closeClaw();
        right(1);
        forward(2.5);
        armUp(0.5);
        forward(0.5);
        armDown(0.35);
        openClaw();
        armUp(0.15);
        backward(1.0);
        armDown(0.5);
        backward(1.5);*/
        // the end
    }
}
