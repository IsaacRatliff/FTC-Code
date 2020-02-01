package workspace;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="10 Pt: Reposition (Red)", group="Linear Opmode")

public class RepositionRed extends AutoOp {
    
    /**
    Starting Position:
        Centered on the border between the 2nd and 3rd tile from the build side.
    
    Objectives:
        1. Drive around the Foundation and push it into the Build Zone.
        2. 
    **/

    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();
        forward(2.5);
        right(1);
        forward(1.0);
        right(1);
        forward(2.25);
        backward(0.25);
        right(1);
        forward(1.75);
    }
}
