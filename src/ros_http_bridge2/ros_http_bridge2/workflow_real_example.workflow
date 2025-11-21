Mode: real

Agents:
    Robot1

Locations:
    Dummy: {position: [0.0, 0.0, 0.0], orientation: [0.0, 0.0, 0.0]}

Trays:
    DummyTray: {
        tray: Dummy
        units: [ { name: U1, pose_index: 0 } ]
        screws: [1 x M4]
        height: 0.1
        initial_pose: [0,0,0,0,0,0]
        operator_pose: [0,0,0,0,0,0]
        final_pose: [0,0,0,0,0,0]
    }

Assembly:
    Robot1 screwPickup: screw 3 ToConfirm;
    Robot1 loadTray: load true ToConfirm;
    Robot1 depositTray: ToConfirm;
    Robot1 mrHoming: ToConfirm;
    Robot1 srHoming: ToConfirm;
    Robot1 positionerRotate: clockwise true ToConfirm;
    Robot1 gyroGrpRot: direction 1 ToConfirm;
    Robot1 present2Op: side 1 face 2 ToConfirm;
    Robot1 presentToScrew: side 2 face 3 ToConfirm;
    Robot1 mrTrolleyVCheck: ToConfirm;
    Robot1 mrTrolleyVCheckErrorCheck: ToConfirm;
    Robot1 stackTray: number 2 ToConfirm;
    Robot1 pickUpTray: ToConfirm;
    Robot1 depositTray: ToConfirm;
    Robot1 present2Op: side 2 face 4 ToConfirm;
    Robot1 presentToScrew: side 1 face 1 ToConfirm;
   
