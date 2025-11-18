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
    manual CallBlock name screwPickup screw 3
    manual CallBlock name loadTray load true
    manual CallBlock name depositTray
    manual CallBlock name mrHoming
    manual CallBlock name srHoming
    manual CallBlock name positionerRotate clockwise true
    manual CallBlock name gyroGrpRot direction 1
    manual CallBlock name present2Op side 1 face 2
    manual CallBlock name presentToScrew side 2 face 3
    manual CallBlock name mrTrolleyVCheck
    manual CallBlock name mrTrolleyVCheckErrorCheck  
    manual CallBlock name stackTray number 2
    manual CallBlock name pickUpTray
    manual CallBlock name depositTray
    manual CallBlock name present2Op side 2 face 4
    manual CallBlock name presentToScrew side 1 face 1
   
