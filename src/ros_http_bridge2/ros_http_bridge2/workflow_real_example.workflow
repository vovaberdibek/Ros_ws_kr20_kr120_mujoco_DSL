Mode: real

Agents:
    Robot1

Locations:
    Placeholder: {position: [0.0, 0.0, 0.0], orientation: [0.0, 0.0, 0.0]}

Trays:
    DemoTray: {
        tray: Demo
        units:[
            { name: U1, pose_index: 0 }
        ]
        screws: [1 x M4]
        height: 0.1
        initial_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        operator_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        final_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }

Assembly:
    manual CallBlock name screwPickup screw 3
    manual CallBlock name loadTray load true
