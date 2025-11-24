Mode: simulation

Agents:
    Robot1
    Robot2
    HumanOP1
    AMR1

Locations:
    Table: {position: [1.252, -0.12, 0.004], orientation: [0, 0, -1.57]}
    RechargeArea: {position: [0.952, -0.29248, 1.1356], orientation: [0, 0, 0]}
    OperatorStation: {position: [1.0, 1.0, 1.55], orientation: [0, 0, 3.14]}
    AssemblyArea: {position: [0, -1.43, 0.75], orientation: [0, 0, 1.57]}
    PickStation: {position: [1.01016, 0.11975, 1.55225], orientation: [1.57, 1.57, 1.57]}


Trays:
    AOCS_Tray: {
        tray: AOC
        units:[
            { name: MTQ12, pose_name: TrayStepPose_0,
              screws: {
                manual_indices: [80,81,84,85],
                auto_indices:   [82,83,86,87]
              }
            },
            { name: MTQ3_MAG, pose_name: TrayStepPose_1,
              screws: {
                manual_indices: [88,90,92,93],
                auto_indices:   [89,91,94,95]
              }
            },
            { name: CMG2, pose_name: TrayStepPose_2,
              screws: {
                manual_indices: [20,27,33],
                auto_indices:   [21,22,23,24,25,26,28,29,31,32,34,35,36,37,38,39]
              }
            },
            { name: CMG1, pose_name: TrayStepPose_4,
              screws: {
                manual_indices: [3,10,17],
                auto_indices:   [0,1,2,4,5,6,7,8,9,11,12,13,14,15,16,18,19]
              } 
            },
            { name: CMG34, pose_name: TrayStepPose_6,
              screws: {
                manual_indices: [40,41,42,47,53,59,63,69,70,71,77],
                auto_indices:   [64,65,66,67,68,43,44,45,46,48,49,50,51,52,54,55,56,57,58,60,61,62,72,73,74,75,76,78,79]
              } 
            },
            ]
        screws: [96 x M4]
        height: 0.25
        initial_pose: [1.01, -0.21, 0.78, 0, 0, 1.57]
        operator_pose: [1.2, 1.1, 1.55, 0, 0, 3.14]
        final_pose: [0, -1.43, 0.75, 0, 0, 1.57]
    }
    

Parameters:
    TipOffset: [-0.299605, 0, 0.260354]
    TrayAngles: [0.314, 0.785] 
    TrayDownSteps: [0, 1, 3, 5, 7, 10, 12, 14]
    RotationSteps: [6, 7, 13, 14]
    TrayHeights: [0.25, 0.2515, 0.2]                 
    OriginToBottomDistances: [0.056]                
    TrayInitOffset: [-0.24184, 0.23975]              


Poses:
    TrayStepPose_0: [-1.57, 0.0, -1.884, 1.20, -0.07, 1.75]
    TrayStepPose_1: [-1.57, -1.57, -1.884, 1.20, -0.07, 1.75]
    TrayStepPose_2: [-1.571, 1.57, 1.256, 1.32, -0.19, 1.75]
    TrayStepPose_3: [-1.57, 1.57, -1.884, 1.20, -0.07, 1.75]
    TrayStepPose_4: [-1.57, -1.57, 1.256, 1.32, -0.19, 1.75]
    TrayStepPose_5: [-1.57, -1.57, -1.884, 1.20, -0.07, 1.75]
    TrayStepPose_6: [-1.57, 3.14, 1.571, 1.32, -0.19, 1.75]
    TrayStepPose_7: [-1.57, 3.14, -1.884, 1.20, -0.07, 1.75]
    TrayStepPose_8: [-1, -1, -1, -1, -1, -1]
    TrayStepPose_9: [-1.57, 1.57, 1.256, 1.32, -0.19, 1.75]
    TrayStepPose_10: [-1.57, 1.57, -1.884, 1.18, -0.05, 1.75]
    TrayStepPose_11: [-1.57, -1.57, 1.256, 1.32, -0.19, 1.75]
    TrayStepPose_12: [-1.57, -1.57, -1.884, 1.18, -0.05, 1.75]
    TrayStepPose_13: [-1.57, 3.14, 1.571, 1.32, -0.19, 1.75]
    TrayStepPose_14: [-1.57, 3.14, -1.884, 1.18, -0.05, 1.75]
    TrayStepPose_15: [-1, -1, -1, -1, -1, -1]
    Table: [1.252, -0.12, 0.004, 0.0, 0.0, -1.57]
    TrayInit: [1.01016, 0.11975, 0.0, 0.0, 0.0, 1.57]
    BottomPanelInit: [1.01016, 0.11975, 1.10925, 0.0, 0.0, 1.57]
    BottomPick: [1.010160, 0.119750, 1.894250, 1.57, 1.57, 1.57]
    Recharge: [0.4004, -0.29248, 1.136, 0.0, 0.0, 0.0]
    Rest: [0.8, -0.8, 1.369, 0.0, 0.0, -0.785]
    Pick: [1.01016, 0.11975, 1.55225, 1.57, 1.57, 1.57]
    PrePick: [1.01016, 0.11975, 1.95375, 1.57, 1.57, 1.57]
    PostPick: [1.010160, 0.119750, 2.353750, 1.57, 1.57, 1.57]
    End: [0.0, -1.43, 0.7865, 0, 1.57, 0]
    PreEnd: [0.0, -1.43, 2.582, 0, 1.57, 0]
    TrayFinalPose: [0.0, -1.43, 0.751, 0.0, 0.0, 1.57]
    TrayPoseOperator: [-1.570796, -1.570796, 3.141593, 1.010160, 1.000000, 1.550000]
    RestPose: [0.8, -0.8, 1.370354, 0, 0, -0.785]
    PrePickPose: [1.01, -0.21, 1.4595, 1.57, 1.57, 1.57]
    PostPickPose: [1.01, -0.21, 1.8595, 1.57, 1.57, 1.57]
    PreRechargePose: [0.952, -0.29248, 1.1156, 0, 0, 0]
    PreEndPose: [0, -1.43, 2.2, 0, 0, -0.785]

TrayStepPoses: [
    TrayStepPose_0,
    TrayStepPose_1,
    TrayStepPose_2,
    TrayStepPose_3,
    TrayStepPose_4,
    TrayStepPose_5,
    TrayStepPose_6,
    TrayStepPose_7,
    TrayStepPose_8,
    TrayStepPose_9,
    TrayStepPose_10,
    TrayStepPose_11,
    TrayStepPose_12,
    TrayStepPose_13,
    TrayStepPose_14,
    TrayStepPose_15
]

Assembly:
    AMR1 addTray: tray_AOCS to AssemblyArea ToConfirm;
    Robot1 pickTray: from PickStation ToConfirm;
    HumanOP1 operatorPositionTray: to OperatorStation unit MTQ12 ToConfirm;
    Robot1 positionTray: unit MTQ12 ToConfirm;
    Robot1 rechargeSequence: unit MTQ12 ToConfirm;
    Robot1 internalScrewingSequence: unit CMG2 ToConfirm;
    Robot1 internalScrew: unit MTQ12 hole 82 ToConfirm;
    Robot1 placeTray: to AssemblyArea ToConfirm;
