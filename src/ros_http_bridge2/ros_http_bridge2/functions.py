import requests
import json

BASE_URL = "http://localhost:8000"  # adjust if your server runs elsewhere

def LoadTray(tray, agent):
    # Exactly as before—just a printout of tray contents
    print(f"Loading tray '{tray['name']}' with agent {agent}.")
    print(f"Tray units: {tray['units']}")
    print(f"Tray screws: {tray['screws']}")

def BringTray(from_loc, to_loc, agent):
    """
    from_loc and to_loc are typically strings (tray names or indices).
    We’ll send them exactly as before, plus agent if needed.
    """
    payload = {
        "tray_name": str(from_loc),
        "location": str(to_loc),
        "agent": agent
    }
    print(f"📡 [HTTP] Bringing tray from {from_loc} to {to_loc} with agent {agent}...")
    try:
        res = requests.post(f"{BASE_URL}/bring_tray", json=payload, timeout=30.0)
        res_json = res.json()
        print(f"BringTray response: {res_json}")
    except requests.RequestException as e:
        print(f"❌ BringTray HTTP call failed: {e}")

def SendInitParameters(params):
    """
    Exactly replicate the same flattening / extraction logic you had in ROS,
    but pack it into a JSON and send to /init_parameters.
    """
    print("📤 Preparing InitParameters payload...")

    # 1) Flatten tray_step_poses (list of 16 poses, each 6 floats → 96 floats)
    flattened_step_poses = [v for pose in params.get('tray_step_poses', []) for v in pose]

    # 2) Pull out main_poses dict
    main_poses = params.get('main_poses', {})
    table_pose       = main_poses.get('Table',       [0.0]*6)
    tray_init_pose   = main_poses.get('TrayInit',    [0.0]*6)
    bottom_init_pose = main_poses.get('BottomPanelInit', [0.0]*6)
    bottom_pick_pose = main_poses.get('BottomPick',  [0.0]*6)
    recharge_pose    = main_poses.get('Recharge',    [0.0]*6)
    rest_pose        = main_poses.get('Rest',        [0.0]*6)
    pick_pose        = main_poses.get('Pick',        [0.0]*6)
    pre_pick_pose    = main_poses.get('PrePick',     [0.0]*6)
    post_pick_pose   = main_poses.get('PostPick',    [0.0]*6)
    end_pose         = main_poses.get('End',         [0.0]*6)
    pre_end_pose     = main_poses.get('PreEnd',      [0.0]*6)

    # 3) Other top‐level arrays
    tray_final_pose   = params.get('tray_final_pose',   [0.0]*6)
    tray_pose_operator= params.get('tray_pose_operator',[0.0]*6)
    tip_offset        = params.get('tip_offset', [0.0,0.0,0.0])
    tray_down_steps   = params.get('tray_down_steps',   [])
    operator_steps    = params.get('operator_steps',    [])
    rotation_steps    = params.get('rotation_steps',    [])
    new_tray_steps    = params.get('new_tray_steps',    [])

    # 4) Tray angles
    tray_angles = params.get('tray_angles', [0.0,0.0])
    angle1, angle2 = tray_angles[0], tray_angles[1]

    # 5) Heights and offsets
    tray_heights     = params.get('tray_heights', [])
    origin_to_bottom = params.get('origin_to_bottom', [])
    tray_init_offset = params.get('tray_init_offset', [])

    # 6) new_dummy (string)
    new_dummy = params.get('new_dummy', "")

    # 7) Build unit‐name & pose‐index lists from params["trays"]
    units = []
    pose_indices = []
    for tray in params.get("trays", {}).values():
        for unit in tray.get("units", []):
            units.append(unit["name"])
            pose_indices.append(unit["pose_index"])

    # 8) Build screws arrays
    screw_quantities = []
    screw_types = []
    for tray in params.get("trays", {}).values():
        for screw in tray.get("screws", []):
            screw_quantities.append(screw["quantity"])
            screw_types.append(screw["type"])

    # 9) Assemble full JSON payload exactly as your ROS request would have:
    payload = {
        "tray_step_poses":       flattened_step_poses,
        "table_pose":            table_pose,
        "tray_init_pose":        tray_init_pose,
        "bottom_init_pose":      bottom_init_pose,
        "bottom_pick_pose":      bottom_pick_pose,
        "recharge_pose":         recharge_pose,
        "rest_pose":             rest_pose,
        "pick_pose":             pick_pose,
        "pre_pick_pose":         pre_pick_pose,
        "post_pick_pose":        post_pick_pose,
        "end_pose":              end_pose,
        "pre_end_pose":          pre_end_pose,
        "tray_final_pose":       tray_final_pose,
        "tray_pose_operator":    tray_pose_operator,
        "tip_offset":            tip_offset,
        "tray_down_steps":       tray_down_steps,
        "operator_steps":        operator_steps,
        "rotation_steps":        rotation_steps,
        "new_tray_steps":        new_tray_steps,
        "tray_angle1":           angle1,
        "tray_angle2":           angle2,
        "tray_heights":          tray_heights,
        "origin_to_bottom":      origin_to_bottom,
        "tray_init_offset":      tray_init_offset,
        "new_dummy":             new_dummy,
        "tray_unit_names":       units,
        "tray_unit_pose_indices":pose_indices,
        "screw_quantities":      screw_quantities,
        "screw_types":           screw_types
    }

    # (Optional) Debug print of what you’re sending
    print("🛠 InitParameters payload:")
    print(json.dumps(payload, indent=2))

    try:
        res = requests.post(f"{BASE_URL}/init_parameters", json={"params": payload}, timeout=30.0)
        res_json = res.json()
        print(f"✅ InitParameters response: {res_json}")
        return res_json.get("success", False)
    except requests.RequestException as e:
        print(f"❌ InitParameters HTTP call failed: {e}")
        return False

def AddTray(index):
    try:
        idx = int(index)
    except ValueError:
        print(f"❌ Invalid index for AddTray: {index} (must be integer)")
        return

    payload = {"index": idx}
    print(f"📡 [HTTP] Adding tray at index {idx}...")
    try:
        res = requests.post(f"{BASE_URL}/add_tray", json=payload)
        print(f"AddTray response: {res.json()}")
    except requests.RequestException as e:
        print(f"❌ AddTray HTTP call failed: {e}")

def AddTray2(tray_name, object_name):
    payload = {
        "tray_name":   tray_name,
        "object_name": object_name
    }
    print(f"📡 [HTTP] AddTray2: tray={tray_name}, object={object_name}...")
    try:
        res = requests.post(f"{BASE_URL}/add_tray2", json=payload, timeout=30.0)
        print(f"AddTray2 response: {res.json()}")
    except requests.RequestException as e:
        print(f"❌ AddTray2 HTTP call failed: {e}")

def PickTray(index):
    try:
        idx = int(index)
    except ValueError:
        print(f"❌ Invalid index for PickTray: {index}")
        return

    payload = {"index": idx}
    print(f"📡 [HTTP] Picking tray index {idx}...")
    try:
        res = requests.post(f"{BASE_URL}/pick_tray", json=payload, timeout=30.0)
        print(f"PickTray response: {res.json()}")
    except requests.RequestException as e:
        print(f"❌ PickTray HTTP call failed: {e}")

def PositionTray(index):
    try:
        idx = int(index)
    except ValueError:
        print(f"❌ Invalid index for PositionTray: {index}")
        return

    payload = {"index": idx}
    print(f"📡 [HTTP] Positioning tray index {idx}...")
    try:
        res = requests.post(f"{BASE_URL}/position_tray", json=payload, timeout=30.0)
        print(f"PositionTray response: {res.json()}")
    except requests.RequestException as e:
        print(f"❌ PositionTray HTTP call failed: {e}")

def OperatorPositionTray(index):
    try:
        idx = int(index)
    except ValueError:
        print(f"❌ Invalid index for OperatorPositionTray: {index}")
        return

    payload = {"index": idx}
    print(f"📡 [HTTP] Operator positioning tray index {idx}...")
    try:
        res = requests.post(f"{BASE_URL}/operator_position_tray", json=payload, timeout=30.0)
        print(f"OperatorPositionTray response: {res.json()}")
    except requests.RequestException as e:
        print(f"❌ OperatorPositionTray HTTP call failed: {e}")

def RechargeSequence(index):
    try:
        idx = int(index)
    except ValueError:
        print(f"❌ Invalid index for RechargeSequence: {index}")
        return

    payload = {"index": idx}
    print(f"📡 [HTTP] RechargeSequence for index {idx}...")
    try:
        res = requests.post(f"{BASE_URL}/recharge_sequence", json=payload, timeout=30.0)
        print(f"RechargeSequence response: {res.json()}")
    except requests.RequestException as e:
        print(f"❌ RechargeSequence HTTP call failed: {e}")

def InternalScrewingSequence(index):
    try:
        idx = int(index)
    except ValueError:
        print(f"❌ Invalid index for InternalScrewingSequence: {index}")
        return

    payload = {"index": idx}
    print(f"📡 [HTTP] InternalScrewingSequence for index {idx}...")
    try:
        res = requests.post(f"{BASE_URL}/internal_screwing_sequence", json=payload, timeout=30.0)
        print(f"InternalScrewingSequence response: {res.json()}")
    except requests.RequestException as e:
        print(f"❌ InternalScrewingSequence HTTP call failed: {e}")

def ExternalScrewingSequence(index):
    try:
        idx = int(index)
    except ValueError:
        print(f"❌ Invalid index for ExternalScrewingSequence: {index}")
        return

    payload = {"index": idx}
    print(f"📡 [HTTP] ExternalScrewingSequence for index {idx}...")
    try:
        res = requests.post(f"{BASE_URL}/external_screwing_sequence", json=payload, timeout=30.0)
        print(f"ExternalScrewingSequence response: {res.json()}")
    except requests.RequestException as e:
        print(f"❌ ExternalScrewingSequence HTTP call failed: {e}")

def PlaceTray(index):
    try:
        idx = int(index)
    except ValueError:
        print(f"❌ Invalid index for PlaceTray: {index}")
        return

    payload = {"index": idx}
    print(f"📡 [HTTP] PlaceTray for index {idx}...")
    try:
        res = requests.post(f"{BASE_URL}/place_tray", json=payload, timeout=30.0)
        print(f"PlaceTray response: {res.json()}")
    except requests.RequestException as e:
        print(f"❌ PlaceTray HTTP call failed: {e}")

def InitialAssemble(tray, agent):
    payload = {
        "tray": tray,
        "agent": agent
    }
    print(f"📡 [HTTP] InitialAssemble tray={tray}, agent={agent}...")
    try:
        res = requests.post(f"{BASE_URL}/initial_assemble", json=payload, timeout=30.0)
        print(f"InitialAssemble response: {res.json()}")
    except requests.RequestException as e:
        print(f"❌ InitialAssemble HTTP call failed: {e}")

def FinishAssemble(tray, agent1, agent2):
    payload = {
        "tray": tray,
        "agent1": agent1,
        "agent2": agent2
    }
    print(f"📡 [HTTP] FinishAssemble tray={tray}, agents={agent1, agent2}...")
    try:
        res = requests.post(f"{BASE_URL}/finish_assemble", json=payload, timeout=30.0)
        print(f"FinishAssemble response: {res.json()}")
    except requests.RequestException as e:
        print(f"❌ FinishAssemble HTTP call failed: {e}")

