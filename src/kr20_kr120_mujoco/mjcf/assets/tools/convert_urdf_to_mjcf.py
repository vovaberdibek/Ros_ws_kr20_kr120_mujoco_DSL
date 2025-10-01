#!/usr/bin/env python3
import sys, os
import mujoco

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input_urdf> <output_mjcf>", file=sys.stderr)
        sys.exit(1)
    in_urdf = os.path.abspath(sys.argv[1])
    out_mjcf = os.path.abspath(sys.argv[2])

    if not os.path.exists(in_urdf):
        print(f"Input URDF not found: {in_urdf}", file=sys.stderr)
        sys.exit(2)

    # NOTE: URIs like package:// will not resolve automatically.
    # Replace them with absolute paths before running if necessary.
    try:
        model = mujoco.MjModel.from_xml_path(in_urdf)
    except Exception as e:
        print("MuJoCo could not load the URDF.", file=sys.stderr)
        print("Common cause: unresolved 'package://' URIs. Replace them with absolute mesh paths.", file=sys.stderr)
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(3)

    # Save to MJCF
    mujoco.mj_saveLastXML(out_mjcf, model)
    print(f"Wrote MJCF to: {out_mjcf}")

if __name__ == '__main__':
    main()
