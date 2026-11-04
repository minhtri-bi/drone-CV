"""Quantify axis-aligned rotor AABB conservatism without changing any profile.

This consumes the existing resolver's output. It is not an independent Gazebo
mesh-loading check and cannot certify a smaller navigation envelope.
"""
import argparse
import hashlib
from itertools import product
import json
from math import hypot, isfinite
from pathlib import Path


def audit(profile: dict) -> dict:
    parts = profile.get("parts")
    if not isinstance(parts,list) or not parts:
        raise ValueError("nonempty complete profile required")
    rows = []
    for part in parts:
        lo,hi = tuple(part["lo"]),tuple(part["hi"])
        if (len(lo)!=3 or len(hi)!=3 or not all(isfinite(v) for v in (*lo,*hi))
                or any(a>b for a,b in zip(lo,hi))):
            raise ValueError("invalid physical part bounds")
        outer = max(hypot(x,y) for x,y in product((lo[0],hi[0]),(lo[1],hi[1])))
        analytic = outer
        if part["role"] == "rotor":
            hub,axis,radius = part["joint_origin_body"],part["joint_axis_body"],part["sweep_radius_m"]
            if (len(hub)!=3 or len(axis)!=3 or not all(isfinite(v) for v in (*hub,*axis,radius))
                    or radius<=0 or abs(axis[0])>1e-12 or abs(axis[1])>1e-12
                    or abs(abs(axis[2])-1)>1e-12):
                raise ValueError("diagnostic only supports resolved body-Z rotor axes")
            # Radial support of a complete level disk: hub offset + sweep radius.
            analytic = hypot(hub[0],hub[1])+radius
            if (hub[0]-radius < lo[0]-1e-12 or hub[0]+radius > hi[0]+1e-12
                    or hub[1]-radius < lo[1]-1e-12 or hub[1]+radius > hi[1]+1e-12):
                raise ValueError("rotor sweep contradicts published outer box")
        rows.append({"part":part["id"], "role":part["role"],
                     "outer_aabb_radial_m":outer,"level_analytic_radial_m":analytic})
    return {"scope":"offline_existing_profile_level_radial_diagnostic_only",
            "part_count":len(parts),"parts":rows,
            "outer_aabb_radial_m":max(row["outer_aabb_radial_m"] for row in rows),
            "level_analytic_radial_m":max(row["level_analytic_radial_m"] for row in rows),
            "vertical_bounds_m":[min(part["lo"][2] for part in parts),max(part["hi"][2] for part in parts)],
            "navigation_clearance_m":.5,"profile_changed":False,"smaller_envelope_authorized":False,
            "independent_gazebo_mesh_loading_verified":False,
            "remaining":"R10 requires independently observed resolved geometry or calibrated multiview fixture; no new execution approved."}


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    raw=args.profile.read_bytes()
    report={**audit(json.loads(raw)),"profile_sha256":hashlib.sha256(raw).hexdigest()}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("x",encoding="utf-8") as output:
        json.dump(report,output,indent=2,allow_nan=False)
        output.write("\n")
    print(json.dumps({key:value for key,value in report.items() if key!="parts"},indent=2))


if __name__=="__main__":
    main()
