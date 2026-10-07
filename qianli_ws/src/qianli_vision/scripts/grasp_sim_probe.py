#!/usr/bin/env python3
"""Offline probe: compare TCP axes/IK and stage tracking in simulation."""
import argparse
import json
import numpy as np
import mujoco
import sim_grasp as S
import sim_mesh_gripper as MG
import sim_ik_dls as IK


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--grid-mm',type=float,nargs=2,default=[111.,20.])
    ap.add_argument('--trials',action='store_true')
    a = ap.parse_args()
    S.OBJ_GRID = tuple(np.array(a.grid_mm)/1000.)
    m = MG.build(.020)
    qadr = S.attach_handles(m)[0]
    if a.trials:
        from sim_grasp_validate import Trial
        for offset in ([8,-4,2],[4,-4,2],[0,0,2],[8,0,2],[0,-4,2]):
            def stage(label,trial):
                print(json.dumps(dict(stage=label,offset=offset,object_position=trial.d.xpos[trial.obj].tolist(),
                                      tcp=trial.tcp().tolist(),tool_angle=IK.tool_angle_deg(m,trial.d),
                                      contact=trial.contacts())),flush=True)
            print(json.dumps(Trial(m,on_stage=stage).execute(offset,retries=0)),flush=True)
        return
    ch,names = S.make_chain()
    for dz,scale in ((.008,1.),(.033,1.),(.068,.80),(.108,.72)):
        target = S.obj_world_pos()+[.008,-.004,dz]
        target[:2] *= scale
        data = mujoco.MjData(m)
        q,err = IK.ik_dls(m,data,qadr,target)
        chain_q = np.zeros(len(ch.links))
        for name,value in zip(S.ARM_JOINTS,q):
            chain_q[names.index(name)] = value
        fk = ch.forward_kinematics(chain_q)
        print(json.dumps(dict(grid_mm=a.grid_mm,target=target.tolist(),err_mm=err*1000,
                              tool_angle=IK.tool_angle_deg(m,data),
                              mujoco_axis=(IK.tcp_of(m,data)[1] @ IK._tool_z_local).tolist(),
                              urdf_axis=fk[:3,2].tolist(),q=q.tolist())),flush=True)


if __name__ == '__main__':
    main()
