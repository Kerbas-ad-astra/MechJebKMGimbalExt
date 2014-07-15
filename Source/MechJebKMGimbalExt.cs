using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MuMech;
using UnityEngine;
using km_Lib;
using System.Reflection;

namespace MuMech
{
    public class MechJebKMGimbalExt : ComputerModule
    {
        public MechJebKMGimbalExt(MechJebCore core) : base(core) { }
        bool isKMLoaded = false;
        private void partModuleUpdate(PartModule pm)
        {
            if (isKMLoaded && pm is KM_Gimbal)
            {
                KM_Gimbal gimbal = (KM_Gimbal)pm;
                Part p = pm.part;
                Vector3 CoM = vessel.findWorldCenterOfMass();
                double maxThrust = 0.0;
                if(p.Modules.Contains("ModuleEngines"))
                    maxThrust = ((ModuleEngines)p.Modules["ModuleEngines"]).maxThrust;
                if(p.Modules.Contains("ModuleEnginesFX"))
                    maxThrust = ((ModuleEnginesFX)p.Modules["ModuleEnginesFX"]).maxThrust;
                if (gimbal.enableGimbal)
                {
                    Vector3 factor = (p.Rigidbody.worldCenterOfMass - CoM);
                    factor *= (float)maxThrust;
                    if (gimbal.enableRoll)
                    {
                        Vector2 rollFactor = new Vector2(factor.x, factor.z);
                        vesselState.torqueAvailable.y += Math.Sin(Math.Abs(Math.Max(gimbal.pitchGimbalRange, gimbal.yawGimbalRange)) * Math.PI / 180) * rollFactor.magnitude;
                    }
                    float mag = factor.magnitude;
                    vesselState.torqueAvailable.x += Math.Sin(Math.Abs(gimbal.pitchGimbalRange) * Math.PI / 180) * mag; // TODO: close enough?
                    vesselState.torqueAvailable.z += Math.Sin(Math.Abs(gimbal.yawGimbalRange) * Math.PI / 180) * mag; // TODO: close enough?
                }
            }
        }
        public override void OnStart(PartModule.StartState state)
        {
            isKMLoaded = AssemblyLoader.loadedAssemblies.Any(a => a.assembly.GetName().Name == "km_Gimbal_2.0");
            print("MechJebKMGimbalExt adding MJ2 callback");
            vesselState.vesselStatePartModuleExtensions.Add(partModuleUpdate);
        }
    }
}
