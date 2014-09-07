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
        
        private bool KM_GimbalIsValid(PartModule p)
        {
            KM_Gimbal gimbal = p as KM_Gimbal;
            return gimbal.initRots.Count() > 0;
        }

        private Vector3d KM_GimbalTorqueVector(PartModule p, int i, Vector3d CoM)
        {
            KM_Gimbal gimbal = p as KM_Gimbal;
            Vector3d torque = Vector3d.zero;

            if (!gimbal.enableGimbal)
                return Vector3d.zero;

            float maxPitchGimbalRange = Math.Min(gimbal.pitchGimbalRange, gimbal.gimbalConstrain);
            float maxYawGimbalRange = Math.Min(gimbal.pitchGimbalRange, gimbal.gimbalConstrain);
            float maxRollGimbalRange = Math.Max(maxPitchGimbalRange, maxYawGimbalRange);

            Vector3d position = gimbal.gimbalTransforms[i].position - CoM;
            double distance = position.magnitude;
            double radius = Vector3.Exclude(Vector3.Project(position, p.vessel.ReferenceTransform.up), position).magnitude;

            torque.x = Math.Sin(Math.Abs(maxPitchGimbalRange) * Math.PI / 180d) * distance;
            torque.z = Math.Sin(Math.Abs(maxYawGimbalRange) * Math.PI / 180d) * distance;

            if (gimbal.enableRoll)
            {
                torque.y = Math.Sin(Math.Abs(maxRollGimbalRange) * Math.PI / 180d) * radius;
            }

            return torque;
        }

        private Quaternion KM_GimbalInitialRot(PartModule p, Transform engineTransform, int i)
        {
            KM_Gimbal gimbal = p as KM_Gimbal;
            return engineTransform.parent.rotation * gimbal.initRots[i];
        }

        public override void OnStart(PartModule.StartState state)
        {
            isKMLoaded = AssemblyLoader.loadedAssemblies.Any(a => a.assembly.GetName().Name == "km_Gimbal_2.0");
            print("MechJebKMGimbalExt adding MJ2 callback");

            if (!VesselState.gimbalExtDict.ContainsKey("KM_Gimbal"))
            {
                VesselState.GimbalExt kmGimbal = new VesselState.GimbalExt() { isValid = KM_GimbalIsValid, initialRot = KM_GimbalInitialRot, torqueVector = KM_GimbalTorqueVector };
                VesselState.gimbalExtDict.Add("KM_Gimbal", kmGimbal);
            }
        }
    }
}
