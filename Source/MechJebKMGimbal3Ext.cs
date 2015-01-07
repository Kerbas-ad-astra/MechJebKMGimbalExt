using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MuMech;
using UnityEngine;
using km_Gimbal;
using System.Reflection;

namespace MuMech
{
    public class MechJebKMGimbal3Ext : ComputerModule
    {
        public MechJebKMGimbal3Ext(MechJebCore core) : base(core) { }
        bool isKMLoaded = false;
        
        private static bool KM_GimbalIsValid(PartModule p)
        {
            KM_Gimbal_3 gimbal = p as KM_Gimbal_3;
            return gimbal.initRots.Any();
        }

        private static Vector3d KM_GimbalTorqueVector(PartModule p, int i, Vector3d CoM)
        {
            KM_Gimbal_3 gimbal = p as KM_Gimbal_3;
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

        private static Quaternion KM_GimbalInitialRot(PartModule p, Transform engineTransform, int i)
        {
            KM_Gimbal_3 gimbal = p as KM_Gimbal_3;
            // Save the current local rot
            Quaternion save = gimbal.gimbalTransforms[i].localRotation;
            // Apply the default rot and let unity compute the world rot
            gimbal.gimbalTransforms[i].localRotation = gimbal.initRots[i];
            Quaternion initalRot = gimbal.gimbalTransforms[i].rotation;
            // Restore the current local rot
            gimbal.gimbalTransforms[i].localRotation = save;
            return initalRot;
        }

        public override void OnStart(PartModule.StartState state)
        {
            isKMLoaded = AssemblyLoader.loadedAssemblies.Any(a => a.assembly.GetName().Name == "km_Gimbal");
            print("MechJebKMGimbal3Ext adding MJ2 callback");

            if(isKMLoaded && !VesselState.SupportsGimbalExtension<KM_Gimbal_3>())
            {
                VesselState.AddGimbalExtension<KM_Gimbal_3>(new VesselState.GimbalExt() { isValid = KM_GimbalIsValid, initialRot = KM_GimbalInitialRot, torqueVector = KM_GimbalTorqueVector });
            }
        }
    }
}
