using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace Vision.Systems.KinectHealth.Libraries
{
    public static class VectorMath
    {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="v"></param>
        /// <param name="u"></param>
        /// <returns>Angles in Radians</returns>
        public static double AngleBetweenUsingDot(Vector3D v, Vector3D u)
        {
            var dot = Vector3D.DotProduct(v, u);
            var mag_v = v.Length;
            var mag_u = u.Length;

            var term = dot / (mag_u * mag_v);

            return Math.Acos(term);
        }

        public static Vector3D projectVectorOntoPlane(Vector3D v, Vector3D u1, Vector3D u2)
        {
            var n = Vector3D.CrossProduct(u1, u2);
            n.Normalize();

            var dot = Vector3D.DotProduct(v, n);

            return v - dot * n;
        }
    }
}
