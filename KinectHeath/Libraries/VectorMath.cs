﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace Vision.Systems.KinectHealth.Libraries
{
    public static class VectorMath
    {

        public static double AngleBetweenUsingDot(Vector3D v, Vector3D u)
        {
            var dot = Vector3D.DotProduct(v, u);
            var mag_v = v.Length;
            var mag_u = u.Length;

            var term = dot / (mag_u * mag_v);

            return Math.Acos(term);
        }
    }
}
