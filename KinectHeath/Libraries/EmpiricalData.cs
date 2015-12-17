using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Vision.Systems.KinectHealth.Libraries
{
    public static class EmpiricalData
    {

        /// <summary>
        /// Experimental value r ( in radians ) as described in 3.3.1 TODO (in paper r is given in degrees ?)
        /// </summary>
        public static readonly double R_UB_FORWARD = VectorMath.degreesToRadians(8);
        
        public static readonly double LL_UB_FORWARD = VectorMath.degreesToRadians(90);

        public static readonly double LU_UB_FORWARD = VectorMath.degreesToRadians(120);

        public static readonly double EU_UB_FORWARD = VectorMath.degreesToRadians(3.67);

        public static readonly double EL_UB_FORWARD = VectorMath.degreesToRadians(-11.78);

        public static readonly double R_UB_LEAN = VectorMath.degreesToRadians(4);

        public static readonly double LL_UB_LEAN = VectorMath.degreesToRadians(85);

        public static readonly double LU_UB_LEAN = VectorMath.degreesToRadians(95);

        public static readonly double EU_UB_LEAN = VectorMath.degreesToRadians(3.48);

        public static readonly double EL_UB_LEAN = VectorMath.degreesToRadians(-8.10);

        public static readonly double R_NECK = VectorMath.degreesToRadians(8);

        public static readonly double LL_NECK = VectorMath.degreesToRadians(80);

        public static readonly double LU_NECK = VectorMath.degreesToRadians(100);

        public static readonly double EU_NECK = VectorMath.degreesToRadians(30.22);

        public static readonly double EL_NECK = VectorMath.degreesToRadians(-11.06);

        public static readonly double R_LOS = VectorMath.degreesToRadians(5);

        public static readonly double LL_LOS = VectorMath.degreesToRadians(0);

        public static readonly double LU_LOS = VectorMath.degreesToRadians(60);

        // NO DEFINITION PRESENT FOR THESE VALUES

        public static readonly double EU_LOS = 0;

        public static readonly double EL_LOS = 0;


        public const double R_VIEWING_DISTANCE = 40; //cm

        public const double LL_VIEWING_DISTANCE = 80; //cm

        public const double LU_VIEWING_DISTANCE = 8; //cm

        // NO DEFINITION PRESENT FOR THESE VALUES

        public static readonly double EU_VIEWING_DISTANCE = 0;

        public static readonly double EL_VIEWING_DISTANCE = 0;

        public static double sigma(double r) { return r / (1.96 * 2); }
    }
}
