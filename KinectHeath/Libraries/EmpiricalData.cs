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
        public const double r_ub = (8 * Math.PI) / 180;

        public const double sigma_ref_ub = EmpiricalData.r_ub / (1.96 * 2);
    }
}
