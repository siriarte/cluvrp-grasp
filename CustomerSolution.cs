using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace cluvrp_grasp
{
    class CustomerSolution
    {
        public List<int>[][] customersCircuit { set; get; }
        public double totalRouteDistance { set; get; }

        public CustomerSolution(List<int>[][] customersPath, double totalRouteDistance)
        {
            this.customersCircuit = customersPath;
            this.totalRouteDistance = totalRouteDistance;
        }
    }
}
