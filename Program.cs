using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace cluvrp_grasp
{
    class Program
    {

        static string instanceSetPath = "../../instances/prueba";
       
        static void Main(string[] args)
        {
            Logger logger = Logger.GetInstance();
            CluVRPInstance[] instancias = InstanceParser.loadGVRPSetOfInstances(instanceSetPath);

            foreach(CluVRPInstance instance in instancias)
            {
                ClusterGRASP clusterSolution = new ClusterGRASP(instance);
                clusterSolution.Grasp(1000, 0.2 , 0.8);
                logger.logLine(clusterSolution.bestSolution.totalRouteDistance.ToString());
                CustomerGRASP customerSolution = new CustomerGRASP(instance, clusterSolution.bestSolution);
                customerSolution.Grasp(1000, 0.1);
                customerSolution._costumerSolution.printSolution();
                customerSolution._costumerSolution.verifySolution(instance);
            }

        }
    }
        
}    
