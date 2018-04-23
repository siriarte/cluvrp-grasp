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
                clusterSolution.Grasp(1000, 0.8 , 0.8);
                clusterSolution.bestSolution.verifySolution(instance);
                CustomerGRASP customerSolution = new CustomerGRASP(instance, clusterSolution.bestSolution);
                customerSolution.Grasp(1000, 0.8);
                customerSolution._costumerSolution.verifySolution(instance, customerSolution._customersDistanceMatrix);
                customerSolution._costumerSolution.printSolution();
            }

        }
    }
        
}    
