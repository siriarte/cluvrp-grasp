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
                for (int i = 0; i <= 10; i = i + 2)
                {
                    double alpha1 = i * 1.0 / 10;

                    for (int j = 0; j <= 10; j = j + 2)
                    {
                        double alpha2 = j * 1.0 / 10;

                        ClusterGRASP clusterGrasp = new ClusterGRASP(instance);
                        CluVRPSolution cluVRPSolution = clusterGrasp.Grasp(100, 0.8, 0.8);
                        cluVRPSolution.verifyClusterSolution(instance);

                        CustomerGRASP customerGrasp = new CustomerGRASP(instance, cluVRPSolution);
                        customerGrasp.Grasp(100, 0.8);
                        cluVRPSolution.verifyCustomerSolution(instance);
                        
                        cluVRPSolution.printSolution();

                        string s = "Solution: " + customerGrasp.solution.totalCustomerRouteDistance + "     alpha1 = " + alpha1 + "    alpha2 = " + alpha2;
                        System.Console.WriteLine(s);
                        logger.logLine(s);

                    }
                }
            }
            System.Console.ReadLine();
        }
    }
        
}    
