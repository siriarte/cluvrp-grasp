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
            logger.setVerbose(true);

            CluVRPInstance[] instancias = InstanceParser.loadGVRPSetOfInstances(instanceSetPath);

            foreach(CluVRPInstance instance in instancias)
            {
                double bestDistance = double.MaxValue;
                string fitAlgoBestSol = "";
                for (int i = 0; i <= 10; i = i + 2)
                {
                    double alphaCapacity = i * 1.0 / 10;

                    for (int j = 0; j <= 10; j = j + 2)
                    {
                        double alphaDistance = j * 1.0 / 10;

                        ClusterGRASP clusterGrasp = new ClusterGRASP(instance);
                        CluVRPSolution cluVRPSolution = clusterGrasp.Grasp(500, alphaCapacity, alphaDistance);

                        if (cluVRPSolution.clusterRouteForVehicule != null)
                        {
                            cluVRPSolution.verifyClusterSolution(instance);

                            CustomerGRASP customerGrasp = new CustomerGRASP(instance, cluVRPSolution);
                            customerGrasp.Grasp(200, alphaDistance);

                            cluVRPSolution.verifyCustomerSolution(instance);

                            string algorithm = string.Join("|", customerGrasp.solution.fitAlgorithmCounter);
                            //string s1 = instance.file_name + '\t' + customerGrasp.solution.totalCustomerRouteDistance + '\t' + alphaCapacity + '\t' + alphaDistance + '\t' + algorithm;
                            //Console.WriteLine(s1);

                            if (customerGrasp.solution.totalCustomerRouteDistance < bestDistance)
                            {
                                bestDistance = customerGrasp.solution.totalCustomerRouteDistance;
                                fitAlgoBestSol = algorithm;
                            }
                        }
                        else
                        {
                            string error = "No solution for: " + instance.file_name + '\t' + alphaCapacity + '\t' + alphaDistance;
                            Console.WriteLine(error);
                       }
                       
                    }
                }
                string s = instance.file_name + '\t' + bestDistance + '\t' + fitAlgoBestSol;
                logger.logLine(s);
            }
            System.Console.ReadLine();
        }
    }
        
}    
