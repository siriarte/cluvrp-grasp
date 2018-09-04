using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace cluvrp_grasp
{
    enum CluVRPVersion { TwoPhase, Complete }
    enum CluVRPType { Normal, Weak }

    class CluVRPGrasp
    {       

        // Main GRASP handle main iteration depending if use Complete version or TwoPhase
        public static CluVRPSolution Grasp(CluVRPInstance instance, Parameters parameters)
        {
            // For best solution
            CluVRPSolution bestSolution = new CluVRPSolution();
          
            // Main cycle
            int iterator = 0;
            while(iterator < parameters.CluVRP_GRASPIterations)
            {
                // New Grasp for Cluster level instance
                ClusterGRASP clusterGrasp = new ClusterGRASP(instance, parameters);

                // Execute Grasp procedure
                CluVRPSolution cluVRPSolution = clusterGrasp.Grasp();

                // If solutions is not available avalaible
                if (cluVRPSolution.clusterRouteForVehicule == null)
                {
                    continue;
                }
          
                // Verify if cluster solution is correct
                cluVRPSolution.verifyClusterSolution(instance);

                // New Grasp for Cluster level instance
                CustomerGRASP customerGrasp = new CustomerGRASP(instance, cluVRPSolution, parameters);

                // Execute Grasp procedure
                customerGrasp.Grasp();

                // Verify if customer solution is correct
                cluVRPSolution.verifyCustomerSolution(instance);
                

                // Update best solution
                if (cluVRPSolution.totalCustomerRouteDistance < bestSolution.totalCustomerRouteDistance)
                {
                    bestSolution = cluVRPSolution;
                }
                
                // Increase iterator
                iterator++;
            }
                        
           // Return best solution
           return bestSolution;
        }

        // Main GRASP (for Weak cluster constrains) handle main iteration depending if use Complete version or TwoPhase
        public static CluVRPSolution GraspForWeakCustomer(CluVRPInstance instance, Parameters parameters)
        {
            // For best solution
            CluVRPSolution bestSolution = new CluVRPSolution();

            // Main cycle
            int iterator = 0;
            while (iterator < parameters.CluVRP_GRASPIterations)
            {
                // New Grasp for Cluster level instance
                ClusterGRASP clusterGrasp = new ClusterGRASP(instance, parameters);

                // Execute Grasp procedure
                CluVRPSolution cluVRPSolution = clusterGrasp.Grasp();

                // If solutions is not avalaible continue next iteration
                if (cluVRPSolution.clusterRouteForVehicule == null)
                {
                    continue;
                }
                
                // Verify if cluster solution is correct
                cluVRPSolution.verifyClusterSolution(instance);

                // New Grasp for Cluster level instance
                CustomerWeakGRASP customerGrasp = new CustomerWeakGRASP(instance, cluVRPSolution, parameters);

                // Execute Grasp procedure
                customerGrasp.Grasp();

                // Verify if customer solution is correct
                cluVRPSolution.verifyCustomerWeakSolution(instance);
                
                // Update best solution
                if (cluVRPSolution.totalCustomerRouteDistance < bestSolution.totalCustomerRouteDistance)
                {
                    bestSolution = cluVRPSolution;
                }

                // Increase iterator
                iterator++;
            }

            // Return best solution
            return bestSolution;
        }


    }
}
