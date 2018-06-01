using System;
using System.Collections.Generic;
using System.Linq;

namespace cluvrp_grasp
{
    class Program
    {
        // Files paths
        static string INSTANCE_SET_FILE_PATH = "../../instances/prueba";
        static string PARAMETERS_FILE_PATH = "../../configurations/parameters.json";
        static string INSTANCE_SET_FILE_PATH_PERFORMANCE_TEST_1 = "../../instances/performance_test_1";

        // For complete run
        static void completeRun()
        {            
            // Get parameters to run instances
            List<Parameters> parametersList = Parameters.parseParameterFile(PARAMETERS_FILE_PATH);

            // Get logger
            Logger logger = Logger.GetInstance();

            // To logger verbose on
            logger.setVerbose(true);

            // Get instances 
            CluVRPInstance[] instancias = InstanceParser.loadGVRPSetOfInstances(INSTANCE_SET_FILE_PATH);

            // For each parameter configuration
            foreach (Parameters parameters in parametersList)
            {
                // String for parameter set
                string actualParameters = Functions.parametersToString(parameters);
                logger.logLine(actualParameters);

                // For each instance
                foreach (CluVRPInstance instance in instancias)
                {
                    // Star watch to calculate time
                    var watch = System.Diagnostics.Stopwatch.StartNew();

                    // Set max distance value
                    double bestDistance = double.MaxValue;
                    string fitAlgoBestSol = "";
               
                    // New Grasp for Cluster level instance
                    ClusterGRASP clusterGrasp = new ClusterGRASP(instance, parameters);

                    // Execute Grasp procedure
                    CluVRPSolution cluVRPSolution = clusterGrasp.Grasp();

                    // If solutions is avalaible
                    if (cluVRPSolution.clusterRouteForVehicule != null)
                    {
                        // Verify if cluster solution is correct
                        cluVRPSolution.verifyClusterSolution(instance);

                        // New Grasp for Cluster level instance
                        CustomerGRASP customerGrasp = new CustomerGRASP(instance, cluVRPSolution, parameters);

                        // Execute Grasp procedure
                        customerGrasp.Grasp();

                        // Verify if customer solution is correct
                        cluVRPSolution.verifyCustomerSolution(instance);

                        // Sets strings to show
                        string algorithm = '[' + string.Join(",", customerGrasp.solution.fitAlgorithmCounter) + ']';
                        string s1 = instance.file_name + '\t' + customerGrasp.solution.totalCustomerRouteDistance + '\t' + 
                            parameters.Cluster_AlphaCapacity + '\t' + parameters.Cluster_AlphaDistance + '\t' + algorithm;
                        //Console.WriteLine(s1);

                        // Check if solution improve 
                        if (customerGrasp.solution.totalCustomerRouteDistance < bestDistance)
                        {
                            // Set new solution 
                            bestDistance = customerGrasp.solution.totalCustomerRouteDistance;
                            fitAlgoBestSol = algorithm;
                        }
                    }
                    // If not solution for cluster was reached
                    else
                    {
                        // Show error
                        string error = "No solution for: " + instance.file_name + '\t' + parameters.Cluster_AlphaCapacity + '\t' + parameters.Cluster_AlphaDistance;
                        Console.WriteLine(error);
                    }

                    // Stop timer watchers
                    watch.Stop();

                    // Set final string result 
                    var elapsedMs = watch.ElapsedMilliseconds;
                    string outLine = instance.file_name + '\t' + bestDistance.ToString("0.00") + '\t' + (elapsedMs*1.0/1000). ToString("0.00")+ "s" + '\t' + fitAlgoBestSol;

                    // Log solution
                    logger.logLine(outLine);
                }

                // New line for new config
                logger.logLine("");
            }

            // Wait for key to close
            System.Console.ReadLine();
        }
        
        // For testing cases
        static void firstPerformanceCase(string parametersFilePath, string instanceSetFilePath, string logFilePath)
        {
            // Star watch to calculate total process time
            var totalWatch = System.Diagnostics.Stopwatch.StartNew();

            // To compare result solutions
            double[] solutionToCompare = new double[] { 253, 522, 687, 804, 914};
            int[] sizeOfSolutionToCompare = new int[] { 16, 32, 66, 151, 200};

            // For best solution
            double[] bestIndividualTotalDistance = new double[solutionToCompare.Length];
            double[] bestIndividualPropDistance = new double[solutionToCompare.Length];
            Functions.Populate(bestIndividualTotalDistance, double.MaxValue);
            string[] bestIndividualParameteres = new string[solutionToCompare.Length];
            double[] bestSolPropDistances = new double[solutionToCompare.Length];
            Functions.Populate(bestSolPropDistances, double.MaxValue);
            double[] bestSolDistances = new double[solutionToCompare.Length];
            List<List<LocalSearch>> bestSolClusterLSOrder = new List<List<LocalSearch>>();
            List<List<LocalSearch>> bestSolCustomerLSOrder = new List<List<LocalSearch>>();
            string bestSolParameters = "";

            // Get parameters to run instances
            List<Parameters> parametersList = Parameters.parseParameterFile(parametersFilePath);

            // Get logger
            Logger logger = Logger.GetInstance();

            // To logger verbose on
            logger.setVerbose(true);
            logger.setLogFilePath(logFilePath);

            // Get instances 
            CluVRPInstance[] instancias = InstanceParser.loadGVRPSetOfInstances(instanceSetFilePath);

            // Log run
            logger.logLine("*****************************************************");
            logger.logLine("* STARTING TEST:");
            logger.logLine("* CONFIG -> " + parametersFilePath);
            logger.logLine("* SET INSTANCE -> " + instanceSetFilePath);
            logger.logLine("* LOG FILE -> " + logger.getLogFilePath());
            logger.logLine("******************************************************");
            logger.logLine("");

            // For each parameter configuration
            foreach (Parameters parameters in parametersList)
            {
                // String for parameter set and print
                logger.logLine("=============================================");
                logger.logLine("=       EXECUTING NEW TEST CASE             =");
                logger.logLine("=============================================");
                string actualParameters = Functions.parametersToString(parameters);
                logger.logLine(actualParameters);

                // For this configuration run
                double[] propDistances = new double[solutionToCompare.Length];
                double[] distances = new double[solutionToCompare.Length];
                List<List<LocalSearch>> LSClusterOrder = new List<List<LocalSearch>>();
                List<List<LocalSearch>> LSCustomerOrder = new List<List<LocalSearch>>();

                // For each instance
                int instanceCounter = 0;
                foreach (CluVRPInstance instance in instancias)
                {
                    // Star watch to calculate time
                    var watch = System.Diagnostics.Stopwatch.StartNew();

                    // Set max distance value
                    double distance = 0;
                    string fitAlgoBestSol = "";

                    // New Grasp for Cluster level instance
                    ClusterGRASP clusterGrasp = new ClusterGRASP(instance, parameters);

                    // Execute Grasp procedure
                    CluVRPSolution cluVRPSolution = clusterGrasp.Grasp();

                    // If solutions is avalaible
                    if (cluVRPSolution.clusterRouteForVehicule != null)
                    {
                        // Verify if cluster solution is correct
                        cluVRPSolution.verifyClusterSolution(instance);

                        // New Grasp for Cluster level instance
                        CustomerGRASP customerGrasp = new CustomerGRASP(instance, cluVRPSolution, parameters);

                        // Execute Grasp procedure
                        customerGrasp.Grasp();

                        // Verify if customer solution is correct
                        cluVRPSolution.verifyCustomerSolution(instance);

                        // Sets strings to show - FOR DEBUG
                        string algorithm = '[' + string.Join(",", customerGrasp.solution.fitAlgorithmCounter) + ']';
                        string s1 = instance.file_name + '\t' + customerGrasp.solution.totalCustomerRouteDistance + '\t' +
                            parameters.Cluster_AlphaCapacity + '\t' + parameters.Cluster_AlphaDistance + '\t' + algorithm;
                        //Console.WriteLine(s1);

                        // For this instance solution
                        distance = customerGrasp.solution.totalCustomerRouteDistance;
                        fitAlgoBestSol = algorithm;
                        LSClusterOrder.Add(customerGrasp.solution.bestClusterLSOrder);
                        LSCustomerOrder.Add(customerGrasp.solution.bestCustomerLSOrder);

                    }
                    // If not solution for cluster was reached
                    else
                    {
                        // Show error
                        string error = "No solution for: " + instance.file_name + '\t' + parameters.Cluster_AlphaCapacity + '\t' + parameters.Cluster_AlphaDistance;
                        Console.WriteLine(error);
                    }

                    // Stop timer watchers
                    watch.Stop();

                    // Set final string result 
                    var elapsedMs = watch.ElapsedMilliseconds;
                    string outLine = instance.file_name + '\t' + distance.ToString("0.00") + '\t' + (elapsedMs * 1.0 / 1000).ToString("0.00") + "s" + '\t' + fitAlgoBestSol;

                    // Update solution results
                    distances[instanceCounter] = Math.Truncate(100 * distance) / 100;
                    propDistances[instanceCounter] = (distance - solutionToCompare[instanceCounter]) * 100 / solutionToCompare[instanceCounter];
                    propDistances[instanceCounter] = Math.Truncate(100 * propDistances[instanceCounter]) / 100;

                    // Update individual instance solution
                    if (distances[instanceCounter] < bestIndividualTotalDistance[instanceCounter])
                    {
                        bestIndividualTotalDistance[instanceCounter] = distances[instanceCounter];
                        bestIndividualPropDistance[instanceCounter] = (distance - solutionToCompare[instanceCounter]) * 100 / solutionToCompare[instanceCounter];
                        bestIndividualPropDistance[instanceCounter] = Math.Truncate(100 * bestIndividualPropDistance[instanceCounter]) / 100;
                        bestIndividualParameteres[instanceCounter] = actualParameters;
                    }

                    // Increase distance counter
                    instanceCounter++;   

                    // Log solution
                    logger.logLine(outLine);
                }

                // Compare best AVG 
                if(propDistances.Sum() < bestSolPropDistances.Sum())
                {
                    // Update to new best set solution
                    bestSolPropDistances = propDistances;
                    bestSolParameters = actualParameters;
                    bestSolDistances = distances;
                    bestSolClusterLSOrder = LSClusterOrder;
                    bestSolCustomerLSOrder = LSCustomerOrder;

                    // Show AVG distance
                    logger.logLine("");
                    logger.logLine("-----------------------------------------------------------------------");
                    logger.logLine("-------------------------NEW BEST DISTANCE-----------------------------");
                    logger.logLine("-----------------------------------------------------------------------");
                    logger.logLine("NEW PROPORTIONAL BEST SET DISTANCE -> " + bestSolPropDistances.Sum());
                    logger.logLine("DISTANCES -> " + Functions.arrayToString(bestSolDistances));
                    logger.logLine("PROP DISTANCES -> " + Functions.arrayToString(bestSolPropDistances));                   
                    logger.logLine("-----------------------------------------------------------------------");
                    logger.logLine("-----------------------------------------------------------------------");
                    logger.logLine("-----------------------------------------------------------------------");
                }

                // New line for new config
                logger.logLine("");
            }

            // Stop timer watchers
            totalWatch.Stop();
            var totalElapsedseconds = totalWatch.ElapsedMilliseconds / 1000;

            // Log best solution
            logger.logLine("");
            logger.logLine("--------------");
            logger.logLine("BEST SOLUTION:");
            logger.logLine("--------------");
            logger.logLine("TOTAL TIME -> " + totalElapsedseconds + " seconds" );
            logger.logLine("PROPORTIONAL BEST SET DISTANCE -> " + bestSolPropDistances.Sum());
            logger.logLine("DISTANCES -> " + Functions.arrayToString(bestSolDistances));
            logger.logLine("PROPORTIONA DIFF DISTANCES -> " + Functions.arrayToString(bestSolPropDistances));
            double totalAvg = Math.Truncate(100 * bestSolPropDistances.Sum() / bestSolPropDistances.Length) / 100;
            logger.logLine("TOTAL DISTANCE AVERAGE -> " + totalAvg);
            logger.logLine("");
            logger.logLine("PARAMETERS: ");
            logger.logLine("***********");
            logger.logLine(bestSolParameters);
            logger.logLine("");
            logger.logLine("CLUSTER LS ORDER:");
            logger.logLine("*****************");
            for (int i = 0; i < bestSolClusterLSOrder.Count; i++ )
                logger.logLine((Functions.arrayToString(bestSolClusterLSOrder[i])));
            logger.logLine("");
            logger.logLine("CUSTOMER LS ORDER:");
            logger.logLine("******************");
            for (int i = 0; i < bestSolCustomerLSOrder.Count; i++)
                logger.logLine((Functions.arrayToString(bestSolCustomerLSOrder[i])));
            logger.logLine("");
            logger.logLine("");
            logger.logLine("============================");
            logger.logLine("= BEST INDIVIDUAL RESULTS =:");
            logger.logLine("============================");
            totalAvg = Math.Truncate(100 * bestIndividualPropDistance.Sum() / bestIndividualPropDistance.Length) / 100;
            logger.logLine("DISTANCES -> " + Functions.arrayToString(bestIndividualTotalDistance));
            logger.logLine("PROPORTIONA DIFF DISTANCES -> " + Functions.arrayToString(bestIndividualPropDistance));
            logger.logLine("PROPORTIONAL TOTAL DISTANCE -> " + bestIndividualPropDistance.Sum().ToString("00.00"));
            logger.logLine("TOTAL DISTANCE AVERAGE -> " + totalAvg);
            logger.logLine("");
            logger.logLine("");
            for (int i = 0; i < bestSolClusterLSOrder.Count; i++)
            {
                logger.logLine("-----------------------------------------------------------------");
                logger.logLine("CONFIGURATION FOR INSTANCE " + i);
                logger.logLine("*****************************");
                logger.logLine(bestIndividualParameteres[i].ToString());
                logger.logLine("-----------------------------------------------------------------");
                logger.logLine("");
            }
                
            // Wait for key to close
            System.Console.ReadLine();
        }

        static void Main(string[] args)
        {
            firstPerformanceCase(args[0], args[1], args[2]);
           
        }
}
        
}    
