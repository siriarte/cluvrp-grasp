using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
namespace cluvrp_grasp
{
    class Program
    {
        // Main Function
        static void GraspProcedure(string parametersFilePath, string instanceSetFilePath, string logFilePath, double[] solutionToCompare, CluVRPType cluvrpType = CluVRPType.Normal)
        {
            // Star watch to calculate total process time
            var totalWatch = System.Diagnostics.Stopwatch.StartNew();

            // For best solution set
            CluVRPSolution solution = new CluVRPSolution();
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
            double totalAvg = double.MaxValue;
           
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
            logger.logLine("*****************************************************" + '\n' +
                            "* STARTING TEST:" + '\n' + 
                            "* CONFIG -> " + parametersFilePath + '\n' + 
                            "* SET INSTANCE -> " + instanceSetFilePath + '\n' + 
                            "* LOG FILE -> " + logger.getLogFilePath() + '\n' + 
                            "******************************************************" + '\n');
         

            // For each parameter configuration
            foreach (Parameters parameters in parametersList)
            {
                // String for parameter set and print
                logger.logLine("=============================================" + '\n' +
                               "=       EXECUTING NEW TEST CASE             =" + '\n' +
                               "=============================================" + '\n');
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

                    // Calculate solution for normal CluVRP or weak cluster constrains
                    if (cluvrpType == CluVRPType.Normal)
                    {                        
                        solution = CluVRPGrasp.Grasp(instance, parameters);
                    }else
                    {
                        solution = CluVRPGrasp.GraspForWeakCustomer(instance, parameters);
                    }
                    // If not possible solution
                    if (solution.clusterRouteForVehicule == null)
                    {
                        logger.logLine(instance.file_name + '\t' + "No solution for this instance");
                        propDistances[instanceCounter] = double.MaxValue;
                        instanceCounter++;
                        continue;
                    }

                    // Sets strings to show - FOR DEBUG
                    string algorithm = '[' + string.Join(",", solution.fitAlgorithmCounter) + ']';
                    string s1 = instance.file_name + '\t' + solution.totalCustomerRouteDistance + '\t' +
                        parameters.Cluster_AlphaCapacity + '\t' + parameters.Cluster_AlphaDistance + '\t' + algorithm;
                    //Console.WriteLine(s1);

                    // For this instance solution
                    distance = solution.totalCustomerRouteDistance;
                    distance = Math.Truncate(distance);
                    fitAlgoBestSol = algorithm;
                    LSClusterOrder.Add(solution.bestClusterLSOrder);
                    LSCustomerOrder.Add(solution.bestCustomerLSOrder);
    
                    // Stop timer watchers
                    watch.Stop();

                    // Set final string result 
                    var elapsedMs = watch.ElapsedMilliseconds;
                    string outLine = instance.file_name + '\t' + distance.ToString("0") + '\t' + (elapsedMs * 1.0 / 1000).ToString("0.00") + "s" + '\t' + fitAlgoBestSol;

                    // Update solution results
                    distances[instanceCounter] = distance;
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
                    totalAvg = Math.Truncate(100 * bestSolPropDistances.Sum() / bestSolPropDistances.Length) / 100;

                    // Show AVG distance
                    logger.logLine("");
                    logger.logLine("-----------------------------------------------------------------------");
                    logger.logLine("-------------------------NEW BEST DISTANCE-----------------------------");
                    logger.logLine("-----------------------------------------------------------------------");
                    logger.logLine("NEW PROPORTIONAL BEST SET DISTANCE -> " + bestSolPropDistances.Sum());
                    logger.logLine("DISTANCES -> " + Functions.arrayToString(bestSolDistances));
                    logger.logLine("PROP DISTANCES -> " + Functions.arrayToString(bestSolPropDistances));
                    logger.logLine("TOTAL DISTANCE AVERAGE -> " + totalAvg);
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
            totalAvg = Math.Truncate(100 * bestSolPropDistances.Sum() / bestSolPropDistances.Length) / 100;
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
            for (int i = 0; i < bestSolClusterLSOrder.Count; i++)
            {
                logger.logLine("-----------------------------------------------------------------");
                logger.logLine("");
                logger.logLine("CONFIGURATION FOR INSTANCE " + i);
                logger.logLine("*****************************");
                logger.logLine(bestIndividualParameteres[i].ToString());
            }

            // End
            return;
        }

        static void Main(string[] args)
        {

            // Default type of clvrp
            CluVRPType cluVRPType = CluVRPType.Normal;

            // Check numbers of parameters
            if (args.Length < 4)
            {
                Console.WriteLine(
                    "Parameter is missing: " + '\n' +
                    "Use: cluvrp_grasp parametersFilePath instanceSetFilePath logFilePath arrayOfSolutions" + '\n' + '\n'
                );
                return;
            }
            // If type of cluvrp is defined set it
            else if(args.Length == 5){
                cluVRPType = (CluVRPType)int.Parse(args[4]);
            }

            // Set variables
            string parametersFilePath = args[0];
            string instanceSetFilePath = args[1];
            string logFilePath = args[2];
            string arrayOfSolutions = args[3];

            // Check if parameters are corrects
            if(!(File.Exists(parametersFilePath) && File.Exists(instanceSetFilePath) && arrayOfSolutions.Length != 0))
            {
                Console.WriteLine("Some parameter is incorrect or file not exists");
                return;
            }

            // Try to parse array of solutions for compare
            double[] solutionToCompare = new double[0];
            try
            {
                solutionToCompare = arrayOfSolutions.Split(',').Select(double.Parse).ToArray();
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
                return;
            }

            // Excute GRASP
            GraspProcedure(parametersFilePath, instanceSetFilePath, logFilePath, solutionToCompare, cluVRPType);

            // End
            return;
        }

    }
        
}    
