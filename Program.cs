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

            // For watch for each instance execution
            long elapsedMs = 0;

            // For best solution set
            CluVRPSolution solution = new CluVRPSolution();
            double[] bestIndividualTotalDistance = new double[solutionToCompare.Length];
            double[] bestIndividualPropDistance = new double[solutionToCompare.Length];
            double[] bestIndividualTime = new double[solutionToCompare.Length];
            Functions.Populate(bestIndividualTotalDistance, double.MaxValue);
            string[] bestIndividualParameteres = new string[solutionToCompare.Length];
            double[] bestSolPropDistances = new double[solutionToCompare.Length];
            Functions.Populate(bestSolPropDistances, double.MaxValue);
            double[] bestSolDistances = new double[solutionToCompare.Length];
            double[] bestSolTimes = new double[solutionToCompare.Length];
            List<List<LocalSearch>> bestSolClusterLSOrder = new List<List<LocalSearch>>();
            List<List<LocalSearch>> bestSolCustomerLSOrder = new List<List<LocalSearch>>();
            string bestSolParameters = "";
            double totalAvg = double.MaxValue;
            double totalAvgTime = double.MaxValue;

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
                double[] times = new double[solutionToCompare.Length];
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

                    // Set execution time
                    elapsedMs = watch.ElapsedMilliseconds;
                    double elapsedSeconds = Math.Round(elapsedMs * 1.0 / 1000, 2);

                    // Update solution results
                    distances[instanceCounter] = distance;
                    propDistances[instanceCounter] = (distance - solutionToCompare[instanceCounter]) * 100 / solutionToCompare[instanceCounter];
                    propDistances[instanceCounter] = Math.Truncate(100 * propDistances[instanceCounter]) / 100;
                    times[instanceCounter] = elapsedSeconds;

                    // Update individual instance solution
                    if (distances[instanceCounter] < bestIndividualTotalDistance[instanceCounter])
                    {
                        bestIndividualTotalDistance[instanceCounter] = distances[instanceCounter];
                        bestIndividualPropDistance[instanceCounter] = (distance - solutionToCompare[instanceCounter]) * 100 / solutionToCompare[instanceCounter];
                        bestIndividualPropDistance[instanceCounter] = Math.Truncate(100 * bestIndividualPropDistance[instanceCounter]) / 100;
                        bestIndividualParameteres[instanceCounter] = actualParameters;
                        bestIndividualTime[instanceCounter] = elapsedSeconds;
                    }

                    // Log solution
                    string outLine = instance.file_name + '\t' + distance.ToString("0") + '\t' + propDistances[instanceCounter] + "%" + '\t' + (elapsedMs * 1.0 / 1000).ToString("0.00") + "s" + '\t' + fitAlgoBestSol;
                    logger.logLine(outLine);

                    // Increase distance counter
                    instanceCounter++;
                }

                // Show total final proporcional differente and times
                totalAvg = Math.Truncate(100 * propDistances.Sum() / propDistances.Length) / 100;
                totalAvgTime = Math.Truncate(100 * times.Sum() / times.Length) / 100;
                logger.logLine("");
                logger.logLine("####################################");
                logger.logLine("# TOTAL PROP DIFFERENCE -> " + totalAvg);
                logger.logLine("# TOTAL TIME -> " + times.Sum().ToString("0.00"));
                logger.logLine("# TOTAL AVERAGE TIME -> " + totalAvgTime.ToString("0.00"));
                logger.logLine("####################################");

                // Compare best AVG 
                if (propDistances.Sum() < bestSolPropDistances.Sum())
                {
                    // Update to new best set solution
                    bestSolPropDistances = propDistances;
                    bestSolParameters = actualParameters;
                    bestSolDistances = distances;
                    bestSolTimes = times;
                    bestSolClusterLSOrder = LSClusterOrder;
                    bestSolCustomerLSOrder = LSCustomerOrder;
                    totalAvg = Math.Truncate(100 * bestSolPropDistances.Sum() / bestSolPropDistances.Length) / 100;
                    totalAvgTime = Math.Truncate(100 * bestSolTimes.Sum() / bestSolTimes.Length) / 100;
                    bestSolTimes = times;

                    // Show AVG distance
                    logger.logLine("");
                    logger.logLine("-----------------------------------------------------------------------");
                    logger.logLine("-------------------------NEW BEST DISTANCE-----------------------------");
                    logger.logLine("-----------------------------------------------------------------------");
                    //logger.logLine("NEW PROPORTIONAL BEST SET DISTANCE -> " + bestSolPropDistances.Sum());
                    logger.logLine("DISTANCES -> " + Functions.arrayToString(bestSolDistances));
                    logger.logLine("PROP DISTANCES -> " + Functions.arrayToString(bestSolPropDistances));
                    logger.logLine("TOTAL AVERAGE DIFERENCE DISTANCE -> " + totalAvg);
                    logger.logLine("TOTAL AVERAGE TIME -> " + totalAvgTime);
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
            logger.logLine("******************");
            logger.logLine("* BEST SOLUTION: *");
            logger.logLine("******************");
            logger.logLine("TOTAL TIME -> " + totalElapsedseconds + " seconds" );
            //logger.logLine("PROPORTIONAL BEST SET DISTANCE -> " + bestSolPropDistances.Sum());
            //logger.logLine("DISTANCES -> " + Functions.arrayToString(bestSolDistances));
            //logger.logLine("PROPORTIONA DIFF DISTANCES -> " + Functions.arrayToString(bestSolPropDistances));
            totalAvg = Math.Truncate(100 * bestSolPropDistances.Sum() / bestSolPropDistances.Length) / 100;
            totalAvgTime = Math.Truncate(100 * bestSolTimes.Sum() / bestSolTimes.Length) / 100;
            logger.logLine("TOTAL AVERAGE DIFERENCE DISTANCE -> " + totalAvg);
            logger.logLine("TOTAL AVERAGE TIME -> " + totalAvgTime);
            logger.logLine("");
            logger.logLine("LIST OF RESULTS");
            logger.logLine("***************");
            for (int i = 0; i < bestSolDistances.Length; i++)
            {
                string outLine = instancias[i].file_name + '\t' + bestSolDistances[i] + '\t' + bestSolPropDistances[i] + "%" +  '\t' + bestSolTimes[i];
                logger.logLine(outLine);
            }
            logger.logLine("");
            logger.logLine("PARAMETERS: ");
            logger.logLine("***********");
            logger.logLine(bestSolParameters);
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
            totalAvgTime = Math.Truncate(100 * bestIndividualTime.Sum() / bestIndividualTime.Length) / 100;
            //logger.logLine("DISTANCES -> " + Functions.arrayToString(bestIndividualTotalDistance));
            //logger.logLine("PROPORTIONA DIFF DISTANCES -> " + Functions.arrayToString(bestIndividualPropDistance));
            //logger.logLine("PROPORTIONAL TOTAL DISTANCE -> " + bestIndividualPropDistance.Sum().ToString("00.00"));
            logger.logLine("TOTAL AVERAGE DIFERENCE DISTANCE -> " + totalAvg);
            logger.logLine("TOTAL AVERAGE TIME -> " + totalAvgTime);
            logger.logLine("");
            logger.logLine("LIST OF RESULTS");
            logger.logLine("***************");
            for (int i = 0; i < bestIndividualTotalDistance.Length; i++)
            {
                string outLine = instancias[i].file_name + '\t' + bestIndividualTotalDistance[i] + '\t' + bestIndividualPropDistance[i] + "%" + '\t' + bestIndividualTime[i];
                logger.logLine(outLine);
            }
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
                    "Use: cluvrp_grasp parametersFilePath instanceSetFilePath logFilePath arrayOfSolutionsFilePath" + '\n' + '\n'
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
            string arrayOfSolutionsFilePath = args[3];

            // Check if parameters are corrects
            if(!(File.Exists(parametersFilePath) && File.Exists(instanceSetFilePath) && File.Exists(arrayOfSolutionsFilePath)))
            {
                Console.WriteLine("Some parameter is incorrect or file not exists");
                return;
            }

            // Try to parse array of solutions for compare
            double[] solutionToCompare = new double[0];
            try
            {
                string[] arrayOfSolutions = System.IO.File.ReadAllLines(arrayOfSolutionsFilePath);
                if(arrayOfSolutions.Length == 0)
                {
                    Console.WriteLine("Solution to compare file is empty");
                }
                solutionToCompare = arrayOfSolutions[0].Split(',').Select(double.Parse).ToArray();
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
                return;
            }

            // To compare in instances with results to compare
            int instanceNumber  = File.ReadLines(instanceSetFilePath).Count();
            if(instanceNumber != solutionToCompare.Length)
            {
                Console.WriteLine("Different number of instances and solutions to compare");
                return;
            }

            // Excute GRASP
            GraspProcedure(parametersFilePath, instanceSetFilePath, logFilePath, solutionToCompare, cluVRPType);

            // End
            return;
        }

    }
        
}    
