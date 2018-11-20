using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Globalization;

namespace cluvrp_grasp
{
    class Program
    {
        // Main Function
        static void GraspProcedureCycleParameters(string parametersFilePath, string instanceSetFilePath, string logFilePath, double[] solutionToCompare)
        {
            // Star watch to calculate total process time
            var totalWatch = System.Diagnostics.Stopwatch.StartNew();

            // For watch for each instance execution
            long elapsedMs = 0;

            // For best solution set
            Dictionary<CluVRPInstance, CluVRPSolution> bestSolutionForInstance = new Dictionary<CluVRPInstance, CluVRPSolution>();
            CluVRPSolution solution;
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
                    
                    // Create a new solution
                    solution = new CluVRPSolution(instance);

                    // Calculate solution for normal CluVRP or weak cluster constrains
                    solution = CluVRPGrasp.Grasp(instance, parameters);
       
                    // If not possible solution
                    if (solution.clusterRouteForVehicule == null)
                    {
                        logger.logLine(instance.file_name + '\t' + "No solution for this instance");
                        propDistances[instanceCounter] = 10;
                        instanceCounter++;
                        continue;
                    }

                    // Sets strings to show - FOR DEBUG
                    string s1 = instance.file_name + '\t' + solution.totalCustomerRouteDistance + '\t' +
                        parameters.Cluster_AlphaCapacity + '\t' + parameters.Cluster_AlphaDistance + '\t';
                    //Console.WriteLine(s1);

                    // For this instance solution
                    distance = solution.totalCustomerRouteDistance;
                    //solution.customersPaths[0][5][0] = 20;
                    //solution.customersPaths[0][5][1] = 19;
                    //distance = Functions.calculateTotalTravelDistance(solution.customersPaths, instance.customersDistanceMatrix);
                    if (instance.instance_type == Instance.GoldenIzquierdo)
                    {
                        distance = Math.Truncate(100 * distance) / 100;
                    }
                    else
                    {
                        distance = Math.Truncate(distance);
                    }
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
                        bestSolutionForInstance[instance] = solution;
                    }

                    // Log solution
                    string s_distance; 
                    if (instance.instance_type == Instance.GoldenIzquierdo)
                    {
                        s_distance = distance.ToString("0.00");
                    }else
                    {
                        s_distance = distance.ToString("0");
                    }
                    string outLine = instance.file_name + '\t' + s_distance + '\t' + propDistances[instanceCounter] + "%" + '\t' + (elapsedMs * 1.0 / 1000).ToString("0.00") + "s" + '\t' + fitAlgoBestSol;
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
            
            foreach (CluVRPInstance instance in bestSolutionForInstance.Keys)
            {
                CluVRPSolution.solutionDrawPythonCode(instance, bestSolutionForInstance[instance]);
            }

            // Pause
            //System.Console.ReadKey();
            
            // End
            return;
        }
        

        // Main Function
        static void GraspProcedure(string parametersFilePath, string instanceSetFilePath, string logFilePath, double[] solutionToCompare, CluVRPVersion cluVRPVersion)
        {

            // Star watch to calculate total process time
            var totalWatch = System.Diagnostics.Stopwatch.StartNew();

            // For watch for each instance execution
            long elapsedMs = 0;

            // Number of instances
            int instancesNumber = solutionToCompare.Length;

            // For best results 
            Dictionary<CluVRPInstance, CluVRPSolution> bestSolutionForInstance = new Dictionary<CluVRPInstance, CluVRPSolution>();
            CluVRPSolution solution;
            double[] bestSolutionTotalDistance = new double[instancesNumber];
            double[] bestSolutionPropDistance = new double[instancesNumber];
            double[] bestSolutionTime = new double[instancesNumber];
            Functions.Populate(bestSolutionTotalDistance, double.MaxValue);
            string[] bestSolutionParameters = new string[instancesNumber];
            
            // For Average results
            double[] totalDistance = new double[instancesNumber];
            double[] totalTime = new double[instancesNumber];
            double[] solutionAvgPropDistance = new double[instancesNumber];
            int[] instanceOKsolutions = new int[instancesNumber];

            // Get parameters to run instances
            List<Parameters> parametersList = Parameters.parseParameterFile(parametersFilePath);

            // Get logger
            Logger logger = Logger.GetInstance();

            // To logger verbose on/off
            logger.setVerbose(false);
            logger.setLogFilePath(logFilePath);

            // Get instances 
            CluVRPInstance[] instancias = InstanceParser.loadGVRPSetOfInstances(instanceSetFilePath);

            // Log run
            logger.logLine("*****************************************************" + '\n' +
                            "* STARTING TEST:" + '\n' +
                            "* DATE -> " + DateTime.Now.ToString() + '\n' +
                            "* CONFIG -> " + parametersFilePath + '\n' +
                            "* SET INSTANCE -> " + instanceSetFilePath + '\n' +
                            "* LOG FILE -> " + logger.getLogFilePath() + '\n' +
                            "*****************************************************" + '\n');

            // For each instance
            int instanceIterator = 0;
            foreach (CluVRPInstance instance in instancias)
            {
                // For this instance
                double distance;

                // Create new solution
                solution = new CluVRPSolution(instance);

                // For each parameter configuration
                foreach (Parameters parameters in parametersList)
                {
                    // Star watch to calculate time
                    var watch = System.Diagnostics.Stopwatch.StartNew();

                    // If CluVRP_Version in parameters is None(3) it has to be taken from command line
                    if(parameters.CluVRP_Version == CluVRPVersion.None)
                    {
                        parameters.CluVRP_Version = cluVRPVersion;
                    } 

                    // Actual Parameter
                    string actualParameters = Functions.parametersToString(parameters);
                    //logger.logLine(actualParameters);

                    // Calculate solution for normal CluVRP or weak cluster constrains
                    solution = CluVRPGrasp.Grasp(instance, parameters);

                    // If not possible solution
                    if (solution.clusterRouteForVehicule == null)
                    {
                        continue;
                    }

                    // Increase instance OK solution iteration
                    instanceOKsolutions[instanceIterator]++;
   
                    // For this instance solution
                    distance = solution.totalCustomerRouteDistance;                    

                    // Izquierdo instances results are not integers
                    if (instance.instance_type == Instance.GoldenIzquierdo)
                    {
                        distance = Math.Truncate(100 * distance) / 100;
                    }
                    else
                    {
                        distance = Math.Truncate(distance);
                    }

                    // Stop timer watchers
                    watch.Stop();

                    // Set execution time
                    elapsedMs = watch.ElapsedMilliseconds;
                    double elapsedSeconds = Math.Round(elapsedMs * 1.0 / 1000, 2);

                    // Update solution results
                    totalDistance[instanceIterator] += distance;
                    totalTime[instanceIterator] += elapsedSeconds;

                    // Update individual instance solution
                    if (distance < bestSolutionTotalDistance[instanceIterator])
                    {
                        bestSolutionTotalDistance[instanceIterator] = distance;
                        bestSolutionPropDistance[instanceIterator] = (distance - solutionToCompare[instanceIterator]) * 100 / solutionToCompare[instanceIterator];
                        bestSolutionPropDistance[instanceIterator] = Math.Truncate(100 * bestSolutionPropDistance[instanceIterator]) / 100;
                        bestSolutionParameters[instanceIterator] = actualParameters;
                        bestSolutionTime[instanceIterator] = elapsedSeconds;
                        bestSolutionForInstance[instance] = solution;
                    }

                }

                // Calculate averages
                double averageDistance = totalDistance[instanceIterator] / instanceOKsolutions[instanceIterator];
                double averageTime = totalTime[instanceIterator] / instanceOKsolutions[instanceIterator];
                averageTime = Math.Round(averageTime, 2);
                double averagePropDistance = (averageDistance - solutionToCompare[instanceIterator]) * 100 / solutionToCompare[instanceIterator];
                averagePropDistance = Math.Truncate(100 * averagePropDistance) / 100;
                bestSolutionPropDistance[instanceIterator] = Math.Truncate(100 * bestSolutionPropDistance[instanceIterator]) / 100;
                solutionAvgPropDistance[instanceIterator] = averagePropDistance;

                // For log solution
                string s_distance;
                string s_averageDistance;
                string s_bestSolutionPropDistance = bestSolutionPropDistance[instanceIterator].ToString("0.00");
                string s_bestSolutionTime = bestSolutionTime[instanceIterator].ToString("0.00");
                string s_averagePropDistance = averagePropDistance.ToString("0.00");
                string s_averageTime = averageTime.ToString("0.00");
                string s_fileName = Path.GetFileName(instancias[instanceIterator].file_name);
                if (instance.instance_type == Instance.GoldenIzquierdo)
                {
                    s_distance = bestSolutionTotalDistance[instanceIterator].ToString("0.00");
                    s_averageDistance = averageDistance.ToString("0.00");
                }
                else
                {
                    s_distance = bestSolutionTotalDistance[instanceIterator].ToString("0");
                    s_averageDistance = averageDistance.ToString("0");
                }

                // Print solution
                string outLine = s_fileName + '\t' + '\t' + s_distance + '\t' + s_bestSolutionPropDistance + "%" + '\t' + s_bestSolutionTime + "s" + '\t' + '\t' + s_averageDistance + '\t' + s_averagePropDistance + "%" + '\t' +  s_averageTime + "s";
                logger.logLine(outLine);
                logger.logTimeAndIterations(solution, instance.file_name);

                // Increase distance counter
                instanceIterator++;
            }
               
            // Stop timer watchers
            totalWatch.Stop();
            var totalElapsedseconds = totalWatch.ElapsedMilliseconds / 1000;

            // Total values
            string s_totalPropBestDistance = (bestSolutionPropDistance.Sum() / instancesNumber).ToString("0.00");
            string s_totalPropAvgDistance = (solutionAvgPropDistance.Sum() / instancesNumber).ToString("0.00");

            // Show parameters for best solution
            logger.logLine("");
            logger.logLine("*************************************");
            logger.logLine("* PARAMETERS FOR BEST SOLUTIONS:    *");
            logger.logLine("*************************************");
            logger.logLine("TOTAL TIME -> " + totalElapsedseconds + " seconds");
            logger.logLine("TOTAL BEST PROP DISTANCE -> " + s_totalPropBestDistance + "%");
            logger.logLine("TOTAL AVG PROP DISTANCE -> " + s_totalPropAvgDistance + "%");
            logger.logLine("");
            for (int i = 0; i < bestSolutionParameters.Length; i++)
            {
                logger.logLine("-----------------------------------------------------------------");
                logger.logLine("");
                logger.logLine("CONFIGURATION FOR INSTANCE " + i);
                logger.logLine("*****************************");
                logger.logLine(bestSolutionParameters[i].ToString());
            }
          
            // Draw PNG solution
            foreach (CluVRPInstance instance in bestSolutionForInstance.Keys)
            {
                //CluVRPSolution.solutionDrawPythonCode(instance, bestSolutionForInstance[instance]);
            }

            // Pause
            //System.Console.ReadKey();

            // End
            return;
        }

        // Main call
        static void Main(string[] args)
        {

            // Default type of clvrp
            CluVRPVersion cluVRPVersion = CluVRPVersion.Strong;

            // Check numbers of parameters
            if (args.Length < 4)
            {
                Console.WriteLine(
                    "Parameter is missing: " + '\n' +
                    "Use: cluvrp_grasp parametersFilePath instanceSetFilePath logFilePath solutionsFilePath typeOfCluVRP" + '\n' + '\n'
                );
                return;
            }
            // If type of cluvrp is defined set it
            else if(args.Length == 5){
                cluVRPVersion = (CluVRPVersion)int.Parse(args[4]);
            }

            // Set variables
            string parametersFilePath = args[0];
            string instanceSetFilePath = args[1];
            string logFilePath = args[2];
            string solutionsFilePath = args[3];

            // Check if parameters are corrects
            if(!(File.Exists(parametersFilePath) && File.Exists(instanceSetFilePath) && File.Exists(solutionsFilePath)))
            {
                Console.WriteLine("Some parameter is incorrect or file not exists");
                return;
            }

            // Try to parse array of solutions for compare
            double[] solutionToCompare = Functions.createSolutionArrayForInstances(instanceSetFilePath, solutionsFilePath);
  
            // Excute GRASP
            GraspProcedure(parametersFilePath, instanceSetFilePath, logFilePath, solutionToCompare, cluVRPVersion);

            // End
            return;
        }

    }
        
}    
