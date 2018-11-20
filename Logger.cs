using System;
using cluvrp_grasp;

namespace cluvrp_grasp
{

    public class Logger
    {
        // Private variables
        private static string defaultLogFilePath = "../../grasp.log";
        private static Logger instance = null;
        private string logFilePath;
        private bool verbose;

        // To get singleton instance 
        public static Logger GetInstance()
        {
            if (instance == null)
                instance = new Logger(defaultLogFilePath, false);

            return instance;
        }

        // Constructor for singleton
        private Logger(string logFilePath, bool verbose)
        {
            this.logFilePath = logFilePath;
            this.verbose = verbose;
        }

        // Set log file path
        public void setLogFilePath(string filePath)
        {
            this.logFilePath = filePath;
        }

        // Set Verbose
        public void setVerbose(bool verbose)
        {
            this.verbose = verbose;
        }

        // To log a single string
        public void logLine(string line)
        {
            try
            {
                using (System.IO.StreamWriter file = System.IO.File.AppendText(logFilePath))
                {
                    file.WriteLine(line);
                    if (verbose)
                    {
                        Console.WriteLine(line);
                    }
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }

        }

        // To log an array of string
        public void logBuffer(string[] buffer)
        {
            try
            {
                using (System.IO.StreamWriter file = System.IO.File.AppendText(logFilePath))
                {
                    foreach (string line in buffer)
                    {
                        file.WriteLine(line);
                        if (verbose)
                        {
                            Console.WriteLine(line);
                        }
                    }
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }
        }

        // Get the log file path
        public string getLogFilePath()
        {
            return this.logFilePath;
        }

        internal void logTimeAndIterations(CluVRPSolution solution, string instance)
        {
            string line = instance + '\t' +
                solution.cluVRPIterations.ToString() + '\t' +
                
                //solution.clusterLevelIterations.ToString() + '\t' +
                //solution.customerLevelIterations.ToString() + '\t' +
                //solution.LSCycleterations.ToString() + '\t' +

                (solution.clusterLevelTime).ToString() + '\t' +
                (solution.customerLevelTime).ToString() + '\t' +
                (solution.cluster_LSCycleTime).ToString() + '\t' +
                (solution.customer_LSCycleTime).ToString() + '\t' +

                solution.cluster_twoOpt_iterations.ToString() + '\t' +
                (solution.cluster_twoOpt_time).ToString() + '\t' +

                solution.cluster_relocate_iterations.ToString() + '\t' +
                (solution.cluster_relocate_time).ToString() + '\t' +

                solution.cluster_exchange_iterations.ToString() + '\t' +
                (solution.cluster_exchange_time).ToString() + '\t' +

                solution.cluster_swapClusters_iterations.ToString() + '\t' +
                (solution.cluster_swapClusters_time).ToString() + '\t' +

                solution.cluster_swapVehicle_iterations.ToString() + '\t' +
                (solution.cluster_swapVehicle_time).ToString() + '\t' +

                solution.cluster_insertVehicle_iterations.ToString() + '\t' +
                (solution.cluster_insertVehicle_time).ToString() + '\t' +

                solution.customer_twoOpt_iterations.ToString() + '\t' +
                (solution.customer_twoOpt_time).ToString() + '\t' +

                solution.customer_relocate_iterations.ToString() + '\t' +
                (solution.customer_relocate_time).ToString() + '\t' +

                solution.customer_exchange_iterations.ToString() + '\t' +
                (solution.customer_exchange_time).ToString() + '\t' +

                solution.customer_swapCustomers_iterations.ToString() + '\t' +
                (solution.customer_swapCustomers_time).ToString();

          try
            {
                using (System.IO.StreamWriter file = System.IO.File.AppendText(logFilePath + "_time"))
                {
                    file.WriteLine(line);
                    if (verbose)
                    {
                        Console.WriteLine(line);
                    }
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }
        }
    }

}
