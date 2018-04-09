using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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

        public void setLogFilePath(string filePath)
        {
            this.logFilePath = filePath;
        }

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


    }
}
