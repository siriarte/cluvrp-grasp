using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CluVRP_GRASP
{

    public class Logger
    {
        static string defaultLogFilePath = "grasp.log";
        private static Logger instance = null;

        string logFilePath;
        bool verbose;

        public static Logger GetInstance()
        {
            if (instance == null)
                instance = new Logger(defaultLogFilePath, false);

            return instance;
        }

        public Logger(string logFilePath, bool verbose)
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
