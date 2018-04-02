using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CluVRP_GRASP
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
                double bestDistance = Grasp.ConstructGreedySolution(instance, 1000);
                string result = String.Format("{0}\t{1}", instance.file_name(), bestDistance.ToString());
                Logger.GetInstance().logLine(result);
            }
            
        }
    }
        
}    
