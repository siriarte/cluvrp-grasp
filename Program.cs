using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CluVRP_GRASP
{
    class Program
    {
        static void Main(string[] args)
        {

            Logger logger = Logger.GetInstance();
            CluVRPInstance[] instancias = InstanceParser.loadGVRPSetOfInstances("prueba");
            logger.logLine("CLIENTES " + instancias[0].gvrp_sets());

        }
    }
        
}    
