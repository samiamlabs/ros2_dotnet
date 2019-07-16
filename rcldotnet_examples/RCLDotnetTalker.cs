using System;
using System.Threading;

using rclcs;
using ROS2.Utils;

namespace ConsoleApplication
{
    public class RCLDotnetTalker
    {
        public static void Main(string[] args)
        {
            Console.WriteLine("Ctor");
            Context ctx = new Context();
            Console.WriteLine("Ctor1");
            Rclcs.Init(ctx);
            Console.WriteLine("Ctor2");
            INode node = Rclcs.CreateNode("talker", ctx);
            Console.WriteLine("Ctor3");
            IPublisher<std_msgs.msg.String> chatter_pub = node.CreatePublisher<std_msgs.msg.String>("chatter");
            Console.WriteLine("Ctor4");
            std_msgs.msg.String msg = new std_msgs.msg.String();
            Console.WriteLine("Ctor5");
            int i = 1;

            while (Rclcs.Ok(ctx))
            {
                Console.WriteLine("Ctor6");
                msg.Data = "Hello World: " + i;
                i++;
                Console.WriteLine("Publishing: \"" + msg.Data + "\"");
                chatter_pub.Publish(msg);

                // Sleep a little bit between each message
                Thread.Sleep(1000);
            }
            Rclcs.Shutdown(ctx);
        }
    }
}
