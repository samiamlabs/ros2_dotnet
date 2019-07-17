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
            Context ctx = new Context();
            Rclcs.Init(ctx);
            INode node = Rclcs.CreateNode("talker", ctx);
            IPublisher<std_msgs.msg.String> chatter_pub = node.CreatePublisher<std_msgs.msg.String>("chatter");
            std_msgs.msg.String msg = new std_msgs.msg.String();
            int i = 1;

            while (Rclcs.Ok(ctx))
            {
                // adamdbrw - if no at least small sleep is made before
                // the first published message, it doesn't reach subscribers
                // Needs investigation for how to put a valid check here
                // TODO (adam) replace with a proper check
                Thread.Sleep(500);
                msg.Data = "Hello World: " + i;
                i++;
                Console.WriteLine("Publishing: \"" + msg.Data + "\"");
                chatter_pub.Publish(msg);
                Thread.Sleep(500);
            }
            Rclcs.Shutdown(ctx);
        }
    }
}
