using NUnit.Framework;
using System;
using System.Linq;

namespace rclcs.Test
{
    [TestFixture]
    public class LargeMessageTest
    {
        Context pub_context;
        Context sub_context;
        Node pub_node;
        Node sub_node;

        [SetUp]
        public void SetUp()
        {
            pub_context = new Context();
            sub_context = new Context();

            Rclcs.Init(pub_context);
            Rclcs.Init(sub_context);

            pub_node = new Node("pub_node", pub_context);
            sub_node = new Node("sub_node", sub_context);
        }

        [TearDown]
        public void TearDown()
        {
            pub_node.Dispose();
            sub_node.Dispose();
            Rclcs.Shutdown(pub_context);
            Rclcs.Shutdown(sub_context);
        }

        [Test]
        public void EmptyBoolPubSub()
        {
            bool callbackTriggered = false;
            var subscription = sub_node.CreateSubscription<std_msgs.msg.Bool>(
              "test_topic", (received_msg) => { callbackTriggered = true; });

            var publisher = pub_node.CreatePublisher<std_msgs.msg.Bool>("test_topic");
            var msg = new std_msgs.msg.Bool();

            publisher.Publish(msg);

            Rclcs.SpinOnce(sub_node, sub_context, 0.5);

            Assert.That(callbackTriggered, Is.True);

            publisher.Dispose();
            subscription.Dispose();
        }

        [Test]
        public void ImagePubSub()
        {
            bool callbackTriggered = false;

            var subscription = sub_node.CreateSubscription<sensor_msgs.msg.Image>(
              "test_topic", (received_msg) =>
              {
                callbackTriggered = true;
                // Assert.That(received_msg.Data.Length, Is.EqualTo(10));
              });

            var publisher = pub_node.CreatePublisher<sensor_msgs.msg.Image>("test_topic");
            var msg = new sensor_msgs.msg.Image();
            msg.Data = new byte[1];
            msg.Data[0] = 1;
            publisher.Publish(msg);

            Rclcs.SpinOnce(sub_node, sub_context, 0.5);

            Assert.That(callbackTriggered, Is.True);

            publisher.Dispose();
            subscription.Dispose();
        }

        [Test]
        public void EmptyDefaultsPubSub()
        {
            bool callbackTriggered = false;
            var subscription = sub_node.CreateSubscription<test_msgs.msg.Defaults>(
              "test_topic", (received_msg) => { callbackTriggered = true; });

            var publisher = pub_node.CreatePublisher<test_msgs.msg.Defaults>("test_topic");
            var msg = new test_msgs.msg.Defaults();

            publisher.Publish(msg);

            Rclcs.SpinOnce(sub_node, sub_context, 0.5);

            Assert.That(callbackTriggered, Is.True);

            publisher.Dispose();
            subscription.Dispose();
        }

        [Test]
        public void EmptyStringsPubSub()
        {
            bool callbackTriggered = false;
            var subscription = sub_node.CreateSubscription<test_msgs.msg.Strings>(
              "test_topic", (received_msg) => { callbackTriggered = true; });

            var publisher = pub_node.CreatePublisher<test_msgs.msg.Strings>("test_topic");
            var msg = new test_msgs.msg.Strings();

            publisher.Publish(msg);

            Rclcs.SpinOnce(sub_node, sub_context, 0.5);

            Assert.That(callbackTriggered, Is.True);

            publisher.Dispose();
            subscription.Dispose();
        }

        [Test]
        public void TfPubSub()
        {
            bool callbackTriggered = false;
            var subscription = sub_node.CreateSubscription<tf2_msgs.msg.TFMessage>(
              "test_topic", (received_msg) => 
              { 
                callbackTriggered = true; 
                Assert.That(received_msg.Transforms.Length, Is.EqualTo(2));
              });

            var publisher = pub_node.CreatePublisher<tf2_msgs.msg.TFMessage>("test_topic");
            var msg = new tf2_msgs.msg.TFMessage();
            var transforms = new geometry_msgs.msg.TransformStamped[2];
            transforms[0] = new geometry_msgs.msg.TransformStamped();
            transforms[1] = new geometry_msgs.msg.TransformStamped();
            msg.Transforms = transforms; 

            publisher.Publish(msg);
            Rclcs.SpinOnce(sub_node, sub_context, 0.5);

            Assert.That(callbackTriggered, Is.True);

            publisher.Dispose();
            subscription.Dispose();
        }


        [Test]
        public void LaserScanPubSub()
        {
            bool callbackTriggered = false;

            var subscription = sub_node.CreateSubscription<sensor_msgs.msg.LaserScan>(
              "test_topic", (received_msg) =>
              {
                callbackTriggered = true;
                // received_msg.ReadNativeMessage();
                // Assert.That(received_msg.Ranges.Length, Is.EqualTo(10));
              });

            var publisher = pub_node.CreatePublisher<sensor_msgs.msg.LaserScan>("test_topic");
            var msg = new sensor_msgs.msg.LaserScan();
            msg.Ranges = new float[10];
            Assert.That(msg.Ranges.Length, Is.EqualTo(10));

            var copiedMsg = new sensor_msgs.msg.LaserScan();
            copiedMsg.ReadNativeMessage(msg.Handle);

            Assert.That(copiedMsg.Ranges.Length, Is.EqualTo(10));

            msg.WriteNativeMessage();
            publisher.Publish(msg);
            Rclcs.SpinOnce(sub_node, sub_context, 0.5);

            Assert.That(callbackTriggered, Is.True);

            publisher.Dispose();
            subscription.Dispose();
        }

        // [Test]
        // public void MultiArrayDimensionPubSub()
        // {
        //     bool callbackTriggered = false;
        //     var subscription = sub_node.CreateSubscription<std_msgs.msg.MultiArrayDimension>(
        //       "test_topic", (received_msg) => { callbackTriggered = true; });

        //     var publisher = pub_node.CreatePublisher<std_msgs.msg.MultiArrayDimension>("test_topic");

        //     var msg = new std_msgs.msg.MultiArrayDimension();

        //     publisher.Publish(msg);

        //     Rclcs.SpinOnce(sub_node, sub_context, 0.5);

        //     Assert.That(callbackTriggered, Is.True);

        //     publisher.Dispose();
        //     subscription.Dispose();
        // }

        // [Test]
        // public void MultiArrayLayoutPubSub()
        // {
        //     bool callbackTriggered = false;
        //     var subscription = sub_node.CreateSubscription<std_msgs.msg.MultiArrayLayout>(
        //       "test_topic", (received_msg) => { callbackTriggered = true; });
        //
        //     var publisher = pub_node.CreatePublisher<std_msgs.msg.MultiArrayLayout>("test_topic");
        //
        //     var msg = new std_msgs.msg.MultiArrayLayout();
        //
        //     publisher.Publish(msg);
        //
        //     Rclcs.SpinOnce(sub_node, sub_context, 0.5);
        //
        //     Assert.That(callbackTriggered, Is.True);
        //
        //     publisher.Dispose();
        //     subscription.Dispose();
        // }

        // [Test]
        // public void Float64MultiArrayPubSub()
        // {
        //     bool callbackTriggered = false;
        //     var subscription = sub_node.CreateSubscription<std_msgs.msg.Float64MultiArray>(
        //       "test_topic", (received_msg) => { callbackTriggered = true; });

        //     var publisher = pub_node.CreatePublisher<std_msgs.msg.Float64MultiArray>("test_topic");

        //     // var multiArrayDimension = new std_msgs.msg.MultiArrayDimension[1];
        //     // multiArrayDimension[0] = new std_msgs.msg.MultiArrayDimension();
        //     // var multiArrayLayout = new std_msgs.msg.MultiArrayLayout();
        //     // multiArrayLayout.Dim = multiArrayDimension;

        //     var msg = new std_msgs.msg.Float64MultiArray();
        //     // msg.Layout = multiArrayLayout;
        //     // msg.Data = new double[2];

        //     publisher.Publish(msg);

        //     Rclcs.SpinOnce(sub_node, sub_context, 0.5);

        //     Assert.That(callbackTriggered, Is.True);

        //     publisher.Dispose();
        //     subscription.Dispose();
        // }

        // [Test]
        // public void ImageSub()
        // {
        //     bool callbackTriggered = false;
        //     var subscription = sub_node.CreateSubscription<sensor_msgs.msg.Image>(
        //       "/image", (received_msg) =>
        //       {
        //         callbackTriggered = true;
        //         Assert.That(received_msg.Data[0], Is.EqualTo(5));
        //       });

        //     Rclcs.SpinOnce(sub_node, sub_context, 3.0);

        //     Assert.That(callbackTriggered, Is.True);

        //     subscription.Dispose();
        // }

        // TODO(sam): implement byte type in generators
        // [Test]
        // public void ByteMultiArrayPubSub()
        // {
        //     bool callbackTriggered = false;
        //     var subscription = sub_node.CreateSubscription<std_msgs.msg.ByteMultiArray>(
        //       "test_topic", (received_msg) => { callbackTriggered = true; });
        //
        //     var publisher = pub_node.CreatePublisher<std_msgs.msg.ByteMultiArray>("test_topic");
        //     var msg = new std_msgs.msg.ByteMultiArray();
        //     publisher.Publish(msg);
        //
        //     Rclcs.SpinOnce(sub_node, sub_context, 0.5);
        //
        //     Assert.That(callbackTriggered, Is.True);
        //
        //     publisher.Dispose();
        //     subscription.Dispose();
        // }

        // [Test]
        // public void EmptyUnboundedSequenses()
        // {
        //     bool callbackTriggered = false;
        //     var subscription = node.CreateSubscription<test_msgs.msg.UnboundedSequences>(
        //       "test_topic", (received_msg) => { callbackTriggered = true; });
        //
        //     var publisher = node.CreatePublisher<test_msgs.msg.UnboundedSequences>("test_topic");
        //     var msg = new test_msgs.msg.UnboundedSequences();
        //
        //     msg.Basic_types_values = new test_msgs.msg.BasicTypes[0];
        //     msg.Constants_values = new test_msgs.msg.Constants[0];
        //     msg.Defaults_values = new test_msgs.msg.Defaults[0];
        //
        //     publisher.Publish(msg);
        //     Rclcs.SpinOnce(node, context, 0.1);
        //
        //     Assert.That(callbackTriggered, Is.True);
        //
        //     publisher.Dispose();
        //     subscription.Dispose();
        // }


    }
}
