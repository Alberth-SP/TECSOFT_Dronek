using AR.Drone.Client;
using AR.Drone.Client.Command;
using AR.Drone.Data.Navigation;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;

namespace Kindrone
{
    class DroneController
    {
        public class DroneCommandChangedEventArgs
        {
            public string CommandText { get; set; }
        }

        public delegate void DroneCommandChangedDelegate(object sender, DroneCommandChangedEventArgs args);

        public event DroneCommandChangedDelegate DroneCommandChanged;

        private DroneClient _client;
        private MqttClient client;
        public int speed;
        public bool enable = false;

        public DroneController()
        {
            client = new MqttClient("192.168.0.20", 1883, false, null);
            client.Connect(Guid.NewGuid().ToString());
            _client = new DroneClient("192.168.1.1");
        }

        public DroneController(DroneClient client)
        {
            _client = client;
        }

        public void Start()
        {
            _client.Start();
            _client.FlatTrim();
            enable = true;
        }

        public void Stop()
        {
            _client.Stop();
            enable = false;
            client.Publish("motor", Encoding.UTF8.GetBytes("0"), 0, false);
        }

        public void Hover()
        {
            _client.Hover();
        }

        public void Emergency()
        {
            _client.Emergency();
        }

        public void ResetEmergency()
        {
            _client.ResetEmergency();
        }

        public void SubscribeToGestures()
        {
            // Right Hand
            GestureDetection.RightHandUpDownChanged += GestureDetection_RightHandUpDownChanged;
            GestureDetection.RightHandLeftRightChanged += GestureDetection_RightHandLeftRightChanged;
            GestureDetection.RightHandBackForwardsChanged += GestureDetection_RightHandBackForwardsChanged;

            // Left Hand
            GestureDetection.LeftHandUpDownChanged += GestureDetection_LeftHandUpDownChanged;
            GestureDetection.LeftHandLeftRightChanged += GestureDetection_LeftHandLeftRightChanged;
            GestureDetection.LeftHandBackForwardsChanged += GestureDetection_LeftHandBackForwardsChanged;
        }

        void GestureDetection_LeftHandBackForwardsChanged(object sender, HandPositionChangedArgs args)
        {
            switch (args.Position)
            {
                case HandPosition.Center:
                    break;
                case HandPosition.Backwards:
                    if (_client.NavigationData.State == (NavigationState.Landed | NavigationState.Command))
                        _client.FlatTrim();
                        DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "NULL " });
                    break;
                case HandPosition.Forwards:
                    _client.Hover();
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "NULL" });
                    break;
            }
        }

        void GestureDetection_RightHandBackForwardsChanged(object sender, HandPositionChangedArgs args)
        {
            switch (args.Position)
            {
                case HandPosition.Center:
                    break;
                case HandPosition.Backwards:
                    _client.Progress(FlightMode.Progressive, pitch: 0.05f);
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Ir hacia atras" });
                    if(enable)client.Publish("motor", Encoding.UTF8.GetBytes("9"), 0, false);
                    break;
                case HandPosition.Forwards:
                    _client.Progress(FlightMode.Progressive, pitch: -0.05f);
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "ir hacia adelante" });
                    if (enable) client.Publish("motor", Encoding.UTF8.GetBytes("8"), 0, false);
                    break;
            }
        }

        void GestureDetection_RightHandLeftRightChanged(object sender, HandPositionChangedArgs args)
        {
            switch (args.Position)
            {
                case HandPosition.Center:
                    break;
                case HandPosition.Left:
                    _client.Progress(FlightMode.Progressive, roll: -0.05f);
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Ir hacia la izquierda" });
                    if (enable) client.Publish("motor", Encoding.UTF8.GetBytes("7"), 0, false);
                    break;
                case HandPosition.Right:
                    _client.Progress(FlightMode.Progressive, roll: 0.05f);
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Ir hacia la derecha" });
                    if (enable) client.Publish("motor", Encoding.UTF8.GetBytes("6"), 0, false);
                    break;
            }
        }

        void GestureDetection_LeftHandLeftRightChanged(object sender, HandPositionChangedArgs args)
        {
            switch (args.Position)
            {
                case HandPosition.Center:
                    break;
                case HandPosition.Left:
                    _client.Progress(FlightMode.Progressive, yaw: 0.25f);
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "NULL" });
                    break;
                case HandPosition.Right:
                    _client.Progress(FlightMode.Progressive, yaw: -0.25f);
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "NULL" });
                    break;
            }
        }

        void GestureDetection_RightHandUpDownChanged(object sender, HandPositionChangedArgs args)
        {
            switch (args.Position)
            {
                case HandPosition.Up:
                    _client.Progress(FlightMode.Progressive, gaz: 0.25f);
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "NULL" });
                    if (enable) client.Publish("motor", Encoding.UTF8.GetBytes("arriba"), 0, false);
                    break;
                case HandPosition.Center:
                    break;
                case HandPosition.Down:
                    _client.Progress(FlightMode.Progressive, gaz: -0.25f);
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "NULL" });
                    if (enable) client.Publish("motor", Encoding.UTF8.GetBytes("abajo"), 0, false);
                    break;
            }
        }

        void GestureDetection_LeftHandUpDownChanged(object sender, HandPositionChangedArgs args)
        {
            switch (args.Position)
            {
                case HandPosition.Up:
                    _client.Takeoff();
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Elevarse" });
                    //if (speed < 5) speed++;
                    if (enable) client.Publish("motor", Encoding.UTF8.GetBytes("1"), 0, false);
                    break;
                case HandPosition.Center:
                    break;
                case HandPosition.Down:
                    _client.Land();
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Descender" });
                    //if (speed < 5) speed--;
                    if (enable) client.Publish("motor", Encoding.UTF8.GetBytes("0"), 0, false);
                    break;
            }
        }
    }
}
