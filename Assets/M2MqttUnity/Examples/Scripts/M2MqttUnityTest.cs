/*
The MIT License (MIT)

Copyright (c) 2018 Giovanni Paolo Vigano'

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.UI;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;
using M2MqttUnity;
using Defective.JSON;

/// <summary>
/// Examples for the M2MQTT library (https://github.com/eclipse/paho.mqtt.m2mqtt),
/// </summary>
namespace M2MqttUnity.Examples
{
    /// <summary>
    /// Script for testing M2MQTT with a Unity UI
    /// </summary>
    public class M2MqttUnityTest : M2MqttUnityClient
    {
        [Tooltip("Set this to true to perform a testing cycle automatically on startup")]
        public bool autoTest = false;
        [Header("User Interface")]
        public InputField consoleInputField;
        public Toggle encryptedToggle;
        public InputField addressInputField;
        public InputField portInputField;
        public Button connectButton;
        public Button disconnectButton;
        public Button testPublishButton;
        public Button clearButton;

        private List<string> eventMessages = new List<string>();
        private bool updateUI = false;
        private bool connected = false;
        private List<double> outputTime = new List<double>();
        private List<double> inputTime = new List<double>();

        public bool getConnectionStatus()
        {
            return connected;
        }

        public void TestPublish()
        {
            // SubscribeTopics();
            // client.Publish("/til-tak/drammen/production/line/robot/control", System.Text.Encoding.UTF8.GetBytes("{'timestamp': '2024-05-31T20:48:31:00Z','joint_1': 0,'joint_2': 90,'joint_3': 1,'joint_4': 91,'joint_5': 2,'joint_6': 92.23}"), MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, false);
            // Debug.Log("Test message published");
            // AddUiMessage("Test message published.");
            // List<float> jointAngles = new List<float>(6);
            float[] jointAngles = new float[6];
            jointAngles[0] = 145.0f;
            jointAngles[1] = -45.0f;
            jointAngles[2] = 90.0f;
            jointAngles[3] = 0.0f;
            jointAngles[4] = 0.0f;
            jointAngles[5] = 0.0f;
            // jointAngles.Add(0.0f);
            // jointAngles.Add(90.0f);
            // jointAngles.Add(1.0f);
            // jointAngles.Add(91.0f);
            // jointAngles.Add(2.0f);
            // jointAngles.Add(92.02f);
            // PublishJoints(jointAngles);
        }

        public void PublishMessage(string topic, string msg)
        {
            client.Publish(topic, System.Text.Encoding.UTF8.GetBytes(msg), MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, false);
            if (topic == "/til-tak/drammen/production/line/latency-test/output")
            {
                outputTime.Add(Time.timeAsDouble);
            }
        }

        public void PublishSetpoint(float[] coordinates, float[] eulerAngles)
        {
            if (coordinates.Length != 3 || eulerAngles.Length != 3)
            {
                return;
            }
            //string msg = "{\"timestamp\": \"2024-05-31T20:48:31:00Z\", \"x\": " + coordinates[0].ToString().Replace(",", ".");
            string msg = "{\"x\": " + coordinates[0].ToString().Replace(",", ".");
            msg += ", \"y\": " + coordinates[1].ToString().Replace("," , ".");
            msg += ", \"z\": " + coordinates[2].ToString().Replace("," , ".");
            msg += ", \"roll\": " + eulerAngles[0].ToString().Replace("," , ".");
            msg += ", \"pitch\": " + eulerAngles[1].ToString().Replace("," , ".");
            msg += ", \"yaw\": " + eulerAngles[2].ToString().Replace("," , ".");
            msg += "}";
            // client.Publish("/til-tak/drammen/production/line/robot/data", System.Text.Encoding.UTF8.GetBytes(msg), MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, false);
            // Change to publish to the control topic once it is connected.
            client.Publish("/til-tak/drammen/production/line/robot/control", System.Text.Encoding.UTF8.GetBytes(msg), MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, false);
            Debug.Log("Test message published");
            AddUiMessage("Test message published.");
        }

        public void SetBrokerAddress(string brokerAddress)
        {
            if (addressInputField && !updateUI)
            {
                this.brokerAddress = brokerAddress;
            }
        }

        public void SetBrokerPort(string brokerPort)
        {
            if (portInputField && !updateUI)
            {
                int.TryParse(brokerPort, out this.brokerPort);
            }
        }

        public void SetEncrypted(bool isEncrypted)
        {
            this.isEncrypted = isEncrypted;
        }


        public void SetUiMessage(string msg)
        {
            if (consoleInputField != null)
            {
                consoleInputField.text = msg;
                updateUI = true;
            }
        }

        public void AddUiMessage(string msg)
        {
            if (consoleInputField != null)
            {
                consoleInputField.text += msg + "\n";
                updateUI = true;
            }
        }

        protected override void OnConnecting()
        {
            base.OnConnecting();
            SetUiMessage("Connecting to broker on " + brokerAddress + ":" + brokerPort.ToString() + "...\n");
        }

        protected override void OnConnected()
        {
            base.OnConnected();
            SetUiMessage("Connected to broker on " + brokerAddress + "\n");

            connected = true;
            if (autoTest)
            {
                TestPublish();
            }
        }

        protected override void SubscribeTopics()
        {
            client.Subscribe(new string[] { "/til-tak/drammen/production/line/robot/data" }, new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
            client.Subscribe(new string[] { "/til-tak/drammen/production/line/latency-test/input" }, new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
            client.Subscribe(new string[] { "/til-tak/drammen/production/line/robot/control" }, new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
        }

        protected override void UnsubscribeTopics()
        {
            client.Unsubscribe(new string[] { "M2MQTT_Unity/test" });
        }

        protected override void OnConnectionFailed(string errorMessage)
        {
            AddUiMessage("CONNECTION FAILED! " + errorMessage);
        }

        protected override void OnDisconnected()
        {
            AddUiMessage("Disconnected.");
            connected = false;
        }

        protected override void OnConnectionLost()
        {
            AddUiMessage("CONNECTION LOST!");
        }

        private void UpdateUI()
        {
            if (client == null)
            {
                if (connectButton != null)
                {
                    connectButton.interactable = true;
                    disconnectButton.interactable = false;
                    testPublishButton.interactable = false;
                }
            }
            else
            {
                if (testPublishButton != null)
                {
                    testPublishButton.interactable = client.IsConnected;
                }
                if (disconnectButton != null)
                {
                    disconnectButton.interactable = client.IsConnected;
                }
                if (connectButton != null)
                {
                    connectButton.interactable = !client.IsConnected;
                }
            }
            if (addressInputField != null && connectButton != null)
            {
                addressInputField.interactable = connectButton.interactable;
                addressInputField.text = brokerAddress;
            }
            if (portInputField != null && connectButton != null)
            {
                portInputField.interactable = connectButton.interactable;
                portInputField.text = brokerPort.ToString();
            }
            if (encryptedToggle != null && connectButton != null)
            {
                encryptedToggle.interactable = connectButton.interactable;
                encryptedToggle.isOn = isEncrypted;
            }
            if (clearButton != null && connectButton != null)
            {
                clearButton.interactable = connectButton.interactable;
            }
            updateUI = false;
        }

        protected override void Start()
        {
            SetUiMessage("Ready.");
            updateUI = true;
            base.Start();
        }

        protected override void DecodeMessage(string topic, byte[] message)
        {
            string msg = System.Text.Encoding.UTF8.GetString(message);
            Debug.Log("Topic: " + topic);
            Debug.Log("Received: " + msg);

            if (topic == "/til-tak/drammen/production/line/robot/data")
            {
                float[] anglesInTemp = new float[6];
                JSONObject data = new JSONObject(msg);
                print(data.ToString());
                CultureInfo ci = (CultureInfo)CultureInfo.CurrentCulture.Clone();
                ci.NumberFormat.CurrencyDecimalSeparator = ".";

                try
                {
                    anglesInTemp[0] = (float)(double.Parse(data.GetField("joint_1").ToString().Trim('"'), NumberStyles.Any, ci));
                    anglesInTemp[1] = (float)(double.Parse(data.GetField("joint_2").ToString().Trim('"'), NumberStyles.Any, ci));
                    anglesInTemp[2] = (float)(double.Parse(data.GetField("joint_3").ToString().Trim('"'), NumberStyles.Any, ci));
                    anglesInTemp[3] = (float)(double.Parse(data.GetField("joint_4").ToString().Trim('"'), NumberStyles.Any, ci));
                    anglesInTemp[4] = (float)(double.Parse(data.GetField("joint_5").ToString().Trim('"'), NumberStyles.Any, ci));
                    anglesInTemp[5] = (float)(double.Parse(data.GetField("joint_6").ToString().Trim('"'), NumberStyles.Any, ci));
                    this.gameObject.GetComponent<UDPConverter>().anglesIn = anglesInTemp;
                    string path = "Assets/sensordata.txt"; 
                    StreamWriter writer = new StreamWriter(path, true);
                    writer.WriteLine(Time.time.ToString().Replace(",", ".") 
                        + "," + anglesInTemp[0].ToString().Replace(",", ".")
                        + "," + anglesInTemp[1].ToString().Replace(",", ".")
                        + "," + anglesInTemp[2].ToString().Replace(",", ".")
                        + "," + anglesInTemp[3].ToString().Replace(",", ".")
                        + "," + anglesInTemp[4].ToString().Replace(",", ".")
                        + "," + anglesInTemp[5].ToString().Replace(",", "."));
                    writer.Close();

                }
                catch
                {
                    print("ERROR: Message not in the correct format");
                }

            }
            else if (topic == "/til-tak/drammen/production/line/robot/control")
            {
                float[] anglesInTemp = new float[6];
                JSONObject data = new JSONObject(msg);
                print(data.ToString());
                CultureInfo ci = (CultureInfo)CultureInfo.CurrentCulture.Clone();
                ci.NumberFormat.CurrencyDecimalSeparator = ".";

                try
                {
                    anglesInTemp[0] = (float)(double.Parse(data.GetField("x").ToString().Trim('"'), NumberStyles.Any, ci));
                    anglesInTemp[1] = (float)(double.Parse(data.GetField("y").ToString().Trim('"'), NumberStyles.Any, ci));
                    anglesInTemp[2] = (float)(double.Parse(data.GetField("z").ToString().Trim('"'), NumberStyles.Any, ci));
                    anglesInTemp[3] = (float)(double.Parse(data.GetField("roll").ToString().Trim('"'), NumberStyles.Any, ci));
                    anglesInTemp[4] = (float)(double.Parse(data.GetField("pitch").ToString().Trim('"'), NumberStyles.Any, ci));
                    anglesInTemp[5] = (float)(double.Parse(data.GetField("yaw").ToString().Trim('"'), NumberStyles.Any, ci));
                    string path = "Assets/controldata.txt";
                    StreamWriter writer = new StreamWriter(path, true);
                    writer.WriteLine(Time.time.ToString().Replace(",", ".")
                        + "," + anglesInTemp[0].ToString().Replace(",", ".")
                        + "," + anglesInTemp[1].ToString().Replace(",", ".")
                        + "," + anglesInTemp[2].ToString().Replace(",", ".")
                        + "," + anglesInTemp[3].ToString().Replace(",", ".")
                        + "," + anglesInTemp[4].ToString().Replace(",", ".")
                        + "," + anglesInTemp[5].ToString().Replace(",", "."));
                    writer.Close();
                }
                catch
                {
                    print("ERROR: Message not in the correct format");
                }
            }
            else if (topic == "/til-tak/drammen/production/line/latency-test/input")
            {
                inputTime.Add(Time.timeAsDouble);
                List<double> latency = new List<double>();
                string outputString = "";
                string inputString = "";
                string latencyString = "";
                for (int i = 0; i < inputTime.Count; i++)
                {
                    latency.Add(inputTime[i] - outputTime[i]);

                    outputString += outputTime[i].ToString().Replace(",", ".") + ",";
                    inputString += inputTime[i].ToString().Replace(",", ".") + ",";
                    latencyString = latency[i].ToString().Replace(",", ".") + ",";
                }

                print("Output time  : " + outputString);
                print("Input time   : " + inputString); 
                print("Latency time : " + latencyString);
                string path = "Assets/latencydata.txt";
                StreamWriter writer = new StreamWriter(path, true);
                writer.WriteLine(latencyString);
                writer.Close();
            }
        }

        private void StoreMessage(string eventMsg)
        {
            eventMessages.Add(eventMsg);
        }

        private void ProcessMessage(string msg)
        {
            AddUiMessage("Received: " + msg);
        }

        protected override void Update()
        {
            base.Update(); // call ProcessMqttEvents()

            if (eventMessages.Count > 0)
            {
                foreach (string msg in eventMessages)
                {
                    ProcessMessage(msg);
                }
                eventMessages.Clear();
            }
            if (updateUI)
            {
                UpdateUI();
            }
        }

        private void OnDestroy()
        {
            Disconnect();
        }

        private void OnValidate()
        {
            if (autoTest)
            {
                autoConnect = true;
            }
        }
    }
}
