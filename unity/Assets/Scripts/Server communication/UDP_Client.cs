using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

// Sources: CoPilot and "https://codemaker2016.medium.com/device-communications-in-unity-3d-2a41652064ed"


public class UDP_Client : MonoBehaviour
{
    private UdpClient client;

    private int localPort = 51815;

    private IPEndPoint remoteEndPoint;
    public UDPConverter ConvertedAngles;
    private Thread receiveThread;
    private bool startFinished = false;
    public static bool readyToRun = true;
    private int i = 0;
    private bool gripperAction = false;
    private bool nextLoop = false;
    public static bool gripCube = false;
    public static bool gripCubeChanged = false;
    public static bool skipGrip = true;
    public static bool startGripSent = false;


    private int closeClientCounter = 0;

    [HideInInspector] public static bool coordinatesPushed = false;
    [HideInInspector] public static string sendGripperString = "";
    [HideInInspector] public static bool pushGripperData = false;
    [HideInInspector] public static bool gripperOn = false;
    [HideInInspector] public static bool loopMode = false;
    [HideInInspector] public static bool pushMessage = false;
    [HideInInspector] public static string sendString = "";
    [HideInInspector] public static bool openGripper = false;

    void Start()
    {
        if(SavedOptions.serverStarted)
        {
            remoteEndPoint = new IPEndPoint(IPAddress.Parse(SavedOptions.serverIP), SavedOptions.serverPort);
            print(IPAddress.Parse(SavedOptions.serverIP));
            UDPTest();
            startFinished = true;
            // Sending a starting setpoint
            sendData("0#-0.7810206#0.3133704#-211.8052#0#0");
            // Opening gripper
            sendData("g0");
            // Making the client the admin on node-red
            sendData("admin1");
            sendData("0#-0.7810206#0.3133704#-211.8052#0#0");
        }
    }

    void UDPTest()
    {
        try
        {
            client = new UdpClient(localPort);
            client.Connect(SavedOptions.serverIP, SavedOptions.serverPort);
        }
        catch(SocketException ex)
        {
            if(ex.SocketErrorCode == SocketError.AddressAlreadyInUse)
            {
                print("Port " + localPort + " is already in use");
            }
            else
            {
                print("Exception thrown " + ex.Message);
            }
        }
        catch(Exception errorLog)
        {
            print("Exception thrown " + errorLog.Message);
        }  

        if(client.Client.Connected)
        {
            print("Connected to server clientside");
        }
        else
        {
            print("Not connected to server clientside, retrying");
            UDPTest();
        }

        receiveThread = new Thread(new ThreadStart(receiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    void Update()
    {
        if(SavedOptions.serverStarted && startFinished)
        {
            udpPrinter();
        }

        if(pushMessage)
        {
            sendData(sendString);
            pushMessage = false;
        }

        if(openGripper)
        {
            sendData("g0");
            openGripper = false;
        }
    }

    void udpPrinter()
    {
        if(client == null || !client.Client.Connected)
        {
            try
            {
                client.Connect(SavedOptions.serverIP, SavedOptions.serverPort);
                print("Reconnected to server clientside");
            }
            catch(Exception errorLog)
            {
                print("Exception thrown " + errorLog.Message);
            }
            receiveThread.Abort();
            receiveThread.Start();
        }

        if(ConvertedAngles.getAngles() != null)
        {
            if(i >= SetpointSaver.pushedData.Count && !loopMode && SetpointSaver.pushedData.Count > 0)
            {
                coordinatesPushed = false;
                i = 0;
                SetpointSaver.pushedData.Clear();
                print("Cleared the pushed data list");
            }
            else if(loopMode && i == SetpointSaver.pushedData.Count && nextLoop)
            {
                i = 0;
                nextLoop = false;
                print("Loop command ran");
            }
            bool runIfStatement = true;
            if(coordinatesPushed && runIfStatement && i < SetpointSaver.pushedData.Count)
            {
                if((!SetpointSaver.pushedData[i].Contains("g")) && readyToRun)
                {
                    sendData(SetpointSaver.pushedData[i]);
                    i++;
                }
                else if(readyToRun)
                {
                    string dataString = SetpointSaver.pushedData[i];
                    if (dataString.Contains("g"))
                    {
                        toggleGripper();
                        gripperAction = true;
                    }
                    i++;
                    dataString = dataString.Remove(dataString.Length - 2);
                    sendData(dataString);
                    readyToRun = false;
                }
            }
        }
        else 
        {
            print("ConvertedAngles.getAngles() is null");
        }
    }

    [HideInInspector] public static void toggleGripper()
    {
        if(!gripperOn)
        {
            sendGripperString = "g1";
        }
        else
        {
            sendGripperString = "g0";
        }
        pushGripperData = true;
        gripperOn = !gripperOn;
    }

    void receiveData()
    {
        while(true)
        {    
            try
            {
                byte[] receivedBytes = client.Receive(ref remoteEndPoint);
                string receivedData = Encoding.ASCII.GetString(receivedBytes);     
                checkString(receivedData);       
            }
            catch(Exception errorLog)
            {
                print("Exception thrown " + errorLog.Message);
            }
        }
    }

    public static void sendMessage(string message)
    {
        sendString = message;
        pushMessage = true;
    }

    private void sendData(string pushString)
    {   
        try
        {
            byte[] sendBytes = Encoding.ASCII.GetBytes(pushString);
            client.Send(sendBytes, sendBytes.Length);
            if(pushString.Contains("g"))
            {
                print("Sending: " + pushString);
            }
        }
        catch(Exception errorLog)
        {
            print("Exception thrown " + errorLog.Message);
        }
    }

    private void checkString(string receivedData)
    {
        if(receivedData.Contains("gf"))
        {
            SetpointSaver.gripperFailed = true;
        }
        else if(receivedData.Contains("STOPPED"))
        {
            if(gripperAction)
            {
                sendData(sendGripperString);
                print("Cobot nextLoop with gripper action");
                gripperAction = false;
            }
            else if(loopMode)
            {
                print("Cobot nextLoop with loopmode");
                nextLoop = true;
            }
            else if(AutoPickup.autoPickup)
            {
                AutoPickup.lastOrderCompleted = true;
                print("Stopped and readied next round");
            }
        }
        else if(receivedData.Contains("READY"))
        {
            readyToRun = true;
            if(skipGrip)
            {
                skipGrip = false;
                gripCubeChanged = false;
            }
            else
            {
                gripCube = !gripCube;
                gripCubeChanged = true;
            }
        }
        else if(receivedData.Contains("#c"))
        {
            ConvertedAngles.arucoConverter(receivedData);
            ArucoBox.coordinatesChanged = true;
        }
        else if(receivedData.Contains("#t"))
        {
            receivedData += "#" + DateTime.Now.ToString("HH:mm:ss.fff");
            print("Sending timestamped: " + receivedData);
            sendData(receivedData);
        }
        else 
        {
            ConvertedAngles.recieveRealAngles(receivedData);
        }
    }

    public static void openGrip()
    {
        openGripper = true;
    }

    void OnApplicationQuit()
    {
        CloseClient();
    }

    void OnDestroy()
    {
        if(startFinished)
        {
            CloseClient();
        }
    }

    private void CloseClient()
    {
        if(closeClientCounter == 0)
        {
            sendData("admin2");
            if (receiveThread != null)
            {
                receiveThread.Abort();
                receiveThread = null;
            }
            if (client != null)
            {
                client.Close();
                client = null;
            }
            print("UDP client closed");
            closeClientCounter++;
        }    
        
    }
}