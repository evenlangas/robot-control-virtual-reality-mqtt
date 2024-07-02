using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SavedOptions : MonoBehaviour
{
    public static int serverPort = 5500;
    public static string serverIP = "192.168.8.150";
    public static bool serverStarted = false;

    void Awake()
    {
        DontDestroyOnLoad(this.gameObject);
    }

    public void insertIP(string IP)
    {
        serverIP = IP;
    }

    public void insertPort(string Port)
    {
        int.TryParse(Port, out serverPort);
    }

    public void startServer()
    {
        serverStarted = true;
    }

}
