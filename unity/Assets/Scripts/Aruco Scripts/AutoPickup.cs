using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using UnityEngine.UIElements;

public class AutoPickup : MonoBehaviour
{
    [HideInInspector] public static bool autoPickup = false;
    [HideInInspector] public static bool lastOrderCompleted = true;
    [HideInInspector] public static Vector3 boxLastPosition;
    public GameObject setpointBall; 
    public GameObject arucoTracer; 
    public UDPConverter converter;
    private string[] gripPointList;
    private float VelocityCheck_x = 0;
    private float VelocityCheck2_x = 0;
    public static bool towerStack = false;
    private int i = 0;
    public VisualStacking stacker;

    void Update()
    {

        if(autoPickup && lastOrderCompleted)
        {
            if(checkBoxVelocity())
            {            
                runAutoPickup();
                lastOrderCompleted = false;
            }
        }

        if(Input.GetKeyDown("4"))
        {
            i = 0;
            stacker.cubeDestroyer();
        }
    }

    bool checkBoxVelocity()
    {
        if(boxLastPosition.x < -0.285 && VelocityCheck2_x == boxLastPosition.x)
        {
            print("Executing pickup order");
            boxLastPosition = new Vector3(0, 0, 0);
            return true;
        }

        VelocityCheck2_x = VelocityCheck_x;;
        VelocityCheck_x = boxLastPosition.x;
        return false;
    }

    void runAutoPickup()
    {
        float midPointHeight = 0.098f + i * 0.051f;
        lastOrderCompleted = false;
        string midPoint = "0#-0.7810206#0.3133704#-211.8052#0#0";
        string dropPointUp2 = "0.4878578#-0.6524279#" + midPointHeight.ToString().Replace(",", ".")  + "#-180#0#0";
        string dropPointUp = "0.4876066#-0.78209#" + midPointHeight.ToString().Replace(",", ".")  + "#-180#0#0";

        string[] stackList = new string[] {
            "0.4878578#-0.7704819#0.04611461#-180#0#0#g0",
            "0.4878578#-0.7704819#0.09811124#-180#0#0#g0",
            "0.4878578#-0.7704819#0.1493808#-180#0#0#g0",
            "0.4878578#-0.7704819#0.2003808#-180#0#0#g0",
            "0.4878578#-0.7704819#0.2503808#-180#0#0#g0",
            "0.4878578#-0.7704819#0.3003808#-180#0#0#g0",
            "0.4878578#-0.7704819#0.3503808#-180#0#0#g0",
            "0.4878578#-0.7704819#0.4003808#-180#0#0#g0",
            "0.4878578#-0.7704819#0.4503808#-180#0#0#g0",
            "0.4878578#-0.7704819#0.5003808#-180#0#0#g0",
            "0.4878578#-0.7704819#0.5503808#-180#0#0#g0",
            "0.4878578#-0.7704819#0.603808#-180#0#0#g0",
            
        };

        string dropPoint;
        if(towerStack)
        {
            dropPoint = stackList[i];

            i++;
            print("'i' is now: " + i);
            if(i == stackList.Length)
            {
                i = 0;
            }
        }
        else
        {
            dropPoint = stackList[0];
            i = 0;
        }      

        getGripCoordinates();

        ArucoBox.arucoBox.transform.localPosition = new Vector3(0.515999973f,0.0279999971f,-0.25f);
        arucoTracer.transform.position = ArucoBox.arucoBox.transform.localPosition;
        boxLastPosition = new Vector3(0.515999973f,0.0279999971f,-0.25f);

        string[] orderList = new string[] {gripPointList[0], gripPointList[1], gripPointList[2], 
         dropPointUp2, dropPointUp, dropPoint, dropPointUp, dropPointUp2, midPoint }; //midPoint,

        SetpointSaver.pushedData = orderList.ToList();
        UDP_Client.coordinatesPushed = true;
        ArucoBox.readyToVisualStack = true;
    }

    void getGripCoordinates()
    {
        float gripperRotationY = ArucoBox.yRotation;       
        bool adjustedCompleted = false;
        while(!adjustedCompleted)
        {
            if(gripperRotationY < 0)
            {
                gripperRotationY += 90;
            }
            else if(gripperRotationY > 90)
            {
                gripperRotationY -= 90;
            }
            else
            {
                adjustedCompleted = true;
            }
        }

        Vector3 cubeGripAngle = new Vector3(0, gripperRotationY, -90);
        Vector3 cubeGripPosition = arucoTracer.transform.localPosition;

        gripPointList = converter.getCartesianAutoPickup(cubeGripPosition, cubeGripAngle);        
    }
}
