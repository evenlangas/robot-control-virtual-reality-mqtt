using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.UrdfImporter;
using UnityEngine;
using UnityEngine.UIElements;

public class LinkAngleController : MonoBehaviour
{
    
    // Set up all links as variables
    
    public ArticulationBody link_0;
    public ArticulationBody link_1;
    public ArticulationBody link_2;
    public ArticulationBody link_3;
    public ArticulationBody link_4;
    public ArticulationBody link_5;
    public ArticulationBody link_6;

    public GameObject ikSetpoint;

    [SerializeField] private bool ReadFromRos;
    [SerializeField] private bool UseInverseKinematics;
    public static bool readFromRos;
    public static bool useInverseKinematics;

    public UDPConverter convertedAngles;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame

    void Update()
    {
        // Setter verdien for en statisk variabel, slik at den kan brukes i UDP_Client.cs
        readFromRos = ReadFromRos;
        useInverseKinematics = UseInverseKinematics;

        if(readFromRos)
        {
            anglesIn();
        }     
        else   
        {
            anglesOut();
        }
    }

    // Sends angle to command window and ROS2
    private int i = 0;
    void anglesOut()
    {
        i++;
    
        
        if(useInverseKinematics)
        {
            Vector3 setpointPosition = ikSetpoint.transform.position;
            

            Debug.Log("Setpoint Position " + setpointPosition);

            i = 0;

            
        }
        else if(i == 2) // change number opposite of "i" to send coordinates less times per second
        {
            

            i = 0;
        }
    }

    // Takes angle from ROS2 (when set up)
   private int t = 0; 

    void anglesIn()
    {
        float[] anglesIn = convertedAngles.anglesIn;
        
        t++;
        //Debug.Log(anglesIn[0] + " " + anglesIn[1] + " " + anglesIn[2] + " " + anglesIn[3] + " " + anglesIn[4] + " " + anglesIn[5] + " x" + t);

        link_1.SetDriveTarget(ArticulationDriveAxis.X, anglesIn[0]);
        link_2.SetDriveTarget(ArticulationDriveAxis.X, anglesIn[1]);
        link_3.SetDriveTarget(ArticulationDriveAxis.X, anglesIn[2]);
        link_4.SetDriveTarget(ArticulationDriveAxis.X, anglesIn[3]);
        link_5.SetDriveTarget(ArticulationDriveAxis.X, anglesIn[4]);
        link_6.SetDriveTarget(ArticulationDriveAxis.X, anglesIn[5]);
        
    }  
}
