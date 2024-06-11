using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class SetpointSaver : MonoBehaviour
{
    public GameObject setPointBall;

    private Vector3 setPoint;
    private List<Vector3> setPointList = new List<Vector3>();
    private List<Vector3> setPointListAngles = new List<Vector3>();
    private List<GameObject> cloneList = new List<GameObject>();
    public UDPConverter converter;
    private bool gripperMesh = false;

    [HideInInspector] public static List<string> pushedData = new List<string>();
    [HideInInspector] public static bool gripperSuccess = false;
    [HideInInspector] public static bool gripperFailed = false;

    private List<int> gripperIndexes = new List<int>();

    private int t = 0;

    [HideInInspector]
    public void pushAllCoordinates()
    {
        UDP_Client.coordinatesPushed = true;

        List<string> serverData = converter.getCartesianList(setPointList, setPointListAngles, gripperIndexes);
        
        pushedData = serverData;

        resetList();
    }

    public void saveCoordinates()
    {
        setPoint = setPointBall.transform.localPosition;
        Vector3 setPointAngles = setPointBall.transform.localEulerAngles;
        
        setPointList.Add(setPoint);
        setPointListAngles.Add(setPointAngles);

        cloneList.Add(visualClone());

        t++;        
    }

    [HideInInspector] public void saveCoordinatesGripped()
    {
        gripperMesh = true;
        saveCoordinates();
        gripperSuccess = false;

        gripperIndexes.Add(cloneList.Count);
        print("Gripper index: " + gripperIndexes[gripperIndexes.Count-1]);
    }
    

    public void resetList()
    {
        setPointList.Clear();
        setPointListAngles.Clear();
        gripperIndexes.Clear();
        UDP_Client.readyToRun = true;

        if(cloneList!=null)
        {
            foreach (GameObject clone in cloneList)
            {
                Destroy(clone);
            }
            cloneList.Clear();
        }

        t = 0;
    }

    private GameObject visualClone()
    {
        GameObject clone = Instantiate(setPointBall);
        clone.transform.position = setPointBall.transform.position;
        clone.transform.eulerAngles = setPointBall.transform.eulerAngles;

        Camera cloneCamera = clone.GetComponentInChildren<Camera>();
        if (cloneCamera != null)
        {
            Destroy(cloneCamera);
        }


        if(gripperMesh)
        {
            Material cloneMaterial = clone.GetComponent<Renderer>().material;
            cloneMaterial.color = new Color(0f, 255f, 0.5f, 255f);
            
            gripperMesh = false;
        }

        return clone;
    }
}
