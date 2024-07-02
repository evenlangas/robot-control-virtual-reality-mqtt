using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArucoBox : MonoBehaviour
{
    public GameObject arucoBoxInspector;
    public GameObject arucoTracerBoxInspector;
    public static GameObject arucoBox;
    private bool notDetected = true;
    public MeshRenderer arucoRenderer;
    public VisualStacking stacker;
    //private MeshRenderer arucoGrippedRenderer;


    [HideInInspector] public static Vector3 coordinates;
    [HideInInspector] public static bool coordinatesChanged = false;
    [HideInInspector] public static bool readyToVisualStack = false;
    [HideInInspector] public static float yRotation = 0;
    [SerializeField] GameObject grippedCube;

    // Start is called before the first frame update
    void Start()
    {
        arucoBox = arucoBoxInspector;
        arucoRenderer.enabled = false;

        //arucoGrippedRenderer = arucoBox.GetComponentInChildren<MeshRenderer>();

    }

    // Update is called once per frame
    void Update()
    {
        if(notDetected && coordinatesChanged)
        {
            arucoRenderer.enabled = true;
            notDetected = false;
        }  

        if(coordinatesChanged)
        {
            arucoBox.transform.localPosition = new Vector3(-coordinates.x, arucoBox.transform.localPosition.y, coordinates.z);
            arucoBox.transform.localEulerAngles = new Vector3(0, yRotation, 0);
            arucoTracerBoxInspector.transform.position = arucoBox.transform.position;
            coordinatesChanged = false;
        }
        AutoPickup.boxLastPosition = arucoBox.transform.localPosition;

        visualPickedUp();

    }

    void visualPickedUp()
    {
        if(UDP_Client.gripCube && UDP_Client.gripCubeChanged)
        {
            grippedCube.SetActive(true);
            arucoBox.SetActive(false);

            arucoBox.transform.localPosition = new Vector3(0.280000001f,0.0279999971f,0.0083999997f);
            arucoBox.transform.localEulerAngles = new Vector3(0, yRotation, 0);
            arucoTracerBoxInspector.transform.position = arucoBox.transform.position;

            notDetected = true;
            arucoRenderer.enabled = false;

            UDP_Client.gripCubeChanged = false;
        }
        else if(UDP_Client.gripCubeChanged)
        {
            grippedCube.SetActive(false);
            arucoBox.SetActive(true);

            arucoBox.transform.localPosition = new Vector3(0.280000001f,0.0279999971f,0.0083999997f);
            arucoBox.transform.localEulerAngles = new Vector3(0, yRotation, 0);
            arucoTracerBoxInspector.transform.position = arucoBox.transform.position;

            UDP_Client.gripCubeChanged = false;

            if(readyToVisualStack)
            {
                stacker.cubeStacker();
            }
        }
    }
}
