using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraScript : MonoBehaviour
{

    [SerializeField] Camera primaryCam;
    [SerializeField] Camera seccondaryCam;
    [SerializeField] Camera thirdCam;
    [SerializeField] Camera fourthCam;
    [SerializeField] Camera fithCam;
    [SerializeField] Camera sixthCam;
    [SerializeField] Camera seventhCam;
    [SerializeField] Camera eighthCam;

    private int camChoice = 1;

    // Start is called before the first frame update
    void Start()
    {
        primaryCam.enabled = true;
        seccondaryCam.enabled = false;
    }

    // Update is called once per frame
    void Update()
    {

        if(Input.GetKeyDown("c"))
        {
            camChoice++;

            disableCams();
            changeCam();
        }
    }

    void disableCams()
    {
        primaryCam.enabled = false;
        seccondaryCam.enabled = false;
        thirdCam.enabled = false;
        fourthCam.enabled = false;
        fithCam.enabled = false;
        sixthCam.enabled = false;
        seventhCam.enabled = false;
        eighthCam.enabled = false;
    }

    void changeCam()
    {
        if(camChoice == 2)
        {
            seccondaryCam.enabled = true;
        }
        else if(camChoice == 3)
        {
            thirdCam.enabled = true;
        }
        else if(camChoice == 4)
        {
            fourthCam.enabled = true; 
        }
        else if(camChoice == 5)
        {
            fithCam.enabled = true; 
        }
        else if(camChoice == 6)
        {
            sixthCam.enabled = true; 
        }
        else if(camChoice == 7)
        {
            seventhCam.enabled = true; 
        }
        else if(camChoice == 8)
        {
            eighthCam.enabled = true; 
        }
        else
        {
            camChoice = 1;
            primaryCam.enabled = true;
        }
    }
}
