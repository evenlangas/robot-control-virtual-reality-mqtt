using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class MainMenu : MonoBehaviour
{
    private bool conveyerOn = true;
    private int conveyerStatus = 1;

    public void launchTwin()
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex + conveyerStatus);
    }

    public void exitProgram()
    {
        Debug.Log("Exiting program");
        Application.Quit();
    }

    public void toggleConveyer()
    {
        conveyerOn = !conveyerOn;
        if(conveyerOn)
        {
            conveyerStatus = 1;
        }
        else
        {
            conveyerStatus = 2;
        }

    }
}
