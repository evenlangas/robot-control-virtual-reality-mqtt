using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class UDPConverter : MonoBehaviour
{
    public LinkAngleController cobotJoints;

    private float[] angles = new float[6];

    public float[] anglesIn = new float[6];
    public float xDivision = 750.0f;
    public float yDivision = 481.52f;

    public float xAdjust = 0.575f;
    public float yAdjust = 0.46f;

    public string getAngles()
    {
        string anglesString = "";

        angles = new float[]
        {
            cobotJoints.link_1.xDrive.target,
            cobotJoints.link_2.xDrive.target,
            cobotJoints.link_3.xDrive.target,
            cobotJoints.link_4.xDrive.target,
            cobotJoints.link_5.xDrive.target,
            cobotJoints.link_6.xDrive.target
        };

        for(int i = 0; i < 6; i++)
        {
            string indexAngle;
            if(i<5)
            {
                indexAngle = angles[i].ToShortString() + "#";
            }
            else   
            {
                indexAngle = angles[i].ToShortString();
            }

            anglesString += indexAngle;
        }

        anglesString = anglesString.Replace(",", ".");
        return anglesString;
    }

    public void recieveRealAngles(string anglesString)
    {
        try
        {
            string[] anglesTemp = anglesString.Split('#');

            float[] anglesInTemp = new float[6];

            for (int i = 0; i < 6; i++)
            {
                // Use InvariantCulture to ensure '.' is used as the decimal separator.
                anglesInTemp[i] = float.Parse(anglesTemp[i], System.Globalization.CultureInfo.InvariantCulture);
            }

            anglesIn = anglesInTemp;
        }
        catch (FormatException ex)
        {
            Debug.LogError("FormatException in recieveRealAngles: " + ex.Message);
            Debug.LogError("Received string was: '" + anglesString + "'");
        }
        catch (Exception ex)
        {
            Debug.LogError("Exception in recieveRealAngles: " + ex.Message);
        }
    }

    public string getCartesian()
    {
        Vector3 position = cobotJoints.ikSetpoint.transform.localPosition;
        Vector3 rotation = cobotJoints.ikSetpoint.transform.localRotation.eulerAngles;

        string positionString = position.z.ToString() + "#" + (-position.x).ToString() + "#" + position.y.ToString();
        string rotationString = rotation.x.ToString() + "#" + rotation.z.ToString() + "#" + (-rotation.y).ToString();

        return positionString.Replace(",", ".") + "#" + rotationString.Replace(",", ".");
    }

    public string[] getCartesianAutoPickup(Vector3 position, Vector3 rotation)
    {

        string positionString1 = position.z.ToString() + "#" + (-position.x).ToString() + "#" + (position.y +0.4).ToString();
        string rotationString = (-rotation.z + 90).ToString() + "#" + rotation.x.ToString() + "#" + (-rotation.y).ToString();
        
        string overBox = positionString1.Replace(",", ".") + "#" + rotationString.Replace(",", ".");

        string positionString2 = position.z.ToString() + "#" + (-position.x).ToString() + "#" + (position.y + 0.165).ToString();
    
        string pickupBox = positionString2.Replace(",", ".") + "#" + rotationString.Replace(",", ".");

        string[] coordinateList = new string[] {overBox, pickupBox + "#g1", overBox};

        return coordinateList;
    }

    public List<string> getCartesianList(List<Vector3> pushedPosition, List<Vector3> pushedAngles, List<int> gripperIndexes)
    {      

        List<string> printData = new List<string>();
        
        int indexCounter = 0;
        for (int i = 0; i < pushedPosition.Count; i++)
        {
            string positionString = pushedPosition[i].z.ToString() + "#" + (-pushedPosition[i].x).ToString() + "#" + pushedPosition[i].y.ToString() + "#";
            string rotationString = (-pushedAngles[i].z + 90).ToString() + "#" + pushedAngles[i].x.ToString() + "#" + (-pushedAngles[i].y).ToString();

            string printLine = positionString.Replace(",", ".") + rotationString.Replace(",", ".");

            if(gripperIndexes != null && indexCounter < gripperIndexes.Count && gripperIndexes[indexCounter] - 1 == i)
            {
                printLine += "#g";

                indexCounter++;
            }
            print(printLine);

            printData.Add(printLine);
        }

        return printData;
    }
   
    [HideInInspector] public void arucoConverter(string stringCoordinates)
    {
        stringCoordinates = stringCoordinates.Replace(".", ",");

        string[] coordinatesConverted = stringCoordinates.Split('#');
        // 0 = z, 1 = x; 2 = rotation y

        Vector3 newCoordinates = new Vector3(float.Parse(coordinatesConverted[0]), ArucoBox.coordinates.y, float.Parse(coordinatesConverted[1]));
        Vector3 coordinates = new Vector3((newCoordinates.x / xDivision) - xAdjust, newCoordinates.y, (newCoordinates.z / yDivision) - yAdjust );

        ArucoBox.yRotation = float.Parse(coordinatesConverted[2]);
        ArucoBox.coordinates = coordinates;
    }
}
