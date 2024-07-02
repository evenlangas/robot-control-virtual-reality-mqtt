using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DragAndDrop : MonoBehaviour
{
    Vector3 mousePos;

    private Vector3 getMousePos()
    {
        return Camera.main.WorldToScreenPoint(transform.position);
    }

    private void OnMouseDown()
    {
        mousePos = Input.mousePosition - getMousePos();
    }

    private void OnMouseDrag()
    {
        float angleX = Camera.main.ScreenToWorldPoint(Input.mousePosition - mousePos).x;

        var xDrive = transform.parent.GetComponent<ArticulationBody>().xDrive;
        xDrive.target = angleX * 10;
        transform.parent.GetComponent<ArticulationBody>().xDrive = transform.parent.GetComponent<ArticulationBody>().xDrive;
    }
}
