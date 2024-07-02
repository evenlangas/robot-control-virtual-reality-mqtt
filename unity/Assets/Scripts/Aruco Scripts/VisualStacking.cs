using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

public class VisualStacking : MonoBehaviour
{
    [SerializeField] GameObject firstCube;

    private List<GameObject> cubes = new List<GameObject>();

    private int counter = 0;

    [HideInInspector] public bool cubeIndexChanged = false;
    
    void Start()
    {
        firstCube.SetActive(false);
    }
 
    public void cubeStacker()
    {
        counter++;
        if(counter == 1)
        {
            firstCube.SetActive(true);
            cubes.Append(firstCube);
        }
        else
        {
            GameObject cubeClone = Instantiate(firstCube);

            cubeClone.transform.position = firstCube.transform.position;
            cubeClone.transform.position += new Vector3(0, 0.05f * (counter - 1), 0);

            cubes.Add(cubeClone);
        } 
    }

    public void cubeDestroyer()
    {
        if(counter > 0)
        {
            foreach (GameObject cubeClone in cubes)
            {
                Destroy(cubeClone);
            }
            firstCube.SetActive(false);
        }
    }
}

