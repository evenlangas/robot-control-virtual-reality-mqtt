using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;

public class ConveyerSimpleMovement : MonoBehaviour
{
    public bool enableConveyer = false;
    public float moveSpeed = 0.01f;
    [SerializeField] float speedRatio = 1.0f;

    [SerializeField] GameObject conveyerBelt;
    [SerializeField] Rigidbody moveableObjectRigidbody;
    [SerializeField] GameObject moveableObject;

    [SerializeField] MeshRenderer beltTexture;
    [SerializeField] GameObject respawnPoint;

    private bool directionChanged = false;

    private Vector3 respawnStartposition;
    
    

    // Start is called before the first frame update
    void Start()
    {
        respawnStartposition = respawnPoint.transform.position;
    }

    // Update is called once per frame
    void Update()
    {   
        inputCheck();

        respawnPlacer();
            
        if(enableConveyer)
        {
            conveyerRunning();
        }
    }

    void conveyerRunning()
    {
        Vector3 textureMoveSpeed = new Vector2((Time.time * moveSpeed * speedRatio), 0);

        beltTexture.material.mainTextureOffset = textureMoveSpeed;

        if (moveableObject.transform.position.y > 0.5 && moveableObjectRigidbody.velocity.y < 0.5)
        {
            moveableObjectRigidbody.position += new Vector3(moveSpeed * Time.deltaTime, 0, 0);
        }
        else if (moveableObject.transform.position.y < 0.5)
        {
            moveableObject.transform.position = respawnPoint.transform.position;
        }
    }

    void inputCheck()
    {
        if(Input.GetKeyDown("b"))
        {
            enableConveyer = !enableConveyer;
        }
        
        if(Input.GetKeyDown("n"))
        {
            moveSpeed -= 0.1f;
            if(moveSpeed < 0)
            {
                directionChanged = true;
            }
        }
        else if(Input.GetKeyDown("m"))
        {
            moveSpeed += 0.1f;
            if(moveSpeed > 0)
            {
                directionChanged = false;
            }
        }
    }

    void respawnPlacer()
    {
        if(directionChanged)
        {
            respawnPoint.transform.position = new Vector3(-respawnStartposition.x, respawnStartposition.y, respawnStartposition.z);
        }
        else
        {
            respawnPoint.transform.position = respawnStartposition;
        }
    }
}
