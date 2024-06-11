using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;

public class EndEffectorStatePublisher : MonoBehaviour
{
    public GameObject endEffector;
    public GameObject baseLink; // The base of the robot
    public GameObject ikSetpointReference; // Remove this
    private float baseX;
    private float baseY;
    private float baseZ;
    private float baseRoll;
    private float basePitch;
    private float baseYaw;
    private float elapsedTime = 0f;
    float oldX;
    float oldY;
    float oldZ;
    float oldRoll;
    float oldPitch;
    float oldYaw;
    M2MqttUnity.Examples.M2MqttUnityTest mqttClient;
    private bool firstIteration = true;
    public ActionBasedController rightHand;
    public ActionBasedController leftHand;

    void Start()
    {
        mqttClient = GetComponent<M2MqttUnity.Examples.M2MqttUnityTest>();

        baseX = baseLink.gameObject.transform.position.x;
        baseY = baseLink.gameObject.transform.position.y;
        baseZ = baseLink.gameObject.transform.position.z; 
        baseRoll = baseLink.gameObject.transform.eulerAngles.z;
        basePitch = baseLink.gameObject.transform.eulerAngles.x;
        baseYaw = baseLink.gameObject.transform.eulerAngles.y;
        print("Base link:");
        print("x:" + baseX.ToString() + ", y:" + baseY.ToString() + ", z:" + baseZ.ToString());
        print("roll:" + baseRoll.ToString() + ", pitch:" + basePitch.ToString() + ", yaw:" + baseYaw.ToString());

        float x = endEffector.gameObject.transform.position.x;
        float y = endEffector.gameObject.transform.position.y;
        float z = endEffector.gameObject.transform.position.z;
        float roll = endEffector.gameObject.transform.eulerAngles.x;
        float pitch = endEffector.gameObject.transform.eulerAngles.y;
        float yaw = endEffector.gameObject.transform.eulerAngles.z;
        print("End effector (world):");
        print("x:" + x.ToString() + ", y:" + y.ToString() + ", z:" + z.ToString());
        print("roll:" + roll.ToString() + ", pitch:" + pitch.ToString() + ", yaw:" + yaw.ToString());

        x = endEffector.gameObject.transform.position.x - baseX;
        y = endEffector.gameObject.transform.position.y - baseY;
        z = endEffector.gameObject.transform.position.z - baseZ;
        roll = endEffector.gameObject.transform.eulerAngles.x - baseRoll;
        pitch = endEffector.gameObject.transform.eulerAngles.y - basePitch;
        yaw = endEffector.gameObject.transform.eulerAngles.z - baseYaw;
        print("End effector (relative to base):");
        print("x:" + x.ToString() + ", y:" + y.ToString() + ", z:" + z.ToString());
        print("roll:" + roll.ToString() + ", pitch:" + pitch.ToString() + ", yaw:" + yaw.ToString());

    }

    void Update()
    {
        elapsedTime += Time.deltaTime;
        if (elapsedTime > 0.1f)
        {
            //float x = -(endEffector.gameObject.transform.position.x - baseX);
            //float y = -(endEffector.gameObject.transform.position.z - baseZ);
            //float z = endEffector.gameObject.transform.position.y - baseY;
            //float roll = mapAngles(180 - (endEffector.gameObject.transform.eulerAngles.z - baseRoll + 90));
            //float pitch = mapAngles(endEffector.gameObject.transform.eulerAngles.x - basePitch);
            //float yaw = mapAngles(-(endEffector.gameObject.transform.eulerAngles.y - baseYaw));

            //int nrOfDecimals = 4;
            //float roundingFactor = Mathf.Pow(10, nrOfDecimals);
            //x = Mathf.Round(x * roundingFactor) / roundingFactor;
            //y = Mathf.Round(y * roundingFactor) / roundingFactor;
            //z = Mathf.Round(z * roundingFactor) / roundingFactor;
            //roll = Mathf.Round(roll * roundingFactor) / roundingFactor;
            //pitch = Mathf.Round(pitch * roundingFactor) / roundingFactor;
            //yaw = Mathf.Round(yaw * roundingFactor) / roundingFactor;

            float x = ikSetpointReference.gameObject.transform.localPosition.z;
            float y = -ikSetpointReference.gameObject.transform.localPosition.x;
            float z = ikSetpointReference.gameObject.transform.localPosition.y;
            float roll = mapAngles(-ikSetpointReference.gameObject.transform.localEulerAngles.z+90);
            float pitch = mapAngles(ikSetpointReference.gameObject.transform.localEulerAngles.x);
            float yaw = mapAngles(-ikSetpointReference.gameObject.transform.localEulerAngles.y);

            int nrOfDecimals = 4;
            float roundingFactor = Mathf.Pow(10, nrOfDecimals);
            x = Mathf.Round(x * roundingFactor) / roundingFactor;
            y = Mathf.Round(y * roundingFactor) / roundingFactor;
            z = Mathf.Round(z * roundingFactor) / roundingFactor;
            roll = Mathf.Round(roll * roundingFactor) / roundingFactor;
            pitch = Mathf.Round(pitch * roundingFactor) / roundingFactor;
            yaw = Mathf.Round(yaw * roundingFactor) / roundingFactor;

            if (firstIteration)
            {
                oldX = x;
                oldY = y;
                oldZ = z;
                oldRoll = roll;
                oldPitch = pitch;
                oldYaw = yaw;
                firstIteration = false;
            }

            bool rightTriggerPressed = rightHand.activateAction.action.ReadValue<float>() > 0.5f;
            bool leftTriggerPressed = leftHand.activateAction.action.ReadValue<float>() > 0.5f;

            if ((Mathf.Abs(oldX - x) > 0.01 || Mathf.Abs(oldY - y) > 0.01 || Mathf.Abs(oldZ - z) > 0.01 || Mathf.Abs(oldRoll - roll) > 0.1 || Mathf.Abs(oldPitch - pitch) > 0.1 || Mathf.Abs(oldYaw - yaw) > 0.1) && (rightTriggerPressed || leftTriggerPressed))
            {
                float[] coordinates = {x,y,z};
                float[] eulerAngles = {roll,pitch,yaw};

                string msg = "{\"x\": " + coordinates[0].ToString().Replace(",", ".");
                msg += ", \"y\": " + coordinates[1].ToString().Replace(",", ".");
                msg += ", \"z\": " + coordinates[2].ToString().Replace(",", ".");
                msg += ", \"roll\": " + eulerAngles[0].ToString().Replace(",", ".");
                msg += ", \"pitch\": " + eulerAngles[1].ToString().Replace(",", ".");
                msg += ", \"yaw\": " + eulerAngles[2].ToString().Replace(",", ".");
                msg += "}";
                print("Publishing reference position: " + msg);

                if (mqttClient.getConnectionStatus())
                {
                    mqttClient.PublishSetpoint(coordinates, eulerAngles);
                    oldX = x;
                    oldY = y;
                    oldZ = z;
                    oldRoll = roll;
                    oldPitch = pitch;
                    oldYaw = yaw;
                    elapsedTime = 0f;
                }
            }
        }
        //print("x:"+x.ToString()+", y:"+y.ToString()+", z:"+z.ToString());
    }
    private float mapAngles(float angle)
    {
        while (angle > 180) { angle -= 360f; }
        while (angle < -180) { angle += 360f; }
        return angle;
    }
}
