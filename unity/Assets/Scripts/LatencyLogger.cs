using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LatencyLogger : MonoBehaviour
{
    private float elapsedTime = 0;
    M2MqttUnity.Examples.M2MqttUnityTest mqttClient;

    float[] coordinates = { 150.9776f, 150.9776f, 150.9776f };
    float[] eulerAngles = { 150.9776f, 150.9776f, 150.9776f };
    string msg = "";
    // Start is called before the first frame update
    void Start()
    {
        mqttClient = GameObject.FindObjectOfType<M2MqttUnity.Examples.M2MqttUnityTest>();
        msg = "{\"x\": " + coordinates[0].ToString().Replace(",", ".");
        msg += ", \"y\": " + coordinates[1].ToString().Replace(",", ".");
        msg += ", \"z\": " + coordinates[2].ToString().Replace(",", ".");
        msg += ", \"roll\": " + eulerAngles[0].ToString().Replace(",", ".");
        msg += ", \"pitch\": " + eulerAngles[1].ToString().Replace(",", ".");
        msg += ", \"yaw\": " + eulerAngles[2].ToString().Replace(",", ".");
        msg += "}";
    }

    // Update is called once per frame
    void Update()
    {
        elapsedTime += Time.deltaTime;
        if (elapsedTime > 1f)
        {
            elapsedTime = 0;
            mqttClient.PublishMessage("/til-tak/drammen/production/line/latency-test/output", msg);
        }
    }
}
