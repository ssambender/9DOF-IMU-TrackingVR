// receive uduino code from Uduino asset

using UnityEngine;
using Uduino;

public class ReceiveIMUValues : MonoBehaviour {

    Vector3 position;
    Vector3 rotation;
    public Vector3 rotationOffset ;
    public float speedFactor = 15.0f;
    public string imuName = "r";

    void Start () {
      UduinoManager.Instance.OnDataReceived += ReadIMU;
    }

    void Update() { }

    public void ReadIMU (string data, UduinoDevice device) {
        string[] values = data.Split('/');
        if (values.Length == 5 && values[0] == imuName)
        {
            float w = float.Parse(values[1]);
            float x = float.Parse(values[2]);
            float y = float.Parse(values[3]);
            float z = float.Parse(values[4]);
            this.transform.localRotation = Quaternion.Lerp(this.transform.localRotation,  new Quaternion(w, y, x, z), Time.deltaTime * speedFactor);
        } else if (values.Length != 5)
        {
            Debug.LogWarning(data);
        }
        this.transform.parent.transform.eulerAngles = rotationOffset;
    }
}
