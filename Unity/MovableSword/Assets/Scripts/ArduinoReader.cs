using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO;
using System.IO.Ports;
using System.Text;
using System;
using TMPro;

public class ArduinoReader : MonoBehaviour
{
    public TextMeshProUGUI button_pushes_text;
    public TextMeshProUGUI calibrated_tf_text;
    public TextMeshProUGUI curr_pos_text;
    public TextMeshProUGUI curr_velo_text;
    public TMP_InputField manual_input_quaternion_text;//manually input rotation

    public GameObject controller;

    public TextMeshProUGUI obj1_text;
    public TextMeshProUGUI obj2_text;
    public TextMeshProUGUI obj3_text;
    public TextMeshProUGUI obj4_text;
    private TextMeshProUGUI[] obj_texts;
    public GameObject obj1;
    public GameObject obj2;
    public GameObject obj3;
    public GameObject obj4;
    private GameObject[] objs;
    private int num_objs;
    private string input_from_arduino;
    private bool calibrated;
    private int num_clicks = 0;

    private uint color_not_selected = 0xFFFFFFFF;
    private uint color_selected = 0xFF0079D9;
    private uint color_tranparent = 0x00000000;

    private float x_pos = 0f;
    private float y_pos = 0f;
    private float z_pos = 0f;

    private float x_velo = 0f;
    private float y_velo = 0f;
    private float z_velo = 0f;

    private int curr_prev_accel_replace = 0;
    private int num_prev_accels = 3;
    private static Vector3[] prev_accels = {new Vector3(0,0,0),new Vector3(0,0,0),new Vector3(0,0,0)};

    private float t = 0f;
    private float offset_t = 0f;

    private float g_accel = 80f;//scaled gravity from accelerometer
    public float x_pos_threshold_accel = 9f;
    public float x_neg_threshold_accel = 6f;
    public float y_pos_threshold_accel = 8f;
    public float y_neg_threshold_accel = 7f;
    public float z_pos_threshold_accel = 6f;
    public float z_neg_threshold_accel = 7f;

    private bool x_reverser = false;
    private bool y_reverser = false;
    private bool z_reverser = false;
    private int x_skips = 0;
    private int y_skips = 0;
    private int z_skips = 0;
    private int init_x_dir = 0;
    private int init_y_dir = 0;
    private int init_z_dir = 0;

    private string line = "";

    // SerialPort sp = new SerialPort("/dev/tty.usbmodem70041DD4244C2", 115200); //home arduino
    // SerialPort sp = new SerialPort("/dev/tty.usbmodemF412FA9C889C2", 115200); //school arduino
    SerialPort sp = new SerialPort("COM4", 9600, Parity.None, 8, StopBits.One); //Windows Laptop Bluetooth Connection. Only rate that works with bluetooth
    private Color32 ToColor(uint HexVal)
    {
        byte A = (byte)((HexVal >> 24) & 0xFF);
        byte R = (byte)((HexVal >> 16) & 0xFF);
        byte G = (byte)((HexVal >> 8) & 0xFF);
        byte B = (byte)((HexVal) & 0xFF);
        return new Color32(R, G, B, A);
    }
    // Start is called before the first frame update
    void Start()
    {
        num_objs = 4;
        obj_texts = new TextMeshProUGUI[] {obj1_text,obj2_text,obj3_text,obj4_text};
        objs = new GameObject[] {obj1,obj2,obj3,obj4};
        calibrated = false;
        try
        {
            sp.Open();
            sp.DtrEnable=true;//ABSOLUTELY NECESSARY FOR THE ARDUINO UNO. DO NOT REMOVE LEST YOU WILL BE STUCK FOR WHILE(TRUE){}
            sp.ReadTimeout = 500;//100 is good for wired connection
            Debug.Log("Port is open - "+sp.IsOpen);
        }
        catch (Exception e)
        {
            Debug.Log("Something went wrong opening port "+e.Message);
        }
        
    }

    // Update is called once per frame
    void Update()
    {
        StartCoroutine(ReadFromArduino());
        StartCoroutine(UpdateText());
        

        //space key for testing
        if (Input.GetKeyDown(KeyCode.Space))//testing
        {
            if(calibrated)
            {
                num_clicks = (num_clicks + 1) % num_objs;
            }
            else
            {
                calibrated = true;
            }
        }
        if(Input.GetKeyDown(KeyCode.Return))
        {
            string currline = manual_input_quaternion_text.text;
            string[] splits = currline.Split(',');
            float w = float.Parse(splits[0]);
            float x = float.Parse(splits[1]);
            float y = float.Parse(splits[2]);
            float z = float.Parse(splits[3]);
            Debug.Log(w + ", " + x + ", " + y + ", " + z);
            controller.transform.rotation = new Quaternion(x, -z, y, w);
            
        }
    }

    //Check sp9600 for 1. Button has been clicked if there is a character to be read
    IEnumerator ReadFromArduino()
    {
        while(true)
        {
            // print(sp.IsOpen + " " + sp.BytesToRead);
            if (sp.IsOpen && sp.BytesToRead > 0)
            {
                try
                {
                    char a = (char)sp.ReadByte();
                    // Debug.Log(a);
                    // string line = sp.ReadLine(); //works with wired connection. DOES NOT WORK WITH BLUETOOTH!!!
                    // Debug.Log(line);
                    if(a != '\n')
                    {
                        line += a;
                    }
                    else
                    {
                        if(calibrated)
                        {
                            if(line.Length > 3)//accelerometer data
                            {
                                string[] splits = line.Split(',');
                                float w = float.Parse(splits[0]);
                                float x = float.Parse(splits[1]);
                                float y = float.Parse(splits[2]);
                                float z = float.Parse(splits[3]);
                                // Debug.Log(w + ", " + x + ", " + y + ", " + z);
                                controller.transform.rotation = new Quaternion(x, -z, y, w);
                                
                                //x,-z,y,w//horizontal start
                                //DO NOT SWITCH -Z. IT IS VITAL TO THE SYSTEM WORKING!!!

                                float x_accel = float.Parse(splits[4]);
                                float y_accel = float.Parse(splits[5]);
                                float z_accel = float.Parse(splits[6]);


                                //default from blog... This is what it SHOULD be, however something is flipped, so I am just moving the eqautions around so they actually work.
                                // Vector3 quat_x_row = new Vector3( 1-2*(Mathf.Pow(y,2)+Mathf.Pow(z,2)),                     2*((x*y)-(w*z)),                     2*((w*y)+(x*z)));
                                // Vector3 quat_y_row = new Vector3(                     2*((x*y)+(w*z)), 1-2*(Mathf.Pow(x,2)+Mathf.Pow(z,2)),                     2*((y*z)-(w*x)));
                                // Vector3 quat_z_row = new Vector3(                     2*((x*z)-(w*y)),                     2*((w*x)+(y*z)), 1-2*(Mathf.Pow(x,2)+Mathf.Pow(y,2)));

                                //GO TO https://blog.endaq.com/quaternions-for-orientation#:~:text=Using%20quaternions%2C%20you%20can%20generate%20a%20rotation%20matrix%2C,accelerations%20your%20car%20goes%20through%20when%20it%20turns.daq.com/old-blog/quaternions-for-arientation#:~:text=Using%20quaternions%2C%20you%20can%20generate%20a%20rotation%20matrix%2C,accelerations%20your%20car%20goes%20through%20when%20it%20turns.
                                //Under "USEFUL EQUATIONS" rotation matrix. I think y needs to be flipped.e
                                
                                //v2(switching y with x). x row and y row switched. Don't ask why, but this just works so I am not touching it
                                Vector3 quat_x_row = new Vector3(                     2*((y*x)+(w*z)), 1-2*(Mathf.Pow(y,2)+Mathf.Pow(z,2)),                     2*((x*z)-(w*y)));
                                Vector3 quat_y_row = new Vector3( 1-2*(Mathf.Pow(x,2)+Mathf.Pow(z,2)),                     2*((y*x)-(w*z)),                     2*((w*x)+(y*z)));
                                Vector3 quat_z_row = new Vector3(                     2*((y*z)-(w*x)),                     2*((w*y)+(x*z)), 1-2*(Mathf.Pow(y,2)+Mathf.Pow(x,2)));

                                Vector3 accels = new Vector3(0,0,1);

                                float expected_x_due_to_g = g_accel*(Vector3.Dot(quat_x_row,accels));
                                float expected_y_due_to_g = g_accel*(Vector3.Dot(quat_y_row,accels));
                                float expected_z_due_to_g = g_accel*(Vector3.Dot(quat_z_row,accels));

                                // Debug.Log($"Q:{w},{x},{y},{z}   Actual G:  {x_accel}  {y_accel}  {z_accel}   Expected G: {expected_x_due_to_g} {expected_y_due_to_g} {expected_z_due_to_g}  ");                            // Debug.Log($"Rots:{controller.transform.eulerAngles.x}  {controller.transform.eulerAngles.y}  {controller.transform.eulerAngles.z}");
                                // Debug.Log($"Cosx: {Mathf.Cos(Mathf.PI*controller.transform.eulerAngles.x/180)}  Sinx: {Mathf.Sin(Mathf.PI*controller.transform.eulerAngles.x/180)}  Cosy: {Mathf.Cos(Mathf.PI*controller.transform.eulerAngles.y/180)}  Siny: {Mathf.Sin(Mathf.PI*controller.transform.eulerAngles.y/180)}  Cosz: {Mathf.Cos(Mathf.PI*controller.transform.eulerAngles.z/180)}  Sinz: {Mathf.Sin(Mathf.PI*controller.transform.eulerAngles.z/180)}");

                                
                                Vector3 accel_x_row = new Vector3( 1-2*(Mathf.Pow(y,2)+Mathf.Pow(z,2)),                     2*((x*y)-(w*z)),                     2*((w*y)+(x*z)));
                                Vector3 accel_z_row = new Vector3(                     2*((x*y)+(w*z)), 1-2*(Mathf.Pow(x,2)+Mathf.Pow(z,2)),                     2*((y*z)-(w*x)));
                                Vector3 accel_y_row = new Vector3(                     2*((x*z)-(w*y)),                     2*((w*x)+(y*z)), 1-2*(Mathf.Pow(x,2)+Mathf.Pow(y,2)));
                                // Vector3 accel_x_row = new Vector3( 1-2*(Mathf.Pow(z,2)+Mathf.Pow(y,2)),                     2*((x*(-z))-(w*y)),                     2*((w*(-z))+(x*y)));
                                // Vector3 accel_y_row = new Vector3(                  2*((x*(-z))+(w*y)), 1-2*(Mathf.Pow(x,2)+Mathf.Pow(y,2)),                     2*(((-z)*y)-(w*x)));
                                // Vector3 accel_z_row = new Vector3(                  2*((x*y)-(w*(-z))),                     2*((w*x)+((-z)*y)), 1-2*(Mathf.Pow(x,2)+Mathf.Pow(z,2)));
                                
                                accels = new Vector3(0,0,0);

                                t = Time.time;
                                float deltaT = (t-offset_t);

                                float deltax = 0f;
                                float deltay = 0f;
                                float deltaz = 0f;
                                float scaled_g = g_accel/10; //80 from accelerometer == 10m/s^2 = g

                                //detecting local motion
                                // //movement on x-axis detected
                                // if(x_accel+threshold_accel < expected_x_due_to_g || x_accel-threshold_accel > expected_x_due_to_g)
                                // {
                                //     accels.x = expected_x_due_to_g - x_accel;
                                // }
                                // //movement on y-axis detected
                                // if(y_accel+threshold_accel < expected_y_due_to_g || y_accel-threshold_accel > expected_y_due_to_g)
                                // {
                                //     accels.y = expected_y_due_to_g - y_accel;
                                // }

                                // //movement on z-axis detected
                                // if(z_accel+threshold_accel < expected_z_due_to_g || z_accel-threshold_accel > expected_z_due_to_g)
                                // {
                                //     accels.z = expected_z_due_to_g - z_accel;
                                // }
                                

                                //movement on x-axis detected
                                if(x_accel+x_pos_threshold_accel < expected_x_due_to_g)
                                {
                                    accels.x = x_accel+x_pos_threshold_accel - expected_x_due_to_g;
                                }
                                if(x_accel-x_neg_threshold_accel > expected_x_due_to_g)
                                {
                                    accels.x = x_accel-x_neg_threshold_accel - expected_x_due_to_g;
                                }
                                // int mostly_x = mostly(0);
                                // if(mostly_x != 0)
                                // {
                                //     if(mostly_x > 0)
                                //     {
                                //         if(accels.x > 0)
                                //         {
                                //             x_skips++;
                                //         }
                                //         else if(x_skips > 0)
                                //         {
                                //             x_skips--;
                                //             accels.x = 0;
                                //         }
                                //     }
                                //     else
                                //     {
                                //         if(accels.x < 0)
                                //         {
                                //             x_skips++;
                                //         }
                                //         else if(x_skips < 0)
                                //         {
                                //             x_skips--;
                                //             accels.x = 0;
                                //         }
                                //     }
                                // }

                                // if(((accels.x > 0 || accels.x == 0) && mostly(0) < 0) || ((accels.x < 0 || accels.x == 0) && mostly(0) > 0))
                                // {
                                //     // if(prev_accels.x && !x_reverser
                                //     x_reverser = !x_reverser;
                                    
                                // }
                                // if(x_reverser)
                                // {
                                //     accels.x *= -1;
                                // }

                                //movement on y-axis detected
                                if(y_accel+y_pos_threshold_accel < expected_y_due_to_g)
                                {
                                    accels.y = y_accel+y_pos_threshold_accel - expected_y_due_to_g;
                                }
                                if(y_accel-y_neg_threshold_accel > expected_y_due_to_g)//movement on y-axis detected
                                {
                                    accels.y = y_accel-y_neg_threshold_accel - expected_y_due_to_g;
                                }

                                // int mostly_y = mostly(1);
                                // if(mostly_y != 0)
                                // {
                                //     if(mostly_y > 0)
                                //     {
                                //         if(accels.y > 0)
                                //         {
                                //             y_skips++;
                                //         }
                                //         else if(y_skips > 0)
                                //         {
                                //             y_skips--;
                                //             accels.y = 0;
                                //         }
                                //     }
                                //     else
                                //     {
                                //         if(accels.y < 0)
                                //         {
                                //             y_skips++;
                                //         }
                                //         else if(y_skips < 0)
                                //         {
                                //             y_skips--;
                                //             accels.y = 0;
                                //         }
                                //     }
                                // }
                                // if(((accels.y > 0 || accels.y == 0) && mostly(1) < 0) || ((accels.y < 0 || accels.y == 0) && mostly(1) > 0))
                                // {
                                //     y_reverser = !y_reverser;
                                // }
                                // if(y_reverser)
                                // {
                                //     accels.y *= -1;
                                // }

                                //movement on z-axis detected
                                if(z_accel+z_pos_threshold_accel < expected_z_due_to_g)
                                {
                                    accels.z = z_accel+z_pos_threshold_accel - expected_z_due_to_g;
                                }
                                if(z_accel-z_neg_threshold_accel > expected_z_due_to_g)
                                {
                                    accels.z = z_accel-z_neg_threshold_accel - expected_z_due_to_g;
                                }

                                // int mostly_z = mostly(2);
                                // if(mostly_z != 0)
                                // {
                                //     if(mostly_z > 0)
                                //     {
                                //         if(accels.z > 0)
                                //         {
                                //             z_skips++;
                                //         }
                                //         else if(z_skips > 0)
                                //         {
                                //             z_skips--;
                                //             accels.z = 0;
                                //         }
                                //     }
                                //     else
                                //     {
                                //         if(accels.z < 0)
                                //         {
                                //             z_skips++;
                                //         }
                                //         else if(z_skips < 0)
                                //         {
                                //             z_skips--;
                                //             accels.z = 0;
                                //         }
                                //     }
                                // }

                                // if(((accels.z > 0 || accels.z == 0) && mostly(2) < 0) || ((accels.z < 0 || accels.z == 0) && mostly(2) > 0))
                                // {
                                //     z_reverser = !z_reverser;
                                // }
                                // if(z_reverser)
                                // {
                                //     accels.z *= -1;
                                // }



                                Debug.Log($"Local Accelerations = {accels.x},    {accels.y},   {accels.z}\n{x_reverser},{y_reverser},{z_reverser},");

                                //making the assumption that everything starts at 0 velo >>TODO
                                //This is because decceleration is not coming across.
                                deltax = 5 *.5f * ((Vector3.Dot(accel_x_row,accels))/scaled_g)*deltaT*deltaT + x_velo*deltaT;
                                x_velo = x_velo + ((Vector3.Dot(accel_x_row,accels))/scaled_g)*deltaT;

                                deltay = 5 * .5f * ((Vector3.Dot(accel_y_row,accels))/scaled_g)*deltaT*deltaT + y_velo*deltaT;
                                y_velo = y_velo + ((Vector3.Dot(accel_y_row,accels))/scaled_g)*deltaT;

                                deltaz = 5 * .5f * ((Vector3.Dot(accel_z_row,accels))/scaled_g)*deltaT*deltaT + z_velo*deltaT;
                                z_velo = z_velo + ((Vector3.Dot(accel_z_row,accels))/scaled_g)*deltaT;

                                if(mostly(0) == 0) { x_velo = 0; }
                                if(mostly(1) == 0) { y_velo = 0; }
                                if(mostly(2) == 0) { z_velo = 0; }

                                
                                prev_accels[curr_prev_accel_replace] = accels;
                                curr_prev_accel_replace = (curr_prev_accel_replace + 1) % num_prev_accels;

                                x_pos += deltax;
                                y_pos += deltay;
                                z_pos += deltaz;

                                if(x_pos > 2) { x_pos = 2; }
                                if(x_pos < -2) { x_pos = -2; }
                                if(y_pos > 3) { y_pos = 3; }
                                if(y_pos < 0) { y_pos = 0; }
                                if(z_pos > 2) { z_pos = 2; }
                                if(z_pos < -2) { z_pos = -2; }

                                controller.transform.position = new Vector3(x_pos, y_pos, z_pos);
                                curr_pos_text.text = x_pos.ToString("0.00") +", "+ y_pos.ToString("0.00") +", " + z_pos.ToString("0.00");
                                curr_velo_text.text = x_velo.ToString("0.00") +", "+ y_velo.ToString("0.00") +", " + z_velo.ToString("0.00");
                                // Debug.Log($"Global Pos = {x_pos},    {y_pos},   {z_pos}");
                                offset_t = t;

                                // print($"Current positions = {x_pos},{y_pos},{z_pos}");
                            }
                            else if(line.Length > 0)
                            {
                                num_clicks = (num_clicks + 1) % num_objs;
                            }
                        }
                        else if(line.Equals("calibrated"))
                        {
                            calibrated = true;
                            offset_t = Time.time;
                        }
                        line = "";
                    }
                }
                catch (Exception e)
                {
                    Debug.Log("ERROR - Reading Failed - " + e.Message);
                }
            }
            yield return null;
        }
    }

    IEnumerator UpdateText()
    {
        while(true)
        {
            if(calibrated)
            {
                calibrated_tf_text.text = "true";
                calibrated_tf_text.color = Color.green;
                //make color green
            }
            button_pushes_text.text = num_clicks.ToString();
            for(int i = 0; i < num_objs; i++)
            {
                Color obj_color = ToColor(color_tranparent);
                bool transparent = true;
                Color obj_text_color = ToColor(color_not_selected);

                if(num_clicks == i)
                {
                    transparent = false;
                    obj_text_color = ToColor(color_selected);
                    obj_color = ToColor(color_selected);
                }
                if(transparent)
                {
                    obj_color.a = 0.0f;
                }
                objs[i].GetComponent<MeshRenderer>().material.color = obj_color;


                obj_texts[i].color = obj_text_color;
            }
            yield return null;
        }
    }

    int mostly(int xyz)
    {
        int positive_counter = 0;
        int negative_counter = 0;
        int zero_counter = 0;
        for(int i = 0; i < num_prev_accels; i++)
        {
            // prev_accels[i];
            // return 0;
            if(prev_accels[i][xyz] == 0) { zero_counter++; }
            if(prev_accels[i][xyz] < 0) { negative_counter++; }
            if(prev_accels[i][xyz] > 0) { positive_counter++; }
        }
        if(positive_counter > negative_counter && positive_counter > zero_counter) { return 1; }
        else if(negative_counter > positive_counter && negative_counter > zero_counter) { return -1; }
        else { return 0; }
    }

}
