using System.Collections.Generic;
using UnityEngine;

// TEMPORARY test-only script. Safe to delete once the real
// import/node-placement UI exists.
[DefaultExecutionOrder(-200)]
public class DebugFrameTestSetup : MonoBehaviour
{
    public string TestSTLPath = "";
    public float TestDensity = 1250f;

    void Awake()
    {
        if (!string.IsNullOrEmpty(TestSTLPath))
        {
            CustomFrameLoader.SelectedSTLPath = TestSTLPath;
        }
        CustomFrameLoader.SelectedDensity = TestDensity;

        if (CustomFrameLoader.SelectedNodes.Count == 0)
        {
            CustomFrameLoader.SelectedNodes.Add(new CustomFrameLoader.NodeData { type = CustomFrameLoader.NodeType.Battery, position = new Vector3(0f, 0f, 0.05f), massKg = 0.5f });
            CustomFrameLoader.SelectedNodes.Add(new CustomFrameLoader.NodeData { type = CustomFrameLoader.NodeType.Motor, position = new Vector3(0.15f, 0f, 0.15f), massKg = 0.05f });
            CustomFrameLoader.SelectedNodes.Add(new CustomFrameLoader.NodeData { type = CustomFrameLoader.NodeType.Motor, position = new Vector3(-0.15f, 0f, 0.15f), massKg = 0.05f });
            CustomFrameLoader.SelectedNodes.Add(new CustomFrameLoader.NodeData { type = CustomFrameLoader.NodeType.Motor, position = new Vector3(0.15f, 0f, -0.15f), massKg = 0.05f });
            CustomFrameLoader.SelectedNodes.Add(new CustomFrameLoader.NodeData { type = CustomFrameLoader.NodeType.Motor, position = new Vector3(-0.15f, 0f, -0.15f), massKg = 0.05f });
        }
    }
}