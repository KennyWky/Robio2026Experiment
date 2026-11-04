using System;
using System.IO;
using System.Text;
using UnityEngine;

public class TreePositionExporter : MonoBehaviour
{
    public Transform treeRoot;
    public string treeTag = "Obstacle";
    public bool exportOnStart = true;
    public string fileName = "trees.csv";

    void Start()
    {
        if (exportOnStart) Export();
    }

    [ContextMenu("Export Trees CSV")]
    public void Export()
    {
        string desktop = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
        string path = Path.Combine(desktop, fileName);

        var sb = new StringBuilder();
        sb.AppendLine("x,y,z");

        if (treeRoot != null)
        {
            for (int i = 0; i < treeRoot.childCount; i++)
            {
                var t = treeRoot.GetChild(i);
                Vector3 p = t.position;
                sb.AppendLine($"{p.x:F6},{p.y:F6},{p.z:F6}");
            }
        }
        else
        {
            var trees = GameObject.FindGameObjectsWithTag(treeTag);
            for (int i = 0; i < trees.Length; i++)
            {
                Vector3 p = trees[i].transform.position;
                sb.AppendLine($"{p.x:F6},{p.y:F6},{p.z:F6}");
            }
        }

        File.WriteAllText(path, sb.ToString());
        Debug.Log($"[TreePositionExporter] saved: {path}");
    }
}
