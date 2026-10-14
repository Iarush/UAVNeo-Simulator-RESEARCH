using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;

/// <summary>
/// Copies Assets/MassService~ (bundled Python + udp_listener.py) into a Windows player's
/// StreamingAssets/MassService, where MassServiceLauncher.ResolveServiceRoot looks for it.
///
/// Assets/MassService~ is ignored by the asset importer (the trailing ~), so it never reaches a
/// build on its own. Copied after the build rather than kept under StreamingAssets so the
/// Editor does not import thousands of .py/.pyd/.dll files.
/// </summary>
public class MassServiceBuildStep : IPostprocessBuildWithReport
{
    public int callbackOrder => 0;

    public void OnPostprocessBuild(BuildReport report)
    {
        BuildTarget target = report.summary.platform;
        if (target != BuildTarget.StandaloneWindows64 && target != BuildTarget.StandaloneWindows)
        {
            Debug.LogWarning($"[MassServiceBuildStep] Bundled mass service is Windows only; not copied for {target}. " +
                             "Custom frames in this build need the service started by hand.");
            return;
        }

        string source = Path.Combine(Application.dataPath, "MassService~");
        if (!File.Exists(Path.Combine(source, "python", "python.exe")))
        {
            Debug.LogError($"[MassServiceBuildStep] {source}\\python is missing, so this build has no mass service " +
                           "and custom frames will fall back to the default drone. Run tools/build-mass-service.ps1 and rebuild.");
            return;
        }

        // outputPath is the .exe; the player's data folder sits next to it as <exe name>_Data.
        string exePath = report.summary.outputPath;
        string dataDir = Path.Combine(Path.GetDirectoryName(exePath),
                                      Path.GetFileNameWithoutExtension(exePath) + "_Data");
        string destination = Path.Combine(dataDir, "StreamingAssets", "MassService");

        if (Directory.Exists(destination))
            Directory.Delete(destination, true);
        CopyDirectory(source, destination);

        Debug.Log($"[MassServiceBuildStep] Copied mass service to {destination}");
    }

    private static void CopyDirectory(string source, string destination)
    {
        Directory.CreateDirectory(destination);
        foreach (string file in Directory.GetFiles(source))
            File.Copy(file, Path.Combine(destination, Path.GetFileName(file)), true);
        foreach (string directory in Directory.GetDirectories(source))
            CopyDirectory(directory, Path.Combine(destination, Path.GetFileName(directory)));
    }
}
