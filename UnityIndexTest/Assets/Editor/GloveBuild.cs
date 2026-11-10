using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

// Editor-only (Assets/Editor is never included in the game).
// Menu: Glove > Build Windows player to Desktop
// Command line (Unity closed): Unity.exe -batchmode -projectPath <project> -executeMethod GloveBuild.BuildToDesktop
public static class GloveBuild
{
    // The hand and ball are made at runtime, so no scene asset references their shader and the
    // build would strip it (pink). Keep it in Graphics > Always Included Shaders.
    static readonly string[] RuntimeShaders = {"Legacy Shaders/Diffuse"};

    static void EnsureRuntimeShadersIncluded()
    {
        UnityEngine.Object[] loaded = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/GraphicsSettings.asset");
        UnityEngine.Object settings = loaded != null && loaded.Length > 0 ? loaded[0] : null;
        if (settings == null) { Debug.LogWarning("Glove build: GraphicsSettings.asset not found"); return; }
        var so = new SerializedObject(settings);
        SerializedProperty list = so.FindProperty("m_AlwaysIncludedShaders");
        foreach (string name in RuntimeShaders) {
            Shader shader = Shader.Find(name);
            if (shader == null) { Debug.LogWarning("Glove build: shader not found: " + name); continue; }
            bool present = false;
            for (int i = 0; i < list.arraySize; i++)
                if (list.GetArrayElementAtIndex(i).objectReferenceValue == shader) present = true;
            if (present) continue;
            list.InsertArrayElementAtIndex(list.arraySize);
            list.GetArrayElementAtIndex(list.arraySize - 1).objectReferenceValue = shader;
            Debug.Log("Glove build: added " + name + " to Always Included Shaders");
        }
        so.ApplyModifiedProperties();
        AssetDatabase.SaveAssets();
    }

    [MenuItem("Glove/Build Windows player to Desktop")]
    public static void BuildToDesktop()
    {
        EnsureRuntimeShadersIncluded();
        string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), "HapticGlove_Build");
        var options = new BuildPlayerOptions {
            scenes = new[] {"Assets/Scenes/SampleScene.unity"},
            locationPathName = Path.Combine(dir, PlayerSettings.productName + ".exe"),
            target = BuildTarget.StandaloneWindows64,
            options = BuildOptions.None
        };
        BuildReport report = BuildPipeline.BuildPlayer(options);
        Debug.Log("Glove build: " + report.summary.result + " -> " + dir + " (" + report.summary.totalErrors + " errors)");
        if (Application.isBatchMode) EditorApplication.Exit(report.summary.result == BuildResult.Succeeded ? 0 : 1);
        else if (report.summary.result == BuildResult.Succeeded) EditorUtility.RevealInFinder(options.locationPathName);
    }
}
