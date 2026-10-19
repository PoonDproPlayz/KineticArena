using System.IO;
using CombatSim;
using Mujoco;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace CombatSim.Editor
{
    public static class MujocoCombatArenaSetup
    {
        private const string ModelPath = "Assets/CombatSim/Models/CombatRobot500g.xml";
        private const string SceneFolder = "Assets/Scenes";
        private const string ScenePath = SceneFolder + "/CombatArenaMuJoCo.unity";

        [MenuItem("Tools/CombatSim/Create MuJoCo Combat Arena")]
        public static void CreateScene()
        {
            if (!Application.isBatchMode
                && File.Exists(ScenePath)
                && !EditorUtility.DisplayDialog(
                    "Replace MuJoCo Scene?",
                    ScenePath + " already exists. Replace it?",
                    "Replace",
                    "Cancel"))
            {
                return;
            }

            if (!Application.isBatchMode
                && !EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
            {
                return;
            }

            if (!Directory.Exists(SceneFolder))
            {
                Directory.CreateDirectory(SceneFolder);
                AssetDatabase.Refresh();
            }

            Scene scene = EditorSceneManager.NewScene(
                NewSceneSetup.EmptyScene,
                NewSceneMode.Single);
            GameObject robotRoot = new MjImporterWithAssets().ImportFile(
                Path.GetFullPath(ModelPath));
            if (robotRoot == null)
            {
                throw new IOException("MuJoCo could not import " + ModelPath);
            }

            robotRoot.AddComponent<MujocoRobotController>();
            CreateLighting();
            CreateCamera();

            EditorSceneManager.MarkSceneDirty(scene);
            if (!EditorSceneManager.SaveScene(scene, ScenePath))
            {
                throw new IOException("Could not save the MuJoCo scene to " + ScenePath);
            }

            EditorBuildSettings.scenes = new[]
            {
                new EditorBuildSettingsScene(ScenePath, true)
            };
            AssetDatabase.SaveAssets();
            Debug.Log("MuJoCo combat arena created at " + ScenePath);
        }

        private static void CreateLighting()
        {
            GameObject lightObject = new GameObject("Directional Light");
            Light light = lightObject.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.2f;
            lightObject.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
        }

        private static void CreateCamera()
        {
            GameObject cameraObject = new GameObject("Main Camera");
            cameraObject.tag = "MainCamera";
            // Keep the viewing ray above the near wall and make the whole roof visible.
            cameraObject.transform.position = new Vector3(0f, 2.8f, -3.5f);
            cameraObject.transform.rotation = Quaternion.LookRotation(
                new Vector3(0f, 0.15f, 0f) - cameraObject.transform.position);

            Camera camera = cameraObject.AddComponent<Camera>();
            camera.fieldOfView = 55f;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.07f, 0.09f, 0.12f);
        }
    }
}
