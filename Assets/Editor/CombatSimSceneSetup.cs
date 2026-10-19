using System.IO;
using CombatSim;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace CombatSim.Editor
{
    public static class CombatSimSceneSetup
    {
        private const string SceneFolder = "Assets/Scenes";
        private const string ScenePath = SceneFolder + "/CombatArena.unity";
        private const string MaterialFolder = "Assets/CombatSim/Materials";

        [MenuItem("Tools/CombatSim/Create Legacy PhysX Reference Scene")]
        public static void CreateStarterScene()
        {
            if (!Application.isBatchMode
                && File.Exists(ScenePath)
                && !EditorUtility.DisplayDialog(
                    "Replace Starter Scene?",
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
            }
            AssetDatabase.Refresh();
            EnsureFolder(MaterialFolder);

            Material floorMaterial = GetOrCreateMaterial(
                "ArenaFloor.mat",
                "Arena Floor",
                new Color32(48, 57, 66, 255),
                0.12f,
                0.05f);
            Material wallMaterial = GetOrCreateMaterial(
                "ArenaWalls.mat",
                "Arena Walls",
                new Color32(102, 120, 137, 255),
                0.18f,
                0.12f);
            Material markingMaterial = GetOrCreateMaterial(
                "ArenaMarkings.mat",
                "Arena Markings",
                new Color32(111, 124, 135, 255),
                0.08f,
                0f);
            Material robotMaterial = GetOrCreateMaterial(
                "RobotChassis.mat",
                "Robot Chassis",
                new Color32(0, 174, 220, 255),
                0.25f,
                0.18f);
            Material wheelMaterial = GetOrCreateMaterial(
                "RobotWheels.mat",
                "Robot Wheels",
                new Color32(30, 34, 40, 255),
                0.2f,
                0.02f);
            Material accentMaterial = GetOrCreateMaterial(
                "RobotAccent.mat",
                "Robot Accent",
                new Color32(255, 176, 40, 255),
                0.16f,
                0.08f);
            PhysicsMaterial driveContactMaterial = GetOrCreateDriveContactMaterial();

            Scene scene = EditorSceneManager.NewScene(
                NewSceneSetup.EmptyScene,
                NewSceneMode.Single);

            GameObject arenaObject = new GameObject("Arena");
            ArenaBuilder arena = arenaObject.AddComponent<ArenaBuilder>();
            arena.floorMaterial = floorMaterial;
            arena.wallMaterial = wallMaterial;
            arena.markingMaterial = markingMaterial;
            arena.RebuildArena();

            CreateRobot(robotMaterial, wheelMaterial, accentMaterial, driveContactMaterial);
            CreateLighting();
            CreateCamera();

            EditorSceneManager.MarkSceneDirty(scene);
            if (!EditorSceneManager.SaveScene(scene, ScenePath))
            {
                throw new IOException("Could not save the starter scene to " + ScenePath);
            }

            EditorBuildSettings.scenes = new[]
            {
                new EditorBuildSettingsScene(ScenePath, true)
            };
            AssetDatabase.SaveAssets();
            Selection.activeGameObject = arenaObject;
            Debug.Log("CombatSim starter scene created at " + ScenePath);
        }

        private static void EnsureFolder(string folderPath)
        {
            string parentFolder = Path.GetDirectoryName(folderPath)?.Replace('\\', '/');
            string folderName = Path.GetFileName(folderPath);
            if (string.IsNullOrEmpty(parentFolder)
                || AssetDatabase.IsValidFolder(folderPath))
            {
                return;
            }

            EnsureFolder(parentFolder);
            AssetDatabase.CreateFolder(parentFolder, folderName);
        }

        private static Material GetOrCreateMaterial(
            string assetName,
            string materialName,
            Color color,
            float smoothness,
            float metallic)
        {
            string assetPath = MaterialFolder + "/" + assetName;
            Material material = AssetDatabase.LoadAssetAtPath<Material>(assetPath);
            if (material == null)
            {
                Shader shader = Shader.Find("Standard");
                if (shader == null)
                {
                    throw new System.InvalidOperationException(
                        "Unity's built-in Standard shader could not be found.");
                }

                material = new Material(shader);
                AssetDatabase.CreateAsset(material, assetPath);
            }

            material.name = materialName;
            material.color = color;
            material.SetFloat("_Glossiness", smoothness);
            material.SetFloat("_Metallic", metallic);
            EditorUtility.SetDirty(material);
            return material;
        }

        private static PhysicsMaterial GetOrCreateDriveContactMaterial()
        {
            const string assetPath = MaterialFolder + "/RobotDriveContact.physicMaterial";
            PhysicsMaterial driveMaterial =
                AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(assetPath);
            if (driveMaterial == null)
            {
                driveMaterial = new PhysicsMaterial("Robot Drive Contact");
                AssetDatabase.CreateAsset(driveMaterial, assetPath);
            }

            driveMaterial.name = "Robot Drive Contact";
            driveMaterial.staticFriction = 0f;
            driveMaterial.dynamicFriction = 0f;
            driveMaterial.frictionCombine = PhysicsMaterialCombine.Minimum;
            driveMaterial.bounciness = 0f;
            EditorUtility.SetDirty(driveMaterial);
            return driveMaterial;
        }

        private static void CreateRobot(
            Material chassisMaterial,
            Material wheelMaterial,
            Material accentMaterial,
            PhysicsMaterial driveContactMaterial)
        {
            GameObject robot = new GameObject("Player Robot");
            robot.transform.position = new Vector3(0f, 0.065f, 0f);

            GameObject chassis = GameObject.CreatePrimitive(PrimitiveType.Cube);
            chassis.name = "Chassis";
            chassis.transform.SetParent(robot.transform, false);
            chassis.transform.localScale = new Vector3(0.36f, 0.09f, 0.30f);
            chassis.GetComponent<Renderer>().sharedMaterial = chassisMaterial;
            chassis.GetComponent<Collider>().sharedMaterial = driveContactMaterial;

            GameObject frontMarker = GameObject.CreatePrimitive(PrimitiveType.Cube);
            frontMarker.name = "Front Marker";
            frontMarker.transform.SetParent(robot.transform, false);
            frontMarker.transform.localPosition = new Vector3(0f, 0.049f, 0.135f);
            frontMarker.transform.localScale = new Vector3(0.12f, 0.022f, 0.04f);
            frontMarker.GetComponent<Renderer>().sharedMaterial = accentMaterial;

            Rigidbody body = robot.AddComponent<Rigidbody>();
            body.mass = 5f;
            body.linearDamping = 0.3f;
            body.angularDamping = 0.8f;
            body.interpolation = RigidbodyInterpolation.Interpolate;
            body.collisionDetectionMode = CollisionDetectionMode.Continuous;
            robot.AddComponent<TankDriveController>();

            CreateWheel(robot.transform, -0.14f, -0.02f, -0.105f, wheelMaterial);
            CreateWheel(robot.transform, -0.14f, -0.02f, 0.105f, wheelMaterial);
            CreateWheel(robot.transform, 0.14f, -0.02f, -0.105f, wheelMaterial);
            CreateWheel(robot.transform, 0.14f, -0.02f, 0.105f, wheelMaterial);
        }

        private static void CreateWheel(
            Transform parent,
            float x,
            float y,
            float z,
            Material material)
        {
            GameObject wheel = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            wheel.name = "Wheel (visual)";
            wheel.transform.SetParent(parent, false);
            wheel.transform.localPosition = new Vector3(x, y, z);
            wheel.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
            wheel.transform.localScale = new Vector3(0.09f, 0.035f, 0.09f);
            wheel.GetComponent<Renderer>().sharedMaterial = material;
            Object.DestroyImmediate(wheel.GetComponent<Collider>());
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
            cameraObject.transform.position = new Vector3(0f, 3.5f, -1.65f);
            cameraObject.transform.rotation = Quaternion.LookRotation(Vector3.zero - cameraObject.transform.position);

            Camera camera = cameraObject.AddComponent<Camera>();
            camera.orthographic = true;
            camera.orthographicSize = 1.8f;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.07f, 0.09f, 0.12f);
        }
    }
}
