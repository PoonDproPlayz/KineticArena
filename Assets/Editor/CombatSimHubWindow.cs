using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using CombatSim;
using Mujoco;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace CombatSim.Editor
{
    public sealed class CombatSimHubWindow : EditorWindow
    {
        private const string ArenaScenePath = "Assets/Scenes/CombatArenaMuJoCo.unity";
        private const string RobotAssetsRoot = "Assets/CombatSim/UserRobots";
        private const string GeneratedMaterialsRoot = "Assets/CombatSim/Materials/Generated";
        private const float RetainedStockDriveAndWeaponMassKg = 0.150f;
        private enum Page { Home, RobotEditor }

        private Page page;
        private RobotDesign design;
        private Vector2 scroll;
        private float looseFileUnitToMeters = 0.001f;
        private readonly HashSet<int> collapsedBodies = new HashSet<int>();

        [MenuItem("Tools/CombatSim/Hub and Bot Editor")]
        public static void Open()
        {
            CombatSimHubWindow window = GetWindow<CombatSimHubWindow>();
            window.titleContent = new GUIContent("CombatSim");
            window.minSize = new Vector2(470f, 430f);
            window.Show();
        }

        private void OnEnable()
        {
            if (Selection.activeObject is RobotDesign selected) design = selected;
        }

        private void OnGUI()
        {
            DrawHeader();
            if (page == Page.Home) DrawHome();
            else DrawRobotEditor();
        }

        private void DrawHeader()
        {
            using (new EditorGUILayout.HorizontalScope(EditorStyles.toolbar))
            {
                if (GUILayout.Button("Home", EditorStyles.toolbarButton, GUILayout.Width(58f))) page = Page.Home;
                if (GUILayout.Button("Bot Editor", EditorStyles.toolbarButton, GUILayout.Width(78f))) page = Page.RobotEditor;
                GUILayout.FlexibleSpace();
                GUILayout.Label("MuJoCo · 2500 Hz", EditorStyles.miniLabel);
            }
            GUILayout.Space(10f);
        }

        private void DrawHome()
        {
            GUILayout.Label("CombatSim", EditorStyles.largeLabel);
            GUILayout.Label("CAD driven combat robot practice", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox(
                "Create a robot design, bring in separated Fusion 360 parts, assign material estimates and masses, then attach the design to the MuJoCo practice arena.",
                MessageType.Info);

            design = (RobotDesign)EditorGUILayout.ObjectField("Active robot design", design, typeof(RobotDesign), false);
            GUILayout.Space(8f);

            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button("New Bot Design", GUILayout.Height(32f))) CreateDesign();
                if (GUILayout.Button("Import Fusion Bundle…", GUILayout.Height(32f))) ImportFusionBundle();
            }
            if (GUILayout.Button("Open Bot Editor", GUILayout.Height(30f))) page = Page.RobotEditor;

            GUILayout.Space(14f);
            GUILayout.Label("Practice Arena", EditorStyles.boldLabel);
            if (design != null)
            {
                float totalMass = design.ImportedMassKg;
                float arenaMass = totalMass + RetainedStockDriveAndWeaponMassKg;
                EditorGUILayout.LabelField("Imported part mass", $"{totalMass * 1000f:0} g");
                EditorGUILayout.LabelField("Estimated total after apply", $"{arenaMass * 1000f:0} g / {design.weightLimitKg * 1000f:0} g");
                if (arenaMass > design.weightLimitKg)
                    EditorGUILayout.HelpBox("Imported parts plus the practice robot's stock 150 g wheels and weapon exceed this weight class. Uncheck CAD parts that duplicate those stock mechanisms or adjust masses.", MessageType.Warning);
            }

            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button("Open Arena", GUILayout.Height(34f))) OpenArena(false);
                if (GUILayout.Button("Apply Design and Play", GUILayout.Height(34f))) OpenArena(true);
            }

            GUILayout.Space(14f);
            EditorGUILayout.HelpBox(
                "Controls: W/S drive forward and reverse, A/D turn, Space spins the weapon. The arena is closed by clear walls and a roof. Use the Bot Editor to tune each part before playing.",
                MessageType.None);
        }

        private void DrawRobotEditor()
        {
            design = (RobotDesign)EditorGUILayout.ObjectField("Robot design", design, typeof(RobotDesign), false);
            if (design == null)
            {
                EditorGUILayout.HelpBox("Create a bot design or import a Fusion bundle to begin.", MessageType.Info);
                if (GUILayout.Button("Create New Bot Design")) CreateDesign();
                if (GUILayout.Button("Import Fusion Bundle…")) ImportFusionBundle();
                return;
            }

            EditorGUI.BeginChangeCheck();
            design.designName = EditorGUILayout.TextField("Name", design.designName);
            design.weightLimitKg = EditorGUILayout.FloatField("Weight limit (kg)", design.weightLimitKg);
            design.weaponMaxRpm = EditorGUILayout.FloatField("Weapon maximum RPM", design.weaponMaxRpm);
            design.weaponSpinUpSeconds = EditorGUILayout.FloatField("Weapon spin-up (s)", design.weaponSpinUpSeconds);

            float importedMassKg = design.ImportedMassKg;
            float massKg = importedMassKg + RetainedStockDriveAndWeaponMassKg;
            EditorGUILayout.LabelField("Included imported parts", $"{importedMassKg * 1000f:0.0} g");
            EditorGUILayout.LabelField("Estimated total player mass", $"{massKg * 1000f:0.0} g / {design.weightLimitKg * 1000f:0} g");
            Rect massRect = GUILayoutUtility.GetRect(10f, 14f);
            EditorGUI.ProgressBar(massRect, design.weightLimitKg > 0f ? Mathf.Clamp01(massKg / design.weightLimitKg) : 0f,
                $"{massKg * 1000f:0} g");
            if (massKg > design.weightLimitKg)
                EditorGUILayout.HelpBox("Over weight class. The practice robot keeps its stock wheels and weapon (150 g combined); uncheck imported parts that duplicate them or adjust the body mass.", MessageType.Warning);

            GUILayout.Space(8f);
            DrawImportControls();
            scroll = EditorGUILayout.BeginScrollView(scroll);
            for (int bodyIndex = 0; bodyIndex < design.bodies.Count; bodyIndex++)
            {
                RobotBodyDefinition body = design.bodies[bodyIndex];
                if (body == null) continue;
                DrawBody(body, bodyIndex);
            }
            EditorGUILayout.EndScrollView();

            if (GUILayout.Button("+ Add Fixed Body", GUILayout.Height(26f)))
            {
                Undo.RecordObject(design, "Add robot body");
                design.bodies.Add(new RobotBodyDefinition { bodyName = "Extra Body" });
                MarkDesignDirty();
            }
            GUILayout.Space(6f);
            if (GUILayout.Button("Apply Design to Practice Arena")) OpenArena(true);

            if (EditorGUI.EndChangeCheck()) MarkDesignDirty();
        }

        private void DrawImportControls()
        {
            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button("Import Fusion Bundle…")) ImportFusionBundle();
                if (GUILayout.Button("Add STL / OBJ Part…")) ImportStandalonePart();
            }
            looseFileUnitToMeters = EditorGUILayout.FloatField("Loose file unit → metres", looseFileUnitToMeters);
            EditorGUILayout.HelpBox(
                "The Fusion bundle stores part names, transforms, assigned Fusion material, mass and density. STL/OBJ alone has no material or mass metadata, so enter those below.",
                MessageType.None);
        }

        private void DrawBody(RobotBodyDefinition body, int index)
        {
            using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
            {
                using (new EditorGUILayout.HorizontalScope())
                {
                    bool expanded = !collapsedBodies.Contains(index);
                    bool nextExpanded = EditorGUILayout.Foldout(expanded, $"{body.bodyName}  ·  {body.parts.Count} part(s)", true, EditorStyles.foldoutHeader);
                    if (nextExpanded) collapsedBodies.Remove(index); else collapsedBodies.Add(index);
                    if (GUILayout.Button("×", GUILayout.Width(24f)))
                    {
                        Undo.RecordObject(design, "Remove robot body");
                        design.bodies.RemoveAt(index);
                        MarkDesignDirty();
                        return;
                    }
                }

                if (collapsedBodies.Contains(index)) return;
                body.bodyName = EditorGUILayout.TextField("Body name", body.bodyName);
                body.localPositionMeters = EditorGUILayout.Vector3Field("Position (m)", body.localPositionMeters);
                body.localRotation = Quaternion.Euler(EditorGUILayout.Vector3Field("Rotation (degrees)", body.localRotation.eulerAngles));
                body.useMassOverride = EditorGUILayout.Toggle("Override total body mass", body.useMassOverride);
                if (body.useMassOverride)
                    body.massOverrideKg = EditorGUILayout.FloatField("Body mass (kg)", body.massOverrideKg);
                EditorGUILayout.LabelField("Attachment", "Fixed to chassis");

                for (int partIndex = 0; partIndex < body.parts.Count; partIndex++)
                {
                    RobotPartDefinition part = body.parts[partIndex];
                    if (part == null) continue;
                    DrawPart(body, part, partIndex);
                }

                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button("+ Add STL / OBJ Part")) ImportStandalonePart(body);
                    if (GUILayout.Button("Remove Empty Parts"))
                    {
                        Undo.RecordObject(design, "Remove empty robot parts");
                        body.parts.RemoveAll(part => part == null || part.mesh == null);
                        MarkDesignDirty();
                    }
                }
            }
        }

        private void DrawPart(RobotBodyDefinition body, RobotPartDefinition part, int index)
        {
            using (new EditorGUILayout.VerticalScope("box"))
            {
                using (new EditorGUILayout.HorizontalScope())
                {
                    part.includeInSimulation = EditorGUILayout.Toggle(part.includeInSimulation, GUILayout.Width(18f));
                    part.partName = EditorGUILayout.TextField(part.partName);
                    if (GUILayout.Button("×", GUILayout.Width(24f)))
                    {
                        Undo.RecordObject(design, "Remove robot part");
                        body.parts.RemoveAt(index);
                        MarkDesignDirty();
                        return;
                    }
                }
                EditorGUILayout.ObjectField("Mesh", part.mesh, typeof(Mesh), false);
                int current = RobotMaterialCatalog.Find(part.materialName);
                int selected = EditorGUILayout.Popup("Material", current, RobotMaterialCatalog.Names);
                if (selected != current) RobotMaterialCatalog.ApplyPreset(part, selected);
                if (RobotMaterialCatalog.Find(part.materialName) == 2)
                    part.materialName = EditorGUILayout.TextField("Tool steel grade", part.materialName);
                part.densityKgPerM3 = EditorGUILayout.FloatField("Density (kg/m³)", part.densityKgPerM3);
                part.slidingFrictionEstimate = EditorGUILayout.FloatField("Sliding friction estimate", part.slidingFrictionEstimate);
                part.useMassOverride = EditorGUILayout.Toggle("Override part mass", part.useMassOverride);
                part.massKg = EditorGUILayout.FloatField("CAD mass (kg)", part.massKg);
                if (part.useMassOverride) part.massOverrideKg = EditorGUILayout.FloatField("Part mass (kg)", part.massOverrideKg);
                part.localPositionMeters = EditorGUILayout.Vector3Field("Position (m)", part.localPositionMeters);
                part.localRotation = Quaternion.Euler(EditorGUILayout.Vector3Field("Rotation (degrees)", part.localRotation.eulerAngles));
                if (!string.IsNullOrEmpty(part.sourceFile)) EditorGUILayout.LabelField("Source", part.sourceFile, EditorStyles.miniLabel);
            }
        }

        private void CreateDesign()
        {
            string path = EditorUtility.SaveFilePanelInProject("Create bot design", "NewBotDesign", "asset", "Choose where to save this robot design.", RobotAssetsRoot);
            if (string.IsNullOrEmpty(path)) return;
            EnsureAssetFolder(Path.GetDirectoryName(path).Replace('\\', '/'));
            RobotDesign created = CreateInstance<RobotDesign>();
            created.designName = Path.GetFileNameWithoutExtension(path);
            created.bodies.Add(new RobotBodyDefinition { bodyName = "Main Chassis" });
            AssetDatabase.CreateAsset(created, path);
            AssetDatabase.SaveAssets();
            design = created;
            page = Page.RobotEditor;
            Selection.activeObject = design;
        }

        private void ImportFusionBundle()
        {
            string path = EditorUtility.OpenFilePanel("Import Fusion 360 MuJoCo bundle", "", "json");
            if (string.IsNullOrEmpty(path)) return;
            try
            {
                FusionBundleManifest bundle = JsonUtility.FromJson<FusionBundleManifest>(File.ReadAllText(path));
                if (bundle == null || bundle.parts == null || bundle.parts.Length == 0)
                    throw new InvalidDataException("The selected JSON has no parts array.");

                string bundleFolder = Path.GetDirectoryName(path);
                List<string> missingMeshes = new List<string>();
                foreach (FusionBundlePart sourcePart in bundle.parts)
                {
                    if (sourcePart == null || string.IsNullOrEmpty(sourcePart.file)) continue;
                    string relativeMeshFile = sourcePart.file.Replace('/', Path.DirectorySeparatorChar);
                    string expectedMeshFile = Path.GetFullPath(Path.Combine(bundleFolder, relativeMeshFile));
                    if (!File.Exists(expectedMeshFile))
                    {
                        string partLabel = string.IsNullOrEmpty(sourcePart.partName)
                            ? sourcePart.file
                            : sourcePart.partName + " (" + sourcePart.file + ")";
                        missingMeshes.Add((string.IsNullOrEmpty(sourcePart.bodyName) ? "Body" : sourcePart.bodyName)
                            + ": " + partLabel);
                    }
                }
                if (missingMeshes.Count > 0)
                {
                    throw new FileNotFoundException(
                        "Fusion bundle is incomplete. The manifest references " + missingMeshes.Count
                        + " STL mesh file(s) that are not present beside it. Make sure the Parts folder "
                        + "from the Fusion export is beside manifest.json and contains every referenced STL.\n\n"
                        + string.Join("\n", missingMeshes));
                }

                string designFolderName = Sanitize(string.IsNullOrEmpty(bundle.designName) ? Path.GetFileNameWithoutExtension(path) : bundle.designName);
                string assetFolder = UniqueFolder(RobotAssetsRoot + "/" + designFolderName);
                EnsureAssetFolder(assetFolder + "/Meshes");
                RobotDesign imported = CreateInstance<RobotDesign>();
                imported.designName = string.IsNullOrEmpty(bundle.designName) ? designFolderName : bundle.designName;
                imported.weaponMaxRpm = bundle.weaponMaxRpm > 0 ? bundle.weaponMaxRpm : 30000f;
                imported.weaponSpinUpSeconds = bundle.weaponSpinUpSeconds > 0 ? bundle.weaponSpinUpSeconds : 1.5f;
                Dictionary<string, RobotBodyDefinition> bodies = new Dictionary<string, RobotBodyDefinition>(StringComparer.OrdinalIgnoreCase);

                foreach (FusionBundlePart sourcePart in bundle.parts)
                {
                    if (sourcePart == null || string.IsNullOrEmpty(sourcePart.file)) continue;
                    string relativeMeshFile = sourcePart.file.Replace('/', Path.DirectorySeparatorChar);
                    string meshFile = Path.GetFullPath(Path.Combine(bundleFolder, relativeMeshFile));
                    string bodyName = string.IsNullOrEmpty(sourcePart.bodyName) ? "Main Chassis" : sourcePart.bodyName;
                    if (!bodies.TryGetValue(bodyName, out RobotBodyDefinition body))
                    {
                        body = new RobotBodyDefinition { bodyName = bodyName };
                        if (sourcePart.positionMeters != null && sourcePart.positionMeters.Length >= 3)
                            body.localPositionMeters = new Vector3(sourcePart.positionMeters[0], sourcePart.positionMeters[1], sourcePart.positionMeters[2]);
                        if (sourcePart.rotationUnity != null && sourcePart.rotationUnity.Length >= 4)
                            body.localRotation = new Quaternion(sourcePart.rotationUnity[0], sourcePart.rotationUnity[1], sourcePart.rotationUnity[2], sourcePart.rotationUnity[3]);
                        bodies.Add(bodyName, body);
                        imported.bodies.Add(body);
                    }

                    float meshScale = sourcePart.unitToMeters > 0f ? sourcePart.unitToMeters : (bundle.unitToMeters > 0f ? bundle.unitToMeters : 0.001f);
                    Mesh mesh = LoadCadMesh(meshFile, meshScale);
                    mesh.name = Sanitize(sourcePart.partName ?? Path.GetFileNameWithoutExtension(meshFile));
                    string meshAsset = AssetDatabase.GenerateUniqueAssetPath(assetFolder + "/Meshes/" + mesh.name + ".asset");
                    AssetDatabase.CreateAsset(mesh, meshAsset);
                    RobotPartDefinition part = new RobotPartDefinition
                    {
                        partName = string.IsNullOrEmpty(sourcePart.partName) ? mesh.name : sourcePart.partName,
                        mesh = mesh,
                        sourceFile = Path.GetFileName(meshFile),
                        massKg = Mathf.Max(0f, sourcePart.massKg),
                        densityKgPerM3 = sourcePart.densityKgPerM3 > 0f ? sourcePart.densityKgPerM3 : 1000f,
                        localPositionMeters = Vector3.zero,
                        localRotation = Quaternion.identity
                    };
                    int presetIndex = RobotMaterialCatalog.Find(sourcePart.materialName);
                    RobotMaterialCatalog.ApplyPreset(part, presetIndex);
                    if (!string.IsNullOrEmpty(sourcePart.materialName))
                        part.materialName = sourcePart.materialName;
                    if (sourcePart.densityKgPerM3 > 0f) part.densityKgPerM3 = sourcePart.densityKgPerM3;
                    part.slidingFrictionEstimate = sourcePart.frictionEstimate > 0f
                        ? sourcePart.frictionEstimate
                        : RobotMaterialCatalog.FrictionFor(part.materialName);
                    body.parts.Add(part);
                }

                if (imported.bodies.Count == 0) throw new InvalidDataException("The bundle did not contain any readable part meshes.");
                string designPath = AssetDatabase.GenerateUniqueAssetPath(assetFolder + "/" + Sanitize(imported.designName) + ".asset");
                AssetDatabase.CreateAsset(imported, designPath);
                AssetDatabase.SaveAssets();
                AssetDatabase.Refresh();
                design = imported;
                page = Page.RobotEditor;
                Selection.activeObject = design;
                Debug.Log($"Imported Fusion bundle '{design.designName}' with {bundle.parts.Length} CAD parts.", design);
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
                EditorUtility.DisplayDialog("CAD import failed", exception.Message, "OK");
            }
        }

        private void ImportStandalonePart(RobotBodyDefinition targetBody = null)
        {
            string path = EditorUtility.OpenFilePanelWithFilters("Import CAD mesh", "", new[] { "STL / OBJ", "stl,obj", "STL", "stl", "OBJ", "obj" });
            if (string.IsNullOrEmpty(path)) return;
            try
            {
                if (design == null) throw new InvalidOperationException("Select or create a robot design first.");
                string folder = UniqueFolder(RobotAssetsRoot + "/" + Sanitize(design.designName));
                EnsureAssetFolder(folder + "/Meshes");
                Mesh mesh = LoadCadMesh(path, Mathf.Max(0.000001f, looseFileUnitToMeters));
                mesh.name = Sanitize(Path.GetFileNameWithoutExtension(path));
                string meshPath = AssetDatabase.GenerateUniqueAssetPath(folder + "/Meshes/" + mesh.name + ".asset");
                AssetDatabase.CreateAsset(mesh, meshPath);
                if (targetBody == null)
                {
                    if (design.bodies.Count == 0) design.bodies.Add(new RobotBodyDefinition { bodyName = "Main Chassis" });
                    targetBody = design.bodies[0];
                }
                RobotPartDefinition part = new RobotPartDefinition
                {
                    partName = mesh.name,
                    mesh = mesh,
                    sourceFile = Path.GetFileName(path),
                    massKg = 0f
                };
                RobotMaterialCatalog.ApplyPreset(part, RobotMaterialCatalog.Names.Length - 1);
                Undo.RecordObject(design, "Import CAD mesh");
                targetBody.parts.Add(part);
                MarkDesignDirty();
                AssetDatabase.SaveAssets();
                Selection.activeObject = design;
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
                EditorUtility.DisplayDialog("CAD import failed", exception.Message, "OK");
            }
        }

        private void OpenArena(bool apply)
        {
            if (!File.Exists(ArenaScenePath))
            {
                EditorUtility.DisplayDialog("Arena scene missing", "Create or restore Assets/Scenes/CombatArenaMuJoCo.unity first.", "OK");
                return;
            }
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            Scene scene = EditorSceneManager.OpenScene(ArenaScenePath, OpenSceneMode.Single);
            if (apply && design != null) ApplyDesign(scene, design);
            if (apply) EditorSceneManager.SaveScene(scene);
            page = Page.Home;
            if (apply) EditorApplication.isPlaying = true;
        }

        private static void ApplyDesign(Scene scene, RobotDesign robotDesign)
        {
            GameObject robotRoot = null;
            foreach (GameObject root in scene.GetRootGameObjects())
            {
                foreach (MjBody body in root.GetComponentsInChildren<MjBody>(true))
                    if (body.gameObject.name == "Player_Robot") robotRoot = body.gameObject;
            }
            if (robotRoot == null) throw new InvalidOperationException("The practice scene has no Player_Robot MuJoCo body.");

            CombatSimDesignAttachment attachment = robotRoot.GetComponent<CombatSimDesignAttachment>();
            if (attachment == null) attachment = Undo.AddComponent<CombatSimDesignAttachment>(robotRoot);
            if (attachment.stockChassis == null)
            {
                foreach (MjGeom geom in robotRoot.GetComponentsInChildren<MjGeom>(true))
                    if (geom.gameObject.name == "chassis") { attachment.stockChassis = geom.gameObject; break; }
                if (attachment.stockChassis != null) attachment.stockChassisWasActive = attachment.stockChassis.activeSelf;
            }

            Transform old = robotRoot.transform.Find("Imported Robot Design");
            if (old != null) Undo.DestroyObjectImmediate(old.gameObject);
            if (attachment.stockChassis != null) attachment.stockChassis.SetActive(false);

            GameObject designRoot = new GameObject("Imported Robot Design");
            Undo.RegisterCreatedObjectUndo(designRoot, "Apply CAD robot design");
            designRoot.transform.SetParent(robotRoot.transform, false);

            foreach (RobotBodyDefinition body in robotDesign.bodies)
            {
                if (body == null) continue;
                List<RobotPartDefinition> included = new List<RobotPartDefinition>();
                foreach (RobotPartDefinition part in body.parts)
                    if (part != null && part.includeInSimulation && part.mesh != null) included.Add(part);
                if (included.Count == 0) continue;

                GameObject bodyObject = new GameObject(string.IsNullOrEmpty(body.bodyName) ? "Fixed CAD Body" : body.bodyName);
                bodyObject.transform.SetParent(designRoot.transform, false);
                bodyObject.transform.localPosition = body.localPositionMeters;
                bodyObject.transform.localRotation = body.localRotation;
                bodyObject.AddComponent<MjBody>();

                float rawTotal = 0f;
                foreach (RobotPartDefinition part in included) rawTotal += part.EstimatedMassKg;
                for (int i = 0; i < included.Count; i++)
                {
                    RobotPartDefinition part = included[i];
                    GameObject partObject = new GameObject(string.IsNullOrEmpty(part.partName) ? "CAD Part" : part.partName);
                    partObject.transform.SetParent(bodyObject.transform, false);
                    partObject.transform.localPosition = part.localPositionMeters;
                    partObject.transform.localRotation = part.localRotation;
                    MeshFilter meshFilter = partObject.AddComponent<MeshFilter>();
                    meshFilter.sharedMesh = part.mesh;
                    MeshRenderer renderer = partObject.AddComponent<MeshRenderer>();
                    renderer.sharedMaterial = MaterialFor(part.materialName);

                    MjGeom geom = partObject.AddComponent<MjGeom>();
                    geom.ShapeType = MjShapeComponent.ShapeTypes.Mesh;
                    geom.Mesh.Mesh = part.mesh;
                    geom.Density = Mathf.Max(0f, part.densityKgPerM3);
                    if (body.useMassOverride)
                    {
                        float assignedMass = rawTotal > 0f
                            ? body.massOverrideKg * part.EstimatedMassKg / rawTotal
                            : (i == 0 ? body.massOverrideKg : 0f);
                        geom.Mass = Mathf.Max(0f, assignedMass);
                    }
                    else
                    {
                        geom.Mass = Mathf.Max(0f, part.SimulationMassKg);
                    }
                    MjGeomSettings settings = MjGeomSettings.Default;
                    // Give imported material surfaces priority over the arena's
                    // generic floor/wall geoms so their friction estimate is used.
                    settings.Priority = 1;
                    settings.Friction = new GeomFriction
                    {
                        Sliding = Mathf.Max(0f, part.slidingFrictionEstimate),
                        Torsional = 0.02f,
                        Rolling = 0.001f
                    };
                    geom.Settings = settings;
                    partObject.AddComponent<MjMeshFilter>();
                }
            }

            robotDesign.weaponMaxRpm = Mathf.Max(0f, robotDesign.weaponMaxRpm);
            robotDesign.weaponSpinUpSeconds = Mathf.Max(0.01f, robotDesign.weaponSpinUpSeconds);
            MujocoRobotController controller = robotRoot.GetComponent<MujocoRobotController>();
            if (controller == null) controller = robotRoot.GetComponentInParent<MujocoRobotController>();
            if (controller != null)
            {
                controller.weaponSpeedRpm = robotDesign.weaponMaxRpm;
                controller.weaponSpinUpSeconds = robotDesign.weaponSpinUpSeconds;
                EditorUtility.SetDirty(controller);
            }
            EditorUtility.SetDirty(robotRoot);
        }

        private static Material MaterialFor(string materialName)
        {
            EnsureAssetFolder(GeneratedMaterialsRoot);
            string assetName = Sanitize(materialName);
            string path = GeneratedMaterialsRoot + "/" + assetName + ".mat";
            Material material = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (material != null) return material;
            Shader shader = Shader.Find("Standard");
            if (shader == null) shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null) shader = Shader.Find("Sprites/Default");
            material = new Material(shader) { name = assetName };
            if (material.HasProperty("_Color")) material.color = RobotMaterialCatalog.ColorFor(materialName);
            AssetDatabase.CreateAsset(material, path);
            return material;
        }

        private static Mesh LoadCadMesh(string path, float unitToMeters)
        {
            string extension = Path.GetExtension(path).ToLowerInvariant();
            Mesh mesh = extension == ".obj" ? ParseObj(path, unitToMeters) :
                extension == ".stl" ? ParseStl(path, unitToMeters) :
                throw new InvalidDataException("Only STL and OBJ mesh files are supported in this version.");
            if (mesh.vertexCount == 0 || mesh.triangles.Length < 3)
                throw new InvalidDataException("The selected CAD mesh did not contain any triangles.");
            mesh.RecalculateBounds();
            mesh.RecalculateNormals();
            return mesh;
        }

        private static Mesh ParseStl(string path, float scale)
        {
            byte[] bytes = File.ReadAllBytes(path);
            List<Vector3> vertices = new List<Vector3>();
            List<int> triangles = new List<int>();
            bool binary = false;
            if (bytes.Length >= 84)
            {
                uint triangleCount = BitConverter.ToUInt32(bytes, 80);
                binary = 84L + 50L * triangleCount == bytes.Length;
            }
            if (binary)
            {
                using (BinaryReader reader = new BinaryReader(new MemoryStream(bytes)))
                {
                    reader.BaseStream.Position = 84;
                    uint count = BitConverter.ToUInt32(bytes, 80);
                    for (uint i = 0; i < count; i++)
                    {
                        reader.ReadBytes(12); // stored facet normal
                        Vector3 a = ReadStlVertex(reader, scale);
                        Vector3 b = ReadStlVertex(reader, scale);
                        Vector3 c = ReadStlVertex(reader, scale);
                        AddTriangle(vertices, triangles, a, c, b);
                        reader.ReadUInt16();
                    }
                }
            }
            else
            {
                string[] lines = File.ReadAllLines(path);
                List<Vector3> facet = new List<Vector3>(3);
                foreach (string line in lines)
                {
                    string trimmed = line.Trim();
                    if (!trimmed.StartsWith("vertex", StringComparison.OrdinalIgnoreCase)) continue;
                    string[] values = trimmed.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                    if (values.Length < 4) continue;
                    facet.Add(ConvertCadVertex(ParseFloat(values[1]), ParseFloat(values[2]), ParseFloat(values[3]), scale));
                    if (facet.Count == 3)
                    {
                        AddTriangle(vertices, triangles, facet[0], facet[2], facet[1]);
                        facet.Clear();
                    }
                }
            }
            return MakeMesh(Path.GetFileNameWithoutExtension(path), vertices, triangles);
        }

        private static Vector3 ReadStlVertex(BinaryReader reader, float scale)
        {
            float x = reader.ReadSingle();
            float y = reader.ReadSingle();
            float z = reader.ReadSingle();
            return ConvertCadVertex(x, y, z, scale);
        }

        private static Mesh ParseObj(string path, float scale)
        {
            List<Vector3> sourceVertices = new List<Vector3>();
            List<Vector3> vertices = new List<Vector3>();
            List<int> triangles = new List<int>();
            foreach (string line in File.ReadLines(path))
            {
                string trimmed = line.Trim();
                if (trimmed.StartsWith("v ", StringComparison.Ordinal))
                {
                    string[] values = trimmed.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                    if (values.Length >= 4)
                        sourceVertices.Add(ConvertCadVertex(ParseFloat(values[1]), ParseFloat(values[2]), ParseFloat(values[3]), scale));
                }
                else if (trimmed.StartsWith("f ", StringComparison.Ordinal))
                {
                    string[] values = trimmed.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                    if (values.Length < 4) continue;
                    int[] face = new int[values.Length - 1];
                    for (int i = 1; i < values.Length; i++)
                    {
                        string indexText = values[i].Split('/')[0];
                        int sourceIndex = int.Parse(indexText, CultureInfo.InvariantCulture);
                        face[i - 1] = sourceIndex > 0 ? sourceIndex - 1 : sourceVertices.Count + sourceIndex;
                    }
                    for (int i = 1; i < face.Length - 1; i++)
                        AddTriangle(vertices, triangles, sourceVertices[face[0]], sourceVertices[face[i + 1]], sourceVertices[face[i]]);
                }
            }
            return MakeMesh(Path.GetFileNameWithoutExtension(path), vertices, triangles);
        }

        private static Mesh MakeMesh(string name, List<Vector3> vertices, List<int> triangles)
        {
            Mesh mesh = new Mesh { name = Sanitize(name) };
            if (vertices.Count > 65535) mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangles, 0);
            return mesh;
        }

        // Fusion and MuJoCo use z-up; Unity uses y-up. The axis swap is a
        // reflection, so the triangle winding is reversed when building meshes.
        private static Vector3 ConvertCadVertex(float x, float y, float z, float scale) => new Vector3(x * scale, z * scale, y * scale);
        private static void AddTriangle(List<Vector3> vertices, List<int> triangles, Vector3 a, Vector3 b, Vector3 c)
        {
            int start = vertices.Count;
            vertices.Add(a); vertices.Add(b); vertices.Add(c);
            triangles.Add(start); triangles.Add(start + 1); triangles.Add(start + 2);
        }
        private static float ParseFloat(string value) => float.Parse(value, CultureInfo.InvariantCulture);

        private void MarkDesignDirty()
        {
            if (design == null) return;
            EditorUtility.SetDirty(design);
            AssetDatabase.SaveAssets();
        }

        private static void EnsureAssetFolder(string path)
        {
            path = path.Replace('\\', '/');
            if (AssetDatabase.IsValidFolder(path)) return;
            string parent = Path.GetDirectoryName(path).Replace('\\', '/');
            string leaf = Path.GetFileName(path);
            if (!AssetDatabase.IsValidFolder(parent)) EnsureAssetFolder(parent);
            AssetDatabase.CreateFolder(parent, leaf);
        }

        private static string UniqueFolder(string desired)
        {
            string path = desired;
            int suffix = 2;
            while (AssetDatabase.IsValidFolder(path)) path = desired + " " + suffix++;
            return path;
        }

        private static string Sanitize(string value)
        {
            if (string.IsNullOrWhiteSpace(value)) return "Robot";
            foreach (char invalid in Path.GetInvalidFileNameChars()) value = value.Replace(invalid, '_');
            return value.Replace('/', '_').Replace('\\', '_').Trim();
        }

        [Serializable]
        private sealed class FusionBundleManifest
        {
            public int schemaVersion = 1;
            public string designName;
            public float unitToMeters;
            public float weaponMaxRpm;
            public float weaponSpinUpSeconds;
            public FusionBundlePart[] parts;
        }

        [Serializable]
        private sealed class FusionBundlePart
        {
            public string bodyName;
            public string partName;
            public string file;
            public string materialName;
            public float densityKgPerM3;
            public float massKg;
            public float frictionEstimate;
            public float unitToMeters;
            public float[] positionMeters;
            public float[] rotationUnity;
        }
    }
}
