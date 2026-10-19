using System;
using System.Collections.Generic;
using UnityEngine;

namespace CombatSim
{
    [CreateAssetMenu(menuName = "CombatSim/Robot Design", fileName = "RobotDesign")]
    public sealed class RobotDesign : ScriptableObject
    {
        public string designName = "New 500 g Bot";
        [Min(0.01f)] public float weightLimitKg = 0.5f;
        [Min(0f)] public float weaponMaxRpm = 30000f;
        [Min(0.01f)] public float weaponSpinUpSeconds = 1.5f;
        public List<RobotBodyDefinition> bodies = new List<RobotBodyDefinition>();

        public float ImportedMassKg
        {
            get
            {
                float total = 0f;
                foreach (RobotBodyDefinition body in bodies)
                {
                    if (body == null) continue;
                    bool hasIncludedPart = false;
                    foreach (RobotPartDefinition part in body.parts)
                        if (part != null && part.includeInSimulation && part.mesh != null) { hasIncludedPart = true; break; }
                    if (body.useMassOverride)
                    {
                        if (hasIncludedPart) total += Mathf.Max(0f, body.massOverrideKg);
                        continue;
                    }

                    foreach (RobotPartDefinition part in body.parts)
                    {
                        if (part == null || !part.includeInSimulation) continue;
                        total += part.EstimatedMassKg;
                    }
                }

                return total;
            }
        }
    }

    [Serializable]
    public sealed class RobotBodyDefinition
    {
        public string bodyName = "Chassis";
        public Vector3 localPositionMeters;
        public Quaternion localRotation = Quaternion.identity;
        [Tooltip("When enabled, this total mass is divided among included parts by their CAD mass; if no CAD masses exist it is assigned to the first part.")]
        public bool useMassOverride;
        [Min(0f)] public float massOverrideKg;
        [Tooltip("A fixed MuJoCo body keeps this group rigidly attached to the robot chassis. Independent joints can be assigned in a later editor pass.")]
        public List<RobotPartDefinition> parts = new List<RobotPartDefinition>();
    }

    [Serializable]
    public sealed class RobotPartDefinition
    {
        private static readonly Dictionary<Mesh, float> MeshVolumeCache = new Dictionary<Mesh, float>();
        public string partName = "Part";
        public Mesh mesh;
        public string sourceFile;
        public string materialName = "Unknown / Custom";
        [Min(0f)] public float densityKgPerM3 = 1000f;
        [Min(0f)] public float slidingFrictionEstimate = 0.6f;
        public Vector3 localPositionMeters;
        public Quaternion localRotation = Quaternion.identity;
        public bool includeInSimulation = true;
        public bool useMassOverride;
        [Min(0f)] public float massKg;
        [Min(0f)] public float massOverrideKg;

        public float SimulationMassKg => useMassOverride
            ? Mathf.Max(0f, massOverrideKg)
            : Mathf.Max(0f, massKg);

        public float EstimatedMassKg
        {
            get
            {
                if (useMassOverride) return Mathf.Max(0f, massOverrideKg);
                if (massKg > 0f) return massKg;
                if (mesh == null || densityKgPerM3 <= 0f) return 0f;
                if (!MeshVolumeCache.TryGetValue(mesh, out float volume))
                {
                    Vector3[] vertices = mesh.vertices;
                    int[] triangles = mesh.triangles;
                    double signedVolumeTimesSix = 0.0;
                    for (int i = 0; i + 2 < triangles.Length; i += 3)
                    {
                        Vector3 a = vertices[triangles[i]];
                        Vector3 b = vertices[triangles[i + 1]];
                        Vector3 c = vertices[triangles[i + 2]];
                        signedVolumeTimesSix += Vector3.Dot(a, Vector3.Cross(b, c));
                    }
                    volume = Mathf.Abs((float)(signedVolumeTimesSix / 6.0));
                    MeshVolumeCache[mesh] = volume;
                }
                return volume * densityKgPerM3;
            }
        }
    }

    public static class RobotMaterialCatalog
    {
        // Density and surface friction are editable starting estimates. Friction is
        // a contact-pair property in real life and depends on finish, load and wear.
        public static readonly string[] Names =
        {
            "UHMW polyethylene",
            "7075 aluminum",
            "Tool steel (select grade in notes)",
            "PA12 nylon",
            "Nylon (generic)",
            "ABS",
            "TPU 95A",
            "Platinum silicone 40A",
            "Platinum silicone 20A",
            "Carbon fiber composite",
            "Unknown / Custom"
        };

        private static readonly float[] Densities =
        {
            930f, 2810f, 7800f, 1010f, 1150f, 1040f, 1200f, 1200f, 1100f, 1600f, 1000f
        };

        private static readonly float[] Frictions =
        {
            0.20f, 0.50f, 0.55f, 0.28f, 0.30f, 0.35f, 0.80f, 0.90f, 0.95f, 0.35f, 0.60f
        };

        private static readonly Color[] Colors =
        {
            new Color(0.82f, 0.87f, 0.86f), new Color(0.67f, 0.72f, 0.76f),
            new Color(0.40f, 0.43f, 0.47f), new Color(0.84f, 0.82f, 0.71f),
            new Color(0.82f, 0.78f, 0.65f), new Color(0.30f, 0.31f, 0.34f),
            new Color(0.18f, 0.28f, 0.42f), new Color(0.82f, 0.75f, 0.75f),
            new Color(0.72f, 0.63f, 0.65f), new Color(0.10f, 0.12f, 0.14f),
            new Color(0.48f, 0.52f, 0.57f)
        };

        public static int Find(string materialName)
        {
            for (int i = 0; i < Names.Length; i++)
                if (string.Equals(Names[i], materialName, StringComparison.OrdinalIgnoreCase)) return i;
            if (materialName != null && materialName.IndexOf("steel", StringComparison.OrdinalIgnoreCase) >= 0) return 2;
            if (materialName != null && materialName.IndexOf("7075", StringComparison.OrdinalIgnoreCase) >= 0) return 1;
            if (materialName != null && materialName.IndexOf("uhmw", StringComparison.OrdinalIgnoreCase) >= 0) return 0;
            if (materialName != null && materialName.IndexOf("pa12", StringComparison.OrdinalIgnoreCase) >= 0) return 3;
            if (materialName != null && materialName.IndexOf("nylon", StringComparison.OrdinalIgnoreCase) >= 0) return 4;
            if (materialName != null && materialName.IndexOf("abs", StringComparison.OrdinalIgnoreCase) >= 0) return 5;
            if (materialName != null && materialName.IndexOf("tpu", StringComparison.OrdinalIgnoreCase) >= 0) return 6;
            if (materialName != null && materialName.IndexOf("silicone", StringComparison.OrdinalIgnoreCase) >= 0)
                return materialName.IndexOf("20", StringComparison.OrdinalIgnoreCase) >= 0 ? 8 : 7;
            if (materialName != null && (materialName.IndexOf("carbon", StringComparison.OrdinalIgnoreCase) >= 0 ||
                materialName.IndexOf("composite", StringComparison.OrdinalIgnoreCase) >= 0)) return 9;
            return Names.Length - 1;
        }

        public static void ApplyPreset(RobotPartDefinition part, int index)
        {
            if (part == null) return;
            index = Mathf.Clamp(index, 0, Names.Length - 1);
            part.materialName = Names[index];
            part.densityKgPerM3 = Densities[index];
            part.slidingFrictionEstimate = Frictions[index];
        }

        public static Color ColorFor(string materialName) => Colors[Find(materialName)];
        public static float DensityFor(string materialName) => Densities[Find(materialName)];
        public static float FrictionFor(string materialName) => Frictions[Find(materialName)];
    }
}
