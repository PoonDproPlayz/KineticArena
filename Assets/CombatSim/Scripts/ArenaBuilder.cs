using UnityEngine;

namespace CombatSim
{
    public sealed class ArenaBuilder : MonoBehaviour
    {
        [Min(500f)]
        public float widthMm = 2000f;

        [Min(500f)]
        public float depthMm = 2000f;

        [Min(100f)]
        public float wallHeightMm = 300f;

        [Min(5f)]
        public float wallThicknessMm = 20f;

        public Material floorMaterial;

        public Material wallMaterial;

        public Material markingMaterial;

        private const string GeneratedRootName = "Generated Arena";

        [ContextMenu("Rebuild Arena")]
        public void RebuildArena()
        {
            Transform oldRoot = transform.Find(GeneratedRootName);
            if (oldRoot != null)
            {
                if (Application.isPlaying)
                {
                    Destroy(oldRoot.gameObject);
                }
                else
                {
                    DestroyImmediate(oldRoot.gameObject);
                }
            }

            GameObject root = new GameObject(GeneratedRootName);
            root.transform.SetParent(transform, false);

            float width = Units.MillimetresToMetres(Mathf.Max(500f, widthMm));
            float depth = Units.MillimetresToMetres(Mathf.Max(500f, depthMm));
            float wallHeight = Units.MillimetresToMetres(Mathf.Max(100f, wallHeightMm));
            float wallThickness = Units.MillimetresToMetres(Mathf.Max(5f, wallThicknessMm));

            CreateBox(
                "Floor",
                root.transform,
                new Vector3(0f, -wallThickness / 2f, 0f),
                new Vector3(width, wallThickness, depth),
                floorMaterial);

            float wallY = wallHeight / 2f;
            float halfWidth = width / 2f;
            float halfDepth = depth / 2f;

            CreateBox(
                "North Wall",
                root.transform,
                new Vector3(0f, wallY, halfDepth + wallThickness / 2f),
                new Vector3(width + wallThickness * 2f, wallHeight, wallThickness),
                wallMaterial);
            CreateBox(
                "South Wall",
                root.transform,
                new Vector3(0f, wallY, -halfDepth - wallThickness / 2f),
                new Vector3(width + wallThickness * 2f, wallHeight, wallThickness),
                wallMaterial);
            CreateBox(
                "East Wall",
                root.transform,
                new Vector3(halfWidth + wallThickness / 2f, wallY, 0f),
                new Vector3(wallThickness, wallHeight, depth),
                wallMaterial);
            CreateBox(
                "West Wall",
                root.transform,
                new Vector3(-halfWidth - wallThickness / 2f, wallY, 0f),
                new Vector3(wallThickness, wallHeight, depth),
                wallMaterial);

            CreateMarkings(root.transform, width, depth, wallThickness);
        }

        private static void CreateBox(
            string boxName,
            Transform parent,
            Vector3 position,
            Vector3 size,
            Material material)
        {
            GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = boxName;
            box.transform.SetParent(parent, false);
            box.transform.localPosition = position;
            box.transform.localScale = size;

            if (material != null)
            {
                box.GetComponent<Renderer>().sharedMaterial = material;
            }
        }

        private void CreateMarkings(Transform parent, float width, float depth, float wallThickness)
        {
            if (markingMaterial == null)
            {
                return;
            }

            float lineThickness = 0.004f;
            const float surfaceHeight = 0.001f;
            int widthDivisions = Mathf.FloorToInt(width / 0.5f);
            int depthDivisions = Mathf.FloorToInt(depth / 0.5f);

            for (int i = 1; i < widthDivisions; i++)
            {
                float x = -width / 2f + width * i / widthDivisions;
                CreateMarking(
                    "Arena Grid",
                    parent,
                    new Vector3(x, surfaceHeight, 0f),
                    new Vector3(lineThickness, 0.001f, depth),
                    markingMaterial);
            }

            for (int i = 1; i < depthDivisions; i++)
            {
                float z = -depth / 2f + depth * i / depthDivisions;
                CreateMarking(
                    "Arena Grid",
                    parent,
                    new Vector3(0f, surfaceHeight, z),
                    new Vector3(width, 0.001f, lineThickness),
                    markingMaterial);
            }
        }

        private static void CreateMarking(
            string markingName,
            Transform parent,
            Vector3 position,
            Vector3 size,
            Material material)
        {
            GameObject marking = GameObject.CreatePrimitive(PrimitiveType.Cube);
            marking.name = markingName;
            marking.transform.SetParent(parent, false);
            marking.transform.localPosition = position;
            marking.transform.localScale = size;
            marking.GetComponent<Renderer>().sharedMaterial = material;
            DestroyImmediate(marking.GetComponent<Collider>());
        }
    }
}
