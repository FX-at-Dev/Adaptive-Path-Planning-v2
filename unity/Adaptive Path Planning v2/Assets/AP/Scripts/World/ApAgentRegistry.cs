








using System.Collections.Generic;
using UnityEngine;

namespace Ap
{
    public class ApAgentRegistry : MonoBehaviour
    {
        public ApCatalog catalog;
        [Tooltip("Scale each one so its collider matches the class footprint the stack uses.")]
        public bool fitToClassSize = true;

        [Header("Live (read only)")]
        public int count;
        public int seed;

        readonly Dictionary<int, ApAgent> byId = new Dictionary<int, ApAgent>();
        RunData run;
        System.Random rng = new System.Random(1);

        public IEnumerable<ApAgent> All => byId.Values;


        public void Begin(RunData r, int worldSeed)
        {
            Clear();
            run = r;
            seed = worldSeed;
            rng = new System.Random(worldSeed ^ 0x5eed);
        }

        public void Clear()
        {
            byId.Clear();
            for (int i = transform.childCount - 1; i >= 0; i--)
            {
                var c = transform.GetChild(i).gameObject;
                if (Application.isPlaying) Destroy(c); else DestroyImmediate(c);
            }
            count = 0;
        }



        public ApAgent Get(int id, string cls)
        {
            if (byId.TryGetValue(id, out var a) && a != null) return a;
            if (catalog == null) return null;
            var prefab = catalog.RoadUser(cls, rng);
            if (prefab == null) return null;
            var go = Instantiate(prefab, transform);
            go.name = $"{cls} #{id}";
            a = go.GetComponent<ApAgent>() ?? go.AddComponent<ApAgent>();
            a.agentId = id;
            a.agentClass = cls;
            if (fitToClassSize && run != null) Fit(go, cls);
            go.SetActive(false);
            byId[id] = a;
            count = byId.Count;
            return a;
        }

        void Fit(GameObject go, string cls)
        {
            var col = go.GetComponent<BoxCollider>();
            if (col == null) return;
            var want = run.SizeOf(cls);


            var s = go.transform.localScale;
            go.transform.localScale = new Vector3(want.y / Mathf.Max(col.size.x, 0.01f), s.y, want.x / Mathf.Max(col.size.z, 0.01f));
        }


        public void PreviewFrom(RunData r)
        {
            Begin(r, 26037);
            foreach (var f in r.frames)
                for (int k = 0; k < f.agentId.Length; k++)
                {
                    if (byId.ContainsKey(f.agentId[k])) continue;
                    var a = Get(f.agentId[k], k < f.agentClass.Length ? f.agentClass[k] : "car");
                    if (a == null) continue;
                    a.transform.SetPositionAndRotation(Frames.Pos(f.agents[3 * k], f.agents[3 * k + 1]), Frames.Rot(f.agents[3 * k + 2]));
                    a.gameObject.SetActive(true);
                }
        }
    }
}
